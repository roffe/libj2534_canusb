//
// port (linux): the dongle through the in-tree ftdi_sio driver, which binds
// both the CANUSB and the canusb2 clone as /dev/ttyUSB* - no libftd2xx, no
// libusb, no udev rule. Plus pthreads for the core.
//
#define _GNU_SOURCE // strcasestr
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "j2534.h"
#include "port.h"

static int fd = -1;

// Which tty: name if it is a path, else $CANUSB_PORT, else a
// /dev/serial/by-id entry naming a CANUSB (udev calls them
// usb-LAWICEL_CANUSB_<serial>-if00-port0) - the one whose serial is name
// if there is one, like FT_OPEN_BY_SERIAL_NUMBER on windows.
static const char *find_port(const char *name, char *path, size_t size)
{
	if (name && name[0] == '/')
		return name;
	const char *env = getenv("CANUSB_PORT");
	if (env && *env)
		return env;
	DIR *d = opendir("/dev/serial/by-id");
	if (!d)
		return NULL;
	int best = 0; // 1 = some CANUSB, 2 = the requested serial
	for (struct dirent *e; best < 2 && (e = readdir(d));)
	{
		if (!strcasestr(e->d_name, "CANUSB"))
			continue;
		int score = (name && *name && strstr(e->d_name, name)) ? 2 : 1;
		if (score > best)
		{
			best = score;
			snprintf(path, size, "/dev/serial/by-id/%s", e->d_name);
		}
	}
	closedir(d);
	return best ? path : NULL;
}

long port_open(const char *name, char *err, int errsize)
{
	char buf[PATH_MAX], tty[PATH_MAX];
	const char *path = find_port(name, buf, sizeof(buf));
	if (!path)
	{
		snprintf(err, errsize, "no CANUSB found; set CANUSB_PORT");
		return ERR_DEVICE_NOT_CONNECTED;
	}
	if (!realpath(path, tty)) // short name for the 80 char error text
		snprintf(tty, sizeof(tty), "%s", path);

	fd = open(tty, O_RDWR | O_NOCTTY | O_CLOEXEC);
	if (fd < 0)
	{
		int e = errno;
		snprintf(err, errsize, "open %s: %s%s", tty, strerror(e),
		         e == EACCES ? " (join uucp/dialout)" : "");
		return e == EBUSY ? ERR_DEVICE_IN_USE : ERR_DEVICE_NOT_CONNECTED;
	}
	// Exclusive: a second opener would steal half of the slcan replies
	ioctl(fd, TIOCEXCL);

	struct termios t;
	if (tcgetattr(fd, &t) == 0)
	{
		cfmakeraw(&t);
		t.c_cflag |= CLOCAL | CREAD;
		t.c_cflag &= ~CRTSCTS;
		t.c_cc[VMIN] = 1;
		t.c_cc[VTIME] = 0;
		tcsetattr(fd, TCSANOW, &t);
	}
	// Every command waits for its ack, so the FTDI latency timer is the
	// throughput knob: low_latency makes ftdi_sio set it to 1ms (older
	// kernels default to 16ms). Best effort.
	struct serial_struct ss;
	if (ioctl(fd, TIOCGSERIAL, &ss) == 0)
	{
		ss.flags |= ASYNC_LOW_LATENCY;
		ioctl(fd, TIOCSSERIAL, &ss);
	}
	tcflush(fd, TCIOFLUSH);
	return STATUS_NOERROR;
}

void port_close(void)
{
	close(fd);
	fd = -1;
}

int port_write(const void *buf, int len)
{
	const char *p = buf;
	while (len > 0)
	{
		ssize_t n = write(fd, p, (size_t)len);
		if (n < 0 && errno == EINTR)
			continue;
		if (n <= 0)
			return -1;
		p += n;
		len -= (int)n;
	}
	return 0;
}

int port_read(void *buf, int size)
{
	struct pollfd p = {.fd = fd, .events = POLLIN};
	int r = poll(&p, 1, 50);
	if (r == 0 || (r < 0 && errno == EINTR))
		return 0;
	if (r < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL)))
		return -1; // unplugged
	ssize_t n = read(fd, buf, (size_t)size);
	if (n < 0 && (errno == EINTR || errno == EAGAIN))
		return 0;
	return n > 0 ? (int)n : -1;
}

// ------------------------------------------------ threads, locks, events --

struct mutex
{
	pthread_mutex_t m;
};

mutex_t *mutex_new(void)
{
	mutex_t *m = malloc(sizeof(*m));
	pthread_mutex_init(&m->m, NULL);
	return m;
}

void mutex_free(mutex_t *m)
{
	pthread_mutex_destroy(&m->m);
	free(m);
}

void mutex_lock(mutex_t *m) { pthread_mutex_lock(&m->m); }
void mutex_unlock(mutex_t *m) { pthread_mutex_unlock(&m->m); }

struct event
{
	pthread_mutex_t m;
	pthread_cond_t c;
	int set;
};

event_t *event_new(void)
{
	event_t *e = calloc(1, sizeof(*e));
	pthread_condattr_t a;
	pthread_condattr_init(&a);
	pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
	pthread_mutex_init(&e->m, NULL);
	pthread_cond_init(&e->c, &a);
	pthread_condattr_destroy(&a);
	return e;
}

void event_free(event_t *e)
{
	pthread_cond_destroy(&e->c);
	pthread_mutex_destroy(&e->m);
	free(e);
}

void event_set(event_t *e)
{
	pthread_mutex_lock(&e->m);
	e->set = 1;
	pthread_cond_signal(&e->c);
	pthread_mutex_unlock(&e->m);
}

void event_reset(event_t *e)
{
	pthread_mutex_lock(&e->m);
	e->set = 0;
	pthread_mutex_unlock(&e->m);
}

int event_wait(event_t *e, unsigned ms)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts.tv_sec += ms / 1000;
	ts.tv_nsec += (long)(ms % 1000) * 1000000;
	if (ts.tv_nsec >= 1000000000)
	{
		ts.tv_sec++;
		ts.tv_nsec -= 1000000000;
	}
	pthread_mutex_lock(&e->m);
	while (!e->set && pthread_cond_timedwait(&e->c, &e->m, &ts) == 0)
		;
	int r = e->set;
	e->set = 0; // auto-reset
	pthread_mutex_unlock(&e->m);
	return r;
}

struct thread
{
	pthread_t t;
	void (*fn)(void);
};

static void *thread_main(void *arg)
{
	((thread_t *)arg)->fn();
	return NULL;
}

thread_t *thread_start(void (*fn)(void))
{
	thread_t *t = malloc(sizeof(*t));
	t->fn = fn;
	pthread_create(&t->t, NULL, thread_main, t);
	return t;
}

void thread_join(thread_t *t)
{
	pthread_join(t->t, NULL);
	free(t);
}

static uint64_t now_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

uint32_t ticks_ms(void) { return (uint32_t)(now_us() / 1000); }
uint32_t micros(void) { return (uint32_t)now_us(); }

void sleep_ms(unsigned ms)
{
	struct timespec ts = {ms / 1000, (long)(ms % 1000) * 1000000};
	while (nanosleep(&ts, &ts) != 0 && errno == EINTR)
		;
}
