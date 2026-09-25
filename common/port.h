//
// port: the platform layer under the J2534 core - the dongle's byte stream,
// plus the threads, locks, events and clocks the core needs.
// windows/port.c talks FTDI D2XX, linux/port.c the ftdi_sio tty.
//
#ifndef PORT_H
#define PORT_H

#include <stdint.h>

// Open the dongle. name (may be NULL or empty) picks a specific one, as the
// PassThruOpen pName. Returns a J2534 status; on failure err says why.
long port_open(const char *name, char *err, int errsize);
void port_close(void);
int port_write(const void *buf, int len); // 0 ok, -1 device gone
int port_read(void *buf, int size);       // bytes read, 0 after ~50ms idle, -1 device gone

typedef struct mutex mutex_t;
mutex_t *mutex_new(void);
void mutex_free(mutex_t *m);
void mutex_lock(mutex_t *m);
void mutex_unlock(mutex_t *m);

// Auto-reset: a successful wait consumes the signal
typedef struct event event_t;
event_t *event_new(void);
void event_free(event_t *e);
void event_set(event_t *e);
void event_reset(event_t *e);
int event_wait(event_t *e, unsigned ms); // 1 signalled, 0 timed out

typedef struct thread thread_t;
thread_t *thread_start(void (*fn)(void));
void thread_join(thread_t *t);

uint32_t ticks_ms(void); // monotonic, wraps
uint32_t micros(void);   // monotonic, wraps
void sleep_ms(unsigned ms);

#endif // PORT_H
