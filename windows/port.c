//
// port (windows): the dongle through the FTDI D2XX driver (ftd2xx.dll), so
// it works with the factory configuration - VCP disabled, no COM port ever
// needs to exist. Plus Win32 threads, locks and events for the core.
//
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include "ftd2xx.h"
#include "j2534.h"
#include "port.h"

// D2XX loaded at runtime so a missing driver is a clean error, and the same
// source serves both bitnesses (SysWOW64/System32 both install "ftd2xx.dll")
typedef FT_STATUS(WINAPI *pFT_OpenEx)(PVOID, DWORD, FT_HANDLE *);
typedef FT_STATUS(WINAPI *pFT_Close)(FT_HANDLE);
typedef FT_STATUS(WINAPI *pFT_Read)(FT_HANDLE, LPVOID, DWORD, LPDWORD);
typedef FT_STATUS(WINAPI *pFT_Write)(FT_HANDLE, LPVOID, DWORD, LPDWORD);
typedef FT_STATUS(WINAPI *pFT_Purge)(FT_HANDLE, ULONG);
typedef FT_STATUS(WINAPI *pFT_SetTimeouts)(FT_HANDLE, ULONG, ULONG);
typedef FT_STATUS(WINAPI *pFT_SetLatencyTimer)(FT_HANDLE, UCHAR);
typedef FT_STATUS(WINAPI *pFT_ResetDevice)(FT_HANDLE);
typedef FT_STATUS(WINAPI *pFT_GetQueueStatus)(FT_HANDLE, LPDWORD);
typedef FT_STATUS(WINAPI *pFT_SetEventNotification)(FT_HANDLE, DWORD, PVOID);
typedef FT_STATUS(WINAPI *pFT_SetUSBParameters)(FT_HANDLE, ULONG, ULONG);

static struct
{
	HMODULE mod;
	pFT_OpenEx OpenEx;
	pFT_Close Close;
	pFT_Read Read;
	pFT_Write Write;
	pFT_Purge Purge;
	pFT_SetTimeouts SetTimeouts;
	pFT_SetLatencyTimer SetLatencyTimer;
	pFT_ResetDevice ResetDevice;
	pFT_GetQueueStatus GetQueueStatus;
	pFT_SetEventNotification SetEventNotification;
	pFT_SetUSBParameters SetUSBParameters;
} d2xx;

static FT_HANDLE ft;
static HANDLE rxchar_ev; // FTDI signals this when RX data arrives (FT_EVENT_RXCHAR)

static int load_d2xx(void)
{
	if (d2xx.mod)
		return 1;
	d2xx.mod = LoadLibraryA("ftd2xx.dll");
	if (!d2xx.mod)
		return 0;
	d2xx.OpenEx = (pFT_OpenEx)(void *)GetProcAddress(d2xx.mod, "FT_OpenEx");
	d2xx.Close = (pFT_Close)(void *)GetProcAddress(d2xx.mod, "FT_Close");
	d2xx.Read = (pFT_Read)(void *)GetProcAddress(d2xx.mod, "FT_Read");
	d2xx.Write = (pFT_Write)(void *)GetProcAddress(d2xx.mod, "FT_Write");
	d2xx.Purge = (pFT_Purge)(void *)GetProcAddress(d2xx.mod, "FT_Purge");
	d2xx.SetTimeouts = (pFT_SetTimeouts)(void *)GetProcAddress(d2xx.mod, "FT_SetTimeouts");
	d2xx.SetLatencyTimer = (pFT_SetLatencyTimer)(void *)GetProcAddress(d2xx.mod, "FT_SetLatencyTimer");
	d2xx.ResetDevice = (pFT_ResetDevice)(void *)GetProcAddress(d2xx.mod, "FT_ResetDevice");
	d2xx.GetQueueStatus = (pFT_GetQueueStatus)(void *)GetProcAddress(d2xx.mod, "FT_GetQueueStatus");
	d2xx.SetEventNotification = (pFT_SetEventNotification)(void *)GetProcAddress(d2xx.mod, "FT_SetEventNotification");
	d2xx.SetUSBParameters = (pFT_SetUSBParameters)(void *)GetProcAddress(d2xx.mod, "FT_SetUSBParameters");
	if (!d2xx.OpenEx || !d2xx.Close || !d2xx.Read || !d2xx.Write ||
	    !d2xx.Purge || !d2xx.SetTimeouts || !d2xx.SetLatencyTimer || !d2xx.ResetDevice ||
	    !d2xx.GetQueueStatus || !d2xx.SetEventNotification || !d2xx.SetUSBParameters)
	{
		FreeLibrary(d2xx.mod);
		d2xx.mod = NULL;
		return 0;
	}
	return 1;
}

long port_open(const char *name, char *err, int errsize)
{
	if (!load_d2xx())
	{
		snprintf(err, errsize, "ftd2xx.dll not found - install the FTDI D2XX driver");
		return ERR_DEVICE_NOT_CONNECTED;
	}

	// name picks a dongle by serial number; otherwise the first CANUSB
	FT_STATUS st = FT_DEVICE_NOT_FOUND;
	if (name && *name)
		st = d2xx.OpenEx((PVOID)name, FT_OPEN_BY_SERIAL_NUMBER, &ft);
	if (st != FT_OK)
		st = d2xx.OpenEx((PVOID) "CANUSB", FT_OPEN_BY_DESCRIPTION, &ft);
	if (st != FT_OK)
	{
		snprintf(err, errsize, "no CANUSB found (FT_OpenEx: %lu)", (unsigned long)st);
		return st == FT_DEVICE_NOT_OPENED ? ERR_DEVICE_IN_USE : ERR_DEVICE_NOT_CONNECTED;
	}

	d2xx.ResetDevice(ft);
	// Large USB transfer/buffer size so bursts don't overflow the driver's RX
	// ring before the reader drains it; 2ms latency batches without adding
	// noticeable response delay.
	d2xx.SetUSBParameters(ft, 65536, 65536);
	d2xx.SetLatencyTimer(ft, 2);
	d2xx.SetTimeouts(ft, 100, 500); // read timeout is only a safety net now
	d2xx.Purge(ft, FT_PURGE_RX | FT_PURGE_TX);

	// Register the RX-data event before the reader starts waiting on it
	rxchar_ev = CreateEventA(NULL, FALSE, FALSE, NULL);
	d2xx.SetEventNotification(ft, FT_EVENT_RXCHAR, rxchar_ev);
	return STATUS_NOERROR;
}

void port_close(void)
{
	d2xx.Close(ft);
	CloseHandle(rxchar_ev);
}

int port_write(const void *buf, int len)
{
	DWORD written = 0;
	FT_STATUS st = d2xx.Write(ft, (LPVOID)buf, (DWORD)len, &written);
	return (st == FT_OK && written == (DWORD)len) ? 0 : -1;
}

// Read exactly what the driver has queued (an immediate, non-blocking read),
// else sleep on the RX event until more arrives. Draining available bytes
// avoids FT_Read's "block until buffer full or timeout" behaviour, which
// otherwise throttled us to ~one 20ms read per call and overflowed the
// driver's RX buffer under load - lost frames and stalled acks.
int port_read(void *buf, int size)
{
	DWORD avail = 0, got = 0;
	if (d2xx.GetQueueStatus(ft, &avail) != FT_OK)
		return -1;
	if (avail == 0)
	{
		// Wake as soon as the driver signals data; the 50ms cap lets the
		// caller re-check its run flag if a wake is ever missed.
		WaitForSingleObject(rxchar_ev, 50);
		return 0;
	}
	if (avail > (DWORD)size)
		avail = (DWORD)size;
	if (d2xx.Read(ft, buf, avail, &got) != FT_OK)
		return -1;
	return (int)got;
}

// ------------------------------------------------ threads, locks, events --

struct mutex
{
	CRITICAL_SECTION cs;
};

mutex_t *mutex_new(void)
{
	mutex_t *m = malloc(sizeof(*m));
	InitializeCriticalSection(&m->cs);
	return m;
}

void mutex_free(mutex_t *m)
{
	DeleteCriticalSection(&m->cs);
	free(m);
}

void mutex_lock(mutex_t *m) { EnterCriticalSection(&m->cs); }
void mutex_unlock(mutex_t *m) { LeaveCriticalSection(&m->cs); }

// An event_t * is the Win32 auto-reset event HANDLE itself
event_t *event_new(void) { return (event_t *)CreateEventA(NULL, FALSE, FALSE, NULL); }
void event_free(event_t *e) { CloseHandle((HANDLE)e); }
void event_set(event_t *e) { SetEvent((HANDLE)e); }
void event_reset(event_t *e) { ResetEvent((HANDLE)e); }
int event_wait(event_t *e, unsigned ms) { return WaitForSingleObject((HANDLE)e, ms) == WAIT_OBJECT_0; }

struct thread
{
	HANDLE h;
	void (*fn)(void);
};

static DWORD WINAPI thread_main(LPVOID arg)
{
	((thread_t *)arg)->fn();
	return 0;
}

thread_t *thread_start(void (*fn)(void))
{
	thread_t *t = malloc(sizeof(*t));
	t->fn = fn;
	t->h = CreateThread(NULL, 0, thread_main, t, 0, NULL);
	return t;
}

void thread_join(thread_t *t)
{
	WaitForSingleObject(t->h, 1000);
	CloseHandle(t->h);
	free(t);
}

uint32_t ticks_ms(void) { return GetTickCount(); }

uint32_t micros(void)
{
	static LARGE_INTEGER freq;
	LARGE_INTEGER now;
	if (freq.QuadPart == 0)
		QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&now);
	// Split so the multiply can't overflow (it did after ~10 days of uptime)
	return (uint32_t)((now.QuadPart / freq.QuadPart) * 1000000 +
	                  (now.QuadPart % freq.QuadPart) * 1000000 / freq.QuadPart);
}

void sleep_ms(unsigned ms) { Sleep(ms); }
