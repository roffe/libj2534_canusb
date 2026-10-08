//
// libj2534_canusb: SAE J2534-1 v04.04 PassThru driver for the LAWICEL CANUSB
// (and the canusb2 clone). This is the platform-free core; the byte stream,
// threads and clocks come from port.h - windows/port.c (FTDI D2XX, so the
// dongle works with VCP disabled) or linux/port.c (the ftdi_sio tty).
//
// Scope: raw CAN (protocol 5) and ISO15765 (protocol 6, normal addressing).
// The ISO-TP segmentation runs entirely in this driver on top of raw slcan
// frames; flow controls for incoming segmented messages are sent from the
// periodic thread (the reader thread cannot wait on its own ack).
//
// The device speaks Lawicel slcan: commands acked with CR, rejected with BELL,
// t/T transmits acked with z/Z, received frames pushed as t/T lines. One
// command is in flight at a time (each write waits for its ack).
// ponytail: synchronous acks cost ~1 RTT per transmitted frame; a credit
// window like gocan's is the upgrade path if flashing throughput matters.
//
// NEVER send 'X' to this firmware - it reboots into the DFU bootloader.
//

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "j2534.h"
#include "port.h"
#include "proto.h"

#define DLL_VERSION "1.0.0"
#define API_VERSION "04.04"

#define ACK_TIMEOUT_MS 500
#define MAX_FILTERS 10
#define MAX_PERIODIC 10
#define RXQ_SIZE 16384 // power of two

typedef struct
{
	uint32_t id;
	uint8_t ext;
	uint8_t dlc;
	uint8_t data[8];
	unsigned long ts;
} rxrec_t;

typedef struct
{
	int used;
	unsigned long type; // PASS_FILTER / BLOCK_FILTER / FLOW_CONTROL_FILTER
	unsigned long len;
	uint8_t mask[12];
	uint8_t pattern[12];
	// FLOW_CONTROL_FILTER (ISO15765) only:
	uint32_t fcid; // CAN id we transmit flow controls (and match writes) on
	uint8_t ext;   // 29 bit conversation
	uint8_t pad;   // pad frames to dlc 8 (ISO15765_FRAME_PAD on the FC msg)
} filter_t;

// Reassembled ISO15765 messages and indications waiting for PassThruReadMsgs
typedef struct
{
	unsigned long rxstatus;
	unsigned long ts;
	unsigned len; // bytes in data (4 byte id + payload)
	uint8_t data[4 + ISOTP_MAX];
} isorec_t;
#define ISOQ_SIZE 64 // power of two

// Flow controls the reader thread wants sent (the periodic thread sends them)
typedef struct
{
	uint32_t id;
	uint8_t ext, dlc;
	uint8_t data[8];
} fcreq_t;
#define FCQ_SIZE 8 // power of two

typedef struct
{
	int used;
	unsigned long interval;
	uint32_t due;
	uint32_t id;
	uint8_t ext;
	uint8_t dlc;
	uint8_t data[8];
} periodic_t;

static struct
{
	int open;	   // PassThruOpen done
	int connected; // CAN channel up
	volatile int dead;
	unsigned long baud;

	thread_t *reader;
	volatile int run;

	mutex_t *cmd_lock;	 // serializes command+ack round trips
	event_t *ack_ev;	 // signalled by the reader
	volatile int ack_ok; // 1 = CR/z/Z/reply, 0 = BELL
	char reply[64];		 // last non-frame reply line (V/N/F/E)
	volatile int reply_len;

	mutex_t *q_lock;
	rxrec_t rxq[RXQ_SIZE];
	volatile unsigned q_head, q_tail; // head = write position
	event_t *rx_ev;					  // set on enqueue

	filter_t filters[MAX_FILTERS];
	int have_pass_filter;

	periodic_t periodic[MAX_PERIODIC];
	thread_t *per_thread;
	event_t *per_wake;
	volatile int per_run;

	// ISO15765 state (proto == ISO15765)
	unsigned long proto;			 // CAN or ISO15765 for the open channel
	unsigned long iso_bs, iso_stmin; // advertised in the flow controls we send
	isotp_rx_t iso_rx[MAX_FILTERS];	 // one reassembly per conversation
	isorec_t isoq[ISOQ_SIZE];		 // q_lock protected, like rxq
	volatile unsigned iq_head, iq_tail;
	fcreq_t fcq[FCQ_SIZE]; // q_lock protected
	volatile unsigned fq_head, fq_tail;
	volatile int txconv; // filter index awaiting a flow control, -1 none
	event_t *fc_ev;		 // reader signals when that FC arrives
	volatile int fc_fs, fc_bs, fc_stmin;
} dev;

static char last_error[80] = "";

static void __attribute__((format(printf, 1, 2))) seterr(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(last_error, sizeof(last_error), fmt, ap);
	va_end(ap);
}

// ---------------------------------------------------------------- filters --

// J2534 message bytes for a CAN frame: 4 byte big endian ID, then the data
static unsigned msg_bytes(uint8_t *out, uint32_t id, const uint8_t *data, int dlc)
{
	out[0] = (uint8_t)(id >> 24);
	out[1] = (uint8_t)(id >> 16);
	out[2] = (uint8_t)(id >> 8);
	out[3] = (uint8_t)(id);
	if (dlc > 0)
		memcpy(out + 4, data, (size_t)dlc);
	return (unsigned)(4 + dlc);
}

// A channel with no PASS_FILTER delivers nothing (per spec); with one, a
// frame must match some pass filter and no block filter.
static int filters_accept(const rxrec_t *r)
{
	uint8_t bytes[12];
	unsigned blen = msg_bytes(bytes, r->id, r->data, r->dlc);
	int pass = 0;

	if (!dev.have_pass_filter)
		return 0;

	for (int i = 0; i < MAX_FILTERS; i++)
	{
		const filter_t *f = &dev.filters[i];
		if (!f->used)
			continue;
		if (!filter_match(bytes, blen, f->mask, f->pattern, f->len))
			continue;
		if (f->type == BLOCK_FILTER)
			return 0;
		pass = 1;
	}
	return pass;
}

// ----------------------------------------------------------- reader thread --

// Queue a message (or indication) for an ISO15765 PassThruReadMsgs.
// payload may be NULL for indications (id only, len must be 0).
static void iso_enqueue(uint32_t id, int ext, unsigned long rxstatus,
						const uint8_t *payload, unsigned len)
{
	mutex_lock(dev.q_lock);
	unsigned next = (dev.iq_head + 1) & (ISOQ_SIZE - 1);
	if (next == dev.iq_tail) // full: drop the oldest
		dev.iq_tail = (dev.iq_tail + 1) & (ISOQ_SIZE - 1);
	isorec_t *r = &dev.isoq[dev.iq_head];
	r->rxstatus = rxstatus | (ext ? CAN_29BIT_ID : 0);
	r->ts = micros();
	r->len = msg_bytes(r->data, id, payload, 0);
	if (len)
	{
		memcpy(r->data + 4, payload, len);
		r->len = 4 + len;
	}
	dev.iq_head = next;
	mutex_unlock(dev.q_lock);
	event_set(dev.rx_ev);
}

// Ask the periodic thread to transmit a flow control on the filter's FC id.
static void iso_queue_fc(const filter_t *f)
{
	fcreq_t q = {.id = f->fcid, .ext = f->ext};
	int dlc = isotp_mk_fc(q.data, 0 /* CTS */, (int)dev.iso_bs, (int)dev.iso_stmin);
	if (f->pad)
	{
		memset(q.data + dlc, 0, (size_t)(8 - dlc));
		dlc = 8;
	}
	q.dlc = (uint8_t)dlc;

	mutex_lock(dev.q_lock);
	unsigned next = (dev.fq_head + 1) & (FCQ_SIZE - 1);
	if (next != dev.fq_tail) // full: drop it, the sender will time out and retry
	{
		dev.fcq[dev.fq_head] = q;
		dev.fq_head = next;
	}
	mutex_unlock(dev.q_lock);
	event_set(dev.per_wake);
}

// One received CAN frame on an ISO15765 channel: find its conversation and
// feed the reassembler. Frames matching no flow control filter are dropped.
static void iso_rx_frame(const rxrec_t *r)
{
	uint8_t idb[4];
	msg_bytes(idb, r->id, NULL, 0);

	for (int i = 0; i < MAX_FILTERS; i++)
	{
		filter_t *f = &dev.filters[i];
		if (!f->used || f->ext != r->ext)
			continue;
		if (!filter_match(idb, 4, f->mask, f->pattern, 4))
			continue;

		int fs = 0, bs = 0, stmin = 0;
		isotp_rx_t *rx = &dev.iso_rx[i];
		switch (isotp_rx_feed(rx, r->data, r->dlc, &fs, &bs, &stmin))
		{
		case ISOTP_MSG:
			iso_enqueue(r->id, r->ext, 0, rx->buf, rx->len);
			break;
		case ISOTP_SEND_FC:
			iso_queue_fc(f);
			iso_enqueue(r->id, r->ext, ISO15765_FIRST_FRAME, NULL, 0);
			break;
		case ISOTP_FLOWCTRL:
			if (dev.txconv == i)
			{
				dev.fc_fs = fs;
				dev.fc_bs = bs;
				dev.fc_stmin = stmin;
				event_set(dev.fc_ev);
			}
			break;
		}
		return; // first matching conversation owns the frame
	}
}

static void rx_enqueue(uint32_t id, int ext, const uint8_t *data, int dlc)
{
	rxrec_t r = {.id = id, .ext = (uint8_t)ext, .dlc = (uint8_t)dlc, .ts = micros()};
	memcpy(r.data, data, (size_t)dlc);

	if (dev.proto == ISO15765)
	{
		iso_rx_frame(&r);
		return;
	}

	if (!filters_accept(&r))
		return;

	mutex_lock(dev.q_lock);
	unsigned next = (dev.q_head + 1) & (RXQ_SIZE - 1);
	if (next == dev.q_tail) // full: drop the oldest
		dev.q_tail = (dev.q_tail + 1) & (RXQ_SIZE - 1);
	dev.rxq[dev.q_head] = r;
	dev.q_head = next;
	mutex_unlock(dev.q_lock);
	event_set(dev.rx_ev);
}

static void reader_line(const char *line, int len)
{
	uint32_t id;
	int ext, dlc;
	uint8_t data[8];

	int rc = slcan_decode(line, len, &id, &ext, data, &dlc);
	if (rc == 1)
	{
		rx_enqueue(id, ext, data, dlc);
		return;
	}
	if (rc < 0)
		return; // malformed frame line, drop

	// Everything else answers the command in flight: bare CR, z/Z transmit
	// acks, or a reply line (V/N/F/E) whose CR is the ack.
	if (len > 0)
	{
		int n = len < (int)sizeof(dev.reply) - 1 ? len : (int)sizeof(dev.reply) - 1;
		memcpy(dev.reply, line, (size_t)n);
		dev.reply[n] = '\0';
		dev.reply_len = n;
	}
	dev.ack_ok = 1;
	event_set(dev.ack_ev);
}

// Split the dongle's byte stream into lines. port_read returns what has
// arrived, or 0 after ~50ms idle so dev.run gets rechecked.
static void reader_main(void)
{
	char buf[4096], line[80];
	int linelen = 0;

	while (dev.run)
	{
		int got = port_read(buf, sizeof(buf));
		if (got < 0)
		{
			dev.dead = 1;
			dev.ack_ok = 0;
			event_set(dev.ack_ev);
			return;
		}
		for (int i = 0; i < got; i++)
		{
			char c = buf[i];
			if (c == '\a') // BELL: command rejected
			{
				linelen = 0;
				dev.ack_ok = 0;
				event_set(dev.ack_ev);
			}
			else if (c == '\r')
			{
				reader_line(line, linelen);
				linelen = 0;
			}
			else if (linelen < (int)sizeof(line) - 1)
			{
				line[linelen++] = c;
			}
			else
				linelen = 0; // overlong garbage, resync
		}
	}
}

// -------------------------------------------------------- command channel --

// Write one slcan command and wait for its ack. Returns STATUS_NOERROR,
// ERR_FAILED (BELL) or ERR_TIMEOUT / ERR_DEVICE_NOT_CONNECTED.
static long send_cmd(const char *cmd, int len, char *reply_out, int reply_size)
{
	long ret = STATUS_NOERROR;

	if (dev.dead)
		return ERR_DEVICE_NOT_CONNECTED;

	mutex_lock(dev.cmd_lock);
	event_reset(dev.ack_ev);
	dev.reply_len = 0;

	if (port_write(cmd, len) != 0)
	{
		dev.dead = 1;
		ret = ERR_DEVICE_NOT_CONNECTED;
	}
	else if (!event_wait(dev.ack_ev, ACK_TIMEOUT_MS))
	{
		ret = dev.dead ? ERR_DEVICE_NOT_CONNECTED : ERR_TIMEOUT;
	}
	else if (dev.dead)
		ret = ERR_DEVICE_NOT_CONNECTED;
	else if (!dev.ack_ok)
		ret = ERR_FAILED;
	else if (reply_out)
	{
		strncpy(reply_out, dev.reply, (size_t)reply_size - 1);
		reply_out[reply_size - 1] = '\0';
	}

	mutex_unlock(dev.cmd_lock);
	return ret;
}

// ------------------------------------------------------- periodic messages --

static void periodic_main(void)
{
	while (dev.per_run)
	{
		// Flow controls first - an ECU waits ~1s (N_Br) for ours after a FF
		for (;;)
		{
			fcreq_t q;
			int have = 0;
			mutex_lock(dev.q_lock);
			if (dev.fq_tail != dev.fq_head)
			{
				q = dev.fcq[dev.fq_tail];
				dev.fq_tail = (dev.fq_tail + 1) & (FCQ_SIZE - 1);
				have = 1;
			}
			mutex_unlock(dev.q_lock);
			if (!have)
				break;
			char cmd[32];
			int n = slcan_encode(cmd, q.id, q.ext, q.data, q.dlc);
			if (n > 0)
				send_cmd(cmd, n, NULL, 0); // best effort
		}

		uint32_t now = ticks_ms();
		uint32_t wait = 50;

		for (int i = 0; i < MAX_PERIODIC; i++)
		{
			periodic_t *p = &dev.periodic[i];
			if (!p->used)
				continue;
			if ((int32_t)(now - p->due) >= 0)
			{
				char cmd[32];
				int n = slcan_encode(cmd, p->id, p->ext, p->data, p->dlc);
				if (n > 0)
					send_cmd(cmd, n, NULL, 0); // best effort
				p->due = now + p->interval;
			}
			uint32_t left = p->due - now;
			if (left < wait)
				wait = left ? left : 1;
		}
		event_wait(dev.per_wake, wait);
	}
}

// ------------------------------------------------------------------- open --

// Stop the reader and release everything PassThruOpen set up
static void dev_teardown(void)
{
	dev.run = 0;
	thread_join(dev.reader); // returns within port_read's ~50ms idle wait
	port_close();
	mutex_free(dev.cmd_lock);
	mutex_free(dev.q_lock);
	event_free(dev.ack_ev);
	event_free(dev.rx_ev);
	event_free(dev.per_wake);
	event_free(dev.fc_ev);
}

int32_t J2534_API PassThruOpen(void *pName, uint32_t *pDeviceID)
{
	if (!pDeviceID)
	{
		seterr("PassThruOpen: null device id pointer");
		return ERR_NULL_PARAMETER;
	}
	if (dev.open)
	{
		seterr("device already open");
		return ERR_DEVICE_IN_USE;
	}

	// pName, when given, selects a specific dongle by its serial number
	long rc = port_open((const char *)pName, last_error, sizeof(last_error));
	if (rc != STATUS_NOERROR)
		return rc;

	dev.cmd_lock = mutex_new();
	dev.q_lock = mutex_new();
	dev.ack_ev = event_new();
	dev.rx_ev = event_new();
	dev.per_wake = event_new();
	dev.fc_ev = event_new();
	dev.q_head = dev.q_tail = 0;
	dev.dead = 0;
	dev.run = 1;
	dev.reader = thread_start(reader_main);

	// Sync up: a bare CR flushes the device's line buffer, C closes a channel
	// a previous user may have left open, V proves it speaks slcan.
	char reply[64] = {0};
	send_cmd("\r", 1, NULL, 0);
	send_cmd("C\r", 2, NULL, 0);
	if (send_cmd("V\r", 2, reply, sizeof(reply)) != STATUS_NOERROR || reply[0] != 'V')
	{
		dev_teardown();
		seterr("device does not answer like a CANUSB");
		return ERR_DEVICE_NOT_CONNECTED;
	}

	dev.open = 1;
	*pDeviceID = 1;
	return STATUS_NOERROR;
}

int32_t J2534_API PassThruClose(uint32_t DeviceID)
{
	if (DeviceID != 1 || !dev.open)
	{
		seterr("invalid device id");
		return ERR_INVALID_DEVICE_ID;
	}
	if (dev.connected)
		PassThruDisconnect(1);

	dev_teardown();
	memset(&dev.filters, 0, sizeof(dev.filters));
	dev.open = 0;
	return STATUS_NOERROR;
}

// ---------------------------------------------------------------- channel --

// S0..S8 are the presets every CANUSB has. Other rates go out as an SJA1000 BTR0/BTR1 pair
// (sXXYY, 16 MHz)
static const char *baud_to_cmd(unsigned long baud)
{
	static const struct
	{
		unsigned long baud;
		const char *cmd;
	} map[] = {
		{10000, "S0\r"},
		{20000, "S1\r"},
		{50000, "S2\r"},
		{100000, "S3\r"},
		{125000, "S4\r"},
		{250000, "S5\r"},
		{500000, "S6\r"},
		{800000, "S7\r"},
		{1000000, "S8\r"},
		{615384, "s4037\r"},
		{615000, "s4037\r"}, // Trionic 5 P-bus, TrionicCANLib's CAN_BAUD_BTR_615K
		{83333, "s852B\r"},	 // PCAN_BAUD_83K: 16tq x 750ns, 81% sample point
	};
	for (unsigned i = 0; i < sizeof(map) / sizeof(map[0]); i++)
	{
		if (map[i].baud == baud)
			return map[i].cmd;
	}
	return NULL;
}

int32_t J2534_API PassThruConnect(uint32_t DeviceID, uint32_t ProtocolID,
								  uint32_t Flags, uint32_t BaudRate, uint32_t *pChannelID)
{
	(void)Flags;
	const char *scmd;

	if (DeviceID != 1 || !dev.open)
	{
		seterr("invalid device id");
		return ERR_INVALID_DEVICE_ID;
	}
	if (!pChannelID)
		return ERR_NULL_PARAMETER;
	if (ProtocolID != CAN && ProtocolID != ISO15765)
	{
		seterr("only CAN and ISO15765 are supported");
		return ERR_INVALID_PROTOCOL_ID;
	}
	if (dev.connected)
	{
		seterr("channel already connected");
		return ERR_CHANNEL_IN_USE;
	}
	if (!(scmd = baud_to_cmd(BaudRate)))
	{
		seterr("unsupported bitrate %lu", (unsigned long)BaudRate);
		return ERR_INVALID_BAUDRATE;
	}

	send_cmd("C\r", 2, NULL, 0); // make sure it's closed before S
	send_cmd("Z0\r", 3, NULL, 0);
	if (send_cmd(scmd, (int)strlen(scmd), NULL, 0) != STATUS_NOERROR)
	{
		seterr("device rejected bitrate %lu", (unsigned long)BaudRate);
		return ERR_INVALID_BAUDRATE;
	}
	long rc = send_cmd("O\r", 2, NULL, 0);
	if (rc != STATUS_NOERROR)
	{
		seterr("could not open CAN channel");
		return rc == ERR_FAILED ? ERR_FAILED : rc;
	}

	dev.baud = BaudRate;
	dev.q_head = dev.q_tail = 0;
	dev.iq_head = dev.iq_tail = 0;
	dev.fq_head = dev.fq_tail = 0;
	memset(dev.filters, 0, sizeof(dev.filters));
	dev.have_pass_filter = 0;
	memset(dev.iso_rx, 0, sizeof(dev.iso_rx));
	dev.iso_bs = 0;
	dev.iso_stmin = 0;
	dev.txconv = -1;
	memset(dev.periodic, 0, sizeof(dev.periodic));
	dev.proto = ProtocolID; // set before frames can be routed
	dev.per_run = 1;
	dev.per_thread = thread_start(periodic_main);
	dev.connected = 1;
	*pChannelID = 1;
	return STATUS_NOERROR;
}

int32_t J2534_API PassThruDisconnect(uint32_t ChannelID)
{
	if (ChannelID != 1 || !dev.connected)
	{
		seterr("invalid channel id");
		return ERR_INVALID_CHANNEL_ID;
	}
	dev.per_run = 0;
	event_set(dev.per_wake);
	thread_join(dev.per_thread);

	send_cmd("C\r", 2, NULL, 0);
	dev.connected = 0;
	memset(dev.filters, 0, sizeof(dev.filters));
	dev.have_pass_filter = 0;
	return STATUS_NOERROR;
}

// --------------------------------------------------------------- messages --

int32_t J2534_API PassThruReadMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg,
								   uint32_t *pNumMsgs, uint32_t Timeout)
{
	if (ChannelID != 1 || !dev.connected)
		return ERR_INVALID_CHANNEL_ID;
	if (!pMsg || !pNumMsgs)
		return ERR_NULL_PARAMETER;

	unsigned long want = *pNumMsgs, got = 0;
	uint32_t deadline = ticks_ms() + Timeout;

	while (got < want)
	{
		int have = 0;

		mutex_lock(dev.q_lock);
		if (dev.proto == ISO15765)
		{
			if (dev.iq_tail != dev.iq_head)
			{
				isorec_t *r = &dev.isoq[dev.iq_tail];
				PASSTHRU_MSG *m = &pMsg[got++];
				memset(m, 0, sizeof(*m) - sizeof(m->Data));
				m->ProtocolID = ISO15765;
				m->RxStatus = r->rxstatus;
				m->Timestamp = r->ts;
				m->DataSize = r->len;
				m->ExtraDataIndex = r->len;
				memcpy(m->Data, r->data, r->len);
				dev.iq_tail = (dev.iq_tail + 1) & (ISOQ_SIZE - 1);
				have = 1;
			}
		}
		else if (dev.q_tail != dev.q_head)
		{
			rxrec_t r = dev.rxq[dev.q_tail];
			dev.q_tail = (dev.q_tail + 1) & (RXQ_SIZE - 1);
			PASSTHRU_MSG *m = &pMsg[got++];
			memset(m, 0, sizeof(*m) - sizeof(m->Data));
			m->ProtocolID = CAN;
			m->RxStatus = r.ext ? CAN_29BIT_ID : 0;
			m->Timestamp = r.ts;
			m->DataSize = msg_bytes(m->Data, r.id, r.data, r.dlc);
			m->ExtraDataIndex = m->DataSize;
			have = 1;
		}
		mutex_unlock(dev.q_lock);

		if (have)
			continue;

		if (dev.dead)
		{
			*pNumMsgs = got;
			return ERR_DEVICE_NOT_CONNECTED;
		}
		int32_t left = (int32_t)(deadline - ticks_ms());
		if (Timeout == 0 || left <= 0)
			break;
		event_wait(dev.rx_ev, (unsigned)left);
	}

	*pNumMsgs = got;
	if (got > 0)
		return STATUS_NOERROR;
	return Timeout == 0 ? ERR_BUFFER_EMPTY : ERR_TIMEOUT;
}

// Send one raw CAN frame and wait for the dongle's ack.
static long send_frame(uint32_t id, int ext, const uint8_t *data, int dlc)
{
	char cmd[32];
	int n = slcan_encode(cmd, id, ext, data, dlc);
	if (n < 0)
	{
		seterr("invalid CAN id %08lx", (unsigned long)id);
		return ERR_INVALID_MSG;
	}
	long rc = send_cmd(cmd, n, NULL, 0);
	if (rc == ERR_FAILED)
		seterr("device rejected transmit (bus off / tx full?)");
	return rc;
}

// One ISO15765 write: segment the payload, obey the peer's flow control,
// then queue the TxDone indication the spec asks for.
static long iso_write(PASSTHRU_MSG *m)
{
	if (m->DataSize < 5 || m->DataSize > 4 + ISOTP_MAX)
	{
		seterr("ISO15765 message must be 5..4099 bytes (id + payload)");
		return ERR_INVALID_MSG;
	}

	uint32_t id = ((uint32_t)m->Data[0] << 24) | ((uint32_t)m->Data[1] << 16) |
				  ((uint32_t)m->Data[2] << 8) | m->Data[3];
	int ext = (m->TxFlags & CAN_29BIT_ID) != 0;
	int pad = (m->TxFlags & ISO15765_FRAME_PAD) != 0;
	const uint8_t *p = m->Data + 4;
	unsigned len = m->DataSize - 4;
	uint8_t f[8];
	long rc;

	if (len <= 7) // single frame
	{
		int dlc = isotp_mk_sf(f, p, len);
		if (pad)
		{
			memset(f + dlc, 0, (size_t)(8 - dlc));
			dlc = 8;
		}
		rc = send_frame(id, ext, f, dlc);
		if (rc != STATUS_NOERROR)
			return rc;
		iso_enqueue(id, ext, TX_MSG_TYPE, NULL, 0);
		return STATUS_NOERROR;
	}

	// Segmented: the conversation whose flow control id we transmit on tells
	// us which incoming FC frames pace this transfer.
	int conv = -1;
	for (int i = 0; i < MAX_FILTERS; i++)
	{
		if (dev.filters[i].used && dev.filters[i].fcid == id &&
			dev.filters[i].ext == (uint8_t)ext)
		{
			conv = i;
			break;
		}
	}
	if (conv < 0)
	{
		seterr("no flow control filter matches id %08lx", (unsigned long)id);
		return ERR_NO_FLOW_CONTROL;
	}

	event_reset(dev.fc_ev);
	dev.txconv = conv;

	unsigned off = 6;
	uint8_t sn = 1;
	int waits = 0;
	isotp_mk_ff(f, p, len);
	rc = send_frame(id, ext, f, 8);

	while (rc == STATUS_NOERROR && off < len)
	{
		// N_Bs: 1s for the peer's flow control, per ISO 15765-2
		if (!event_wait(dev.fc_ev, 1000))
		{
			seterr("no flow control from peer");
			rc = ERR_TIMEOUT;
			break;
		}
		if (dev.fc_fs == 1) // WAIT: peer needs more time
		{
			if (++waits > 8)
			{
				seterr("peer stuck in flow control WAIT");
				rc = ERR_TIMEOUT;
			}
			continue;
		}
		if (dev.fc_fs != 0) // overflow or invalid
		{
			seterr("peer aborted transfer (flow status %d)", dev.fc_fs);
			rc = ERR_FAILED;
			break;
		}

		unsigned block = (unsigned)dev.fc_bs; // 0 = the rest in one go
		unsigned gap = isotp_stmin_ms(dev.fc_stmin);
		for (unsigned nsent = 0; off < len && (block == 0 || nsent < block); nsent++)
		{
			if (nsent && gap)
				sleep_ms(gap);
			int dlc = isotp_mk_cf(f, p, len, off, sn);
			off += (unsigned)dlc - 1;
			sn++;
			if (pad)
			{
				memset(f + dlc, 0, (size_t)(8 - dlc));
				dlc = 8;
			}
			rc = send_frame(id, ext, f, dlc);
			if (rc != STATUS_NOERROR)
				break;
		}
	}

	dev.txconv = -1;
	if (rc != STATUS_NOERROR)
		return rc;
	iso_enqueue(id, ext, TX_MSG_TYPE, NULL, 0);
	return STATUS_NOERROR;
}

int32_t J2534_API PassThruWriteMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg,
									uint32_t *pNumMsgs, uint32_t Timeout)
{
	(void)Timeout; // every write is synchronous: sent and acked, or failed

	if (ChannelID != 1 || !dev.connected)
		return ERR_INVALID_CHANNEL_ID;
	if (!pMsg || !pNumMsgs)
		return ERR_NULL_PARAMETER;

	unsigned long want = *pNumMsgs, sent = 0;

	for (; sent < want; sent++)
	{
		PASSTHRU_MSG *m = &pMsg[sent];
		long rc;

		if (m->ProtocolID != dev.proto)
		{
			*pNumMsgs = sent;
			return ERR_MSG_PROTOCOL_ID;
		}

		if (dev.proto == ISO15765)
			rc = iso_write(m);
		else if (m->DataSize < 4 || m->DataSize > 12)
		{
			seterr("CAN message must be 4..12 bytes (id + data)");
			rc = ERR_INVALID_MSG;
		}
		else
		{
			uint32_t id = ((uint32_t)m->Data[0] << 24) | ((uint32_t)m->Data[1] << 16) |
						  ((uint32_t)m->Data[2] << 8) | m->Data[3];
			rc = send_frame(id, (m->TxFlags & CAN_29BIT_ID) != 0,
							m->Data + 4, (int)m->DataSize - 4);
		}

		if (rc != STATUS_NOERROR)
		{
			*pNumMsgs = sent;
			return rc;
		}
	}

	*pNumMsgs = sent;
	return STATUS_NOERROR;
}

// ---------------------------------------------------- filters and periodic --

int32_t J2534_API PassThruStartMsgFilter(uint32_t ChannelID, uint32_t FilterType,
										 PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg,
										 PASSTHRU_MSG *pFlowControlMsg, uint32_t *pFilterID)
{
	if (ChannelID != 1 || !dev.connected)
		return ERR_INVALID_CHANNEL_ID;
	if (!pMaskMsg || !pPatternMsg || !pFilterID)
		return ERR_NULL_PARAMETER;

	if (dev.proto == ISO15765)
	{
		// Only conversations make sense here: mask/pattern select the id we
		// receive on, the flow control msg is the id we transmit on.
		if (FilterType != FLOW_CONTROL_FILTER)
		{
			seterr("ISO15765 channels take flow control filters only");
			return ERR_NOT_SUPPORTED;
		}
		if (!pFlowControlMsg)
			return ERR_NULL_PARAMETER;
		if (pMaskMsg->DataSize != 4 || pPatternMsg->DataSize != 4 ||
			pFlowControlMsg->DataSize != 4)
		{
			seterr("ISO15765 filter msgs must be exactly 4 bytes (CAN id)");
			return ERR_INVALID_MSG;
		}
		if (pFlowControlMsg->TxFlags & ISO15765_ADDR_TYPE)
		{
			// ponytail: normal addressing only; extended addressing (5 byte
			// filters) is the upgrade if a tool ever needs it
			seterr("ISO15765 extended addressing not supported");
			return ERR_NOT_SUPPORTED;
		}

		for (int i = 0; i < MAX_FILTERS; i++)
		{
			filter_t *f = &dev.filters[i];
			if (f->used)
				continue;
			f->type = FLOW_CONTROL_FILTER;
			f->len = 4;
			memcpy(f->mask, pMaskMsg->Data, 4);
			memcpy(f->pattern, pPatternMsg->Data, 4);
			f->fcid = ((uint32_t)pFlowControlMsg->Data[0] << 24) |
					  ((uint32_t)pFlowControlMsg->Data[1] << 16) |
					  ((uint32_t)pFlowControlMsg->Data[2] << 8) |
					  pFlowControlMsg->Data[3];
			f->ext = (pFlowControlMsg->TxFlags & CAN_29BIT_ID) != 0;
			f->pad = (pFlowControlMsg->TxFlags & ISO15765_FRAME_PAD) != 0;
			memset(&dev.iso_rx[i], 0, sizeof(dev.iso_rx[i]));
			f->used = 1;
			*pFilterID = (unsigned long)(i + 1);
			return STATUS_NOERROR;
		}
		return ERR_EXCEEDED_LIMIT;
	}

	if (FilterType == FLOW_CONTROL_FILTER)
	{
		seterr("flow control filters need ISO15765");
		return ERR_NOT_SUPPORTED;
	}
	if (FilterType != PASS_FILTER && FilterType != BLOCK_FILTER)
		return ERR_INVALID_MSG;
	if (pMaskMsg->DataSize < 1 || pMaskMsg->DataSize > 12 ||
		pMaskMsg->DataSize != pPatternMsg->DataSize)
	{
		seterr("mask and pattern must be equal length, 1..12 bytes");
		return ERR_INVALID_MSG;
	}

	for (int i = 0; i < MAX_FILTERS; i++)
	{
		filter_t *f = &dev.filters[i];
		if (f->used)
			continue;
		f->type = FilterType;
		f->len = pMaskMsg->DataSize;
		memcpy(f->mask, pMaskMsg->Data, f->len);
		memcpy(f->pattern, pPatternMsg->Data, f->len);
		f->used = 1;
		if (FilterType == PASS_FILTER)
			dev.have_pass_filter = 1;
		*pFilterID = (unsigned long)(i + 1);
		return STATUS_NOERROR;
	}
	return ERR_EXCEEDED_LIMIT;
}

int32_t J2534_API PassThruStopMsgFilter(uint32_t ChannelID, uint32_t FilterID)
{
	if (ChannelID != 1 || !dev.connected)
		return ERR_INVALID_CHANNEL_ID;
	if (FilterID < 1 || FilterID > MAX_FILTERS || !dev.filters[FilterID - 1].used)
		return ERR_INVALID_FILTER_ID;

	dev.filters[FilterID - 1].used = 0;
	dev.have_pass_filter = 0;
	for (int i = 0; i < MAX_FILTERS; i++)
		if (dev.filters[i].used && dev.filters[i].type == PASS_FILTER)
			dev.have_pass_filter = 1;
	return STATUS_NOERROR;
}

int32_t J2534_API PassThruStartPeriodicMsg(uint32_t ChannelID, PASSTHRU_MSG *pMsg,
										   uint32_t *pMsgID, uint32_t TimeInterval)
{
	if (ChannelID != 1 || !dev.connected)
		return ERR_INVALID_CHANNEL_ID;
	if (!pMsg || !pMsgID)
		return ERR_NULL_PARAMETER;
	if (TimeInterval < 5 || TimeInterval > 65535)
		return ERR_INVALID_TIME_INTERVAL;
	if (pMsg->ProtocolID != dev.proto)
		return ERR_MSG_PROTOCOL_ID;
	// ISO15765 periodics (e.g. tester present) must fit a single frame
	if (dev.proto == ISO15765 ? (pMsg->DataSize < 5 || pMsg->DataSize > 11)
							  : (pMsg->DataSize < 4 || pMsg->DataSize > 12))
		return ERR_INVALID_MSG;

	for (int i = 0; i < MAX_PERIODIC; i++)
	{
		periodic_t *p = &dev.periodic[i];
		if (p->used)
			continue;
		p->id = ((uint32_t)pMsg->Data[0] << 24) | ((uint32_t)pMsg->Data[1] << 16) |
				((uint32_t)pMsg->Data[2] << 8) | pMsg->Data[3];
		p->ext = (pMsg->TxFlags & CAN_29BIT_ID) != 0;
		if (dev.proto == ISO15765)
		{
			int dlc = isotp_mk_sf(p->data, pMsg->Data + 4, pMsg->DataSize - 4);
			if (pMsg->TxFlags & ISO15765_FRAME_PAD)
			{
				memset(p->data + dlc, 0, (size_t)(8 - dlc));
				dlc = 8;
			}
			p->dlc = (uint8_t)dlc;
		}
		else
		{
			p->dlc = (uint8_t)(pMsg->DataSize - 4);
			memcpy(p->data, pMsg->Data + 4, p->dlc);
		}
		p->interval = TimeInterval;
		p->due = ticks_ms(); // first send right away
		p->used = 1;
		event_set(dev.per_wake);
		*pMsgID = (unsigned long)(i + 1);
		return STATUS_NOERROR;
	}
	return ERR_EXCEEDED_LIMIT;
}

int32_t J2534_API PassThruStopPeriodicMsg(uint32_t ChannelID, uint32_t MsgID)
{
	if (ChannelID != 1 || !dev.connected)
		return ERR_INVALID_CHANNEL_ID;
	if (MsgID < 1 || MsgID > MAX_PERIODIC || !dev.periodic[MsgID - 1].used)
		return ERR_INVALID_MSG_ID;
	dev.periodic[MsgID - 1].used = 0;
	return STATUS_NOERROR;
}

// ------------------------------------------------------------------ misc --

int32_t J2534_API PassThruSetProgrammingVoltage(uint32_t DeviceID, uint32_t PinNumber,
												uint32_t Voltage)
{
	(void)PinNumber;
	if (DeviceID != 1 || !dev.open)
		return ERR_INVALID_DEVICE_ID;
	// No programming voltage hardware; only "off" succeeds
	if (Voltage == VOLTAGE_OFF)
		return STATUS_NOERROR;
	seterr("no programmable voltage hardware");
	return ERR_NOT_SUPPORTED;
}

int32_t J2534_API PassThruReadVersion(uint32_t DeviceID, char *pFirmwareVersion,
									  char *pDllVersion, char *pApiVersion)
{
	if (DeviceID != 1 || !dev.open)
		return ERR_INVALID_DEVICE_ID;
	if (!pFirmwareVersion || !pDllVersion || !pApiVersion)
		return ERR_NULL_PARAMETER;

	// V reply is Vhhss: two digits hardware, two digits software
	char reply[64] = {0};
	if (send_cmd("V\r", 2, reply, sizeof(reply)) == STATUS_NOERROR && reply[0] == 'V' &&
		strlen(reply) >= 5)
		snprintf(pFirmwareVersion, 20, "%c%c.%c%c", reply[1], reply[2], reply[3], reply[4]);
	else
		strcpy(pFirmwareVersion, "unknown");
	strcpy(pDllVersion, DLL_VERSION);
	strcpy(pApiVersion, API_VERSION);
	return STATUS_NOERROR;
}

int32_t J2534_API PassThruGetLastError(char *pErrorDescription)
{
	if (!pErrorDescription)
		return ERR_NULL_PARAMETER;
	strncpy(pErrorDescription, last_error, 80);
	pErrorDescription[79] = '\0';
	return STATUS_NOERROR;
}

int32_t J2534_API PassThruIoctl(uint32_t ChannelID, uint32_t IoctlID,
								void *pInput, void *pOutput)
{
	(void)pOutput; // only used by ioctls this device doesn't have

	switch (IoctlID)
	{
	case GET_CONFIG:
	{
		SCONFIG_LIST *list = (SCONFIG_LIST *)pInput;
		if (!list || !list->ConfigPtr)
			return ERR_NULL_PARAMETER;
		for (unsigned long i = 0; i < list->NumOfParams; i++)
		{
			SCONFIG *c = &list->ConfigPtr[i];
			switch (c->Parameter)
			{
			case DATA_RATE:
				c->Value = dev.baud;
				break;
			case LOOPBACK:
				c->Value = 0;
				break;
			case ISO15765_BS:
				c->Value = dev.iso_bs;
				break;
			case ISO15765_STMIN:
				c->Value = dev.iso_stmin;
				break;
			default:
				c->Value = 0; // accepted, meaningless here
				break;
			}
		}
		return STATUS_NOERROR;
	}

	case SET_CONFIG:
	{
		SCONFIG_LIST *list = (SCONFIG_LIST *)pInput;
		if (!list || !list->ConfigPtr)
			return ERR_NULL_PARAMETER;
		for (unsigned long i = 0; i < list->NumOfParams; i++)
		{
			SCONFIG *c = &list->ConfigPtr[i];
			if (c->Parameter == ISO15765_BS || c->Parameter == ISO15765_STMIN)
			{
				if (c->Value > 0xFF)
					return ERR_INVALID_IOCTL_VALUE;
				if (c->Parameter == ISO15765_BS)
					dev.iso_bs = c->Value;
				else
					dev.iso_stmin = c->Value;
			}
			else if (c->Parameter == DATA_RATE && dev.connected)
			{
				const char *scmd = baud_to_cmd(c->Value);
				if (!scmd)
					return ERR_INVALID_BAUDRATE;
				send_cmd("C\r", 2, NULL, 0);
				send_cmd(scmd, (int)strlen(scmd), NULL, 0);
				if (send_cmd("O\r", 2, NULL, 0) != STATUS_NOERROR)
					return ERR_FAILED;
				dev.baud = c->Value;
			}
			// Everything else: accepted and ignored, like the dongle itself
			// does for its unimplemented knobs.
		}
		return STATUS_NOERROR;
	}

	case CLEAR_TX_BUFFER:
		return STATUS_NOERROR; // transmits are synchronous, nothing queued

	case CLEAR_RX_BUFFER:
		if (ChannelID != 1 || !dev.connected)
			return ERR_INVALID_CHANNEL_ID;
		mutex_lock(dev.q_lock);
		dev.q_tail = dev.q_head;
		dev.iq_tail = dev.iq_head;
		mutex_unlock(dev.q_lock);
		return STATUS_NOERROR;

	case CLEAR_MSG_FILTERS:
		if (ChannelID != 1 || !dev.connected)
			return ERR_INVALID_CHANNEL_ID;
		memset(dev.filters, 0, sizeof(dev.filters));
		memset(dev.iso_rx, 0, sizeof(dev.iso_rx));
		dev.have_pass_filter = 0;
		return STATUS_NOERROR;

	case CLEAR_PERIODIC_MSGS:
		if (ChannelID != 1 || !dev.connected)
			return ERR_INVALID_CHANNEL_ID;
		memset(dev.periodic, 0, sizeof(dev.periodic));
		return STATUS_NOERROR;

	default:
		seterr("ioctl %lu not supported", (unsigned long)IoctlID);
		return ERR_NOT_SUPPORTED;
	}
}
