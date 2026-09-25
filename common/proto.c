#include <string.h>
#include "proto.h"

static const char hexdig[] = "0123456789ABCDEF";

static int unhex(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return -1;
}

int slcan_encode(char *out, uint32_t id, int ext, const uint8_t *data, int dlc)
{
	int n = 0;

	if (dlc < 0 || dlc > 8)
		return -1;
	if (ext ? (id > 0x1FFFFFFFu) : (id > 0x7FFu))
		return -1;

	out[n++] = ext ? 'T' : 't';
	for (int shift = (ext ? 28 : 8); shift >= 0; shift -= 4)
		out[n++] = hexdig[(id >> shift) & 0xF];
	out[n++] = hexdig[dlc];
	for (int i = 0; i < dlc; i++)
	{
		out[n++] = hexdig[data[i] >> 4];
		out[n++] = hexdig[data[i] & 0xF];
	}
	out[n++] = '\r';
	return n;
}

int slcan_decode(const char *line, int len, uint32_t *id, int *ext,
                 uint8_t data[8], int *dlc)
{
	int idlen;

	if (len < 1)
		return 0;

	if (line[0] == 't')
		*ext = 0, idlen = 3;
	else if (line[0] == 'T')
		*ext = 1, idlen = 8;
	else
		return 0; // ack, reply, remote frame, FD frame: not ours to queue

	if (len < 1 + idlen + 1)
		return -1;

	uint32_t v = 0;
	for (int i = 1; i <= idlen; i++)
	{
		int d = unhex(line[i]);
		if (d < 0)
			return -1;
		v = (v << 4) | (uint32_t)d;
	}
	*id = v;

	int d = unhex(line[1 + idlen]);
	if (d < 0 || d > 8)
		return -1;
	*dlc = d;

	int datalen = 2 * d;
	int have = len - (1 + idlen + 1);
	// Exact length, or exact plus the 4 digit timestamp suffix
	if (have != datalen && have != datalen + 4)
		return -1;

	for (int i = 0; i < d; i++)
	{
		int hi = unhex(line[1 + idlen + 1 + 2 * i]);
		int lo = unhex(line[1 + idlen + 1 + 2 * i + 1]);
		if (hi < 0 || lo < 0)
			return -1;
		data[i] = (uint8_t)((hi << 4) | lo);
	}

	return 1;
}

int isotp_mk_sf(uint8_t out[8], const uint8_t *p, unsigned len)
{
	out[0] = (uint8_t)len;
	memcpy(out + 1, p, len);
	return (int)len + 1;
}

int isotp_mk_ff(uint8_t out[8], const uint8_t *p, unsigned len)
{
	out[0] = (uint8_t)(0x10 | (len >> 8));
	out[1] = (uint8_t)len;
	memcpy(out + 2, p, 6);
	return 8;
}

int isotp_mk_cf(uint8_t out[8], const uint8_t *p, unsigned len,
                unsigned off, uint8_t sn)
{
	unsigned n = len - off;
	if (n > 7)
		n = 7;
	out[0] = (uint8_t)(0x20 | (sn & 0xF));
	memcpy(out + 1, p + off, n);
	return (int)n + 1;
}

int isotp_mk_fc(uint8_t out[8], int fs, int bs, int stmin)
{
	out[0] = (uint8_t)(0x30 | (fs & 0xF));
	out[1] = (uint8_t)bs;
	out[2] = (uint8_t)stmin;
	return 3;
}

int isotp_rx_feed(isotp_rx_t *r, const uint8_t *d, int dlc,
                  int *fs, int *bs, int *stmin)
{
	if (dlc < 1)
		return ISOTP_NONE;

	switch (d[0] >> 4)
	{
	case 0: // single frame
	{
		unsigned n = d[0] & 0xF;
		if (n < 1 || (int)n > dlc - 1)
			return ISOTP_NONE;
		r->active = 0; // an SF aborts any reassembly in progress (per spec)
		r->len = n;
		memcpy(r->buf, d + 1, n);
		return ISOTP_MSG;
	}
	case 1: // first frame
	{
		if (dlc < 8)
			return ISOTP_NONE;
		unsigned n = ((unsigned)(d[0] & 0xF) << 8) | d[1];
		if (n < 8) // FF with an SF-sized length is malformed
			return ISOTP_NONE;
		r->len = n;
		r->got = 6;
		r->sn = 1;
		r->active = 1;
		memcpy(r->buf, d + 2, 6);
		return ISOTP_SEND_FC;
	}
	case 2: // consecutive frame
	{
		if (!r->active)
			return ISOTP_NONE;
		if ((d[0] & 0xF) != (r->sn & 0xF))
		{
			r->active = 0; // sequence error aborts the message (per spec)
			return ISOTP_NONE;
		}
		unsigned n = r->len - r->got;
		if (n > 7)
			n = 7;
		if ((int)n > dlc - 1)
		{
			r->active = 0;
			return ISOTP_NONE;
		}
		memcpy(r->buf + r->got, d + 1, n);
		r->got += n;
		r->sn++;
		if (r->got >= r->len)
		{
			r->active = 0;
			return ISOTP_MSG;
		}
		return ISOTP_NONE;
	}
	case 3: // flow control
		if (dlc < 3)
			return ISOTP_NONE;
		*fs = d[0] & 0xF;
		*bs = d[1];
		*stmin = d[2];
		return ISOTP_FLOWCTRL;
	}
	return ISOTP_NONE;
}

unsigned isotp_stmin_ms(int stmin)
{
	if (stmin <= 0x7F)
		return (unsigned)stmin;
	if (stmin >= 0xF1 && stmin <= 0xF9)
		return 1; // 100..900us: round up, we have no sub-ms sleep anyway
	return 0x7F;  // reserved values: be conservative (per spec)
}

int filter_match(const uint8_t *msg, unsigned msglen,
                 const uint8_t *mask, const uint8_t *pattern, unsigned flen)
{
	if (msglen < flen)
		return 0;
	for (unsigned i = 0; i < flen; i++)
	{
		if ((msg[i] & mask[i]) != (pattern[i] & mask[i]))
			return 0;
	}
	return 1;
}
