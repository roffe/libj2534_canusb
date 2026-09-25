//
// proto: pure slcan line encode/decode and J2534 filter matching.
// No windows dependencies so the logic is testable on the build host.
//
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

// Encode a classic CAN frame as an slcan transmit line ("t.../T...\r").
// Returns the line length, or -1 for an invalid frame. out must fit 32 bytes.
int slcan_encode(char *out, uint32_t id, int ext, const uint8_t *data, int dlc);

// Decode one received slcan line (without the trailing CR). Returns 1 and
// fills the frame for a t/T data frame, 0 for any other line (acks, replies,
// remote frames), -1 for a malformed t/T line. Tolerates the optional 4 digit
// timestamp suffix the device appends when Z1 is set.
int slcan_decode(const char *line, int len, uint32_t *id, int *ext,
                 uint8_t data[8], int *dlc);

// Byte-wise J2534 filter compare: (msg & mask) == (pattern & mask) over the
// filter's length. A message shorter than the filter never matches.
int filter_match(const uint8_t *msg, unsigned msglen,
                 const uint8_t *mask, const uint8_t *pattern, unsigned flen);

// ------------------- ISO 15765-2 (ISO-TP), normal addressing only ----------

#define ISOTP_MAX 4095

// Frame builders. Each fills out[0..7] and returns the used dlc (caller pads
// to 8 if the J2534 FRAME_PAD flag asks for it).
int isotp_mk_sf(uint8_t out[8], const uint8_t *p, unsigned len);       // 1..7
int isotp_mk_ff(uint8_t out[8], const uint8_t *p, unsigned len);       // 8..4095, consumes 6
int isotp_mk_cf(uint8_t out[8], const uint8_t *p, unsigned len,
                unsigned off, uint8_t sn);                             // next chunk at off
int isotp_mk_fc(uint8_t out[8], int fs, int bs, int stmin);

// One reassembly in progress per ID pair (that is all the protocol allows).
typedef struct
{
	unsigned len, got;
	uint8_t sn, active;
	uint8_t buf[ISOTP_MAX];
} isotp_rx_t;

// isotp_rx_feed results
#define ISOTP_NONE     0 // consumed (partial, malformed or stray) - nothing to do
#define ISOTP_MSG      1 // complete message: r->buf, r->len bytes
#define ISOTP_SEND_FC  2 // first frame accepted - caller must send a flow control
#define ISOTP_FLOWCTRL 3 // frame was a flow control: *fs/*bs/*stmin filled

// Feed one received CAN payload belonging to this conversation.
int isotp_rx_feed(isotp_rx_t *r, const uint8_t *d, int dlc,
                  int *fs, int *bs, int *stmin);

// STmin byte from a flow control -> milliseconds to wait between CFs.
unsigned isotp_stmin_ms(int stmin);

#endif // PROTO_H
