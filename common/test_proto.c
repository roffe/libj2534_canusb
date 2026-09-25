// Host test for the pure slcan/filter logic the J2534 DLL is built on.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "proto.h"

int main(void)
{
	char line[32];
	uint32_t id;
	int ext, dlc, n;
	uint8_t data[8];

	// Standard frame encode: 0x7E0, 2 bytes
	uint8_t d1[] = {0x10, 0xAB};
	n = slcan_encode(line, 0x7E0, 0, d1, 2);
	assert(n == 10 && memcmp(line, "t7E0210AB\r", 10) == 0);

	// Extended frame encode
	n = slcan_encode(line, 0x18DAF110, 1, d1, 2);
	assert(n == 15 && memcmp(line, "T18DAF110210AB\r", 15) == 0);

	// Invalid: 11 bit id too large, dlc too large
	assert(slcan_encode(line, 0x800, 0, d1, 2) == -1);
	assert(slcan_encode(line, 0x7E0, 0, d1, 9) == -1);

	// Decode round trip
	assert(slcan_decode("t7E0210AB", 9, &id, &ext, data, &dlc) == 1);
	assert(id == 0x7E0 && !ext && dlc == 2 && data[0] == 0x10 && data[1] == 0xAB);

	assert(slcan_decode("T18DAF110210AB", 14, &id, &ext, data, &dlc) == 1);
	assert(id == 0x18DAF110 && ext && dlc == 2);

	// DLC 0 frame
	assert(slcan_decode("t1230", 5, &id, &ext, data, &dlc) == 1);
	assert(id == 0x123 && dlc == 0);

	// Timestamp suffix (Z1 mode) tolerated
	assert(slcan_decode("t7E0210AB1A2B", 13, &id, &ext, data, &dlc) == 1);
	assert(id == 0x7E0 && dlc == 2 && data[1] == 0xAB);

	// Non-frame lines are ignored, malformed frames rejected
	assert(slcan_decode("z", 1, &id, &ext, data, &dlc) == 0);
	assert(slcan_decode("V1011", 5, &id, &ext, data, &dlc) == 0);
	assert(slcan_decode("", 0, &id, &ext, data, &dlc) == 0);
	assert(slcan_decode("r1232", 5, &id, &ext, data, &dlc) == 0);
	assert(slcan_decode("t7E02", 5, &id, &ext, data, &dlc) == -1);      // missing data
	assert(slcan_decode("t7EG210AB", 9, &id, &ext, data, &dlc) == -1);  // bad hex
	assert(slcan_decode("t7E0910AB", 9, &id, &ext, data, &dlc) == -1);  // dlc 9

	// Filter match: pass 0x220-0x22F (mask FFFFFFF0 on the 4 byte BE id)
	uint8_t msg[12] = {0x00, 0x00, 0x02, 0x28, 0xAA};
	uint8_t mask[4] = {0xFF, 0xFF, 0xFF, 0xF0};
	uint8_t pat[4] = {0x00, 0x00, 0x02, 0x20};
	assert(filter_match(msg, 5, mask, pat, 4) == 1);
	msg[3] = 0x38; // 0x238: no match
	assert(filter_match(msg, 5, mask, pat, 4) == 0);
	// Message shorter than the filter never matches
	assert(filter_match(msg, 3, mask, pat, 4) == 0);
	// Zero mask matches everything long enough
	uint8_t zmask[4] = {0}, zpat[4] = {0};
	assert(filter_match(msg, 5, zmask, zpat, 4) == 1);

	// ---- ISO-TP ----
	static isotp_rx_t rx; // zeroed
	uint8_t frm[8], payload[300];
	int fs, bs, stmin;
	for (unsigned i = 0; i < sizeof(payload); i++)
		payload[i] = (uint8_t)i;

	// Single frame build + feed roundtrip
	n = isotp_mk_sf(frm, payload, 5);
	assert(n == 6 && frm[0] == 0x05 && frm[5] == 0x04);
	assert(isotp_rx_feed(&rx, frm, n, &fs, &bs, &stmin) == ISOTP_MSG);
	assert(rx.len == 5 && memcmp(rx.buf, payload, 5) == 0);

	// Segmented roundtrip: FF then CFs with wrapping sequence numbers
	n = isotp_mk_ff(frm, payload, 300);
	assert(n == 8 && frm[0] == 0x11 && frm[1] == 0x2C && frm[2] == 0x00);
	assert(isotp_rx_feed(&rx, frm, 8, &fs, &bs, &stmin) == ISOTP_SEND_FC);
	unsigned off = 6;
	uint8_t sn = 1;
	int r = -1;
	while (off < 300)
	{
		n = isotp_mk_cf(frm, payload, 300, off, sn);
		off += (unsigned)n - 1;
		sn++;
		r = isotp_rx_feed(&rx, frm, n, &fs, &bs, &stmin);
		assert(r == (off >= 300 ? ISOTP_MSG : ISOTP_NONE));
	}
	assert(r == ISOTP_MSG && rx.len == 300 && memcmp(rx.buf, payload, 300) == 0);

	// Sequence error aborts, later CFs for the dead message are ignored
	n = isotp_mk_ff(frm, payload, 300);
	assert(isotp_rx_feed(&rx, frm, 8, &fs, &bs, &stmin) == ISOTP_SEND_FC);
	n = isotp_mk_cf(frm, payload, 300, 6, 2); // expected sn 1, send 2
	assert(isotp_rx_feed(&rx, frm, n, &fs, &bs, &stmin) == ISOTP_NONE);
	n = isotp_mk_cf(frm, payload, 300, 13, 2);
	assert(isotp_rx_feed(&rx, frm, n, &fs, &bs, &stmin) == ISOTP_NONE);
	assert(!rx.active);

	// Flow control build + parse
	n = isotp_mk_fc(frm, 0, 8, 0x14);
	assert(n == 3 && frm[0] == 0x30);
	assert(isotp_rx_feed(&rx, frm, n, &fs, &bs, &stmin) == ISOTP_FLOWCTRL);
	assert(fs == 0 && bs == 8 && stmin == 0x14);

	// Stray CF with no reassembly running is ignored
	n = isotp_mk_cf(frm, payload, 300, 6, 1);
	assert(isotp_rx_feed(&rx, frm, n, &fs, &bs, &stmin) == ISOTP_NONE);

	// STmin normalization
	assert(isotp_stmin_ms(0) == 0 && isotp_stmin_ms(0x7F) == 0x7F);
	assert(isotp_stmin_ms(0xF1) == 1 && isotp_stmin_ms(0xF9) == 1);
	assert(isotp_stmin_ms(0x80) == 0x7F && isotp_stmin_ms(0xFF) == 0x7F);

	printf("all proto tests passed\n");
	return 0;
}
