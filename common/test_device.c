// End-to-end J2534 exerciser for a real CANUSB, on either platform:
//
//   build/test_device [bitrate] [--send id hexbytes]        (linux)
//   test_device.exe [bitrate] [--send id hexbytes]          (windows, D2XX driver installed)
//
// Opens the device, prints versions, connects CAN at the given bitrate
// (default 500000), sets a pass-everything filter and dumps received frames
// for 5 seconds. --send transmits one standard-ID frame first.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "j2534.h"

int main(int argc, char **argv)
{
	uint32_t dev = 0, chan = 0, fid = 0, n;
	uint32_t bitrate = 500000;
	long rc;
	char fw[32], dll[32], api[32], err[80];

	if (argc > 1 && argv[1][0] != '-')
		bitrate = (uint32_t)strtoul(argv[1], NULL, 10);

	rc = PassThruOpen(NULL, &dev);
	if (rc)
	{
		PassThruGetLastError(err);
		printf("PassThruOpen failed: %ld (%s)\n", rc, err);
		return 1;
	}

	PassThruReadVersion(dev, fw, dll, api);
	printf("open ok: firmware %s, dll %s, api %s\n", fw, dll, api);

	rc = PassThruConnect(dev, CAN, 0, bitrate, &chan);
	if (rc)
	{
		PassThruGetLastError(err);
		printf("PassThruConnect(%lu) failed: %ld (%s)\n", (unsigned long)bitrate, rc, err);
		PassThruClose(dev);
		return 1;
	}
	printf("connected at %lu bit/s\n", (unsigned long)bitrate);

	// Pass everything: zero mask matches all IDs
	PASSTHRU_MSG mask, pattern;
	memset(&mask, 0, sizeof(mask));
	memset(&pattern, 0, sizeof(pattern));
	mask.ProtocolID = pattern.ProtocolID = CAN;
	mask.DataSize = pattern.DataSize = 4;
	rc = PassThruStartMsgFilter(chan, PASS_FILTER, &mask, &pattern, NULL, &fid);
	printf("pass-all filter: rc=%ld id=%lu\n", rc, (unsigned long)fid);

	for (int i = 1; i < argc - 1; i++)
	{
		if (strcmp(argv[i], "--send") == 0)
		{
			PASSTHRU_MSG tx;
			memset(&tx, 0, sizeof(tx));
			tx.ProtocolID = CAN;
			unsigned long id = strtoul(argv[i + 1], NULL, 16);
			tx.Data[0] = (unsigned char)(id >> 24);
			tx.Data[1] = (unsigned char)(id >> 16);
			tx.Data[2] = (unsigned char)(id >> 8);
			tx.Data[3] = (unsigned char)id;
			tx.DataSize = 4;
			for (int a = i + 2; a < argc && tx.DataSize < 12; a++)
				tx.Data[tx.DataSize++] = (unsigned char)strtoul(argv[a], NULL, 16);
			n = 1;
			rc = PassThruWriteMsgs(chan, &tx, &n, 1000);
			PassThruGetLastError(err);
			printf("send %03lx: rc=%ld sent=%lu %s\n", id, rc, (unsigned long)n, rc ? err : "");
			break;
		}
	}

	printf("listening 5s...\n");
	time_t until = time(NULL) + 5;
	unsigned total = 0;
	while (time(NULL) < until)
	{
		PASSTHRU_MSG msgs[16];
		n = 16;
		rc = PassThruReadMsgs(chan, msgs, &n, 500);
		if (rc != STATUS_NOERROR && rc != ERR_TIMEOUT && rc != ERR_BUFFER_EMPTY)
		{
			PassThruGetLastError(err);
			printf("ReadMsgs failed: %ld (%s)\n", rc, err);
			break;
		}
		for (uint32_t i = 0; i < n; i++)
		{
			PASSTHRU_MSG *m = &msgs[i];
			unsigned long id = ((unsigned long)m->Data[0] << 24) | ((unsigned long)m->Data[1] << 16) |
			                   ((unsigned long)m->Data[2] << 8) | m->Data[3];
			printf("rx %s%08lx [%lu]", (m->RxStatus & CAN_29BIT_ID) ? "x" : " ", id,
			       (unsigned long)m->DataSize - 4);
			for (uint32_t b = 4; b < m->DataSize; b++)
				printf(" %02X", m->Data[b]);
			printf("\n");
			total++;
		}
	}
	printf("%u frames received\n", total);

	PassThruDisconnect(chan);
	PassThruClose(dev);
	printf("closed\n");
	return 0;
}
