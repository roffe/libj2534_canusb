// Runtime smoke test: load the built DLL, resolve every J2534 export and
// call through the stdcall boundary. Run under wine from the Makefile.
// Without an FTDI driver present, PassThruOpen must fail cleanly with
// ERR_DEVICE_NOT_CONNECTED (8) - that exercises the whole load/error path.
#include <windows.h>
#include <stdio.h>

typedef long(__stdcall *tOpen)(void *, unsigned long *);
typedef long(__stdcall *tGetLastError)(char *);

int main(int argc, char **argv)
{
	static const char *names[] = {
		"PassThruOpen", "PassThruClose", "PassThruConnect", "PassThruDisconnect",
		"PassThruReadMsgs", "PassThruWriteMsgs", "PassThruStartPeriodicMsg",
		"PassThruStopPeriodicMsg", "PassThruStartMsgFilter", "PassThruStopMsgFilter",
		"PassThruSetProgrammingVoltage", "PassThruReadVersion",
		"PassThruGetLastError", "PassThruIoctl"};

	if (argc < 2)
		return 1;

	HMODULE h = LoadLibraryA(argv[1]);
	if (!h)
	{
		printf("FAIL: LoadLibrary(%s): %lu\n", argv[1], GetLastError());
		return 1;
	}

	for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++)
	{
		if (!GetProcAddress(h, names[i]))
		{
			printf("FAIL: export %s missing\n", names[i]);
			return 1;
		}
	}

	tOpen open = (tOpen)(void *)GetProcAddress(h, "PassThruOpen");
	tGetLastError gle = (tGetLastError)(void *)GetProcAddress(h, "PassThruGetLastError");

	unsigned long id = 0;
	long rc = open(NULL, &id);
	char err[80] = {0};
	gle(err);
	printf("PassThruOpen rc=%ld err='%s'\n", rc, err);

	// 8 = ERR_DEVICE_NOT_CONNECTED: expected with no ftdi driver installed.
	// 0 would mean a dongle is actually reachable - also fine.
	if (rc == 8 || rc == 0)
	{
		printf("smoke test OK\n");
		return 0;
	}
	return 2;
}
