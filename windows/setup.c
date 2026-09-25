//
// setup: single-file installer/uninstaller for the CANUSB J2534 driver.
// Both DLLs are embedded as resources; install extracts them to
// %ProgramFiles%\libj2534_canusb and registers the PassThruSupport.04.04 keys
// in both registry views (32 bit apps find the x86 DLL, 64 bit apps the x64
// one). Built as a 32 bit exe so the same binary runs on any windows.
//
//   libj2534_canusb_setup.exe            interactive (asks install/remove)
//   libj2534_canusb_setup.exe install
//   libj2534_canusb_setup.exe uninstall
//
#include <windows.h>
#include <shellapi.h>
#include <stdio.h>
#include <string.h>

#define REG_KEY  "SOFTWARE\\PassThruSupport.04.04\\LAWICEL - CANUSB"
#define DLL32    "libj2534_canusb_x86.dll"
#define DLL64    "libj2534_canusb_x64.dll"

// Log to both the console and %TEMP%\libj2534_canusb_setup.log, so the actions
// of the elevated child process are recoverable (its console is separate).
static void logmsg(const char *fmt, ...)
{
	char msg[512], path[MAX_PATH];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);
	if (n < 0)
		return;

	fputs(msg, stdout); // callers include their own trailing newline

	DWORD len = GetEnvironmentVariableA("TEMP", path, sizeof(path));
	if (len == 0 || len >= sizeof(path) - 32)
		return;
	strcat(path, "\\libj2534_canusb_setup.log");
	HANDLE f = CreateFileA(path, FILE_APPEND_DATA, FILE_SHARE_READ, NULL,
	                       OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (f == INVALID_HANDLE_VALUE)
		return;
	SetFilePointer(f, 0, NULL, FILE_END);
	DWORD w;
	WriteFile(f, msg, (DWORD)strlen(msg), &w, NULL);
	CloseHandle(f);
}

static int is_win64(void)
{
	BOOL wow = FALSE;
	IsWow64Process(GetCurrentProcess(), &wow);
	return wow != FALSE;
}

static int is_elevated(void)
{
	HANDLE tok;
	TOKEN_ELEVATION el = {0};
	DWORD len = 0;
	int ret = 0;

	if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &tok))
	{
		if (GetTokenInformation(tok, TokenElevation, &el, sizeof(el), &len))
			ret = el.TokenIsElevated != 0;
		CloseHandle(tok);
	}
	return ret;
}

// Relaunch ourselves with the UAC prompt and wait for the result
static int relaunch_elevated(const char *arg)
{
	char exe[MAX_PATH];
	GetModuleFileNameA(NULL, exe, sizeof(exe));

	SHELLEXECUTEINFOA sei;
	memset(&sei, 0, sizeof(sei));
	sei.cbSize = sizeof(sei);
	sei.fMask = SEE_MASK_NOCLOSEPROCESS;
	sei.lpVerb = "runas";
	sei.lpFile = exe;
	sei.lpParameters = arg;
	sei.nShow = SW_SHOWNORMAL;

	if (!ShellExecuteExA(&sei) || !sei.hProcess)
	{
		printf("elevation cancelled\n");
		return 1;
	}
	WaitForSingleObject(sei.hProcess, INFINITE);
	DWORD code = 1;
	GetExitCodeProcess(sei.hProcess, &code);
	CloseHandle(sei.hProcess);
	return (int)code;
}

static int extract(const char *resname, const char *path)
{
	HRSRC r = FindResourceA(NULL, resname, RT_RCDATA);
	if (!r)
		return -1;
	DWORD size = SizeofResource(NULL, r);
	void *data = LockResource(LoadResource(NULL, r));
	if (!data)
		return -1;

	HANDLE f = CreateFileA(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS,
	                       FILE_ATTRIBUTE_NORMAL, NULL);
	if (f == INVALID_HANDLE_VALUE)
		return -1;
	DWORD written = 0;
	BOOL ok = WriteFile(f, data, size, &written, NULL);
	CloseHandle(f);
	return (ok && written == size) ? 0 : -1;
}

static int write_reg_view(REGSAM view, const char *dllpath)
{
	static const char *zero_protocols[] = {"J1850VPW", "J1850PWM", "ISO9141", "ISO14230"};
	HKEY k;
	DWORD one = 1, zero = 0;

	if (RegCreateKeyExA(HKEY_LOCAL_MACHINE, REG_KEY, 0, NULL, 0,
	                    KEY_WRITE | view, NULL, &k, NULL) != ERROR_SUCCESS)
		return -1;

	RegSetValueExA(k, "Name", 0, REG_SZ, (const BYTE *)"CANUSB", 7);
	RegSetValueExA(k, "Vendor", 0, REG_SZ, (const BYTE *)"LAWICEL", 8);
	RegSetValueExA(k, "FunctionLibrary", 0, REG_SZ, (const BYTE *)dllpath,
	               (DWORD)strlen(dllpath) + 1);
	RegSetValueExA(k, "ConfigApplication", 0, REG_SZ, (const BYTE *)"", 1);
	RegSetValueExA(k, "CAN", 0, REG_DWORD, (const BYTE *)&one, sizeof(one));
	RegSetValueExA(k, "ISO15765", 0, REG_DWORD, (const BYTE *)&one, sizeof(one));
	for (unsigned i = 0; i < sizeof(zero_protocols) / sizeof(zero_protocols[0]); i++)
		RegSetValueExA(k, zero_protocols[i], 0, REG_DWORD, (const BYTE *)&zero, sizeof(zero));

	RegCloseKey(k);
	return 0;
}

// Delete our leaf key from one registry view. The direct form (predefined
// root + full path + explicit WOW64 view flag) is the documented way to
// target a specific view; verified against both views under wine.
static LONG delete_reg_view(REGSAM view)
{
	LONG rc = RegDeleteKeyExA(HKEY_LOCAL_MACHINE, REG_KEY, view, 0);
	logmsg("  RegDeleteKeyEx(view=0x%lx) -> %ld\n", (unsigned long)view, rc);
	return rc;
}

static void target_dir(char *out, size_t size)
{
	char pf[MAX_PATH] = {0};
	// Real Program Files even from a 32 bit process on 64 bit windows
	if (!GetEnvironmentVariableA("ProgramW6432", pf, sizeof(pf)))
		GetEnvironmentVariableA("ProgramFiles", pf, sizeof(pf));
	snprintf(out, size, "%s\\libj2534_canusb", pf);
}

static int do_install(void)
{
	char dir[MAX_PATH], p32[MAX_PATH + 32], p64[MAX_PATH + 32];
	target_dir(dir, sizeof(dir));
	snprintf(p32, sizeof(p32), "%s\\%s", dir, DLL32);
	snprintf(p64, sizeof(p64), "%s\\%s", dir, DLL64);

	if (!CreateDirectoryA(dir, NULL) && GetLastError() != ERROR_ALREADY_EXISTS)
	{
		logmsg("FAILED: cannot create %s (error %lu)\n", dir, GetLastError());
		return 1;
	}

	if (extract("DLL32", p32) != 0)
	{
		logmsg("FAILED: cannot write %s (close apps using the driver?)\n", p32);
		return 1;
	}
	logmsg("installed %s\n", p32);
	if (write_reg_view(KEY_WOW64_32KEY, p32) != 0)
	{
		logmsg("FAILED: cannot write registry (32 bit view)\n");
		return 1;
	}

	if (is_win64())
	{
		if (extract("DLL64", p64) != 0)
		{
			logmsg("FAILED: cannot write %s\n", p64);
			return 1;
		}
		logmsg("installed %s\n", p64);
		if (write_reg_view(KEY_WOW64_64KEY, p64) != 0)
		{
			logmsg("FAILED: cannot write registry (64 bit view)\n");
			return 1;
		}
	}

	logmsg("registered: HKLM\\%s\n", REG_KEY);
	logmsg("done - J2534 applications will now list \"LAWICEL - CANUSB\"\n");
	return 0;
}

static int do_uninstall(void)
{
	char dir[MAX_PATH], p[MAX_PATH + 32];
	int problems = 0;
	target_dir(dir, sizeof(dir));

	LONG r32 = delete_reg_view(KEY_WOW64_32KEY);
	LONG r64 = is_win64() ? delete_reg_view(KEY_WOW64_64KEY) : ERROR_FILE_NOT_FOUND;
	if (r32 == ERROR_SUCCESS || r64 == ERROR_SUCCESS)
		logmsg("registry entries removed\n");
	else if (r32 == ERROR_FILE_NOT_FOUND && r64 == ERROR_FILE_NOT_FOUND)
		logmsg("registry entries were not present\n");
	else
	{
		logmsg("FAILED: could not remove registry entries\n");
		problems = 1;
	}

	const char *dlls[] = {DLL32, DLL64};
	for (int i = 0; i < 2; i++)
	{
		snprintf(p, sizeof(p), "%s\\%s", dir, dlls[i]);
		if (DeleteFileA(p))
			logmsg("removed %s\n", p);
		else if (GetLastError() != ERROR_FILE_NOT_FOUND && GetLastError() != ERROR_PATH_NOT_FOUND)
		{
			logmsg("FAILED: cannot delete %s (in use?)\n", p);
			problems = 1;
		}
	}
	if (!problems)
		RemoveDirectoryA(dir);

	logmsg(problems ? "finished with problems\n" : "uninstalled\n");
	return problems;
}

int main(int argc, char **argv)
{
	const char *mode = argc > 1 ? argv[1] : NULL;
	int interactive = (mode == NULL);
	int ret;

	if (interactive)
	{
		printf("CANUSB J2534 driver setup\n  i = install\n  u = uninstall\n  q = quit\n> ");
		int c = getchar();
		if (c == 'i' || c == 'I')
			mode = "install";
		else if (c == 'u' || c == 'U')
			mode = "uninstall";
		else
			return 0;
	}

	if (strcmp(mode, "install") != 0 && strcmp(mode, "uninstall") != 0)
	{
		printf("usage: %s [install|uninstall]\n", argv[0]);
		return 1;
	}

	if (!is_elevated())
	{
		printf("requesting administrator rights...\n");
		return relaunch_elevated(mode);
	}

	ret = strcmp(mode, "install") == 0 ? do_install() : do_uninstall();

	// Keep the window open when we own the console (double click / UAC spawn)
	DWORD pids[2];
	if (GetConsoleProcessList(pids, 2) == 1)
	{
		printf("\npress enter to close...");
		while (getchar() != '\n' && !feof(stdin))
			;
		getchar();
	}
	return ret;
}
