# libj2534_canusb: J2534 PassThru driver for the LAWICEL CANUSB / canusb2.
#
#   make             both platforms
#   make linux       build/libj2534_canusb.so
#   make windows     build/{x86,x64}/libj2534_canusb.dll + build/libj2534_canusb_setup.exe
#                    (cross-built with mingw-w64)
#   make install     linux: ~/.passthru/libj2534_canusb.so + ~/.passthru/canusb.json
#   make test        host-side test of the slcan/ISO-TP/filter logic
#
# common/ is the platform-free core, windows/ and linux/ the port layer under it.

CC   ?= cc
CC32  = i686-w64-mingw32-gcc
CC64  = x86_64-w64-mingw32-gcc

CFLAGS ?= -O2 -Wall -Wextra
CFLAGS += -Icommon

CORE = common/canusb.c common/proto.c
HDRS = common/j2534.h common/port.h common/proto.h

SO    = build/libj2534_canusb.so
DLL32 = build/x86/libj2534_canusb.dll
DLL64 = build/x64/libj2534_canusb.dll
SETUP = build/libj2534_canusb_setup.exe

WINSRC     = $(CORE) windows/port.c windows/libj2534_canusb.def
WINLDFLAGS = -shared -static-libgcc -Wl,--kill-at -Wl,--enable-stdcall-fixup

all: linux windows

# ------------------------------------------------------------------ linux --

linux: $(SO)

$(SO): $(CORE) linux/port.c $(HDRS)
	@mkdir -p build
	$(CC) $(CFLAGS) -fPIC -fvisibility=hidden -pthread -shared -o $@ $(CORE) linux/port.c

install: $(SO)
	install -Dm755 $(SO) ~/.passthru/libj2534_canusb.so
	install -Dm644 linux/canusb.json ~/.passthru/canusb.json

# ---------------------------------------------------------------- windows --

windows: $(DLL32) $(DLL64) $(SETUP)

$(DLL32): $(WINSRC) $(HDRS) windows/ftd2xx.h
	@mkdir -p build/x86
	$(CC32) $(CFLAGS) $(WINLDFLAGS) -o $@ $(WINSRC)

$(DLL64): $(WINSRC) $(HDRS) windows/ftd2xx.h
	@mkdir -p build/x64
	$(CC64) $(CFLAGS) $(WINLDFLAGS) -o $@ $(WINSRC)

# Single-file installer with both DLLs embedded; 32 bit so it runs anywhere
$(SETUP): windows/setup.c windows/setup.rc $(DLL32) $(DLL64)
	i686-w64-mingw32-windres windows/setup.rc build/setup_res.o
	$(CC32) $(CFLAGS) -D_WIN32_WINNT=0x0601 -o $@ windows/setup.c build/setup_res.o -static-libgcc -lshell32 -ladvapi32

# ------------------------------------------------------------------ tests --

# Host-side check of the pure protocol logic
test:
	@mkdir -p build
	$(CC) $(CFLAGS) -o build/test_proto common/test_proto.c common/proto.c
	build/test_proto

# On-hardware exerciser: build/test_device on linux, build/{x86,x64}/test_device.exe
# to run on windows with a dongle attached
testexes: $(DLL32) $(DLL64)
	$(CC) $(CFLAGS) -pthread -o build/test_device common/test_device.c $(CORE) linux/port.c
	$(CC32) $(CFLAGS) -o build/x86/test_device.exe common/test_device.c -L build/x86 -l:libj2534_canusb.dll -Wl,--enable-stdcall-fixup
	$(CC64) $(CFLAGS) -o build/x64/test_device.exe common/test_device.c -L build/x64 -l:libj2534_canusb.dll

# Load each DLL under wine, resolve all exports and call through the stdcall
# boundary. Needs wine; PassThruOpen failing with 8 (no FTDI driver) is a pass.
winetest: $(DLL32) $(DLL64)
	$(CC32) -O2 -Wall -o build/x86/test_load.exe windows/test_load.c
	$(CC64) -O2 -Wall -o build/x64/test_load.exe windows/test_load.c
	wine build/x86/test_load.exe 'build\x86\libj2534_canusb.dll'
	wine build/x64/test_load.exe 'build\x64\libj2534_canusb.dll'

clean:
	rm -rf build

.PHONY: all linux windows install test testexes winetest clean
