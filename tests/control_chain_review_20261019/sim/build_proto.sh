#!/bin/sh
P="$1"; OUT="$2"; shift 2
ROOT=/c/Users/Admin/Desktop/duoji
/d/SoftWare/mingw64/bin/gcc.exe -std=gnu99 -O2 -w -Wl,--image-base,0x400000 -DPROTO \
  -I"$ROOT/tests/stubs" -Iproto -I"$P/Application" -I"$P/Common" -I"$P/Drivers" -I"$P/System" -I"$P/SRC/Debug" \
  "$@" sim.c -lm -o "$OUT"
