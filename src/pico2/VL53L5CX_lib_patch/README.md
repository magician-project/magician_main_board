# VL53L5CX library patch

Patched sources for **Adafruit_VL53L5CX 1.0.1**, from the CRF development thread
(original archive: `../VL53L5CX_lib_patch.zip`). `MagicianCam4.ino` is built and
released against these.

## Install

Copy the two files over the library sources, keeping a backup of the stock ones:

```sh
LIB=~/Arduino/libraries/Adafruit_VL53L5CX/src
for f in platform.cpp vl53l5cx_api.cpp; do
  [ -f "$LIB/$f.orig" ] || cp -p "$LIB/$f" "$LIB/$f.orig"
  cp "$f" "$LIB/$f"
done
```

The `.stock` files here are the unmodified 1.0.1 sources, kept so the patch stays
diffable after an IDE library update overwrites the installed copies:

```sh
diff -u platform.cpp.stock     platform.cpp
diff -u vl53l5cx_api.cpp.stock vl53l5cx_api.cpp
```

## What it changes

**`platform.cpp`**
- `RdByte` / `WrByte` retry up to 10 times with a 2 ms gap instead of failing on
  the first NACK.
- `WrMulti` uses a fixed 34-byte buffer instead of a VLA sized from
  `maxBufferSize()`, and guards `maxBufferSize() <= 2`; `RdMulti` guards `== 0`.

**`vl53l5cx_api.cpp`**
- `_vl53l5cx_poll_for_answer` checks the match *before* sleeping and returns the
  real status instead of OR-ing it, so a bus error aborts the poll rather than
  spinning out the full 2 s timeout. The `temp_buffer[2] >= 0x7f` MCU-error check
  is gone.
- `vl53l5cx_init` retries the `0x000F` `0x40 -> 0x43 -> 0x40` boot transition up
  to 5 times, reading the register back after each write and only accepting the
  sequence if all three readbacks match.
- The FW-access enable (`0x000E`/`0x7fff`/`0x03`, then poll `0x21 & 0x10`) is
  retried up to 3 times; on some boot cycles the first poll never leaves 0x00.

The rest of the diff is whitespace and brace-style reformatting.

## Status on the FORTH setup

Untested against hardware here so far — the stock library has been working on the
FORTH boards, so whether these retry paths are actually load-bearing on this set
of VL53L5CX modules is still an open question. They cost nothing when the sensor
boots cleanly, so the patched library is the default regardless.
