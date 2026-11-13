# Vendored libraries

The firmware builds without installing any Arduino library besides the
arduino-pico core (5.6.0). The Arduino IDE / arduino-cli compiles everything
under the sketch's `src/` folder automatically.

| Folder | Library | Version | Source | License |
|---|---|---|---|---|
| `Adafruit_VL53L5CX/` | Adafruit VL53L5CX | 1.0.1 + FORTH patch | downloads.arduino.cc/libraries/github.com/adafruit/Adafruit_VL53L5CX-1.0.1.zip | MIT (`LICENSE`); the ST ULD files point to that LICENSE, else as-is |
| `Adafruit_BusIO/` | Adafruit BusIO (dependency of the above) | 1.17.4 | downloads.arduino.cc/libraries/github.com/adafruit/Adafruit_BusIO-1.17.4.zip | MIT, see `LICENSE` |
| `Ethernet/` | Arduino Ethernet (W5500 via `Ethernet.init(cs)`) | 2.0.2 | downloads.arduino.cc/libraries/github.com/arduino-libraries/Ethernet-2.0.2.zip | MIT-style permissive, see file headers (no LICENSE file in the release) |

Only the library sources (plus license / `library.properties`) were copied;
examples, docs and test sketches were left out.

## Local changes

1. **VL53L5CX patch.** `Adafruit_VL53L5CX/platform.cpp` and
   `Adafruit_VL53L5CX/vl53l5cx_api.cpp` are the patched files from
   `../OLD/VL53L5CX_lib_patch/` (I2C retries, bounded `WrMulti` buffer,
   boot-sequence retries). The stock 1.0.1 files are kept there as `*.stock`;
   the downloaded 1.0.1 release matched them byte for byte.
2. **Include paths.** A sketch `src/` folder is not on the include path the way an
   installed library is, so includes between these libraries are quoted and relative:
   - `Adafruit_BusIO/*`: `<Adafruit_X.h>` → `"Adafruit_X.h"`
   - `Adafruit_VL53L5CX/Adafruit_VL53L5CX.h`, `platform.h`:
     `<Adafruit_I2CDevice.h>` / `<Adafruit_BusIO_Register.h>` → `"../Adafruit_BusIO/…"`
   - `Ethernet/utility/w5100.cpp`: `"Ethernet.h"` → `"../Ethernet.h"`
   - `../hardware_config.h` includes `"src/Ethernet/Ethernet.h"` and
     `"src/Adafruit_VL53L5CX/Adafruit_VL53L5CX.h"`.

Because every include is quoted, a copy of these libraries installed in
`~/Arduino/libraries` is never picked up for this sketch.

## Updating

Download the new release, copy its `src/` over the folder, re-apply the patch
(diff against `../OLD/VL53L5CX_lib_patch/*.stock` first) and redo the include
edits above.
