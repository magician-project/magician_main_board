# MagicianCam controller firmware (Pico 2 / RP2350) — how to talk to the board

Firmware **2.2** for the Cam5 / v4 boards. The board is still called "the arduino" in
the grabber and in older notes; it is the same thing as "the controller".

The board accepts the same commands, and sends the same replies and CSV, over two links
at the same time:

| Link | Address | Notes |
|---|---|---|
| USB serial | `/dev/ttyACM*` (stable name: `/dev/serial/by-id/usb-Raspberry_Pi_Pico_2_<serial>-if00`) | 115200 8N1. Boot messages appear only here. Your user needs to be in the `dialout` group. |
| Ethernet (W5500, Telnet server) | `192.168.137.64:23`, netmask `255.255.255.0`, gateway `.1` | Raw TCP (no Telnet negotiation). **One client at a time.** Same subnet as the Doosan controller's default `.100`. |

Replies (`V:`, `P:`, the `x` report) go back on the link that asked. The CSV stream goes
to the link that sent `i`.

---

## Quick start

### USB
```sh
ls -l /dev/serial/by-id/ | grep Pico            # find the board
picocom -b 115200 --omap crlf /dev/ttyACM0      # or: screen /dev/ttyACM0 115200
```
Type `v` + Enter → `V:2.2`. Type `x` + Enter for the full diagnostic report.

Scripted (pyserial):
```python
import serial, time
s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)
s.write(b"v\n"); print(s.readline())          # b'V:2.2\r\n'
s.write(b"p\n"); print(s.readline())          # b'P:11V=1/1,5V=1/1,VBUS=1,ADC3=957mV,...'
```

### Ethernet
Give the PC an address on the same subnet (not `.64`, `.1` or `.100`), e.g.:
```sh
sudo ip addr add 192.168.137.10/24 dev eth0
nc 192.168.137.64 23                            # then type commands, one per line
```
`x` reports `ETH HW: 3` and the IP when the W5500 is detected; `ETH link: 1` with a
cable, `2` without. `ETH HW: 0` means no W5500 answered (see Troubleshooting).

---

## Commands

Every command can be followed by `\n` (the grabber always sends one). Single-letter
commands act on the letter itself. **Line commands** (`E`, `N`, `S`, `P`) are collected up to
the newline and must end with one. A line starting with an unknown letter is discarded
entirely, so its digits are never run as light selections.

### Status and session
| Command | Reply / effect |
|---|---|
| `v` | `V:2.2` |
| `x` | Full diagnostic report (~100 lines, streamed one line per loop): system, network, queues, exposure timing, ToF, light limits & schedule, power rails |
| `p` | One-line power report, see [Power](#power-rails) |
| `i` | Start the CSV stream **and** timer mode (lights step every 100 ms, firing on exposures) |
| `f` | Stop the CSV stream |
| `h` | Reset the report timestamp to 0 |
| `z` | Reboot (lights off, 3-second countdown) |

### Lights
| Command | Effect |
|---|---|
| `0` | All lights off and disarmed; external mode |
| `1`–`6` | Arm that light (1-indexed) for the next exposures; external mode |
| `+` | Advance to the next light and send exactly one CSV row |
| `r` | Timer mode, round-robin order 1-2-3-4-5-6 |
| `t` | Timer mode, opposite order 1-4-2-5-3-6 |
| `y` | Timer mode that stops by itself after 10 s |
| `a` | Depth mode: light chosen from the ToF distances |
| `e` | **Exposure-locked**: one schedule step per camera exposure, one CSV row per exposure |
| `E<us>` | Maximum light-on time per exposure, e.g. `E800` (clamped 50–1000 µs; default 1000) |
| `N<n>` | No light fires more than `n` exposures in a row (`N0` = unlimited, the default). When reached, the next different light in the schedule fires instead; if the schedule holds no other light, that exposure stays dark |
| `S<digits>` | Light schedule, **0-indexed**, up to 60 entries, e.g. `S01235` (skips light 5). Default `S012345` |

Lights only fire on the camera's exposure signal (GP2). With no camera connected,
nothing lights up whatever the mode.

### Power rails
| Command | Effect |
|---|---|
| `P11,0` / `P11,1` | Camera power (11.5 V rail → J16 → camera connector) off / on |
| `PR` / `PR<ms>` | Power-cycle the camera: off for `ms` (default 500, clamped 50–10000), then on again by itself |
| `P5,0` / `P5,1` | 5 V rail off / on. **Off is refused without USB**: the 5 V rail then powers the Pico itself |
| `p` | `P:11V=c/r,5V=c/r,VBUS=0\|1,ADC3=<mV>mV,CYCLE=0\|1,MCP=0\|1,REFUSED=n` |

In the `p` reply, `c/r` is *commanded / read back from the MCP23018 enable pin*: `1/1` on,
`0/0` off, a mismatch means the expander is not holding the state. `?` = unreadable.
There is **no rail voltage sense** on this board, so "read back" does not prove that a
rail actually came up. `ADC3` is the raw GP29 pin voltage (VSYS/3 on an official Pico 2;
unverified on the clone modules fitted). `P:ERR=timeout` means core 0 did not refresh in
250 ms. Lights are switched off before any rail goes down.

---

## CSV stream

17 comma-separated fields per line (the grabber adds its own host timestamp in front):

```
dev_timestamp,Button1,Button2,Distance1,Distance2,Distance3,Light1..Light6,StrobeCounter,StrobeLight,ForcedAdvances,Starved,WatchdogTrips
151522,0,0,F,F,F,1,0,0,0,0,0,0,-1,0,0,0
```

| Field | Meaning |
|---|---|
| `dev_timestamp` | ms since boot or since `h` |
| `Button1/2` | 1 while pressed |
| `Distance1..3` | ToF centre distance in mm; `0` no data/stale, `F` sensor failed, `H` beyond 6000 mm |
| `Light1..6` | Selected light (in `e` mode: the light that actually fired) |
| `StrobeCounter` | Exposures fired since boot |
| `StrobeLight` | Light that fired on that exposure, 0-indexed; `-1` none yet |
| `ForcedAdvances` | Exposures where the `N` limit fired another light |
| `Starved` | Exposures left dark by the `N` limit |
| `WatchdogTrips` | Always 0 (reserved) |

When rows are sent: on every light change (`i`, `r`, `t`, `y`), every 100 ms (`a`), once
per `+`, once per exposure (`e`).

---

## Power-on state

All lights off and disarmed, external mode, CSV off, `N0`, `E1000`, schedule `012345`,
both rails on, ToF ranging at 8×8 / 10 Hz, Ethernet up at `192.168.137.64`.
`E`, `N`, `S` and the rails live in RAM: `0` and `f` do not reset them, a power cycle or
`z` does. The grabber re-sends `E`, `N` and `S` on every run.

---

## From the grabber (`../../../magician_grabber`)

```sh
magician_grabber --pico2 --arduino /dev/ttyACM0 ...          # USB (unchanged default path)
magician_grabber --controller-ip 192.168.137.64 ...          # Ethernet; --arduino-ip is the same option
```
On firmware ≥ 2.2 the grabber sends `v`, `i`, the lighting mode, `E<exposure+150>`, `p`,
`N` (from `--maxrepeat`, else `N0`), `S` (lights left by `--skip`), and `e` for
`--exposure-locked`. On exit it sends `0` and `f`. The power functions
(`arduino_setCameraPower`, `arduino_powerCycleCamera`, `arduino_set5VRail`,
`arduino_queryPowerRails`) are in `arduinoSensor.h`.

---

## Building and flashing

Arduino IDE (or arduino-cli) with the **arduino-pico 5.6.0** core, board
"Raspberry Pi Pico 2". No other libraries need installing: Adafruit_VL53L5CX (patched),
Adafruit BusIO and Ethernet are vendored in [`src/`](src/README.md).

The IDE resets the board into BOOTSEL and copies the UF2. If it reports
*"No drive to deploy"*: unplug, hold **BOOTSEL**, plug in, release, upload again. If the
board does not show up at all (`lsusb | grep 2e8a` is empty), try another cable — charge-only
cables power the board but carry no data.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| No `/dev/ttyACM*`, nothing in `lsusb` | Charge-only USB cable or dead port |
| `x` shows `ETH HW: 0`, `GPIOA 0x03` in the power section | WIZ850io not powered/seated, or faulty (its own pull-up keeps the reset line, GPIOA bit 2, high when it has power). Swap modules to tell board from module |
| `Distance` columns all `F` | ToF sensors not connected or failed at boot (they are only initialised at power-up) |
| `P:11V=1/0` | Expander not holding the camera rail on (reset or brown-out) |
| Commands seem ignored after typing junk | A line starting with an unknown letter is discarded up to the next Enter |
| Another `/dev/ttyACM*` appears | Check `/dev/serial/by-id/`; e.g. a phone's USB modem also shows up as ttyACM |
