# dynamixel-on-device

**Run the Dynamixel control loop on your microcontroller instead of a host PC. Protocol 2.0 with Fast Sync Read, over a 74HC241 half-duplex transceiver or equivalent. One header, no heap, testable on your laptop with no hardware.**

![license](https://img.shields.io/badge/license-AGPL--3.0-blue.svg)
![platform](https://img.shields.io/badge/platform-Arduino%20cores-orange.svg)
![language](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![tests](https://img.shields.io/badge/tests-passing-brightgreen.svg)

`tiny_dxl` is a single-header C++ driver for driving ROBOTIS **Dynamixel X-series** servos
from a microcontroller over a half-duplex bus. It is small, reentrant, and allocation-free,
it implements **Fast Sync Read (0x8A)** for minimum-latency feedback, and its entire wire
protocol is **unit-tested on your laptop with no hardware attached**.

The point of it is to move the control loop off the PC. A Dynamixel chain is usually driven
from a host computer through a U2D2, which puts a USB stack and a non-realtime OS scheduler
inside your feedback path. This driver is small and predictable enough to close that loop on
the microcontroller instead.

## Why use it

- **It is fast where it counts.** A single `fastSyncRead()` pulls position, current,
  velocity, and hardware-error from every servo in one status packet with one bus
  turnaround, rather than a packet-plus-turnaround per servo. At 4 Mbaud that takes the
  read-and-write wire time per control tick from about 0.78 ms to about 0.19 ms.
  **Those two figures are calculated from the Protocol 2.0 frame sizes (bytes x 10 bits /
  baud), not measured on hardware**, and they exclude the bus turnaround, which only a
  capture on a real bus can give you. On that budget a 2 kHz inner current loop fits with
  room left over, where a naive `SyncRead` at 1 Mbaud runs out of budget near 1 kHz. The
  full derivation, and the benchmark table waiting to be filled from hardware, are in
  [`docs/DESIGN.md`](docs/DESIGN.md).
- **It is correct on the wire.** Full Protocol 2.0 byte-stuffing (it survives a payload that
  crosses `0xFD 0xFF`, the classic "works until the position passes 0xFDFF" bug), a
  table-driven CRC-16, and a bounded receive where every failure is counted and nothing is
  silently retried.
- **It is allocation-free with bounded waits.** No `String`, no dynamic allocation, no
  unbounded spins. It is meant to run in a hard inner loop on a Cortex-M7.
- **It comes with a worked safety chain.** Listen-before-transmit so it will not fight a
  second master on the bus, refusal to connect to a half-populated chain, link qualification
  before a baud rate is trusted, the servo-side Bus Watchdog armed as a dead-man, and fail-safe
  (torque off, counted, recoverable) on any hardware-error bit or sustained bus loss. To be
  precise about where this lives: `busQuiet()` and the failure counters are in the library, and
  the rest is in [`examples/wearable_hand_bringup/`](examples/wearable_hand_bringup), because a
  safety policy belongs to the application. The example is meant to be read and adapted, not
  treated as a black box.
- **You can test it without hardware.** A faithful bus emulator runs the frame codec and the
  full control sketch on any machine with a C++ compiler, so you can trust a change before
  you flash it.
- **It is not tied to one board.** Everything is plain C++17 over `<Arduino.h>`,
  `HardwareSerial` and `micros()`. The one platform-specific thing, turning the half-duplex
  transceiver around, is isolated behind four selectable strategies. See
  [Which boards](#which-boards).

## It works, and it is tested

```
$ ./test/run.sh
====  codec: 40 passed, 0 failed        # core without hardware direction
====  codec: 37 passed, 0 failed        # core with hardware direction (Teensy)
====  codec: 25 passed, 0 failed        # mode pinned to HARDWARE (opt-in)
====  codec: 28 passed, 0 failed        # mode pinned to MANUAL (opt-in)
====  motor integration: 41 passed, 0 failed
```

171 assertions across five builds cover the Fast Sync Read parser, the CRC-16, byte-stuffing
across `0xFDFF`, both half-duplex direction strategies, single-master refusal, missing-motor
refusal, link qualification, the hardware-error fail-safe, and the servo Bus Watchdog, all
against an emulated two-servo bus. You need no Teensy, no servos, and no Arduino toolchain to
run them.

The codec suite is compiled four times on purpose, once per direction configuration. Each one
`#if`s out the others, so without the extra builds a path could break unnoticed until someone
flashed the board it belongs to.

## Install

**As an Arduino library.** Clone into your Arduino `libraries/` folder, then
`#include <tiny_dxl.h>`. On Teensy you will need Teensyduino; on other boards, whatever core
your board normally uses.

```
git clone https://github.com/molanocortes/dynamixel-on-device \
  ~/Documents/Arduino/libraries/dynamixel-on-device
```

**Direct.** Drop `src/tiny_dxl.h` next to your sketch and `#include "tiny_dxl.h"`.

## Quickstart

```cpp
#include <tiny_dxl.h>

// Serial1 (pins 0/1) through a 74HC241 buffer; direction on pin 7.
TinyDXL dxl(&Serial1, 7);
const uint8_t ids[2] = {1, 2};

void setup() {
  dxl.begin(4000000);              // 4 Mbaud (the XC330-M181 maximum)
  uint16_t model;
  dxl.ping(ids[0], &model);        // 1230 is the XC330-M181-T
}

void loop() {
  // One Fast Sync Read: 2 servos, each [Present Current(2) Velocity(4) Position(4)].
  // Registers 126..135 are contiguous, so this is a single turnaround.
  uint8_t fb[2 * 10];
  if (dxl.fastSyncRead(126, 10, ids, 2, fb)) {
    // fb[i*10 + 0..1] = current, +2..5 = velocity, +6..9 = position, per servo.
    // ... run your control law ...
  }
  // One Sync Write: Goal Current per servo (register 102, 2 bytes, little-endian mA).
  uint8_t goal[2 * 2] = { /* ... */ };
  dxl.syncWrite(102, 2, ids, 2, goal);
}
```

The bundled example, [`examples/wearable_hand_bringup/`](examples/wearable_hand_bringup), is
a complete real-world application: a Teensy 4.1 wearable articulated-hand rig that runs the
driver at 2 kHz in a blended transparency and assist current-control loop, over a simple
serial menu, with detection, link qualification, and the full fail-safe chain.

## Which boards

A Dynamixel bus is one wire, so something has to turn the transceiver around between "we
talk" and "the servo talks". That is the only part of this driver that is not portable C++,
and it is isolated behind one setting. Everything else, the framing, the CRC, the
byte-stuffing, the parsers, is core-agnostic.

| Direction mode | What drives the pin | Where |
|---|---|---|
| `DXL_DIR_HARDWARE` | The UART peripheral, inside the transmit interrupt. No software in the path, fastest turnaround the wiring allows. | Teensy (`transmitterEnable`) |
| `DXL_DIR_MANUAL` | `digitalWrite` around `write()` + `flush()`. Portable, because `flush()` is defined to return only once the last stop bit has left. | Any Arduino core |
| `DXL_DIR_CALLBACK` | Your function. Use it for a core with a native RS485 half-duplex mode worth having instead of a GPIO toggle, or for odd wiring such as a direction bit behind a shift register. | Anywhere |
| `DXL_DIR_NONE` | The transceiver itself, for auto-direction parts like the MAX13487. No pin needed. | Anywhere |

You normally pick none of these. The default is `DXL_DIR_AUTO`, resolved identically on every
board: `CALLBACK` if you installed a handler, else `NONE` if you passed no pin, else
`HARDWARE` where the core has it, else `MANUAL`. Asking for `HARDWARE` on a core without it
**degrades to `MANUAL` rather than failing quietly**, and `directionMode()` tells you what you
actually got.

```cpp
TinyDXL dxl(&Serial1, 7);                   // what you almost always want, on any board
TinyDXL dxl(&Serial1, 7, DXL_DIR_MANUAL);   // force the portable path
TinyDXL dxl(&Serial1, DXL_NO_PIN);          // self-directing transceiver

// Or drive it yourself, e.g. an ESP32 using native RS485 mode on the RTS pin:
void myDir(void* ctx, bool transmit) { /* ... */ }
dxl.setDirectionHandler(myDir);      // before begin()
dxl.begin(4000000);
```

### What the portability costs

Every board runs the same code and resolves the strategy the same way. That costs a load and
two predictable branches on each side of a frame. Diffing the compiler's output for
`sendPacket`, the transmit hot path, at `-O2`:

| Build | `sendPacket` |
|---|---|
| Original, Teensy-only, no portability layer | 128 instructions |
| **Default, every board** | **159 instructions** |
| `-DTINY_DXL_DIR_MODE=DXL_DIR_HARDWARE` (opt-in) | 128, identical to the original |

**30 instructions per packet.** A 2 kHz tick sends two packets, so about 60 per tick, on the
order of 100 ns on a 600 MHz M7 against a 500 us budget: **under 0.03 % of the tick**. For
scale, that is a small fraction of the time it takes to put a *single byte* on the wire at
4 Mbaud. It sits below the noise floor of anything you could measure on the bus.

That is why there is no special case for the fast boards. One code path, one behaviour, one
constructor signature everywhere, which is worth more than a saving nobody can observe. If
you want the instructions back anyway:

```cpp
#define TINY_DXL_DIR_MODE DXL_DIR_HARDWARE   // before #include <tiny_dxl.h>
```

The mode becomes a compile-time constant, the branches fold away, and for `HARDWARE` and
`NONE` the driver emits no direction code at all. In a pinned build the constructor takes no
mode argument, so asking for a strategy it cannot honour is a **compile error** rather than a
setting that silently does nothing. `./test/run.sh` builds every configuration, so none can
quietly rot.

Caveat on those counts: they were produced on an ARM64 host, not a Cortex-M7, because there
is no Teensy toolchain in this environment. The instruction counts are real; the nanosecond
figure is arithmetic on top of them, not a measurement. Nobody has timed this on an M7.

**Status per platform, stated honestly:**

| Platform | State |
|---|---|
| **Teensy 4.x** | The reference target. Hardware-timed direction, developed and bench-used here. |
| **ESP32, RP2040, STM32, SAMD, AVR** | Should work through `MANUAL`. The logic is verified on the host (below), but **no one has run this on those boards yet**. If you do, please open an issue either way. |
| **Raspberry Pi and other Linux boards** | Not supported. The codec compiles and runs there already, that is how the test suite works, but there is no UART transport, and a non-realtime OS reintroduces exactly the jitter this driver exists to avoid. |

The direction logic is not taken on trust. `./test/run.sh` builds the codec suite **twice**,
once as a software-direction core and once as a hardware-direction one, and asserts that the
pin goes up before the first byte and down after the last, that neither strategy changes the
frame by a single byte, and that an unsupported request degrades instead of going silent.
That is real coverage of the porting seam, but it is host coverage: it cannot tell you your
board's `flush()` returns as late as it promises. Only a scope or a working servo can.

## How it works

The physical layer is one hardware UART plus a 74HC241 octal buffer for half-duplex. The
Teensy core toggles the direction pin inside the UART interrupt (`transmitterEnable`), so the
turnaround is hardware-timed, which is the fastest the wiring allows and avoids bit-banging.

For Fast Sync Read (0x8A), the driver sends the instruction and parses the single
concatenated status packet, which is `[error][id][data][crc]` per servo. It trusts the
on-wire length and verifies the packet CRC. Regular Sync Read (0x82) stays available as a
fallback.

To get everything in one shot, map the exact registers you need, including the
non-contiguous Hardware-Error byte, into one contiguous Indirect Data block. A single Fast
Sync Read then grabs them all.

The full timing budget, the transparency argument (loop latency, phase lag, stable gain,
felt back-drive force), and the benchmark method are in [`docs/DESIGN.md`](docs/DESIGN.md).

## Requirements

- An Arduino core providing `<Arduino.h>`, `HardwareSerial` and `micros()`. Developed on the
  Teensy 4.1 (NXP i.MX RT1062, Cortex-M7) with Teensyduino. See [Which boards](#which-boards)
  for what is tested where.
- A 74HC241, or an equivalent tri-state buffer, for half-duplex direction control, unless
  your transceiver self-directs.
- Dynamixel X-series servos, developed against the XC330-M181-T. Adjust the control-table
  addresses for other models.
- To run the host tests: any C++17 compiler, nothing else.

Two things scale with the board rather than the driver. Baud is limited by your UART, so
4 Mbaud is a Teensy/ESP32-class figure and an AVR will not approach it. And `micros()`
resolution sets how finely the turnaround can be measured, 1 us on ARM but 4 us on AVR.

## Wiring: why a bus this simple needs a buffer

A Dynamixel X-series bus is **one wire**. `DATA` carries both directions, and every device on
the chain, the servos and your microcontroller, shares it. A UART, by contrast, has two
separate pins: `TX` always drives, `RX` always listens. Those two facts do not fit together on
their own, and that mismatch is what the extra chip exists to solve.

If you wired `TX` straight to `DATA`, your microcontroller would drive that wire **all the
time**, including while a servo is trying to answer. Two outputs fighting over one wire means
the data is destroyed and, depending on the parts, the pins can be damaged. So `TX` needs a
switch: connected while you talk, electrically disconnected the rest of the time. That third
state, neither high nor low but *disconnected*, is what a **tri-state buffer** provides, and
it is why a plain logic gate will not do.

The reference build uses a **74HC241** octal tri-state buffer:

```
                          ┌──────────────┐
   MCU TX  ──────────────►│              │──────┐
                          │   74HC241    │      │
   MCU RX  ◄──────────────│              │◄─────┤────►  DATA  (to the servo chain)
                          │              │      │
   DIR pin ──────┬───────►│ /OE (enable) │      │
                 │        └──────────────┘      │
                [R]                            [R] 10k pull-up to 3V3
               pull-down                        │
                 │                              │
                GND                            3V3
```

- **`DIR` high**: the TX path is enabled, your microcontroller drives `DATA`. You are talking.
- **`DIR` low**: the TX path goes high-impedance and disappears from the wire. The servo can
  drive `DATA`, and you hear it on `RX`. You are listening.

Three details matter in practice:

- **Pull-down on `DIR`.** Before your firmware runs, pins float. A pull-down guarantees the
  buffer boots into *receive*, so a half-configured board cannot sit on the bus and jam it.
- **Pull-up on `DATA`.** The line needs a defined idle state, otherwise a floating bus reads
  as noise and you will chase phantom bytes. 10k to 3V3 is the usual value.
- **A small series resistor** toward the servos limits current if two devices ever do
  overlap, which is cheap insurance during bring-up.

**Getting the switch timing right is the hard part**, and it is what the direction modes in
[Which boards](#which-boards) are about. Release the line one bit too early and your last byte
is truncated. Release it too late and you are still driving while the servo starts answering.
Teensy solves this in hardware by toggling `DIR` inside the UART interrupt; elsewhere the
driver does it in software around `flush()`. Both orderings are asserted in the test suite.

**"Or equivalent" means:** any tri-state buffer works (a 74HC125 or 74HC126 for a smaller
package, a proper RS485 transceiver such as a MAX485 or SN75176 if you want differential
signalling and its protection). **Auto-direction** parts like the MAX13487 sense the traffic
and flip themselves, in which case you pass `DXL_NO_PIN` and the driver stops thinking about
direction entirely. Some boards, notably ROBOTIS's own OpenRB-150, have all of this built in.

Whatever you use, keep **one master on the bus at a time**: unplug the U2D2 before the
microcontroller takes over. The driver listens for 120 ms before its first transmission and
refuses to talk if it hears anything, but that is a second line of defence, not the first.

## Contributing

Issues and pull requests are welcome. Please keep the driver allocation-free and every wait
bounded, and make sure `./test/run.sh` is all green before you submit.

## License

Free and open source under the **GNU AGPL-3.0**, see [`LICENSE`](LICENSE). A separate
**commercial license** is available for proprietary or closed-source use, see
[`COMMERCIAL.md`](COMMERCIAL.md). It is free for the community, and commercial users who want
to keep their own work closed help fund it.

## Author

Created by **Juan Sebastian Molano**. If it saves you time, a star is appreciated.
