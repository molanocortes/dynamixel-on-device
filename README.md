# dynamixel-fast-teensy

**A tiny, zero-heap Dynamixel Protocol 2.0 master for Teensy, with Fast Sync Read and a rock-steady 2 kHz current-control loop.**

![license](https://img.shields.io/badge/license-AGPL--3.0-blue.svg)
![platform](https://img.shields.io/badge/platform-Teensy%204.x-orange.svg)
![language](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![tests](https://img.shields.io/badge/tests-62%20passing-brightgreen.svg)

`tiny_dxl` is a single-header C++ driver for driving ROBOTIS **Dynamixel X-series** servos
from a **Teensy 4.x** over a 74HC241 half-duplex bus. It is small, reentrant, and
allocation-free, it implements **Fast Sync Read (0x8A)** for minimum-latency feedback, and
its entire wire protocol is **unit-tested on your laptop with no hardware attached**.

## Why use it

- **It is fast where it counts.** A single `fastSyncRead()` pulls position, current,
  velocity, and hardware-error from every servo in one status packet with one bus
  turnaround, rather than a packet-plus-turnaround per servo. At 4 Mbaud the read-and-write
  wire time per control tick drops from about 0.78 ms to about 0.19 ms. That is the headroom
  that lets a 2 kHz inner current loop hold steady, where a naive `SyncRead` at 1 Mbaud
  runs out of budget near 1 kHz.
- **It is correct on the wire.** Full Protocol 2.0 byte-stuffing (it survives a payload that
  crosses `0xFD 0xFF`, the classic "works until the position passes 0xFDFF" bug), a
  table-driven CRC-16, and a bounded receive where every failure is counted and nothing is
  silently retried.
- **It is allocation-free with bounded waits.** No `String`, no dynamic allocation, no
  unbounded spins. It is meant to run in a hard inner loop on a Cortex-M7.
- **It is safe by construction.** It listens before it transmits (so it will not fight a
  second master on the bus), it requires all expected servos before it connects, it
  qualifies the link before trusting a baud rate, it arms the servo-side Bus Watchdog as a
  dead-man, and it fails safe (torque off, counted, recoverable) on any hardware-error bit
  or on sustained bus loss.
- **You can test it without hardware.** A faithful bus emulator runs the frame codec and the
  full control sketch on any machine with a C++ compiler, so you can trust a change before
  you flash it.

## It works, and it is tested

```
$ ./test/run.sh
====  codec: 21 passed, 0 failed
====  motor integration: 41 passed, 0 failed
```

62 assertions cover the Fast Sync Read parser, the CRC-16, byte-stuffing across `0xFDFF`,
single-master refusal, missing-motor refusal, link qualification, the hardware-error
fail-safe, and the servo Bus Watchdog, all against an emulated two-servo bus. You need no
Teensy, no servos, and no Arduino toolchain to run them.

## Install

**As an Arduino library.** Clone into your Arduino `libraries/` folder, then
`#include <tiny_dxl.h>`. You will need Teensyduino for Teensy 4.x.

```
git clone https://github.com/molanocortes/dynamixel-fast-teensy \
  ~/Documents/Arduino/libraries/dynamixel-fast-teensy
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

- Teensy 4.x, developed on the Teensy 4.1 (NXP i.MX RT1062, Cortex-M7), plus Teensyduino.
- A 74HC241, or an equivalent tri-state buffer, for half-duplex direction control.
- Dynamixel X-series servos, developed against the XC330-M181-T. Adjust the control-table
  addresses for other models.
- To run the host tests: any C++17 compiler, nothing else.

## Hardware note

Give the servo `DATA` line a pull-up to 3V3 and a small series resistor toward the servos,
and give the direction pin a pull-down so the buffer boots into receive. Keep a single master
on the bus at a time. The exact bring-up wiring is in the example header and in
`docs/DESIGN.md`.

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
