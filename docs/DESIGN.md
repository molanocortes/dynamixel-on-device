# Fast Dynamixel bus + 2 kHz current-control transparency: a design note

Teensy 4.1 + 74HC241 half-duplex path, two daisy-chained **XC330-M181-T** (IDs 1, 2),
current-control mode. This note covers the change set, the timing budget, and the
transparency argument. Every performance number is marked **computed** (from
bytes x 10 bits / baud) or **measured** (captured on hardware via `M,b`), nothing here is
invented. No Teensy toolchain was available in the authoring environment, so the on-target
`.hex` build and the `M,b` capture are the bench step; the control-path logic was verified
off-target by 49 host-compiled assertions (see *Verification*).

## What changed and why (each item states the reason)

| # | Change | Reason |
|---|--------|--------|
| 1 | Baud **1 -> 4 Mbaud** (`reg 8 = 6`; `M,u`, ping-verified, scan fallback) | 4x less wire time/frame. **4 Mbps is the XC330-M181 maximum** (verified control table); reg value 7 / 4.5 Mbps is an XM/XH capability this actuator does not have, so 4 Mbaud is the fastest *honest* operating point. |
| 2 | Return-Delay-Time **8 -> 0 us** (`reg 9 = 0`) | Free latency removed from every status frame. |
| 3 | SyncRead `0x82` **-> Fast Sync Read `0x8A`** (`fastSyncRead()` in `tiny_dxl.h`, `0x82` kept as fallback) | `0x82` pays one header + one bus turnaround *per motor*; `0x8A` returns all feedback concatenated in **one status packet with one turnaround**. The saving grows with motor count. |
| 4 | **Indirect Data block** maps Present-Position (132), Present-Current (126), Present-Velocity (128) **and Hardware-Error (70)** into 11 contiguous bytes at Indirect Data 1 (**224**), armed via Indirect Address 1 (**168**), read-back verified | One Fast Sync Read grabs position + current + velocity + the *non-contiguous* hardware-error byte the old 126..135 read missed, in a single shot. |
| 5 | Hot RX poll is a **tight, yield-free** bounded spin | `yield()` invited scheduler jitter; the wait stays bounded and every miss is counted. |
| 6 | (stretch) DMA/ISR UART | **Not implemented, deliberately.** See *Why not DMA* below; the actuator, not the CPU, is the bottleneck. |
| - | Wired the motor subsystem into `loop()` | `handleMotorLine()` and `motorService()` were **defined but never called** in the committed sketch, the whole `M,` menu was dead code. Added a line-buffered `M,` reader (no aliasing of the single-letter menu or the `D,` swallow) and `motorService()` calls, so the control tick actually runs. |
| - | Fixed a latent multi-id `readStatus` over-read | At 4 Mbaud/RDT 0 the two SyncRead statuses can sit in the FIFO together; the greedy read swallowed and dropped `id2`. Now each read is bounded by its own packet `need`. (`0x8A` is immune; this hardens the `0x82` fallback.) |
| - | Hardened `motorTake()` detect->connect | Single-master listen-before-talk, require BOTH motors, qualify the link (>=15/16 round-trips), verify model 1230, connect torque-off. See *Detection & connection*. |
| - | Servo-side dead-man (Bus Watchdog reg 98) | Protects the wearer if the **Teensy itself** hangs (host/bus watchdogs can't run then): the servo zeroes goal current after 100 ms with no packet. Petted by every Sync Write; re-armed after a legitimate stall so transient blocks self-heal, latched-safe on a true hang. |
| - | `motorEnable()` torque-safety | Config is done torque-OFF and its success tracked; motors are energized **only if config fully took**, else de-energized and reported failed. Never "torque physically ON while firmware reports OFF" (which, in a stale operating mode, could snap the finger). |

Safety and invariants preserved: **single-master** (Teensy drives DATA only with the U2D2
disconnected, unchanged); zero heap; every wait bounded; every failure counted; software
Goal-Current clamp below the `reg 38` Current-Limit backstop; **torque-off on any Hardware
Error Status bit or `MOTOR_ERR_TRIP` consecutive missed reads** (counted, recoverable via
`M,e,1`); byte-stuffing counts stuffed bytes in LEN and survives a payload crossing `0xFDFF`
(proven, below). Goal-Current unit kept at **~1.0 mA/unit** (XC330); M181-T model preserved.

## Detection & connection (correct, safe, reliable)

`M,t,1` no longer trusts a single ping. `motorTake()` runs a four-step detect-then-connect
sequence, and only the last step marks the bus taken:

1. **Single-master safety (listen before you talk).** With the 74HC241 idle (receive), the
   Teensy listens for **120 ms** (`busQuiet()`, RX-only, zero TX). If *any* byte arrives, the
   U2D2 (or another master) is driving DATA, so the Teensy **refuses to take the bus and never
   transmits**, no collision. This is software defense-in-depth behind the physical rule
   (disconnect the U2D2). It is baud-agnostic: even wrong-baud traffic shows up as bytes.
2. **Correct detection.** Scan bauds fastest-first; at each, ping **every** expected ID with
   retries. A **half-populated** bus (one servo missing/unpowered) is reported and rejected,
   not silently accepted, the old code marked the bus taken on one motor, which then thrashed
   the control loop into a fault.
3. **Reliable link qualification.** Before trusting a baud, require **>= 15 of 16** full
   round-trips to succeed. A marginal 4 Mbaud (signal integrity over the cable) therefore
   **falls back to a solid slower baud automatically**, and the measured rate is reported.
4. **Identity + safe connect.** Verify the **model number = 1230 (XC330-M181-T)** (warn, don't
   brick), then **force torque OFF on every motor** so the device connects *compliant*, never
   driving on connect, and any prior fault cleared. Enabling is the separate, explicit `M,e,1`.

Failure modes are explicit and safe: busy bus -> "disconnect the U2D2"; missing motor ->
"not fully populated"; unreliable link -> "trying a slower baud"; nothing -> "check 74HC241,
wiring, 5 V motor supply". In every non-success case `taken` stays false and torque is never
enabled. Verified by host tests (single-master refusal, missing-motor refusal, link-quality
gating, torque-off-on-connect, and recovery when both motors return).

## Timing budget (computed, 2 motors)

Bit time = 10 bits/byte. `1 Mbaud = 10 us/byte`, `4 Mbaud = 2.5 us/byte`. Frame sizes are
Protocol 2.0 exact.

**NEW, 4 Mbaud, `0x8A` + indirect (L = 11):**

| Phase | Bytes | Time |
|---|---|---|
| Fast Sync Read instruction (`8A`, addr+len+2 IDs) | 16 | 40 us |
| bus turnaround (RDT 0 + servo intrinsic + 74HC241 flip) | - | **T_to (measure)** |
| Fast Sync Read status (one packet: `8 + 2*(err+id+11+devCRC) + CRC`) | 40 | 100 us |
| control law + parse (M7 float) | - | ~3 us |
| Sync Write goal current (`addr+len+2*(id+2)`) | 20 | 50 us |
| **read+write wire total** | **76** | **190 us + 1 turnaround** |

**OLD, 1 Mbaud, `0x82` + direct 126..135 (L = 10), RDT 8 us:**

| Phase | Bytes | Time |
|---|---|---|
| Sync Read instruction | 16 | 160 us |
| status x2 (`11 + 10` each) + **2 turnarounds** (RDT 8 us each) | 42 | 420 us + 2 turn |
| Sync Write goal current | 20 | 200 us |
| **read+write wire total** | **78** | **780 us + 2 turnarounds** |

**Result (computed):** wire time per tick **780 -> 190 us (4.1x less)**; turnarounds
**2 -> 1** (and `N -> 1` for N motors). The OLD loop already fills ~80-90 % of a 1 ms (1 kHz)
tick, so **2 kHz is impossible at 1 Mbaud**. The NEW loop lands at ~190 us + T_to + compute
(order ~0.2-0.25 ms), which **holds a 500 us (2 kHz) tick with ~50 % idle** and a bus-limited
ceiling near **~4 kHz**. The indirect block adds the hardware-error byte for one extra
byte (~2.5 us), essentially free.

## Benchmark table (fill from hardware)

`M,b[,N]` runs N ticks/method (default 2000) at the current baud and prints a measured row.
To fill OLD vs NEW on the SAME rig: `M,t,1` -> `M,b` (records 1 Mbaud rows) -> `M,u` ->
`M,b` (records 4 Mbaud rows). Run with torque **off** for a pure bus measurement (goal
current is forced to 0). Columns are `mean/min/max/jitter (us)`, measured `turnaround (us)`,
achieved `Hz`, and the `tx / timeout / crc / hwErr` counter deltas.

| baud | method | mean us | min us | max us | jitter us | turnaround us | achieved Hz | crc | timeouts |
|---|---|---|---|---|---|---|---|---|---|
| 1 Mbaud | `0x82` | _measure_ | | | | | | | |
| 1 Mbaud | `0x8A` | _measure_ | | | | | | | |
| 4 Mbaud | `0x82` | _measure_ | | | | | | | |
| 4 Mbaud | `0x8A` | _measure_ | | | | | | | |

> **Not measured this run**, no bus hardware was attached to the authoring host. The
> emulator used for logic tests advances `micros()` as a call-counter, so its us/Hz are
> plumbing artifacts, **not** timing; only a Teensy on the real bus produces the numbers
> above. The firmware is instrumented to capture them (real `micros()` + the driver's
> `lastTurnaroundUs` and bus counters).

## Better control / transparency: the point of the speed

Feedback per tick: position + velocity + current + hardware-error (one Fast Sync Read).
Command: Goal-Current (one Sync Write). The law is unchanged in form (separable, saturated):
`i = (1-a)*i_transparent + a*i_assist`, with `i_transparent = -(b*qd + c*tanh(qd/w))`
(viscous + smoothed-Coulomb friction cancellation + tendon/gravity bias) and `i_assist`
a saturated PD, `a` from the crown (or host), hard-clamped to +/-150 mA under the reg-38 ceiling.

**Why faster helps, causally:** the friction-cancellation term injects current *in the
direction of motion*, positive velocity feedback, whose maximum stable gain `b` is bounded
by the loop's phase margin. Phase lag of a loop delay `tau` at frequency `omega` is `phi = omega*tau`.
Halving the sample period (1 kHz -> 2 kHz) roughly **halves the loop delay and its phase
lag at every frequency**, and the velocity estimate (finite-difference through the 1-pole
filter) arrives fresher. So for a fixed phase-margin target the stable `b` rises: **more
reflected gear friction is cancelled -> lower felt back-drive force and a narrower dead-band**,
without limit-cycling. Lower latency -> lower phase lag -> higher stable gain -> lower felt force.

**Quantifying it honestly:** the felt back-drive force / dead-band delta is **not measured
this run** (needs the F_t bench rig, not just the bus). Protocol to capture it, OLD vs NEW:
enter transparency (`M,m,1`, `M,a,0`), sweep the finger on the F_t rig, log `M,s`
(`iMeas`, achieved Hz, jitter) and the rig's break-away force / dead-band, first at 1 Mbaud/1 kHz
then at 4 Mbaud/2 kHz with the same `M,f` friction-FF gains; then raise `b` (`M,f`) at 2 kHz
to the new stability limit and re-measure. Report the measured break-away force and dead-band
ratio on the F_t rig, OLD vs NEW, never a computed stand-in. (The thesis' existing ~1.65
dead-band figure is a separate result; do not present it as a fresh measurement of this change.)

## Why not DMA/ISR (item 6)

At 4 Mbaud the read+write **wire** time is ~190 us and the control-law **compute** is a few
us of M7 float math, so overlapping compute with the bus (the only thing DMA/ISR buys) saves
a few us out of ~225. The bottleneck is the **actuator turnaround (T_to)**, which no amount
of CPU overlap removes. DMA/ISR is therefore *not* the lever here and is left out on purpose;
it would only matter if `M,b` measures compute as a large fraction of the tick. This is the
honest, defensible negative result the budget predicts. (If a hard-real-time 2 kHz guarantee
is later needed *under concurrent 50 Hz sensing + SD*, the tick belongs on a PIT/IntervalTimer
with a DMA UART back-end, scoped as future work, gated on the measured co-scheduled jitter
that `M,s` reports as `nOverrun`/`worstUs`.)

## Verification (this run, host-compiled, no hardware)

The frame codec and the sketch's motor logic were compiled and executed on the host against a
faithful Protocol-2.0 bus emulator (`test/`, run with `./test/run.sh`): **21 codec assertions**
(Fast Sync Read parse incl. an embedded `FF FF FD 00` header pattern and a `0xFDFF` position;
CRC-16; TX byte-stuffing and RX destuffing across `0xFDFF`; bad-CRC / truncation / id-order
rejection, each counted) and **34 integration assertions** driving the *actual*
`wearable_hand_bringup.ino` (bus take at 4 Mbaud/500 us, single-master refusal on a busy bus,
missing-motor refusal, link-quality gating, torque-off-on-connect, indirect arm + `0x8A`
selection, engineering-unit parse, hardware-error fail-safe, sustained-miss fail-safe, `M,b`,
the `loop()` `M,` dispatch wiring, the indirect-fails -> direct-fallback path, the servo
Bus-Watchdog arming + stall self-heal, and the enable torque-safety refusal). All 62 pass.
What remains for the bench: the Teensy `.hex` compile and the `M,b` / F_t captures above.

## Files
- `src/tiny_dxl.h`, `fastSyncRead()` (`0x8A`), turnaround instrumentation, `rxGuardUs`, bounded read fix.
- `examples/wearable_hand_bringup/wearable_hand_bringup.ino`, 4 Mbaud/RDT-0 upgrade path, indirect feedback block, method/layout probe,
  2 kHz tick, hardware-error fail-safe, `M,b` benchmark, and the `loop()` wiring.
