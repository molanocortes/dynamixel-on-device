// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2026 Juan Sebastian Molano. Dual-licensed; see COMMERCIAL.md.
/*
 * wearable_hand_bringup - Teensy 4.1 wiring check + calibration + SD logger
 * ========================================================================
 * Reference application for the tiny_dxl fast Dynamixel driver: a full wearable
 * articulated-hand rig (4 finger chains, 2-3 orientation sensors, a round
 * display, analog/digital inputs, and two XC330 servos on a 74HC241 half-duplex
 * bus). The servo section (search "servo bus") is the part that exercises
 * tiny_dxl at 2 kHz; the rest is the sensing/logging harness around it.
 *
 * Purpose (day-of-soldering tool):
 *   1) SCAN   - after you solder, confirm every encoder + both IMUs answer.
 *   2) CALIB  - flex each finger through its full range; capture per-channel
 *               min/max (range of motion). Needs NO motors.
 *   3) RECORD - stream all channels + both IMUs at 50 Hz to the Teensy SD card
 *               as a labelled CSV (open-ended: starts on command, stops on
 *               command) - a self-capture over a flexion sweep.
 *
 * ARCHITECTURE (verify connector order against your own schematic):
 *   Encoders : up to 14 AS5600 (addr 0x36) behind 2x TCA9548A muxes
 *              MUX_A @ 0x70 channels 0..7  -> encoder channels 0..7
 *              MUX_B @ 0x71 channels 0..5  -> encoder channels 8..13
 *              Mux main I2C bus  -> Teensy Wire  (SDA 18 / SCL 19), 400 kHz
 *   IMUs     : 3x BNO085. Hand 0x4A on Wire1 (17/16); forearm 0x4B on Wire2
 *              (25/24); THUMB TIP 0x4B on Wire1, routed through the palm PCB
 *              (the BNO085 has only two addresses, so the thumb shares the
 *              hand's bus at the other address - the reentrant driver was
 *              built for exactly this; adjust THUMB_WIRE/ADDR if the bench
 *              routing differs). tiny_bno085 keeps all state per object.
 *   SD       : Teensy 4.1 built-in socket (BUILTIN_SDCARD)
 *
 * SERIAL MENU (115200 baud, send a single letter):
 *   s = re-scan the buses and print the wiring report
 *   c = run a 12 s range-of-motion calibration (flex everything, slowly)
 *   r = start / stop an SD recording (toggles - bench-human convenience)
 *   b = START recording  (explicit, idempotent - what the host bridge sends)
 *   e = STOP  recording  (explicit, idempotent - what the host bridge sends)
 *   v = print the version banner "# ver bringup_12ch <n>" (host handshake)
 *   j = live stream ON  (emit "S,t,enc0..13,hq,fq,imu0,imu1" lines for the host bridge)
 *   k = live stream OFF
 *   ? = help
 *   D,...\n = device-screen state pushed by the host for the SCREEN firmware;
 *             this sketch has no display, so the whole line is swallowed
 *             (never letting payload bytes alias menu letters)
 *   M,...\n = motor commands (Dynamixel XC330 bus via the 74HC241 on Serial1;
 *             the whole line is buffered + dispatched as a unit, so its bytes
 *             never alias the single-letter menu commands above):
 *     M,t,<0|1>        release / TAKE the servo bus. TAKE first LISTENS and refuses
 *                      if another master is driving DATA (single-master: disconnect
 *                      the U2D2); then auto-baud scans (4 Mbps first), requires BOTH
 *                      motors present, qualifies the link (repeated round-trips),
 *                      checks the model, and connects torque OFF (compliant)
 *     M,e,<0|1>        torque off / ON (on-path configures: current mode, RDT 0 us,
 *                      current limit, and arms the one-shot Indirect feedback block)
 *     M,m,<0|1|2>      mode: 0 idle, 1 RUN (blended transparency<->assist), 2 direct current
 *     M,a,<-1|0..1000> assist blend override; -1 = follow the physical crown (default)
 *     M,c,<id>,<mA>    direct current setpoint (mode 2), clamped to +/-150 mA
 *     M,p,<id>,<deg>   assist position setpoint (mode 1), servo horn degrees
 *     M,k,<kp>,<kd>    assist gains [mA/rad, mA/(rad s^-1)]
 *     M,f,<visc>,<coul> transparency friction feed-forward [mA/(rad s^-1), mA]
 *     M,s              print servo-loop + bus statistics (human)
 *     M,u              one-time servo EEPROM upgrade: baud -> 4 Mbps (XC330 max),
 *                      RDT -> 0 us; re-pings to verify, rescans if not
 *     M,b[,N]          bus benchmark: N ticks (default 2000) with Sync Read 0x82
 *                      then Fast Sync Read 0x8A at the current baud; prints
 *                      measured per-tick us / achieved Hz / turnaround / errors
 *   Feedback per tick: ONE Fast Sync Read 0x8A (position + current + velocity +
 *   hardware-error, one turnaround) + ONE Sync Write (goal current); inner loop
 *   up to 2 kHz at 4 Mbaud. Watchdog: 600 ms without any M line demotes
 *   direct-current to RUN and returns blend authority to the crown - the device
 *   degrades to the compliant (transparent-leaning) state, never to a locked one.
 *   Safety: any Hardware Error Status bit or MOTOR_ERR_TRIP consecutive missed
 *   reads -> torque off (counted, recoverable with M,e,1).
 *
 * LIVE STREAM (for the web console bridge): while streaming is ON the sketch
 * emits, at SAMPLE_HZ, one line per sample:
 *   S,<t_ms>,<enc00>..<enc13>,<h_qw,h_qx,h_qy,h_qz>,<f_qw,f_qx,f_qy,f_qz>,<imu0live>,<imu1live>,<emg_env>,<emg_rms>,<emg_present>,<crown_0..1000>,<t_qw,t_qx,t_qy,t_qz>,<imu2live>,<mflags>,<m0_pos>,<m0_vel>,<m0_ma>,<m1_pos>,<m1_vel>,<m1_ma>
 * (fields are append-only across firmware versions: v3 ended at crown, v4
 *  appends the thumb-tip quaternion + live flag, v5 appends servo telemetry:
 *  mflags = taken | torque<<1 | mode<<2 | fault<<4, pos deg / vel dps / mA;
 *  hosts parse by index)
 * Encoders that did not answer at the last scan stream -1. Re-scan ('s') after
 * you wire a new channel so it starts streaming. Scan/calib/record still work.
 *
 * Libraries: Wire + SD are built in; install "Adafruit BNO08x" via Library Mgr.
 * No warranty: compile in Arduino IDE and bench-test before trusting a joint.
 */

#include <Wire.h>
#include <SD.h>
#include <tiny_bno085.h>   // reentrant per-instance driver (Adafruit lib is single-instance)
#include <tiny_dxl.h>      // minimal Protocol 2.0 master (74HC241 half-duplex on Serial1)

// ---- configuration ---------------------------------------------------------
#define MUX_BUS      Wire          // encoders' mux main bus (pins 18/19)
// Each BNO085 gets its OWN I2C bus (kills the two-device clock-stretch contention).
// Verified by raw bus scan: 0x4A is on Wire1 (17/16), 0x4B is on Wire2 (25/24).
// IMU_ADDR = {0x4A, 0x4B}, so index 0 -> Wire1, index 1 -> Wire2.
TwoWire* const IMU_WIRE[3] = { &Wire1, &Wire2, &Wire1 };   // hand, forearm, thumb
const uint8_t  MUX_ADDR[2] = {0x70, 0x71};
const uint8_t  CH_PER_MUX  = 8;        // TCA9548A has 8 channels
const uint8_t  N_CHANNELS  = 14;       // encoder channels in use (0..13)
const uint8_t  AS5600_ADDR = 0x36;
const uint8_t  REG_STATUS  = 0x0B;
const uint8_t  REG_ANGLE_HI= 0x0E;
const uint8_t  IMU_ADDR[3] = {0x4A, 0x4B, 0x4B};   // thumb = 0x4B on Wire1 (palm PCB)
const char*    IMU_NAME[3] = {"hand", "forearm", "thumb"};
const float    SAMPLE_HZ   = 50.0f;
const uint8_t  EMG_PIN     = 14;       // MyoWare ENVELOPE output on A0; oversampled -> env + rms
const uint8_t  FW_VERSION  = 5;        // bumped when the serial contract grows ('v' reports it)
const uint8_t  N_IMU       = 3;        // hand, forearm, thumb tip
const uint32_t IMU_STALE_MS = 500;     // no rotation report this long = the sensor silently died
// Crown potentiometer (pin 27 = A13, same wiring + calibrated active band as
// the v9 dashboard: the pot's usable travel is 300..720 of the 10-bit range).
// Continuous transparency control, Apple-crown style: low = fully transparent
// (device renders zero force), high = fully assisted. Streamed as 0..1000.
const uint8_t  POT_PIN         = 27;
const int      POT_ACTIVE_LOW  = 300;
const int      POT_ACTIVE_HIGH = 720;

// ---- state -----------------------------------------------------------------
bool     chLive[N_CHANNELS];           // did this encoder answer at scan?
float    chMin[N_CHANNELS], chMax[N_CHANNELS];
float    frameDeg[N_CHANNELS];         // ONE acquisition per frame, shared by SD row + S-line
TinyBNO085 bno[3] = { TinyBNO085(&Wire1, 0x4A),    // hand    - Wire1 (17/16)
                      TinyBNO085(&Wire2, 0x4B),    // forearm - own bus (25/24)
                      TinyBNO085(&Wire1, 0x4B) };  // thumb tip - shares Wire1 (palm PCB)
bool     imuLive[3] = {false, false, false};
float    imuQ[3][4];                   // w,x,y,z per IMU
uint32_t imuFreshMs[3] = {0, 0, 0};    // last rotation report (staleness watchdog)
uint32_t imuSvcMs = 0;                 // last imuService pass (self-stall forgiveness)
bool     sdOK = false;
File     recFile;
bool     recording = false;
uint32_t recStart = 0, recRows = 0;
uint16_t recSeq = 0;                   // next REC file number (collision-free names)
uint32_t lastSample = 0;
bool     streaming = false;            // live serial stream for the web-console bridge
bool     discardLine = false;          // swallowing a "D,...\n" device-screen line
bool     inMotorLine = false;          // buffering an "M,...\n" motor-command line
char     mBuf[48];                     // the motor-line buffer (dispatched whole on '\n')
uint8_t  mLen = 0;
// EMG (MyoWare envelope on pin 14): oversampled ~1 kHz between 50 Hz frames, reduced
// to mean (envelope) + RMS and appended to the S-line. The host runs the Fable
// activation module (effort_control BayesianAmplitude + normalization) on this.
uint32_t emgSum = 0, emgSumSq = 0;
uint16_t emgCount = 0;
uint32_t emgLastU = 0;
float    emgEnv = 0.0f, emgRms = 0.0f;
bool     emgHave = false;
float    crownFilt = -1.0f;            // EMA of the crown pot, 0..1000 (-1 = no sample yet)

// ---- mux + encoder helpers (raw I2C, matches as5600_dual_reader) -----------
void muxSelect(uint8_t muxAddr, uint8_t ch) {
  MUX_BUS.beginTransmission(muxAddr);
  MUX_BUS.write(1 << ch);
  MUX_BUS.endTransmission();
}
void muxDisable(uint8_t muxAddr) {       // deselect all channels
  MUX_BUS.beginTransmission(muxAddr);
  MUX_BUS.write((uint8_t)0);
  MUX_BUS.endTransmission();
}
// map encoder channel 0..13 -> (mux, channel)
void chToMux(uint8_t ch, uint8_t &muxAddr, uint8_t &muxCh) {
  muxAddr = MUX_ADDR[ch / CH_PER_MUX];
  muxCh   = ch % CH_PER_MUX;
}
uint16_t readAngleRaw() {                // on the currently-selected channel
  MUX_BUS.beginTransmission(AS5600_ADDR);
  MUX_BUS.write(REG_ANGLE_HI);
  if (MUX_BUS.endTransmission(false) != 0) return 0xFFFF;
  if (MUX_BUS.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return 0xFFFF;
  uint8_t hi = MUX_BUS.read(), lo = MUX_BUS.read();
  return ((uint16_t)hi << 8) | lo;
}
bool encoderPresent() {                  // on the currently-selected channel
  MUX_BUS.beginTransmission(AS5600_ADDR);
  MUX_BUS.write(REG_STATUS);
  return (MUX_BUS.endTransmission(false) == 0 &&
          MUX_BUS.requestFrom(AS5600_ADDR, (uint8_t)1) == 1);
}
// Read every live channel in one batched sweep. Both muxes answer the same
// slave address (0x36), so the invariant is: never two muxes active at once.
// The old per-channel pattern (select + read + disable = 3 transactions each)
// honored that with 42 transactions per frame; this honors it with one disable
// per MUX SWITCH instead of per channel (selecting a channel on the same mux
// atomically replaces its channel mask), cutting frame I2C time ~30 %.
// Dead channels (chLive false) are skipped without touching the bus, as before.
void readAllChannels(float out[N_CHANNELS]) {
  for (uint8_t m = 0; m < 2; m++) {
    muxDisable(MUX_ADDR[1 - m]);                    // the other mux stays silent
    const uint8_t chLo = m * CH_PER_MUX;
    uint8_t chHi = chLo + CH_PER_MUX;
    if (chHi > N_CHANNELS) chHi = N_CHANNELS;
    for (uint8_t ch = chLo; ch < chHi; ch++) {
      if (!chLive[ch]) { out[ch] = -1.0f; continue; }
      muxSelect(MUX_ADDR[m], ch % CH_PER_MUX);
      uint16_t raw = readAngleRaw();
      out[ch] = (raw == 0xFFFF) ? -1.0f : (raw & 0x0FFF) * 360.0f / 4096.0f;
    }
    muxDisable(MUX_ADDR[m]);
  }
}

// ---- servo bus: 74HC241 half-duplex on Serial1, XC330 in current mode --------
// Design: the CONTROL LAW RUNS HERE, next to the actuator, at up to 2 kHz.
// The host only sends setpoints and mode requests; if it dies, the watchdog
// demotes toward the compliant transparent state (never a locked one).
// Per tick: ONE Fast Sync Read 0x8A (position + current + velocity + hardware-
// error concatenated in a single status packet with ONE bus turnaround, via an
// Indirect Data block) + ONE Sync Write (goal current). At 4 Mbaud with RDT = 0
// the read+write is ~0.2 ms, so a 500 us tick (2 kHz) holds with margin; at
// slower buses the tick period stretches honestly (see motorTake). Lower loop
// latency -> lower phase lag -> the transparency term stays stable at higher
// gain -> lower felt back-drive force. Measure it all with M,b.
const uint8_t  DXL_DIR_PIN   = 7;      // 74HC241 1OE+2OE (HIGH = transmit)
const uint8_t  DXL_IDS[2]    = {1, 2};
const uint8_t  N_MOTOR       = 2;
const float    I_CAP_MA      = 150.0f; // ecosystem hard ceiling (matches the host clamp)
const uint16_t I_LIMIT_REG   = 350;    // servo-side Current Limit (reg 38): fault backstop
const float    CNT2DEG       = 0.087891f;              // 4096 counts / rev
const float    CNT2RAD       = 0.087891f * PI / 180.0f;
const float    VELU2DPS      = 0.229f * 6.0f;          // 0.229 rpm/unit -> deg/s
const float    QD_LP_ALPHA   = 0.222f; // 1-pole derivative filter, fc ~= 40 Hz @ 1 kHz
const uint32_t MOTOR_WD_MS   = 600;    // host-silence watchdog (demote, don't lock)
const uint8_t  MOTOR_ERR_TRIP= 25;     // consecutive bus misses before torque-off

// --- fast-bus operating point (verified against the XC330-M181 control table) --
// 4 Mbps is the XC330-M181 MAXIMUM (baud reg 8 value 6); reg value 7 / 4.5 Mbps
// belongs to the larger XM/XH series, not this actuator, so 4 Mbaud is the
// fastest HONEST operating point here. RDT 0 shaves the servo's return delay off
// every status frame. Both are one-time EEPROM writes via 'M,u'.
const uint32_t DXL_TARGET_BAUD = 4000000; // XC330-M181 ceiling
const uint8_t  DXL_BAUD_CODE   = 6;       // reg 8 = 6 -> 4 Mbps
const uint8_t  DXL_RDT_UNITS   = 0;       // reg 9, unit 2 us -> 0 us return delay
// Feedback registers folded into ONE contiguous Indirect Data block so a single
// Fast Sync Read grabs position + current + velocity + the (non-contiguous)
// hardware-error byte in one turnaround. Indirect Address 1 = 168 (stride 2),
// Indirect Data 1 = 224 (stride 1); both RAM on the XC330, so re-armed each
// enable (no EEPROM wear). Order = position first (control-critical), then
// current, velocity, hardware-error.
const uint16_t IND_ADDR_BASE = 168, IND_DATA_BASE = 224;
const uint16_t A_PRES_CUR = 126, A_HW_ERR = 70;
const uint8_t  IND_SRC[11] = {132,133,134,135, 126,127, 128,129,130,131, 70};
const uint8_t  IND_LEN      = 11;         // pos(4)+cur(2)+vel(4)+hwErr(1)
const uint8_t  HW_FAULT_MASK = 0x3F;      // any Hardware Error Status bit -> fail safe

// --- detection / connection robustness (correct + safe + reliable) -----------
const uint8_t  DETECT_PING_TRIES = 3;     // per-id ping attempts at a candidate baud
const uint8_t  LINK_QUAL_ROUNDS  = 16;    // round-trip probes used to qualify a baud
const uint8_t  LINK_QUAL_MIN     = 15;    // require >= this many (tolerate 1 warm-up miss)
const uint32_t BUS_QUIET_US      = 120000;// single-master listen window before we EVER transmit
const uint16_t XC330_M181_MODEL  = 1230;  // Model Number (reg 0) a PING must return
// Servo-side dead-man (Bus Watchdog, reg 98, unit 20 ms). If the servo sees no
// packet within this window it zeroes goal current ON ITS OWN - the one thing
// that protects the wearer if the TEENSY hangs (the host- and bus-watchdogs
// cannot run then). Every Sync Write pets it; it re-arms after a legitimate stall
// so transient blocks self-heal, but a true hang stays latched-safe (it latches
// by design and needs an explicit clear).
const uint8_t  BUS_WATCHDOG_UNITS = 5;    // 5 * 20 ms = 100 ms
const uint32_t BUS_WATCHDOG_US    = 100000;
const uint16_t A_BUS_WATCHDOG      = 98;

TinyDXL dxl(&Serial1, DXL_DIR_PIN);
struct {
  bool     taken = false, torque = false, fault = false;
  uint8_t  mode = 0;                   // 0 idle, 1 RUN (blend), 2 direct current
  float    kp = 400.0f, kd = 8.0f;     // assist PD [mA/rad, mA/(rad/s)]
  float    frV = 3.0f, frC = 20.0f;    // transparency friction ff [mA/(rad/s), mA]
  float    frW = 0.5f;                 // Coulomb tanh width [rad/s]
  float    aOverride = -1.0f;          // blend 0..1; <0 = follow the crown pot
  float    qSet[2] = {0, 0};           // assist setpoints [rad]
  float    iSet[2] = {0, 0};           // direct current setpoints [mA]
  float    q[2] = {0, 0}, qdF[2] = {0, 0}, qPrev[2] = {0, 0};
  float    posDeg[2] = {0, 0}, velDps[2] = {0, 0}, iMeas[2] = {0, 0};
  bool     havePrev = false;
  uint8_t  errRun = 0;
  uint32_t tickUs = 1000;              // period, set from the bus baud
  uint32_t nextTick = 0, lastCmdMs = 0;
  uint32_t nTicks = 0, nMiss = 0, nOverrun = 0, worstUs = 0, sumUs = 0;
  // fast-bus feedback layout (chosen at enable; see motorEnable)
  bool     useFast = true;             // 0x8A Fast Sync Read available (else 0x82)
  bool     useIndirect = true;         // one-shot indirect block armed (else direct 126..135)
  uint16_t readAddr = A_PRES_CUR;      // 224 (indirect) or 126 (direct fallback)
  uint8_t  readLen  = 10;              // 11 (indirect) or 10 (direct)
  int8_t   offPos = 6, offCur = 0, offVel = 2, offErr = -1; // byte offsets in a device block
  uint8_t  hwErr[2] = {0, 0};          // last Hardware Error Status per motor
  uint32_t lastHwPollMs = 0;           // low-rate hw-error poll (only in the direct fallback)
  bool     wdRearm = false;            // re-arm the servo Bus Watchdog after a stall (self-heal)
} mc;

uint8_t motorFlags() {
  return (mc.taken ? 1 : 0) | (mc.torque ? 2 : 0) | ((mc.mode & 3) << 2) |
         (mc.fault ? 16 : 0);
}

void motorTorqueOff() {
  if (mc.taken)
    for (uint8_t i = 0; i < N_MOTOR; i++) dxl.writeU8(DXL_IDS[i], 64, 0);
  mc.torque = false;
  mc.mode = 0;
}

// Tick period tracks the wire budget so every baud stays correct, just slower:
// 4/3 Mbaud -> 2 kHz, >=1 Mbaud -> 1 kHz, 115200 -> 250 Hz, 57600 -> 100 Hz.
uint32_t tickUsForBaud(uint32_t baud) {
  return (baud >= 3000000) ? 500 : (baud >= 1000000 ? 1000 : (baud >= 115200 ? 4000 : 10000));
}
bool motorPing(uint8_t id, uint16_t* model) {   // bounded retries for the first, warm-up transaction
  for (uint8_t t = 0; t < DETECT_PING_TRIES; t++) if (dxl.ping(id, model)) return true;
  return false;
}

// Take the bus = DETECT the motors correctly, then CONNECT safely + reliably.
// Order matters:
//   1) single-master check: LISTEN first; refuse to transmit if the U2D2 (or any
//      other master) is already driving DATA. We never collide on the bus.
//   2) scan bauds fastest-first; at each, ping EVERY expected id (with retries).
//      Require ALL present - a half-populated bus is a fault, not a connection.
//   3) qualify the link: many round-trips must succeed before we trust the baud,
//      so a marginal 4 Mbaud (signal integrity) falls back to a solid slower one.
//   4) verify identity (model number) and force torque OFF (compliant, safe) on
//      connect. Only then is the bus "taken".
void motorTake() {
  static const uint32_t CAND[6] = {4000000, 1000000, 2000000, 3000000, 115200, 57600};
  // (1) single-master safety: is anyone else driving DATA right now?
  dxl.begin(CAND[0]);
  if (!dxl.busQuiet(BUS_QUIET_US)) {
    dxl.end();
    mc.taken = false;
    Serial.println(F("# motor: DATA is BUSY - another master is driving the bus. Disconnect the "
                     "U2D2 before the Teensy takes the bus (single-master rule). Not taken."));
    return;
  }
  bool sawPartial = false;
  for (uint8_t b = 0; b < 6; b++) {
    dxl.begin(CAND[b]);
    delayMicroseconds(300);
    // (2) require EVERY expected motor at this baud
    uint16_t model[N_MOTOR]; bool present[N_MOTOR]; uint8_t nPresent = 0;
    for (uint8_t i = 0; i < N_MOTOR; i++) {
      model[i] = 0;
      present[i] = motorPing(DXL_IDS[i], &model[i]);
      if (present[i]) nPresent++;
    }
    if (nPresent == 0) continue;                       // nothing here; try the next baud
    if (nPresent < N_MOTOR) {                          // partial bus: report, keep looking
      sawPartial = true;
      Serial.printf("# motor @ %lu baud: only %u/%u present (", (unsigned long)CAND[b], nPresent, N_MOTOR);
      for (uint8_t i = 0; i < N_MOTOR; i++)
        Serial.printf("id%u %s%s", DXL_IDS[i], present[i] ? "OK" : "MISSING", i + 1 < N_MOTOR ? ", " : "");
      Serial.println(F(")"));
      continue;
    }
    // (3) qualify the link: repeated full round-trips must (almost) all succeed
    uint8_t good = 0;
    for (uint8_t r = 0; r < LINK_QUAL_ROUNDS; r++) {
      bool all = true;
      for (uint8_t i = 0; i < N_MOTOR; i++) if (!dxl.ping(DXL_IDS[i])) { all = false; break; }
      if (all) good++;
    }
    if (good < LINK_QUAL_MIN) {
      Serial.printf("# motor @ %lu baud: link UNRELIABLE (%u/%u round-trips) - trying a slower baud\n",
                    (unsigned long)CAND[b], good, LINK_QUAL_ROUNDS);
      continue;
    }
    // (4) identity check (warn, do not brick) + SAFE connect (torque OFF)
    for (uint8_t i = 0; i < N_MOTOR; i++)
      if (model[i] != XC330_M181_MODEL)
        Serial.printf("# motor WARN: id%u model %u != expected XC330-M181-T (%u)\n",
                      DXL_IDS[i], model[i], XC330_M181_MODEL);
    for (uint8_t i = 0; i < N_MOTOR; i++) dxl.writeU8(DXL_IDS[i], 64, 0);  // compliant on connect
    mc.taken = true; mc.torque = false; mc.mode = 0; mc.fault = false; mc.errRun = 0;
    mc.tickUs = tickUsForBaud(CAND[b]);
    mc.nextTick = micros();
    Serial.printf("# motor CONNECTED @ %lu baud: id%u model %u, id%u model %u; link %u/%u; "
                  "tick %lu us (%lu Hz); torque OFF (compliant). Enable with M,e,1\n",
                  (unsigned long)CAND[b], DXL_IDS[0], model[0], DXL_IDS[1], model[1],
                  good, LINK_QUAL_ROUNDS, (unsigned long)mc.tickUs,
                  (unsigned long)(1000000UL / mc.tickUs));
    if (CAND[b] != DXL_TARGET_BAUD)
      Serial.printf("# motor hint: run M,u to move the servos to %lu baud (4 Mbps = XC330 max)\n",
                    (unsigned long)DXL_TARGET_BAUD);
    return;
  }
  dxl.end();
  mc.taken = false;
  if (sawPartial)
    Serial.println(F("# motor: bus reachable but NOT fully populated - a servo is missing/unpowered. "
                     "Check the daisy-chain, the 5 V motor supply, and both IDs. Not taken."));
  else
    Serial.println(F("# motor: no servo answered on any baud (check 74HC241, wiring, 5 V motor supply). Not taken."));
}

void motorRelease() {
  motorTorqueOff();
  mc.taken = false;
  dxl.end();
  Serial.println(F("# motor bus released"));
}

// Map the feedback registers into Indirect Data 1..11 so ONE Fast Sync Read
// grabs position + current + velocity + the non-contiguous hardware-error byte.
// Each Indirect Address k (2 B, at 168 + 2k) holds the source register address
// mirrored into Indirect Data k (1 B, at 224 + k). Written with torque off, then
// READ BACK and verified: if the map does not take (odd firmware / model), the
// caller falls back to the proven direct 126..135 read. RAM on the XC330, so
// this is a per-enable arm, not an EEPROM-wear item.
bool motorConfigIndirect(uint8_t id) {
  for (uint8_t k = 0; k < IND_LEN; k++)
    if (!dxl.writeU16(id, IND_ADDR_BASE + 2 * k, IND_SRC[k])) return false;
  for (uint8_t k = 0; k < IND_LEN; k++) {   // read-back verify (no blind trust)
    uint8_t rb[2];
    if (!dxl.read(id, IND_ADDR_BASE + 2 * k, 2, rb)) return false;
    if ((uint16_t)(rb[0] | (rb[1] << 8)) != IND_SRC[k]) return false;
  }
  return true;
}

// Arm (or re-arm) the servo-side Bus Watchdog: clear any latched error (write 0),
// then set the timeout. RAM register, valid with torque on or off.
void motorArmWatchdog(uint8_t id) {
  dxl.writeU8(id, A_BUS_WATCHDOG, 0);                 // clear a latched Bus Watchdog Error
  dxl.writeU8(id, A_BUS_WATCHDOG, BUS_WATCHDOG_UNITS);// arm: zero goal if no packet in 100 ms
}

// Configure-and-enable. CONFIG is done first with torque OFF and its success is
// tracked; the motors are energized (torque ON + Bus Watchdog armed) ONLY if the
// whole config took. On any lost config write we de-energize and report failure -
// we never leave a servo driven (possibly in a stale mode) while reporting OFF.
// Layered graceful degradation of feedback:
//   indirect block (adds hardware-error) -> direct 126..135 (no hw-error, polled)
//   Fast Sync Read 0x8A                   -> Sync Read 0x82
void motorEnable() {
  if (!mc.taken) { Serial.println(F("# motor: take the bus first (M,t,1)")); return; }
  bool ok = true, indOK = true;
  for (uint8_t i = 0; i < N_MOTOR; i++) {             // CONFIG (torque stays OFF)
    uint8_t id = DXL_IDS[i];
    ok &= dxl.writeU8(id, 64, 0);          // torque off for the EEPROM writes
    ok &= dxl.writeU8(id, 11, 0);          // operating mode 0 = current control
    ok &= dxl.writeU8(id, 9, DXL_RDT_UNITS); // return delay 0 us (was 8; free latency)
    ok &= dxl.writeU16(id, 38, I_LIMIT_REG); // hardware current ceiling (fault backstop)
    indOK &= motorConfigIndirect(id);      // arm the one-shot feedback block (RAM)
  }
  // pick the feedback layout from what actually armed
  mc.useIndirect = indOK;
  if (indOK) { mc.readAddr = IND_DATA_BASE; mc.readLen = IND_LEN;
               mc.offPos = 0; mc.offCur = 4; mc.offVel = 6; mc.offErr = 10; }
  else       { mc.readAddr = A_PRES_CUR;    mc.readLen = 10;
               mc.offPos = 6; mc.offCur = 0; mc.offVel = 2; mc.offErr = -1; }
  // probe Fast Sync Read (0x8A); fall back to Sync Read (0x82) if it does not verify
  uint8_t probe[N_MOTOR * 16];
  mc.useFast = dxl.fastSyncRead(mc.readAddr, mc.readLen, DXL_IDS, N_MOTOR, probe);
  if (!mc.useFast) dxl.syncRead(mc.readAddr, mc.readLen, DXL_IDS, N_MOTOR, probe);
  if (!ok) {                                          // config incomplete -> stay SAFE
    for (uint8_t i = 0; i < N_MOTOR; i++) dxl.writeU8(DXL_IDS[i], 64, 0);  // de-energize
    mc.torque = false; mc.fault = true; mc.mode = 0;
    Serial.println(F("# motor ENABLE FAILED (a config write was lost) - torque OFF, staying safe"));
    return;
  }
  for (uint8_t i = 0; i < N_MOTOR; i++) motorArmWatchdog(DXL_IDS[i]);       // servo dead-man
  bool ton = true;
  for (uint8_t i = 0; i < N_MOTOR; i++) ton &= dxl.writeU8(DXL_IDS[i], 64, 1);  // ENERGIZE
  if (!ton) {                                         // torque-on not confirmed -> stay SAFE
    for (uint8_t i = 0; i < N_MOTOR; i++) dxl.writeU8(DXL_IDS[i], 64, 0);
    mc.torque = false; mc.fault = true; mc.mode = 0;
    Serial.println(F("# motor ENABLE FAILED (torque-on not acked) - torque OFF, staying safe"));
    return;
  }
  mc.torque = true; mc.fault = false;
  if (mc.mode == 0) mc.mode = 1;           // enabling means: run the blended law
  mc.havePrev = false;                     // clean derivative restart
  mc.hwErr[0] = mc.hwErr[1] = 0;
  mc.wdRearm = false;
  mc.nextTick = micros();                  // tick immediately so the watchdog gets petted
  Serial.printf("# motor torque ON (current mode, RDT 0us, bus-watchdog %u ms) | feedback %s via %s (%u B/motor)\n",
                (unsigned)(BUS_WATCHDOG_UNITS * 20),
                mc.useIndirect ? "pos+cur+vel+hwErr" : "pos+cur+vel (hwErr polled)",
                mc.useFast ? "FAST sync read 0x8A" : "sync read 0x82", mc.readLen);
}

// One-time EEPROM upgrade to the fast bus. Explicit command, never automatic:
// rewrites servo EEPROM (baud reg 8 = 6 -> 4 Mbps, the XC330-M181 ceiling; RDT 0)
// and re-opens the bus. Each servo answers its baud write at the OLD baud, then
// switches; we re-open at the target and PING to verify. If a servo does not
// come up (marginal signal integrity at 4 Mbps over this cable), we do not fake
// success: we rescan and report the real baud it actually answered on.
void motorUpgrade() {
  if (!mc.taken) { Serial.println(F("# motor: take the bus first (M,t,1)")); return; }
  for (uint8_t i = 0; i < N_MOTOR; i++) {
    dxl.writeU8(DXL_IDS[i], 64, 0);              // torque off (EEPROM writes)
    dxl.writeU8(DXL_IDS[i], 9, DXL_RDT_UNITS);   // return delay 0 us
    dxl.writeU8(DXL_IDS[i], 8, DXL_BAUD_CODE);   // 6 = 4 Mbps (status returns at the old baud)
  }
  dxl.end();
  dxl.begin(DXL_TARGET_BAUD);
  delay(2);
  bool ok = dxl.ping(DXL_IDS[0]) && dxl.ping(DXL_IDS[1]);
  if (ok) {
    mc.tickUs = tickUsForBaud(DXL_TARGET_BAUD);  // 2 kHz: the 4 Mbps budget supports it
    mc.nextTick = micros();
    Serial.printf("# motor upgrade to %lu baud: OK (tick 500 us / 2 kHz)\n",
                  (unsigned long)DXL_TARGET_BAUD);
  } else {
    Serial.println(F("# motor upgrade: no ping at 4 Mbps - rescanning (staying honest about the real baud)"));
    motorTake();                                 // rediscover + set tickUs from the actual baud
  }
}

// The inner current-control tick (up to 2 kHz at 4 Mbaud): ONE Fast Sync Read
// (position + current + velocity + hardware-error in one turnaround) + ONE Sync
// Write (goal current). Lower loop latency = lower phase lag = the friction/
// impedance terms stay stable at higher gain = lower felt back-drive force.
// Blended control: i = (1-a)*i_transparent + a*i_assist, a = crown (or host
// override). a=0 renders zero force plus friction cancellation (best
// transparency); a=1 is a saturated PD toward the assist setpoint. One law,
// no mode-switching transients: the crown IS the controller interpolation.
void motorTick() {
  uint32_t t0 = micros();
  uint8_t rx[N_MOTOR * 16], rerr[N_MOTOR];
  bool ok = mc.useFast
    ? dxl.fastSyncRead(mc.readAddr, mc.readLen, DXL_IDS, N_MOTOR, rx, rerr)
    : dxl.syncRead(mc.readAddr, mc.readLen, DXL_IDS, N_MOTOR, rx);
  if (!ok) {
    mc.nMiss++;
    if (++mc.errRun >= MOTOR_ERR_TRIP) {   // sustained bus loss: fail safe, stop driving
      motorTorqueOff();
      mc.fault = true;
    }
    return;
  }
  mc.errRun = 0;
  float dt = mc.tickUs * 1e-6f;
  float alpha = mc.aOverride;
  if (alpha < 0.0f) alpha = (crownFilt < 0.0f) ? 0.0f : crownFilt * 0.001f;
  uint8_t goal[N_MOTOR * 2];
  bool hwFault = false;
  for (uint8_t i = 0; i < N_MOTOR; i++) {
    const uint8_t* p = rx + i * mc.readLen;
    int16_t curU = (int16_t)(p[mc.offCur] | (p[mc.offCur + 1] << 8));
    int32_t velU = (int32_t)(p[mc.offVel] | (p[mc.offVel + 1] << 8) |
                             ((uint32_t)p[mc.offVel + 2] << 16) | ((uint32_t)p[mc.offVel + 3] << 24));
    int32_t posU = (int32_t)(p[mc.offPos] | (p[mc.offPos + 1] << 8) |
                             ((uint32_t)p[mc.offPos + 2] << 16) | ((uint32_t)p[mc.offPos + 3] << 24));
    if (mc.offErr >= 0) mc.hwErr[i] = p[mc.offErr];   // hardware-error rides the fast read
    if (mc.hwErr[i] & HW_FAULT_MASK) hwFault = true;
    mc.iMeas[i] = (float)curU;                        // 1 mA / unit (XC330)
    mc.velDps[i] = velU * VELU2DPS;
    mc.posDeg[i] = (posU - 2048) * CNT2DEG;
    mc.q[i] = (posU - 2048) * CNT2RAD;
    float qdRaw = mc.havePrev ? (mc.q[i] - mc.qPrev[i]) / dt : 0.0f;
    mc.qPrev[i] = mc.q[i];
    mc.qdF[i] += QD_LP_ALPHA * (qdRaw - mc.qdF[i]);   // 1-pole derivative filter (fc ~ rate-dep)
    float iCmd = 0.0f;
    if (mc.mode == 1) {
      // transparency: cancel identified friction (viscous + smoothed Coulomb)
      float iT = -(mc.frV * mc.qdF[i] + mc.frC * tanhf(mc.qdF[i] / mc.frW));
      // assist: saturated PD toward the host setpoint
      float iA = mc.kp * (mc.qSet[i] - mc.q[i]) - mc.kd * mc.qdF[i];
      iCmd = (1.0f - alpha) * iT + alpha * iA;
    } else if (mc.mode == 2) {
      iCmd = mc.iSet[i];
    }
    iCmd = constrain(iCmd, -I_CAP_MA, I_CAP_MA);      // software clamp below the reg-38 ceiling
    int16_t g = (int16_t)lrintf(iCmd);
    goal[i * 2] = (uint8_t)(g & 0xFF);
    goal[i * 2 + 1] = (uint8_t)((g >> 8) & 0xFF);
  }
  mc.havePrev = true;
  if (hwFault) {                            // a servo raised a hardware-error bit: stop, fail safe
    motorTorqueOff();
    mc.fault = true;
    return;
  }
  if (mc.wdRearm && mc.torque) {            // self-heal: a long stall may have latched the servo watchdog
    for (uint8_t i = 0; i < N_MOTOR; i++) motorArmWatchdog(DXL_IDS[i]);
    mc.wdRearm = false;
  }
  if (mc.torque) dxl.syncWrite(102, 2, DXL_IDS, N_MOTOR, goal);  // also pets the servo Bus Watchdog
  uint32_t el = micros() - t0;
  mc.nTicks++;
  mc.sumUs += el;
  if (el > mc.worstUs) mc.worstUs = el;
}

void motorService() {
  if (!mc.taken) return;
  uint32_t now = micros();
  if ((int32_t)(now - mc.nextTick) < 0) return;
  int32_t late = (int32_t)(now - mc.nextTick);           // how late this tick arrived
  if ((uint32_t)late > mc.tickUs / 2) mc.nOverrun++;
  if (late > (int32_t)BUS_WATCHDOG_US && mc.torque)      // a stall this long may have latched the
    mc.wdRearm = true;                                   // servo Bus Watchdog: re-arm on the next tick
  mc.nextTick += mc.tickUs;
  if ((int32_t)(now - mc.nextTick) >= 0) mc.nextTick = now + mc.tickUs; // resync after stall
  // host-silence watchdog: degrade toward compliance, never toward a lock
  if (mc.torque && millis() - mc.lastCmdMs > MOTOR_WD_MS) {
    if (mc.mode == 2) mc.mode = 1;       // stale direct current is unsafe: blend law
    mc.aOverride = -1.0f;                // the physical crown regains authority
  }
  // direct-fallback only: the hardware-error byte is not in the fast block, so
  // poll it off the hot path (~20 Hz) and fail safe on any fault bit.
  if (mc.torque && mc.offErr < 0 && millis() - mc.lastHwPollMs > 50) {
    mc.lastHwPollMs = millis();
    for (uint8_t i = 0; i < N_MOTOR; i++) {
      uint8_t e;
      if (dxl.read(DXL_IDS[i], A_HW_ERR, 1, &e)) {
        mc.hwErr[i] = e;
        if (e & HW_FAULT_MASK) { motorTorqueOff(); mc.fault = true; }
      }
    }
  }
  motorTick();
}

void motorStats() {
  uint32_t mean = mc.nTicks ? mc.sumUs / mc.nTicks : 0;
  Serial.printf("# motor stats: %lu ticks @ %lu us target, mean %lu us, worst %lu us, "
                "%lu overruns, %lu missed reads\n",
                (unsigned long)mc.nTicks, (unsigned long)mc.tickUs, (unsigned long)mean,
                (unsigned long)mc.worstUs, (unsigned long)mc.nOverrun, (unsigned long)mc.nMiss);
  Serial.printf("# dxl bus: %lu tx, %lu timeouts, %lu crc, %lu hw-err\n",
                (unsigned long)dxl.txCount, (unsigned long)dxl.rxTimeouts,
                (unsigned long)dxl.crcErrors, (unsigned long)dxl.errStatus);
  Serial.printf("# dxl link: %lu baud, %s%s, last turnaround %lu us, hwErr [0x%02X 0x%02X]\n",
                (unsigned long)dxl.baud(), mc.useFast ? "Fast Sync Read 0x8A" : "Sync Read 0x82",
                mc.useIndirect ? " + indirect(pos,cur,vel,hwErr)" : " + direct 126..135",
                (unsigned long)dxl.lastTurnaroundUs, mc.hwErr[0], mc.hwErr[1]);
  mc.nTicks = mc.nMiss = mc.nOverrun = mc.worstUs = mc.sumUs = 0;
}

int motorIdIndex(int id) {
  for (uint8_t i = 0; i < N_MOTOR; i++) if (DXL_IDS[i] == id) return i;
  return -1;
}

// In-firmware benchmark: run n read+write ticks with BOTH methods (0x82 then
// 0x8A) at the CURRENT baud and feedback block, and print a measured row each:
// per-tick mean/min/max/jitter (us), measured bus turnaround (us), achieved Hz,
// and the delta of the bus error counters. Goal current is forced to 0 (safe:
// run with torque off for a pure bus measurement). To fill the full OLD-vs-NEW
// table, run 'M,b' before 'M,u' (1 Mbaud) and again after (4 Mbaud).
void motorBench(uint16_t n) {
  if (!mc.taken) { Serial.println(F("# motor: take the bus first (M,t,1)")); return; }
  if (n == 0) n = 2000;
  if (n > 20000) n = 20000;
  Serial.printf("# BENCH %u ticks/method @ %lu baud, %u B/motor, %s (goal=0, torque %s)\n",
                n, (unsigned long)dxl.baud(), mc.readLen,
                mc.useIndirect ? "indirect" : "direct", mc.torque ? "ON" : "off");
  Serial.println(F("# method meanUs minUs maxUs jitUs  turnUs(mean/min/max)  achHz    dTx dTo dCrc dHw"));
  uint8_t rx[N_MOTOR * 16], rerr[N_MOTOR], goal[N_MOTOR * 2];
  for (uint8_t j = 0; j < N_MOTOR * 2; j++) goal[j] = 0;
  for (uint8_t meth = 0; meth < 2; meth++) {
    uint32_t tx0 = dxl.txCount, to0 = dxl.rxTimeouts, cr0 = dxl.crcErrors, he0 = dxl.errStatus;
    uint32_t okN = 0, sumUs = 0, minUs = 0xFFFFFFFF, maxUs = 0;
    uint32_t sumTn = 0, minTn = 0xFFFFFFFF, maxTn = 0;
    uint32_t tStart = micros();
    for (uint16_t k = 0; k < n; k++) {
      uint32_t a = micros();
      bool ok = (meth == 0)
        ? dxl.syncRead(mc.readAddr, mc.readLen, DXL_IDS, N_MOTOR, rx)
        : dxl.fastSyncRead(mc.readAddr, mc.readLen, DXL_IDS, N_MOTOR, rx, rerr);
      if (ok) dxl.syncWrite(102, 2, DXL_IDS, N_MOTOR, goal);
      uint32_t el = micros() - a;
      if (ok) {
        okN++; sumUs += el; if (el < minUs) minUs = el; if (el > maxUs) maxUs = el;
        uint32_t tn = dxl.lastTurnaroundUs;
        sumTn += tn; if (tn < minTn) minTn = tn; if (tn > maxTn) maxTn = tn;
      }
    }
    uint32_t elapsed = micros() - tStart;
    uint32_t mean = okN ? sumUs / okN : 0, meanTn = okN ? sumTn / okN : 0;
    float achHz = elapsed ? (float)n * 1e6f / (float)elapsed : 0.0f;
    if (!okN) { minUs = 0; minTn = 0; }
    Serial.printf("# %-6s %6lu %5lu %5lu %5lu  %5lu/%5lu/%5lu  %7.0f  %4lu %3lu %4lu %3lu\n",
                  meth ? "0x8A" : "0x82",
                  (unsigned long)mean, (unsigned long)minUs, (unsigned long)maxUs,
                  (unsigned long)(maxUs - minUs), (unsigned long)meanTn,
                  (unsigned long)minTn, (unsigned long)maxTn, achHz,
                  (unsigned long)(dxl.txCount - tx0), (unsigned long)(dxl.rxTimeouts - to0),
                  (unsigned long)(dxl.crcErrors - cr0), (unsigned long)(dxl.errStatus - he0));
  }
  mc.nextTick = micros();   // resync the scheduler after the busy benchmark
}

// Dispatch one complete "M,..." line (the loop() reader hands it over whole,
// so payload bytes can never alias single-letter menu commands).
void handleMotorLine(const char* line) {
  mc.lastCmdMs = millis();
  char sub = 0;
  float a = 0, b = 0;
  int n = sscanf(line, "M,%c,%f,%f", &sub, &a, &b);
  if (n < 1) return;
  switch (sub) {
    case 't': (a >= 0.5f) ? motorTake() : motorRelease(); break;
    case 'e': (a >= 0.5f) ? motorEnable() : motorTorqueOff(); break;
    case 'm': if (a >= 0 && a <= 2) { mc.mode = (uint8_t)a; if (mc.mode == 0) motorTorqueOff(); } break;
    case 'a': mc.aOverride = (a < 0) ? -1.0f : constrain(a, 0.0f, 1000.0f) * 0.001f; break;
    case 'c': { int i = motorIdIndex((int)a); if (i >= 0 && n >= 3) mc.iSet[i] = constrain(b, -I_CAP_MA, I_CAP_MA); } break;
    case 'p': { int i = motorIdIndex((int)a); if (i >= 0 && n >= 3) mc.qSet[i] = b * PI / 180.0f; } break;
    case 'k': if (n >= 3 && a >= 0 && b >= 0) { mc.kp = min(a, 2000.0f); mc.kd = min(b, 100.0f); } break;
    case 'f': if (n >= 3 && a >= 0 && b >= 0) { mc.frV = min(a, 50.0f); mc.frC = min(b, 60.0f); } break;
    case 's': motorStats(); break;
    case 'u': motorUpgrade(); break;
    case 'b': motorBench(n >= 2 ? (uint16_t)a : 2000); break;
  }
}

// ---- scan / report ---------------------------------------------------------
// detectAll(): quiet re-detection so sensors are HOT-PLUGGABLE (no reboot).
// - re-probes every mux channel -> chLive[]
// - re-inits any IMU that is not already live (a live IMU is left running so it
//   never loses its fusion lock). Called by the verbose scan ('s'), by the quiet
//   rescan ('R', which the host bridge sends ~every 1.5 s), and once at boot.
void detectAll() {
  for (uint8_t ch = 0; ch < N_CHANNELS; ch++) {
    uint8_t m, c; chToMux(ch, m, c);
    muxSelect(m, c);
    chLive[ch] = encoderPresent();
    muxDisable(m);
  }
  for (uint8_t i = 0; i < N_IMU; i++) {   // hot-plug: (re)start an IMU that appeared
    if (imuLive[i] && bno[i].present()) continue;
    imuLive[i] = bno[i].present() ? bno[i].begin() : false;
    if (imuLive[i]) imuFreshMs[i] = millis();   // grace: first report may take a moment
  }
}

void scanAll() {
  detectAll();
  Serial.println(F("\n--- WIRING SCAN ---------------------------------------"));
  for (uint8_t i = 0; i < 2; i++) {
    MUX_BUS.beginTransmission(MUX_ADDR[i]);
    bool ok = (MUX_BUS.endTransmission() == 0);
    Serial.printf("  MUX %c @ 0x%02X : %s\n", 'A' + i, MUX_ADDR[i], ok ? "OK" : "MISSING");
  }
  uint8_t nLive = 0;
  for (uint8_t ch = 0; ch < N_CHANNELS; ch++) {
    uint8_t m, c; chToMux(ch, m, c);
    if (chLive[ch]) nLive++;
    Serial.printf("  ENC ch%02u (mux 0x%02X:%u) : %s\n", ch, m, c,
                  chLive[ch] ? "OK 0x36" : "-- none");
  }
  for (uint8_t i = 0; i < N_IMU; i++)
    Serial.printf("  IMU %u (%s) @ 0x%02X on Wire%c : %s\n", i + 1, IMU_NAME[i],
                  IMU_ADDR[i], IMU_WIRE[i] == &Wire1 ? '1' : '2',
                  imuLive[i] ? "OK" : "MISSING");
  Serial.printf("  SD card : %s\n", sdOK ? "OK" : "MISSING");
  Serial.printf("  SUMMARY : %u/%u encoders, %u/%u IMUs live\n",
                nLive, N_CHANNELS, (imuLive[0] + imuLive[1] + imuLive[2]), N_IMU);
  Serial.println(F("-------------------------------------------------------"));
}

// ---- imu -------------------------------------------------------------------
// A misbehaving IMU can hold the bus low and make begin_I2C() block forever,
// freezing the whole sketch at boot. Guard every IMU init: release a stuck bus,
// and only call the blocking begin_I2C after the address cleanly ACKs a bounded
// probe. A bad/absent IMU then just reads "missing" instead of killing the board.
bool i2cPresent(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;   // Teensy Wire has an internal timeout
}
void busRecover(uint8_t sclPin, uint8_t sdaPin) {  // free an I2C bus if a slave holds SDA low
  pinMode(sclPin, OUTPUT);
  pinMode(sdaPin, INPUT_PULLUP);
  for (uint8_t k = 0; k < 9 && digitalRead(sdaPin) == LOW; k++) {
    digitalWrite(sclPin, LOW);  delayMicroseconds(5);
    digitalWrite(sclPin, HIGH); delayMicroseconds(5);
  }
}
void imuBusRecover() { busRecover(16, 17); busRecover(24, 25); }  // Wire1 (17/16) + Wire2 (25/24)
// Both BNO085s run SIMULTANEOUSLY at full rate: TinyBNO085 keeps all protocol
// state per object (no globals), each sensor owns a whole I2C bus, and poll()
// is bounded and non-blocking. No round-robin, no shared-context hang.
void imuBegin() {
  for (uint8_t i = 0; i < N_IMU; i++) {
    imuLive[i] = bno[i].begin();
    if (imuLive[i]) imuFreshMs[i] = millis();
  }
}
void imuService() {                  // both sensors, every loop pass, independent buses
  uint32_t now = millis();
  // Self-stall forgiveness: if WE did not run for a while (calibrate/fullScan
  // block deliberately), the silence is ours, not the sensors'. Reset their
  // freshness clocks instead of condemning healthy IMUs.
  if (now - imuSvcMs > IMU_STALE_MS / 2) {
    for (uint8_t i = 0; i < N_IMU; i++) imuFreshMs[i] = now;
  }
  imuSvcMs = now;
  for (uint8_t i = 0; i < N_IMU; i++) {
    if (!imuLive[i]) continue;
    for (uint8_t k = 0; k < 4 && bno[i].poll(); k++) {}  // drain up to 4 packets
    if (bno[i].fresh) {
      bno[i].fresh = false;          // read-and-clear: "new since last service"
      imuQ[i][0] = bno[i].qw; imuQ[i][1] = bno[i].qx;
      imuQ[i][2] = bno[i].qy; imuQ[i][3] = bno[i].qz;
      imuFreshMs[i] = now;
    } else if (now - imuFreshMs[i] > IMU_STALE_MS) {
      // Silent freeze: the chip still ACKs address probes but stopped
      // reporting (brownout / internal reset). Report it honestly - the
      // stream's live flag drops, the host bridge notices the drop and sends
      // 'R', and detectAll() re-begins the sensor. Closes the loop where a
      // frozen IMU streamed its last quaternion forever as "live".
      imuLive[i] = false;
    }
  }
}

// ---- calibration -----------------------------------------------------------
void calibrate() {
  for (uint8_t ch = 0; ch < N_CHANNELS; ch++) { chMin[ch] = 9999; chMax[ch] = -9999; }
  Serial.println(F("\n>>> CALIBRATION: slowly flex EVERY finger through its full"));
  Serial.println(F(">>> range (open <-> closed) for 12 seconds..."));
  uint32_t t0 = millis();
  while (millis() - t0 < 12000) {
    float d[N_CHANNELS];
    readAllChannels(d);
    for (uint8_t ch = 0; ch < N_CHANNELS; ch++) {
      if (!chLive[ch] || d[ch] < 0) continue;
      if (d[ch] < chMin[ch]) chMin[ch] = d[ch];
      if (d[ch] > chMax[ch]) chMax[ch] = d[ch];
    }
    motorService();   // keep the motor fail-safe live during this 12 s blocking capture
    delay(10);
  }
  Serial.println(F("\n--- RANGE OF MOTION (per channel) ---------------------"));
  for (uint8_t ch = 0; ch < N_CHANNELS; ch++) {
    if (!chLive[ch]) continue;
    Serial.printf("  ch%02u : min %6.1f  max %6.1f  ROM %6.1f deg\n",
                  ch, chMin[ch], chMax[ch], chMax[ch] - chMin[ch]);
  }
  Serial.println(F("(Flex one finger at a time and watch which channel moves to"));
  Serial.println(F(" learn the channel->finger/joint map; note it down.)"));
  Serial.println(F("-------------------------------------------------------"));
}

// ---- SD recording ----------------------------------------------------------
void recToggle() {
  if (!recording) {
    if (!sdOK) { Serial.println(F("No SD card - cannot record.")); return; }
    // collision-free name: FILE_WRITE appends, so a reused name would bury a
    // second header mid-file. Scan past anything already on the card.
    char name[24];
    do {
      snprintf(name, sizeof(name), "REC%05u.CSV", recSeq++);
    } while (SD.exists(name) && recSeq < 60000);
    recFile = SD.open(name, FILE_WRITE);
    if (!recFile) { Serial.println(F("Could not open file.")); return; }
    // header: time + 14 encoder angles + 2 quaternions
    recFile.print("t_ms");
    for (uint8_t ch = 0; ch < N_CHANNELS; ch++) recFile.printf(",enc%02u", ch);
    recFile.print(",h_qw,h_qx,h_qy,h_qz,f_qw,f_qx,f_qy,f_qz,emg_env,emg_rms,crown"
                  ",t_qw,t_qx,t_qy,t_qz,thumb_live\n");
    recording = true; recStart = millis(); recRows = 0;
    Serial.printf(">>> RECORDING to %s (press 'r' again to stop)\n", name);
  } else {
    recFile.close(); recording = false;
    Serial.printf(">>> STOPPED. %lu rows over %.1f s saved to SD.\n",
                  (unsigned long)recRows, (millis() - recStart) / 1000.0f);
  }
}
void recWrite(uint32_t t) {
  recFile.printf("%lu", (unsigned long)t);
  for (uint8_t ch = 0; ch < N_CHANNELS; ch++) {
    recFile.printf(",%.2f", frameDeg[ch]);   // the frame's ONE acquisition
  }
  for (uint8_t i = 0; i < 2; i++)
    recFile.printf(",%.4f,%.4f,%.4f,%.4f", imuQ[i][0], imuQ[i][1], imuQ[i][2], imuQ[i][3]);
  recFile.printf(",%.1f,%.1f", emgEnv, emgRms);
  recFile.printf(",%d", (int)(crownFilt < 0.0f ? 0 : crownFilt));
  recFile.printf(",%.4f,%.4f,%.4f,%.4f,%d",
                 imuQ[2][0], imuQ[2][1], imuQ[2][2], imuQ[2][3], imuLive[2] ? 1 : 0);
  recFile.print('\n');
  recRows++;
}

// ---- live stream (for the web-console bridge) ------------------------------
// One compact CSV line per sample. Prefix 'S,' so the bridge can pick these out
// from human-readable menu output. Uses the last scan's chLive[] gating.
void emitStream(uint32_t t) {
  Serial.print("S,");
  Serial.print(t);
  for (uint8_t ch = 0; ch < N_CHANNELS; ch++) {
    Serial.print(',');
    Serial.print(frameDeg[ch], 2);           // identical values to the SD row
  }
  for (uint8_t i = 0; i < 2; i++)
    for (uint8_t k = 0; k < 4; k++) { Serial.print(','); Serial.print(imuQ[i][k], 4); }
  Serial.print(','); Serial.print(imuLive[0] ? 1 : 0);
  Serial.print(','); Serial.print(imuLive[1] ? 1 : 0);
  Serial.print(','); Serial.print(emgEnv, 1);   // MyoWare envelope, mean ADC (0..1023)
  Serial.print(','); Serial.print(emgRms, 1);   // envelope RMS over the frame
  Serial.print(','); Serial.print(emgHave ? 1 : 0);
  Serial.print(','); Serial.print((int)(crownFilt < 0.0f ? 0 : crownFilt)); // crown 0..1000 (v3)
  for (uint8_t k = 0; k < 4; k++) { Serial.print(','); Serial.print(imuQ[2][k], 4); }  // thumb (v4)
  Serial.print(','); Serial.print(imuLive[2] ? 1 : 0);
  Serial.print('\n');
}

// ---- full I2C scanner (diagnostic: what actually ACKs, and where) ----------
void scanChannelAddrs(bool skipMux) {
  uint8_t n = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (skipMux && (a == 0x70 || a == 0x71)) continue;
    MUX_BUS.beginTransmission(a);
    if (MUX_BUS.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  if (n == 0) Serial.print(F(" (nothing)"));
  Serial.println();
}
void fullScan() {
  Serial.println(F("\n--- FULL I2C SCAN -------------------------------------"));
  for (uint8_t i = 0; i < 2; i++) muxDisable(MUX_ADDR[i]);
  Serial.print(F("  MAIN bus (Wire 18/19)   :"));
  scanChannelAddrs(false);                       // expect 0x70 0x71
  Serial.print(F("  Wire1 IMU bus (17/16)   :"));
  { uint8_t n = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      Wire1.beginTransmission(a);
      if (Wire1.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
    }
    if (n == 0) Serial.print(F(" (nothing)")); Serial.println(); }
  Serial.print(F("  Wire2 IMU bus (25/24)   :"));
  { uint8_t n = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      Wire2.beginTransmission(a);
      if (Wire2.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
    }
    if (n == 0) Serial.print(F(" (nothing)")); Serial.println(); }
  for (uint8_t ch = 0; ch < N_CHANNELS; ch++) {  // each mux channel: expect 0x36
    uint8_t m, c; chToMux(ch, m, c);
    muxSelect(m, c);
    Serial.printf("  ch%02u (mux 0x%02X:%u)       :", ch, m, c);
    scanChannelAddrs(true);
    muxDisable(m);
  }
  Serial.println(F("-------------------------------------------------------"));
}

// ---- main ------------------------------------------------------------------
void printHelp() {
  Serial.println(F("\nCommands: s=scan  A=full I2C scan  c=calibrate  r=record  j=stream  k=stop  ?=help"));
}
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  MUX_BUS.begin();  MUX_BUS.setClock(400000);
  imuBusRecover();                     // release both IMU buses if a slave holds SDA low
  Wire1.begin();  Wire1.setClock(400000);   // forearm 0x4B, own bus
  Wire2.begin();  Wire2.setClock(400000);   // hand 0x4A, own bus (pins 25/24)
  for (uint8_t i = 0; i < 2; i++) muxDisable(MUX_ADDR[i]);
  pinMode(EMG_PIN, INPUT);             // MyoWare envelope on A0 (analog in)
  sdOK = SD.begin(BUILTIN_SDCARD);
  Serial.println(F("\nbringup_12ch - Teensy 4.1"));
  imuBegin();
  scanAll();
  printHelp();
}
void loop() {
  motorService();   // servo control tick (no-op until the bus is taken with M,t,1)
  // Drain everything pending (a backlog builds during the deliberate blocking
  // commands). Line-oriented input ("D,...\n" screen pushes and "M,...\n" motor
  // commands) is swallowed as a unit so payload bytes can never alias the
  // single-letter menu commands.
  while (Serial.available()) {
    char c = Serial.read();
    if (discardLine) { if (c == '\n') discardLine = false; continue; }
    if (inMotorLine) {                      // accumulate a whole M-line, dispatch on newline
      if (c == '\n') { mBuf[mLen] = 0; handleMotorLine(mBuf); inMotorLine = false; mLen = 0; }
      else if (mLen >= sizeof(mBuf) - 1) {  // over-long line: dispatch what we have, swallow the tail
        mBuf[mLen] = 0; handleMotorLine(mBuf); inMotorLine = false; mLen = 0; discardLine = true;
      } else mBuf[mLen++] = c;
      continue;
    }
    if (c == 'D') { discardLine = true; }   // device-screen line: for the screen firmware, not us
    else if (c == 'M') { inMotorLine = true; mLen = 0; mBuf[mLen++] = 'M'; }  // motor-command line
    else if (c == 's') scanAll();
    else if (c == 'A') fullScan();
    else if (c == 'c') calibrate();
    else if (c == 'r') recToggle();                       // human toggle
    else if (c == 'b') { if (!recording) recToggle(); }   // host: explicit start (idempotent)
    else if (c == 'e') { if (recording)  recToggle(); }   // host: explicit stop  (idempotent)
    else if (c == 'v') Serial.printf("# ver bringup_12ch %u\n", FW_VERSION);
    else if (c == 'j') { streaming = true;  Serial.println(F("# stream ON")); }
    else if (c == 'k') { streaming = false; Serial.println(F("# stream OFF")); }
    else if (c == 'R') detectAll();   // quiet hot-plug rescan (host bridge sends this)
    else if (c == '?') printHelp();
  }
  motorService();   // keep the control tick alive after draining a serial backlog
  imuService();
  // oversample the MyoWare envelope (pin 14) at ~1 kHz between the 50 Hz frames
  uint32_t nowU = micros();
  if (nowU - emgLastU >= 1000) {
    emgLastU = nowU;
    uint16_t v = analogRead(EMG_PIN);
    emgSum += v; emgSumSq += (uint32_t)v * v; emgCount++;
  }
  const uint32_t PERIOD_MS = (uint32_t)(1000.0f / SAMPLE_HZ);
  uint32_t now = millis();
  if (now - lastSample >= PERIOD_MS) {
    // Advance by the period so the average rate is exactly SAMPLE_HZ (the old
    // `lastSample = now` slipped by the loop latency every frame). After a real
    // stall (SD flush, calibrate) resync instead of bursting catch-up frames:
    // bunched rows would be worse data than honestly missing rows.
    lastSample += PERIOD_MS;
    if (now - lastSample >= PERIOD_MS) lastSample = now;
    // reduce the EMG oversample to envelope (mean) + RMS, then reset the accumulator
    emgHave = (emgCount > 0);
    if (emgHave) {
      emgEnv = (float)emgSum / (float)emgCount;
      emgRms = sqrtf((float)emgSumSq / (float)emgCount);
    } else { emgEnv = 0.0f; emgRms = 0.0f; }
    emgSum = 0; emgSumSq = 0; emgCount = 0;
    // crown pot: sample per frame, light EMA so the streamed value is stable
    // enough that every UI dial can track it directly (band -> 0..1000)
    {
      int raw = analogRead(POT_PIN);
      float t01 = (float)(raw - POT_ACTIVE_LOW) / (float)(POT_ACTIVE_HIGH - POT_ACTIVE_LOW);
      if (t01 < 0.0f) t01 = 0.0f;
      if (t01 > 1.0f) t01 = 1.0f;
      float v = t01 * 1000.0f;
      crownFilt = (crownFilt < 0.0f) ? v : crownFilt + 0.25f * (v - crownFilt);
    }
    // ONE sensor acquisition per frame, shared by the SD row and the S-line:
    // halves the I2C budget when recording + streaming, and makes the stored
    // and streamed values identical for a given timestamp.
    if (recording || streaming) readAllChannels(frameDeg);
    motorService();                 // the 50 Hz frame is the longest blocker: service between its steps
    if (recording) recWrite(now);
    if (streaming) emitStream(now);
    motorService();
  }
}
