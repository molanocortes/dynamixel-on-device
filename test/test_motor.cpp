// Integration proof: compile + run the REAL bringup_12ch.ino motor subsystem
// against the emulated two-servo bus. Verifies the wiring the sketch was missing
// (M-line dispatch + motorService tick), the indirect-block + Fast Sync Read
// enable path, the per-offset parse, the hardware-error fail-safe, the sustained-
// miss fail-safe, and the M,b benchmark. No hardware.
#include <Arduino.h>
#include "emu.h"
#include "../examples/wearable_hand_bringup/wearable_hand_bringup.ino"

static int PASS = 0, FAILN = 0;
#define CHECK(d, c) do { if (c) { PASS++; printf("  ok   %s\n", d); } else { FAILN++; printf(" FAIL  %s\n", d); } } while (0)
static bool approx(float a, float b, float tol) { float d = a - b; return (d < 0 ? -d : d) <= tol; }

static void seedFeedback(uint8_t id, int32_t pos, int16_t cur, int32_t vel, uint8_t hw) {
  // indirect data block at 224: pos(4) cur(2) vel(4) hwErr(1) -- LITTLE ENDIAN
  EMU_REG[id][224] = pos & 0xFF; EMU_REG[id][225] = (pos >> 8) & 0xFF;
  EMU_REG[id][226] = (pos >> 16) & 0xFF; EMU_REG[id][227] = (pos >> 24) & 0xFF;
  EMU_REG[id][228] = cur & 0xFF; EMU_REG[id][229] = (cur >> 8) & 0xFF;
  EMU_REG[id][230] = vel & 0xFF; EMU_REG[id][231] = (vel >> 8) & 0xFF;
  EMU_REG[id][232] = (vel >> 16) & 0xFF; EMU_REG[id][233] = (vel >> 24) & 0xFF;
  EMU_REG[id][234] = hw;
  EMU_ERR[id] = 0;
}

int main() {
  seedFeedback(1, 2148, 25, 10, 0);      // +100 counts, +25 mA, +10 vel-units, no fault
  seedFeedback(2, 1948, -30, -5, 0);     // -100 counts, -30 mA

  printf("\n[1] take the bus: detect both motors, qualify, connect SAFE\n");
  handleMotorLine("M,t,1");
  CHECK("bus taken", mc.taken);
  CHECK("tick set to 500 us (2 kHz) at >=3 Mbaud", mc.tickUs == 500);
  CHECK("opened at 4 Mbaud (target)", dxl.baud() == 4000000);
  CHECK("connected torque OFF (compliant/safe, not driving)", !mc.torque);
  CHECK("no fault on a clean connect", !mc.fault);

  printf("\n[2] enable: indirect block armed + Fast Sync Read chosen\n");
  handleMotorLine("M,e,1");
  CHECK("torque ON", mc.torque);
  CHECK("indirect feedback armed", mc.useIndirect);
  CHECK("Fast Sync Read 0x8A selected", mc.useFast);
  CHECK("read addr = 224 (Indirect Data 1), len = 11", mc.readAddr == 224 && mc.readLen == 11);
  CHECK("offsets: pos0 cur4 vel6 err10", mc.offPos==0 && mc.offCur==4 && mc.offVel==6 && mc.offErr==10);
  CHECK("servo Bus Watchdog armed on enable (reg 98 = 5 = 100 ms)", EMU_REG[1][98]==5 && EMU_REG[2][98]==5);

  printf("\n[3] one tick parses the fast block into engineering units\n");
  mc.mode = 1;                            // blended transparency<->assist
  mc.aOverride = 0.0f;                    // fully transparent (a=0)
  motorTick();
  CHECK("motor 1 position ~ +8.79 deg (100 counts)", approx(mc.posDeg[0], 8.789f, 0.05f));
  CHECK("motor 2 position ~ -8.79 deg", approx(mc.posDeg[1], -8.789f, 0.05f));
  CHECK("motor 1 current = 25 mA (1 mA/unit)", approx(mc.iMeas[0], 25.0f, 0.5f));
  CHECK("motor 2 current = -30 mA", approx(mc.iMeas[1], -30.0f, 0.5f));
  CHECK("hardware-error bytes clear", mc.hwErr[0]==0 && mc.hwErr[1]==0);
  CHECK("no fault", !mc.fault && mc.torque);

  printf("\n[4] a Hardware Error Status bit trips the fail-safe (torque off)\n");
  seedFeedback(2, 1948, -30, -5, 0x20);   // overload bit on motor 2
  motorTick();
  CHECK("fault latched", mc.fault);
  CHECK("torque dropped", !mc.torque);
  CHECK("mode returned to idle", mc.mode == 0);

  printf("\n[5] re-enable clears the fault; sustained bus loss also fails safe\n");
  seedFeedback(1, 2148, 25, 10, 0); seedFeedback(2, 1948, -30, -5, 0);
  handleMotorLine("M,e,1");
  CHECK("fault cleared on re-enable", !mc.fault && mc.torque);
  EMU_DROP_ONE_BYTE = true;               // every read now truncates -> miss
  for (int k = 0; k < MOTOR_ERR_TRIP + 2; k++) motorTick();
  EMU_DROP_ONE_BYTE = false;
  CHECK("consecutive misses -> torque off", !mc.torque && mc.fault);
  CHECK("misses counted (never silently retried)", mc.nMiss >= MOTOR_ERR_TRIP);

  printf("\n[6] M,b benchmark runs both methods against the bus\n");
  seedFeedback(1, 2148, 25, 10, 0); seedFeedback(2, 1948, -30, -5, 0);
  handleMotorLine("M,e,1");
  uint32_t tx0 = dxl.txCount;
  handleMotorLine("M,b,40");
  CHECK("benchmark issued transactions for both methods", dxl.txCount > tx0 + 40);

  printf("\n[7] indirect map fails -> graceful fallback to the DIRECT 126..135 read\n");
  handleMotorLine("M,t,0");                 // release
  handleMotorLine("M,t,1");                 // re-take
  // seed the DIRECT contiguous block: current@126(2) vel@128(4) pos@132(4)
  EMU_REG[1][126]=0x19; EMU_REG[1][127]=0x00;                       // cur = 25
  EMU_REG[1][128]=0x0A; EMU_REG[1][129]=EMU_REG[1][130]=EMU_REG[1][131]=0; // vel = 10
  EMU_REG[1][132]=0x64; EMU_REG[1][133]=0x08; EMU_REG[1][134]=EMU_REG[1][135]=0; // pos = 2148
  EMU_REG[2][126]=0xE2; EMU_REG[2][127]=0xFF;                       // cur = -30
  EMU_REG[2][132]=0x9C; EMU_REG[2][133]=0x07; EMU_REG[2][134]=EMU_REG[2][135]=0; // pos = 1948
  EMU_BREAK_INDIRECT = true;
  handleMotorLine("M,e,1");
  EMU_BREAK_INDIRECT = false;
  CHECK("indirect NOT armed (read-back mismatch caught)", !mc.useIndirect);
  CHECK("fell back to direct: addr 126, len 10", mc.readAddr == 126 && mc.readLen == 10);
  CHECK("direct offsets: cur0 vel2 pos6 err-none", mc.offCur==0 && mc.offVel==2 && mc.offPos==6 && mc.offErr==-1);
  mc.mode = 1; mc.aOverride = 0.0f;
  motorTick();
  CHECK("direct parse: motor1 pos ~ +8.79 deg", approx(mc.posDeg[0], 8.789f, 0.05f));
  CHECK("direct parse: motor1 current = 25 mA", approx(mc.iMeas[0], 25.0f, 0.5f));
  CHECK("direct parse: motor2 current = -30 mA", approx(mc.iMeas[1], -30.0f, 0.5f));

  printf("\n[8] loop() M-line reader dispatches (the wiring that was missing)\n");
  mc.torque = true;                        // pretend enabled
  Serial.feed("M,e,0\n");                   // torque OFF via the serial reader path
  loop();                                   // one pass drains + dispatches the line
  CHECK("loop() routed 'M,e,0' to handleMotorLine (torque off)", !mc.torque);

  printf("\n[9] single-master safety: refuse to take a BUSY bus (U2D2 active)\n");
  handleMotorLine("M,t,0");                 // release
  g_bus_traffic = true;                     // another master starts driving DATA
  mc.taken = true;                          // (prove motorTake actively clears it)
  handleMotorLine("M,t,1");
  g_bus_traffic = false;
  Serial1.rxhead = Serial1.rxtail = 0;      // clear the injected noise
  CHECK("refused to take while another master drives DATA", !mc.taken);
  CHECK("did not enable torque on a contended bus", !mc.torque);

  printf("\n[10] missing motor: refuse a half-populated bus\n");
  EMU_ABSENT_ID = 2;                        // motor 2 unpowered / unplugged
  mc.taken = true;
  handleMotorLine("M,t,1");
  EMU_ABSENT_ID = 255;
  CHECK("refused: bus not fully populated (a motor is missing)", !mc.taken);

  printf("\n[11] recovery: both motors back -> clean take again\n");
  handleMotorLine("M,t,1");
  CHECK("takes cleanly once both motors answer", mc.taken && !mc.torque);

  printf("\n[12] enable with a LOST config write -> stays SAFE (torque OFF, fault)\n");
  handleMotorLine("M,e,0");                 // torque-off baseline
  EMU_DROP_ONE_BYTE = true;                 // every ack truncates -> config writes fail
  handleMotorLine("M,e,1");
  EMU_DROP_ONE_BYTE = false;
  CHECK("did NOT energize on a failed config write", !mc.torque);
  CHECK("fault flagged, not silently reported OFF", mc.fault);

  printf("\n[13] servo Bus Watchdog self-heal: a long stall re-arms the dead-man\n");
  seedFeedback(1, 2148, 25, 10, 0); seedFeedback(2, 1948, -30, -5, 0);
  handleMotorLine("M,e,1");                 // torque on, watchdog armed
  CHECK("re-enabled cleanly, watchdog armed", mc.torque && EMU_REG[1][98]==5);
  EMU_REG[1][98] = 0xFF; EMU_REG[2][98] = 0xFF;   // pretend the servo watchdog LATCHED (-1)
  mc.mode = 1; mc.aOverride = 0.0f;
  g_micros += 200000;                       // simulate a 200 ms stall (> the 100 ms watchdog)
  motorService();                           // must detect the stall, re-arm, and resume
  CHECK("watchdog re-armed after the stall (reg 98 back to 5)", EMU_REG[1][98]==5 && EMU_REG[2][98]==5);
  CHECK("re-arm flag consumed", !mc.wdRearm);
  CHECK("still driving after self-heal", mc.torque && !mc.fault);

  printf("\n%s  motor integration: %d passed, %d failed\n", FAILN ? "XXXX" : "====", PASS, FAILN);
  return FAILN ? 1 : 0;
}
