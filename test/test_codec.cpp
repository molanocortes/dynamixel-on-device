// Host-side codec proof for tiny_dxl.h: Fast Sync Read (0x8A) parse, CRC-16,
// byte-stuffing across 0xFDFF (TX stuffing + RX destuffing), and the failure
// paths (bad CRC, truncation, id mis-order). Compiles the REAL driver against an
// Arduino stub + a faithful bus emulator. No hardware, no STL.
#include "Arduino.h"
#include "emu.h"
#include "tiny_dxl.h"     // the real driver under test
#include <stdio.h>

static int PASS = 0, FAIL = 0;
#define CHECK(desc, cond) do { if (cond) { PASS++; printf("  ok   %s\n", desc); } \
  else { FAIL++; printf(" FAIL  %s\n", desc); } } while (0)

static HardwareSerial S;
static TinyDXL dxl(&S, 7);

static void seed(uint8_t id, uint16_t a, const uint8_t* b, int n) {
  for (int k = 0; k < n; k++) EMU_REG[id][a + k] = b[k];
}

int main() {
  dxl.begin(4000000);
  const uint8_t ids[2] = {1, 2};

  printf("\n[1] Fast Sync Read (0x8A) round-trip, adversarial data\n");
  // 11-byte indirect block at 224. Deliberately embed FF FF FD 00 (the header
  // run) and a 0xFDFF position value: the FSR status is NOT stuffed, so a correct
  // fixed-length parse must return these verbatim, unbroken by the pattern.
  const uint8_t d1[11] = {0xFF,0xFF,0xFD,0x00, 0x10,0x20, 0x30,0x40,0x50,0x60, 0x01};
  const uint8_t d2[11] = {0xFD,0xFF,0x00,0x00, 0xAA,0xBB, 0x01,0x02,0x03,0x04, 0x00};
  seed(1, 224, d1, 11); seed(2, 224, d2, 11);
  EMU_ERR[1] = 0x00; EMU_ERR[2] = 0x00;
  uint8_t rx[2*11]; uint8_t err[2];
  bool ok = dxl.fastSyncRead(224, 11, ids, 2, rx, err);
  CHECK("fastSyncRead returns true on a valid frame", ok);
  CHECK("device 1 data intact (embedded FF FF FD 00 header pattern)", memcmp(rx, d1, 11) == 0);
  CHECK("device 2 data intact (0xFDFF position value)", memcmp(rx+11, d2, 11) == 0);
  CHECK("per-device error bytes captured", err[0]==0 && err[1]==0);

  printf("\n[2] Fast Sync Read surfaces a servo hardware-error bit\n");
  EMU_ERR[2] = 0x08;                                   // e.g. overheating alert on device 2
  uint32_t e0 = dxl.errStatus;
  ok = dxl.fastSyncRead(224, 11, ids, 2, rx, err);
  CHECK("still parses with an error bit set", ok);
  CHECK("error byte reported for device 2", err[1]==0x08);
  CHECK("errStatus counter incremented", dxl.errStatus == e0 + 1);
  EMU_ERR[2] = 0x00;

  printf("\n[3] Fast Sync Read rejects a corrupted overall CRC\n");
  uint32_t c0 = dxl.crcErrors;
  EMU_CORRUPT_CRC = true;
  ok = dxl.fastSyncRead(224, 11, ids, 2, rx, err);
  EMU_CORRUPT_CRC = false;
  CHECK("returns false on bad CRC", !ok);
  CHECK("crcErrors incremented (not silently retried)", dxl.crcErrors == c0 + 1);

  printf("\n[4] Fast Sync Read rejects a truncated frame (bounded, no hang)\n");
  uint32_t t0 = dxl.rxTimeouts;
  EMU_DROP_ONE_BYTE = true;
  ok = dxl.fastSyncRead(224, 11, ids, 2, rx, err);
  EMU_DROP_ONE_BYTE = false;
  CHECK("returns false on truncation", !ok);
  CHECK("counted as a timeout", dxl.rxTimeouts == t0 + 1);

  printf("\n[5] Fast Sync Read rejects an id / ordering mismatch\n");
  uint32_t c1 = dxl.crcErrors;
  EMU_SWAP_IDS = 1;
  ok = dxl.fastSyncRead(224, 11, ids, 2, rx, err);
  EMU_SWAP_IDS = 0;
  CHECK("returns false when ids come back out of order", !ok);
  CHECK("counted (id check fired)", dxl.crcErrors == c1 + 1);

  printf("\n[6] Sync Read (0x82) RX destuffing across 0xFDFF (classic bug)\n");
  const uint8_t sd[10] = {0xFF,0xFF,0xFD, 0x11,0x22, 0xFF,0xFF,0xFF,0xFD, 0x33};
  seed(1, 100, sd, 10);
  uint8_t sr[10];
  ok = dxl.syncRead(100, 10, ids, 1, sr);
  CHECK("syncRead returns true", ok);
  CHECK("destuffed params equal the originals", memcmp(sr, sd, 10) == 0);

  printf("\n[7] TX stuffing: Write-then-Read round-trip with FF FF FD payload\n");
  const uint8_t wr[8] = {0xFF,0xFF,0xFD, 0x55, 0xFF,0xFF,0xFD, 0x66};
  int tx_before = S.txlen;
  bool w = dxl.write(1, 200, wr, 8);
  bool sawStuff = false;                               // inserted FD FD after FF FF on the wire
  for (int i = tx_before; i + 3 < S.txlen; i++)
    if (S.txbuf[i]==0xFF && S.txbuf[i+1]==0xFF && S.txbuf[i+2]==0xFD && S.txbuf[i+3]==0xFD) sawStuff = true;
  uint8_t rb[8];
  bool r = dxl.read(1, 200, 8, rb);
  CHECK("write acked", w);
  CHECK("read acked", r);
  CHECK("instruction byte-stuffed on the wire (FD FD after FF FF)", sawStuff);
  CHECK("write/read round-trip preserves the FF FF FD payload", memcmp(rb, wr, 8) == 0);

  printf("\n[8] Ping sanity\n");
  uint16_t model = 0;
  CHECK("ping id 1 succeeds", dxl.ping(1, &model));
  CHECK("model number parsed (1230 = XC330-M181-T)", model == 1230);

  // ---- half-duplex direction control, the one platform-specific part ----
  // Two failure modes destroy a bus: releasing the driver before the last stop
  // bit (truncated frame) and holding it after (fighting the servo's reply).
  // The stub logs every level change against the TX byte count so both are
  // visible. This block is compiled in every direction configuration.
#if TINY_DXL_HAS_HW_DIR
  printf("\n[9] HARDWARE direction: the UART owns the pin, software never touches it\n");
  CHECK("a core with transmitterEnable resolves to HARDWARE",
        dxl.directionMode() == DXL_DIR_HARDWARE);
  CHECK("begin() handed the pin to the UART exactly once", S.te_calls == 1);
  CHECK("handed over the right pin", S.te_pin == 7);
  dirlog_reset();
  dxl.ping(1, &model);
  CHECK("no GPIO toggling in the hot path (the ISR does it)", g_dir_n == 0);
#else
  printf("\n[9] MANUAL direction: the pin brackets the frame exactly\n");
  CHECK("a core without transmitterEnable resolves to MANUAL",
        dxl.directionMode() == DXL_DIR_MANUAL);
  CHECK("never called transmitterEnable on a core that lacks it", S.te_calls == 0);
  dirlog_reset();
  int tx0 = S.txlen;
  dxl.ping(1, &model);
  CHECK("exactly one assert + one release per transaction", g_dir_n == 2);
  CHECK("asserted HIGH before any byte was written",
        g_dir_n >= 1 && g_dir_log[0].level == HIGH && g_dir_log[0].txlen == tx0);
  CHECK("released LOW only after the whole frame was written",
        g_dir_n >= 2 && g_dir_log[1].level == LOW && g_dir_log[1].txlen == S.txlen);
  CHECK("left the transceiver receiving", digitalRead(7) == LOW);
  CHECK("direction pin was configured as an output", g_pin_mode[7] == OUTPUT);
#endif

#ifndef TINY_DXL_DIR_MODE   // a pinned build has only one strategy to compare
  printf("\n[10] the direction strategy never changes the bytes on the wire\n");
  static HardwareSerial S2;
  static TinyDXL other(&S2, 7, DXL_DIR_NONE);   // NONE = nothing toggles; pure frame
  other.begin(4000000);
  int a0 = S.txlen, b0 = S2.txlen;
  dxl.write(1, 200, wr, 8);
  other.write(1, 200, wr, 8);
  int na = S.txlen - a0, nb = S2.txlen - b0;
  CHECK("same frame length regardless of direction mode", na == nb);
  CHECK("same frame bytes regardless of direction mode",
        na == nb && memcmp(S.txbuf + a0, S2.txbuf + b0, na) == 0);
  dirlog_reset();
  other.ping(1, &model);
  CHECK("NONE mode never touches a GPIO (self-directing transceiver)", g_dir_n == 0);

  printf("\n[11] CALLBACK direction: your own handler drives the transceiver\n");
  static int cb_tx = 0, cb_rx = 0, cb_last = -1;
  struct CB {
    static void go(void* ctx, bool transmit) {
      (void)ctx;
      if (transmit) { cb_tx++; cb_last = 1; } else { cb_rx++; cb_last = 0; }
    }
  };
  static HardwareSerial S3;
  static TinyDXL cbd(&S3, DXL_NO_PIN);
  cbd.setDirectionHandler(&CB::go);
  cbd.begin(4000000);
  CHECK("a handler installed before begin() selects CALLBACK",
        cbd.directionMode() == DXL_DIR_CALLBACK);
  CHECK("begin() leaves the handler in receive", cb_last == 0);
  cb_tx = cb_rx = 0;
  dirlog_reset();
  CHECK("ping over a callback-driven transceiver succeeds", cbd.ping(1, &model));
  CHECK("handler asserted once and released once", cb_tx == 1 && cb_rx == 1);
  CHECK("handler finished in receive", cb_last == 0);
  CHECK("CALLBACK mode touches no GPIO", g_dir_n == 0);

  printf("\n[12] Direction mode resolution degrades safely\n");
  static HardwareSerial S4;
  static TinyDXL nopin(&S4, DXL_NO_PIN);            // no pin, no handler
  nopin.begin(4000000);
  CHECK("no pin and no handler resolves to NONE", nopin.directionMode() == DXL_DIR_NONE);
  static HardwareSerial S5;
  static TinyDXL askhw(&S5, 7, DXL_DIR_HARDWARE);   // explicit request for hardware timing
  askhw.begin(4000000);
#if TINY_DXL_HAS_HW_DIR
  CHECK("an explicit HARDWARE request is honoured where the core supports it",
        askhw.directionMode() == DXL_DIR_HARDWARE);
#else
  CHECK("asking for HARDWARE on a core without it degrades to MANUAL, not silence",
        askhw.directionMode() == DXL_DIR_MANUAL);
#endif
  CHECK("the driver still completes a transaction either way", askhw.ping(1, &model));
#endif  // TINY_DXL_DIR_MODE

  printf("\n%s  codec: %d passed, %d failed\n", FAIL ? "XXXX" : "====", PASS, FAIL);
  return FAIL ? 1 : 0;
}
