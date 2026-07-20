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

  printf("\n%s  codec: %d passed, %d failed\n", FAIL ? "XXXX" : "====", PASS, FAIL);
  return FAIL ? 1 : 0;
}
