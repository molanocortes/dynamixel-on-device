// STL-free Protocol-2.0 DYNAMIXEL bus emulator for host-side codec testing.
// Destuffs the master's instruction like a real servo, then enqueues the correct
// status frames: byte-STUFFED params on READ / SYNC READ (0x02/0x82) so the
// driver's destuffer is exercised, and an UNSTUFFED single concatenated frame on
// FAST SYNC READ (0x8A), exactly as Protocol 2.0 / DynamixelSDK define it.
// CRC-16 is an independent copy of the table: a passing test proves both sides.
#pragma once
#include "Arduino.h"

static const uint16_t EMU_TBL[256] = {
 0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
 0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
 0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
 0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
 0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
 0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
 0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
 0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
 0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
 0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
 0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
 0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
 0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
 0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
 0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
 0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202
};
static uint16_t emu_crc(const uint8_t* d, int n) {
  uint16_t crc = 0;
  for (int i = 0; i < n; i++) { uint16_t idx = ((crc >> 8) ^ d[i]) & 0xFF; crc = (crc << 8) ^ EMU_TBL[idx]; }
  return crc;
}
static int emu_stuff(const uint8_t* in, int n, uint8_t* out) {
  int m = 0, ff = 0;
  for (int i = 0; i < n; i++) { uint8_t b = in[i]; out[m++] = b;
    if (ff >= 2 && b == 0xFD) { out[m++] = 0xFD; ff = 0; } else ff = (b == 0xFF) ? ff + 1 : 0; }
  return m;
}
static int emu_destuff(const uint8_t* in, int n, uint8_t* out) {
  int m = 0, ff = 0;
  for (int i = 0; i < n; i++) { uint8_t b = in[i]; out[m++] = b;
    if (ff >= 2 && b == 0xFD) { if (i + 1 < n && in[i+1] == 0xFD) { i++; ff = 0; continue; } }
    ff = (b == 0xFF) ? ff + 1 : 0; }
  return m;
}

// per-id register file + status-error slot
static uint8_t EMU_REG[8][320];
static uint8_t EMU_ERR[8];
static bool EMU_CORRUPT_CRC = false;     // test hook: mangle the overall CRC
static bool EMU_DROP_ONE_BYTE = false;   // test hook: truncate the frame
static int  EMU_SWAP_IDS = 0;            // test hook: emit ids out of order (fast path)
static bool EMU_BREAK_INDIRECT = false;  // test hook: indirect-address reads mismatch (force fallback)
static uint8_t EMU_ABSENT_ID = 255;      // test hook: this id never answers (missing motor); 255 = none
static inline uint8_t emu_reg(uint8_t id, uint16_t a) { return (id < 8 && a < 320) ? EMU_REG[id][a] : 0; }

static void emu_frame_push(HardwareSerial& s, uint8_t* f, int flen) {
  uint16_t crc = emu_crc(f, flen);
  if (EMU_CORRUPT_CRC) crc ^= 0xFFFF;
  f[flen++] = crc & 0xFF; f[flen++] = crc >> 8;
  if (EMU_DROP_ONE_BYTE) flen--;                 // truncate: driver must reject
  for (int i = 0; i < flen; i++) s.rxpush(f[i]);
}

static void emu_push_status(HardwareSerial& s, uint8_t id, uint8_t err,
                            const uint8_t* params, int np, bool stuffParams) {
  uint8_t sp[600]; int spn = np;
  if (stuffParams) spn = emu_stuff(params, np, sp); else memcpy(sp, params, np);
  uint16_t len = (uint16_t)(2 + spn + 2);        // instr + err + params + crc
  uint8_t f[700]; int k = 0;
  f[k++]=0xFF; f[k++]=0xFF; f[k++]=0xFD; f[k++]=0x00; f[k++]=id;
  f[k++]=len & 0xFF; f[k++]=len >> 8; f[k++]=0x55; f[k++]=err;
  for (int i = 0; i < spn; i++) f[k++] = sp[i];
  emu_frame_push(s, f, k);
}

static void emu_push_fast(HardwareSerial& s, const uint8_t* ids, int nids,
                          uint16_t addr, uint8_t len) {
  uint8_t order[8]; memcpy(order, ids, nids);
  if (EMU_SWAP_IDS && nids >= 2) { uint8_t t = order[0]; order[0] = order[1]; order[1] = t; }
  uint8_t f[900]; int k = 0;
  f[k++]=0xFF; f[k++]=0xFF; f[k++]=0xFD; f[k++]=0x00; f[k++]=0xFE;
  int lenpos = k; k += 2; f[k++] = 0x55;
  for (int d = 0; d < nids; d++) {
    uint8_t id = order[d];
    uint8_t blk[40]; int b = 0;
    blk[b++] = EMU_ERR[id & 7]; blk[b++] = id;
    for (uint8_t j = 0; j < len; j++) blk[b++] = emu_reg(id, addr + j);
    uint16_t dcrc = emu_crc(blk, b);             // per-device CRC over [err,id,data]
    for (int i = 0; i < b; i++) f[k++] = blk[i];
    f[k++] = dcrc & 0xFF; f[k++] = dcrc >> 8;
  }
  uint16_t len16 = (uint16_t)(k - 7) + 2;        // bytes after LEN, + overall crc
  f[lenpos] = len16 & 0xFF; f[lenpos+1] = len16 >> 8;
  emu_frame_push(s, f, k);
}

void emu_service(HardwareSerial& s) {
  const uint8_t* t = s.txbuf;
  int i = s.tx_parsed;
  if (s.txlen - i < 10) { s.tx_parsed = s.txlen; return; }
  if (!(t[i]==0xFF && t[i+1]==0xFF && t[i+2]==0xFD && t[i+3]==0x00)) { s.tx_parsed = s.txlen; return; }
  uint8_t id = t[i+4];
  uint16_t len = t[i+5] | (t[i+6] << 8);
  int end = i + 7 + len;
  uint8_t instr = t[i+7];
  uint8_t prm[600]; int pn = emu_destuff(&t[i+8], (end - 2) - (i + 8), prm);
  s.tx_parsed = end;

  if (id == EMU_ABSENT_ID && id != 0xFE) return;        // absent servo: silence -> master times out

  if (instr == 0x01) {                                  // PING -> model(2)+fw(1); 1230 = 0x04CE
    uint8_t p[3] = {0xCE, 0x04, 0x2A}; emu_push_status(s, id, 0, p, 3, false);
  } else if (instr == 0x03) {                           // WRITE addr(2)+data
    uint16_t a = prm[0] | (prm[1] << 8);
    for (int k = 2; k < pn; k++) if (id < 8 && a + (k-2) < 320) EMU_REG[id][a + (k-2)] = prm[k];
    emu_push_status(s, id, 0, 0, 0, false);
  } else if (instr == 0x02) {                           // READ addr(2)+len(2)
    uint16_t a = prm[0] | (prm[1] << 8), n = prm[2] | (prm[3] << 8);
    uint8_t data[300]; for (uint16_t k = 0; k < n; k++) data[k] = emu_reg(id, a + k);
    if (EMU_BREAK_INDIRECT && a >= 168 && a < 224)       // force indirect read-back mismatch
      for (uint16_t k = 0; k < n; k++) data[k] = 0xFF;
    emu_push_status(s, id, 0, data, n, true);
  } else if (instr == 0x82) {                           // SYNC READ -> per-id stuffed status
    uint16_t a = prm[0] | (prm[1] << 8), n = prm[2] | (prm[3] << 8);
    for (int k = 4; k < pn; k++) {
      uint8_t sid = prm[k];
      uint8_t data[300]; for (uint16_t j = 0; j < n; j++) data[j] = emu_reg(sid, a + j);
      emu_push_status(s, sid, EMU_ERR[sid & 7], data, n, true);
    }
  } else if (instr == 0x8A) {                           // FAST SYNC READ -> one unstuffed frame
    uint16_t a = prm[0] | (prm[1] << 8), n = prm[2] | (prm[3] << 8);
    emu_push_fast(s, &prm[4], pn - 4, a, (uint8_t)n);
  }
  // SYNC WRITE 0x83 (broadcast): no status.
}
