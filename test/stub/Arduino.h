// Minimal, STL-free Arduino.h stub so tiny_dxl.h compiles + runs on the host
// (this box has the C toolchain but not libc++). Provides only what tiny_dxl.h
// touches: fixed-width ints, memcpy/memmove, a monotonically advancing micros(),
// and a HardwareSerial backed by an in-memory servo-bus emulator (emu.h).
#pragma once
#include <stdint.h>
#include <string.h>
#include <stddef.h>

// micros(): +1 us per call so bounded deadline loops always terminate.
static uint32_t g_micros = 0;
static inline uint32_t micros() { return ++g_micros; }
static inline void yield() {}   // Arduino cooperative-yield hook (no-op on host)

struct HardwareSerial;
void emu_service(HardwareSerial& s);

struct HardwareSerial {
  uint8_t rxq[4096]; int rxhead = 0, rxtail = 0;   // bytes the bus presents to the master
  uint8_t txbuf[16384]; int txlen = 0;             // full master TX history
  int tx_parsed = 0;                               // how far the emulator has answered

  void begin(uint32_t) {}
  void end() {}
  void transmitterEnable(uint8_t) {}
  int  available() { return rxtail - rxhead; }
  int  read() { return (rxhead >= rxtail) ? -1 : rxq[rxhead++]; }
  void rxpush(uint8_t b) {                          // emulator enqueues a response byte
    if (rxhead == rxtail) { rxhead = 0; rxtail = 0; }   // compact once fully drained
    if (rxtail < (int)sizeof(rxq)) rxq[rxtail++] = b;
  }
  size_t write(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; i++) if (txlen < (int)sizeof(txbuf)) txbuf[txlen++] = p[i];
    return n;
  }
  size_t write(uint8_t b) { if (txlen < (int)sizeof(txbuf)) txbuf[txlen++] = b; return 1; }
  void flush() { emu_service(*this); }             // master finished a packet -> servo answers
};
