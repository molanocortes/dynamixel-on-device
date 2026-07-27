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

// ---- GPIO stub, instrumented -------------------------------------------
// The half-duplex direction pin is the one platform-specific thing in the
// driver, so the tests need to SEE it, not just trust it. Every level change
// is logged against the master's TX byte count, which is what lets a test
// assert "DE went high before the first byte and low after the last one" and
// catch the two failure modes that actually destroy a bus: releasing early
// (truncated frame) and releasing late (fighting the servo's reply).
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

struct DirEvent { uint8_t pin; uint8_t level; int txlen; };
static DirEvent g_dir_log[256];
static int g_dir_n = 0;
static uint8_t g_pin_level[64] = {0};
static uint8_t g_pin_mode[64] = {0};
static int g_tx_witness = 0;              // set by HardwareSerial::write()

static inline void dirlog_reset() { g_dir_n = 0; }
static inline void pinMode(uint8_t p, uint8_t m) { if (p < 64) g_pin_mode[p] = m; }
static inline void digitalWrite(uint8_t p, uint8_t v) {
  if (p < 64) g_pin_level[p] = v;
  if (g_dir_n < 256) g_dir_log[g_dir_n++] = DirEvent{p, v, g_tx_witness};
}
static inline int digitalRead(uint8_t p) { return (p < 64) ? g_pin_level[p] : 0; }

struct HardwareSerial;
void emu_service(HardwareSerial& s);

struct HardwareSerial {
  uint8_t rxq[4096]; int rxhead = 0, rxtail = 0;   // bytes the bus presents to the master
  uint8_t txbuf[16384]; int txlen = 0;             // full master TX history
  int tx_parsed = 0;                               // how far the emulator has answered

  void begin(uint32_t) {}
  void end() {}
  // Teensy's hardware direction hook. Counted so the tests can prove the
  // HARDWARE path really hands the pin to the UART instead of toggling GPIO.
  int te_calls = 0; int te_pin = -1;
  void transmitterEnable(uint8_t p) { te_calls++; te_pin = p; }
  int  available() { return rxtail - rxhead; }
  int  read() { return (rxhead >= rxtail) ? -1 : rxq[rxhead++]; }
  void rxpush(uint8_t b) {                          // emulator enqueues a response byte
    if (rxhead == rxtail) { rxhead = 0; rxtail = 0; }   // compact once fully drained
    if (rxtail < (int)sizeof(rxq)) rxq[rxtail++] = b;
  }
  size_t write(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; i++) if (txlen < (int)sizeof(txbuf)) txbuf[txlen++] = p[i];
    g_tx_witness = txlen;
    return n;
  }
  size_t write(uint8_t b) {
    if (txlen < (int)sizeof(txbuf)) txbuf[txlen++] = b;
    g_tx_witness = txlen;
    return 1;
  }
  void flush() { emu_service(*this); }             // master finished a packet -> servo answers
};
