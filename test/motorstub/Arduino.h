// Full-enough Teensy 4.1 Arduino stub to COMPILE + RUN the real bringup_12ch.ino
// on the host against an emulated servo bus. Single translation unit only.
#pragma once
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

// ---- time -------------------------------------------------------------------
static uint32_t g_micros = 0;
void emu_tick_traffic();                 // test hook: injects "another master" bus traffic
static inline uint32_t micros() { ++g_micros; emu_tick_traffic(); return g_micros; }
static inline uint32_t millis() { return g_micros / 1000; }
static inline void delay(uint32_t ms) { g_micros += ms * 1000; }
static inline void delayMicroseconds(uint32_t us) { g_micros += us; }
static inline void yield() {}

// ---- pins / analog (no-op on host) ------------------------------------------
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LSBFIRST 0
#define MSBFIRST 1
static inline void pinMode(uint8_t, uint8_t) {}
static inline void digitalWrite(uint8_t, uint8_t) {}
static inline int  digitalRead(uint8_t) { return 1; }
static inline int  analogRead(uint8_t) { return 512; }
static inline void analogReadResolution(uint8_t) {}
static inline void analogReadAveraging(uint8_t) {}

// ---- math / helpers ---------------------------------------------------------
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif
#define F(x) (x)
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
template <class A, class B> static inline A tmin(A a, B b) { return a < (A)b ? a : (A)b; }
template <class A, class B> static inline A tmax(A a, B b) { return a > (A)b ? a : (A)b; }
#define min(a, b) tmin(a, b)
#define max(a, b) tmax(a, b)

// ---- HardwareSerial: emulated half-duplex servo bus (emu_service in emu.h) ---
struct HardwareSerial;
void emu_service(HardwareSerial& s);
struct HardwareSerial {
  uint8_t rxq[4096]; int rxhead = 0, rxtail = 0;
  uint8_t txbuf[16384]; int txlen = 0;
  int tx_parsed = 0;
  void begin(uint32_t) {}
  void end() {}
  void transmitterEnable(uint8_t) {}
  int  available() { return rxtail - rxhead; }
  int  read() { return (rxhead >= rxtail) ? -1 : rxq[rxhead++]; }
  void rxpush(uint8_t b) { if (rxhead == rxtail) { rxhead = 0; rxtail = 0; } if (rxtail < (int)sizeof(rxq)) rxq[rxtail++] = b; }
  size_t write(const uint8_t* p, size_t n) { for (size_t i = 0; i < n; i++) if (txlen < (int)sizeof(txbuf)) txbuf[txlen++] = p[i]; return n; }
  size_t write(uint8_t b) { if (txlen < (int)sizeof(txbuf)) txbuf[txlen++] = b; return 1; }
  void flush() { emu_service(*this); }
};

// ---- Serial console (stdout for output; injectable input for loop() tests) --
struct SerialConsole {
  const char* inbuf = nullptr; int inpos = 0, inlen = 0;
  void feed(const char* s) { inbuf = s; inpos = 0; inlen = (int)strlen(s); }
  void begin(uint32_t) {}
  operator bool() const { return true; }
  int available() { return inbuf ? (inlen - inpos) : 0; }
  int read() { return (inbuf && inpos < inlen) ? inbuf[inpos++] : -1; }
  void print(const char* s) { fputs(s, stdout); }
  void print(char c) { fputc(c, stdout); }
  void print(int v) { printf("%d", v); }
  void print(unsigned v) { printf("%u", v); }
  void print(long v) { printf("%ld", v); }
  void print(unsigned long v) { printf("%lu", v); }
  void print(double v) { printf("%g", v); }
  void print(double v, int d) { printf("%.*f", d, v); }
  void println() { fputc('\n', stdout); }
  void println(const char* s) { fputs(s, stdout); fputc('\n', stdout); }
  int  printf(const char* f, ...) { va_list ap; va_start(ap, f); int r = vprintf(f, ap); va_end(ap); return r; }
};

static SerialConsole Serial;
static HardwareSerial Serial1;   // the emulated Dynamixel bus (dxl talks to this)

// Test hook: when g_bus_traffic is set, another master appears to be driving DATA,
// so bytes trickle into the RX line as time advances (micros() calls this). This
// lets busQuiet() observe a busy bus and lets motorTake() refuse (single-master).
static bool g_bus_traffic = false;
inline void emu_tick_traffic() { if (g_bus_traffic && (g_micros % 500u) == 0u) Serial1.rxpush(0xAB); }
