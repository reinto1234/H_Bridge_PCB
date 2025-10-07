#pragma once

#include <Arduino.h>
#include <math.h>
#include <float.h>
#include <string.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
  #include "esp_heap_caps.h"
  #include "esp_timer.h"
  #include "esp_log.h"
}

#if __has_include(<esp32-hal-psram.h>)
  #include <esp32-hal-psram.h>
#endif

#ifndef SPICOMMON_BUSFLAG_MASTERMODE
#define SPICOMMON_BUSFLAG_MASTERMODE 0
#endif

// --- Tweakables specific to output measurements (moved here) ---
#ifndef OM_BIT_ONE_IS_POSITIVE
#define OM_BIT_ONE_IS_POSITIVE 1
#endif

// Decimation and full-scale (moved from main)
#ifndef OM_OSR
#define OM_OSR 2000u          // MUST be multiple of 8
#endif

#ifndef OM_FS_MV
#define OM_FS_MV 320.0f
#endif

// Analyzer cadence defaults (kept here so the class can use them if needed)
#ifndef OM_ANALYZE_EVERY_MS
#define OM_ANALYZE_EVERY_MS 1
#endif

// Default buffer sizes for the measurement engine
#ifndef OM_DECIM_RING_DEFAULT
#define OM_DECIM_RING_DEFAULT 256
#endif

#ifndef OM_MAX_SCAN_DEFAULT
#define OM_MAX_SCAN_DEFAULT 128
#endif

#ifndef OM_MAX_PERIOD_BUF
#define OM_MAX_PERIOD_BUF 128
#endif

class OutputMeasurements {
public:
  struct Snapshot {
    float     freq_hz;
    float     rms_mV;
    uint16_t  n_period;
  };

  // Construct with the data-rate you expect initially (e.g., SPI clock) so fs_decim starts sensible.
  OutputMeasurements(uint32_t osr = OM_OSR,
                     float fs_mv = OM_FS_MV,
                     float initial_bit_rate_bps = 0.0f,
                     size_t decim_ring_len = OM_DECIM_RING_DEFAULT,
                     size_t max_scan_len   = OM_MAX_SCAN_DEFAULT,
                     size_t max_period_len = OM_MAX_PERIOD_BUF);

  // Allocates buffers (heap/PSRAM if present) and resets state. Returns false on fatal allocation failure.
  bool init(const char* log_name = "OM");

  // Feed raw RX bytes and update CIC / decimated ring; dt_us is wall time since last call for bitrate smoothing.
  void processRxBytesAndUpdateBitrate(const uint8_t* buf, size_t nbytes, uint64_t dt_us);

  // Perform one analysis step: mean removal, hysteretic rising ZC, freq, exact RMS.
  void analyzeStep();

  // Copy out the latest snapshot (thread-safe via mutex). Returns true if valid (n_period>0 or freq finite).
  bool getSnapshot(Snapshot& out) const;

  // Copy out the most recent one-period waveform captured during analyzeStep into dst (dst_len is in/out).
  // Returns number of samples copied.
  uint16_t copyLastPeriod(float* dst, uint16_t dst_len) const;

  // Performance timing (us) around analyzeStep()
  inline int analyzeStartUs() const { return t_us_start; }
  inline int analyzeEndUs()   const { return t_us_end;   }

  // Accessors
  inline float bitRateBps() const { return bit_rate_bps; }
  inline uint32_t osr()     const { return OSR; }

  // Max samples returned per incremental pull
  static constexpr uint16_t OM_SEQ_MAX_CHUNK = 50;
  inline float    getDecimFsHz() const { return bit_rate_bps / (float)OSR; }
  inline uint32_t getSeq32() const { return decim_seq32; }

  // Copy up to the last 50 *newest* samples produced since `last_seq` into `dst`.
  // Returns #copied and sets *next_seq = current producer sequence (i.e., "now").
  // Order: newest-first (dst[0] is the newest).
  uint16_t copySinceSeq32(uint32_t last_seq, float* dst, uint16_t dst_len, uint32_t* next_seq) const;


private:
  // ----- CIC3 continuous state -----
  struct cic3_state_t {
    long long i1 = 0, i2 = 0, i3 = 0;   // integrators
    long long c1 = 0, c2 = 0, c3 = 0;   // comb delays (at decim instants)
    uint16_t  osr_byte_phase = 0;       // 0..(OSR/8 - 1)
  };

  volatile uint32_t decim_seq32 = 0; // ++ in pushDecim() per decimated sample

  // Per-byte CIC LUT entry
  struct bytecic_t { int8_t S1; int16_t PS1; int16_t SS; };

  // Static LUT shared across instances
  static bytecic_t s_lut[256];
  static bool s_lut_built;
  static void buildByteCicLut();

  // Helpers
  static inline float cicToVolts(long long y, uint32_t OSR, float FS_mV) {
    const float G = (float)((double)OSR * (double)OSR * (double)OSR);
    return ((float)y / G) * FS_mV * 1e-3f;
  }

  static inline void updateBitRate(float& smoothed_bps, uint64_t dt_us, size_t nbytes) {
    if (dt_us == 0) return;
    const float inst = ((float)nbytes * 8.0f) / ((float)dt_us * 1e-6f);
    smoothed_bps = 0.9f * smoothed_bps + 0.1f * inst;
  }

  inline void pushDecim(float v) {
    decim_ring[widx] = v;
    widx = (widx + 1) % DECIM_RING;
    if (count < DECIM_RING) count++;
    __atomic_fetch_add(&decim_seq32, 1u, __ATOMIC_RELEASE);
  }

  static size_t copyRecentWindowWithStats(
      const float* ring, size_t ring_len, size_t widx, size_t count, size_t max_scan,
      float* dst, float& mean, float& vmin, float& vmax);

  static bool findLastTwoRisingZcHyst(const float* win, size_t N,
                                      float& t0, float& t1, int& zcCount,
                                      float hyst_mV);

  static inline float fracRisingZcHyst(float a, float b, float h) {
    const float A = a + h;
    const float B = b - h;
    const float denom = B - A;
    if (denom == 0.0f) return 0.5f;
    float f = -(A) / denom;
    if (f <= 0.0f) f = 1e-6f;
    else if (f >= 1.0f) f = 1.0f - 1e-6f;
    return f;
  }

  static float computeRmsOnePeriodExact(const float* win, size_t N,
                                        float t0, float t1, float fs_decim);

private:
  // Config
  uint32_t OSR;
  float    FS_mV;

  // CIC + bitrate
  cic3_state_t cic;
  float        bit_rate_bps;

  // Decimated ring
  size_t DECIM_RING;
  size_t MAX_SCAN;
  size_t MAX_PERIOD;

  float* decim_ring = nullptr;
  float* win        = nullptr;
  float* period_buf = nullptr;

  size_t widx  = 0;
  size_t count = 0;

  // Snapshot
  Snapshot snap {NAN, NAN, 0};
  mutable SemaphoreHandle_t snap_mutex = nullptr;

  // Perf timing
  volatile int t_us_start = 0;
  volatile int t_us_end   = 0;

  // Logging tag
  const char* log_tag = "OM";
};
