// ==== AMC1306 continuous capture via SPI DMA (Arduino-ESP32) ====
// - Triple-buffer SPI DMA (no pauses), CIC3 per BYTE (fast), continuous state
// - Analyzer task (100 ms): mean-removal, rising-ZC with hysteresis, freq & RMS
// - Printer task (10 s): prints last full period CSV + freq_hz + rms_mV
// - Watchdog-friendly; no large stacks; big buffers on heap/PSRAM
//
// Suggested decimation: OSR = 2048 (power-of-two; OSR % 8 == 0 required)

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <float.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
  #include "driver/spi_master.h"
  #include "driver/gpio.h"
  #include "esp_system.h"
  #include "esp_heap_caps.h"
  #include "esp_timer.h"
  #include "esp_log.h"
}

#if __has_include(<esp32-hal-psram.h>)
  #include <esp32-hal-psram.h>
#endif

// ---------- Pins (adjust to your wiring) ----------
#define PIN_SCLK   17   // AMC1306 CLKIN  -> ESP32 SCLK
#define PIN_MISO   18   // AMC1306 DOUT   -> ESP32 MISO
#define PIN_MOSI   -1   // not used (we TX dummy)
#define PIN_CS     -1   // no CS on AMC1306

// ---------- SPI / DMA ----------
#define SPI_HOST_USE      VSPI_HOST
#define DMA_CHAN          1
#define F_CLK_HZ          5000000      // try 10-20 MHz; keep within board limits
#define SPI_MODE          1             // if data looks wrong, try 0

#define CHUNK_BYTES       4096          // per DMA transaction
#define QUEUE_DEPTH       3             // triple-buffer
#define NUM_DMA_BUFS      3

// ---------- Modulator polarity ----------
#define BIT_ONE_IS_POSITIVE  1          // set 0 if your DOUT polarity is inverted

// ---------- CIC / Decimation ----------
#define OSR               2000u         // MUST be multiple of 8 (byte-step CIC)
#define FS_MV             320.0f       // full-scale in mV

// ---------- Analyzer cadence ----------
#define ANALYZE_EVERY_MS  10           // compute freq/RMS every 100 ms
#define PRINT_EVERY_MS    10000         // print CSV every 10 s

// ---------- Decimated ring (heap) ----------
#define DECIM_RING_DEFAULT   8192       // history of decimated samples
#define MAX_SCAN_DEFAULT     4096       // window size for period search
#define MAX_PERIOD_BUF       1024       // max samples to print for one period

#ifndef SPICOMMON_BUSFLAG_MASTER
#define SPICOMMON_BUSFLAG_MASTER 0
#endif

static const char* TAG = "AMC1306";

// ===== CIC3 continuous state =====
typedef struct {
  long long i1, i2, i3;   // integrators
  long long c1, c2, c3;   // comb delays (at decim instants)
  uint16_t  osr_byte_phase; // 0..(OSR/8 - 1)
} cic3_state_t;

static cic3_state_t g_cic;

// ===== Decimated ring in heap/PSRAM =====
static size_t DECIM_RING = DECIM_RING_DEFAULT;
static size_t MAX_SCAN   = MAX_SCAN_DEFAULT;
static float* g_decim_ring = nullptr;
static size_t g_widx = 0;
static size_t g_count = 0;

// Writer: push newest decimated sample
static inline void push_decim(float v) {
  g_decim_ring[g_widx] = v;
  g_widx = (g_widx + 1) % DECIM_RING;
  if (g_count < DECIM_RING) g_count++;
}

// ===== Measured bit-rate (for accurate freq) =====
static double g_bit_rate_bps = (double)F_CLK_HZ; // moving average
static inline void update_bit_rate_bps(uint64_t dt_us) {
  if (dt_us == 0) return;
  double inst = ((double)CHUNK_BYTES * 8.0) / ((double)dt_us * 1e-6);
  // light smoothing
  g_bit_rate_bps = 0.9 * g_bit_rate_bps + 0.1 * inst;
}

// ===== Byte-step CIC LUT =====
typedef struct { int8_t S1; int16_t PS1; int16_t SS; } bytecic_t;
static bytecic_t g_lut[256];

static void build_byte_cic_lut() {
  for (int b = 0; b < 256; ++b) {
    int s[8];
#if BIT_ONE_IS_POSITIVE
    for (int k = 0; k < 8; ++k) s[k] = ((b >> (7 - k)) & 1) ? +1 : -1;
#else
    for (int k = 0; k < 8; ++k) s[k] = ((b >> (7 - k)) & 1) ? -1 : +1;
#endif
    int S1 = 0, prefix[8];
    for (int k = 0; k < 8; ++k) { S1 += s[k]; prefix[k] = (k ? prefix[k-1] : 0) + s[k]; }
    int PS1 = 0; for (int j = 0; j < 8; ++j) PS1 += prefix[j];
    int SS = 0; for (int m = 0; m < 8; ++m) SS += (9 - (m + 1)) * prefix[m];
    g_lut[b].S1 = (int8_t)S1; g_lut[b].PS1 = (int16_t)PS1; g_lut[b].SS = (int16_t)SS;
  }
}

static inline float cic_to_mV(long long y) {
  const double G = (double)OSR * (double)OSR * (double)OSR;
  return (float)((double)y / G * (double)FS_MV);
}

// ===== Snapshot shared between analyzer and printer =====
typedef struct {
  float   freq_hz;
  float   rms_mV;
  uint16_t n_period;
  // period samples copied here (heap buffer)
} meas_snapshot_t;

static float* g_period_buf = nullptr;  // size MAX_PERIOD_BUF
static meas_snapshot_t g_snap = {NAN, NAN, 0};
static SemaphoreHandle_t g_snap_mutex = nullptr;

// ===== Utilities to copy a window from the ring with stats =====
static size_t copy_recent_window_with_stats(float* dst /*size MAX_SCAN*/,
                                            double& mean, float& vmin, float& vmax) {
  size_t avail = g_count;
  if (avail == 0) return 0;
  size_t N = (MAX_SCAN > avail) ? avail : MAX_SCAN;

  // Take a consistent snapshot of indices
  size_t widx = g_widx;
  size_t start = (widx + DECIM_RING - N) % DECIM_RING;

  double acc = 0.0;
  vmin = +FLT_MAX;
  vmax = -FLT_MAX;
  for (size_t i = 0; i < N; ++i) {
    size_t idx = (start + i) % DECIM_RING;
    float v = g_decim_ring[idx];
    dst[i] = v;
    acc += (double)v;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }
  mean = acc / (double)N;
  return N;
}

// ===== Rising ZC with hysteresis (streaming; tiny stack) =====
static bool find_last_two_rising_zc_hyst(const float* win, size_t N,
                                         double& t0, double& t1, int& zcCount,
                                         float hyst_mV) {
  t0 = t1 = NAN; zcCount = 0;
  if (N < 4) return false;

  bool below = (win[0] <= -hyst_mV);
  double last2 = NAN, last1 = NAN;

  for (size_t i = 1; i < N; ++i) {
    float v = win[i];
    if (below && v >= +hyst_mV) {
      float a = win[i - 1], b = win[i];
      float A = a + hyst_mV, B = b - hyst_mV;
      double denom = (double)B - (double)A;
      double frac  = (denom != 0.0) ? (-(double)A) / denom : 0.0;
      if (frac < 0.0) frac = 0.0; if (frac > 1.0) frac = 1.0;
      double idx = (double)(i - 1) + frac;
      last2 = last1; last1 = idx; ++zcCount; below = false;
    } else if (v <= -hyst_mV) {
      below = true;
    }
  }

  if (zcCount >= 2 && last2 < last1) { t0 = last2; t1 = last1; return true; }
  return false;
}

// ===== SPI DMA sampler with CIC per BYTE =====
typedef struct {
  spi_device_handle_t dev;
  spi_transaction_t   trans[NUM_DMA_BUFS];
  uint8_t*            rxbuf[NUM_DMA_BUFS];
  uint8_t*            txdummy;
} sampler_ctx_t;

static void spi_init_and_start(sampler_ctx_t* s) {
  memset(s, 0, sizeof(*s));
  build_byte_cic_lut();
  memset(&g_cic, 0, sizeof(g_cic));

  // DMA buffers
  for (int i = 0; i < NUM_DMA_BUFS; ++i) {
    s->rxbuf[i] = (uint8_t*)heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!s->rxbuf[i]) { ESP_LOGE(TAG, "RX DMA alloc fail"); abort(); }
  }
  s->txdummy = (uint8_t*)heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!s->txdummy) { ESP_LOGE(TAG, "TX dummy alloc fail"); abort(); }
  memset(s->txdummy, 0xFF, CHUNK_BYTES);

  // SPI bus
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num     = PIN_MOSI;
  buscfg.miso_io_num     = PIN_MISO;
  buscfg.sclk_io_num     = PIN_SCLK;
  buscfg.quadwp_io_num   = -1;
  buscfg.quadhd_io_num   = -1;
  buscfg.max_transfer_sz = CHUNK_BYTES;
  buscfg.flags           = SPICOMMON_BUSFLAG_MASTER;
  buscfg.intr_flags      = 0;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_USE, &buscfg, DMA_CHAN));

  // Device
  spi_device_interface_config_t devcfg = {};
  devcfg.mode             = SPI_MODE;
  devcfg.clock_speed_hz   = F_CLK_HZ;
  devcfg.spics_io_num     = PIN_CS;
#ifdef SPI_DEVICE_NO_DUMMY
  devcfg.flags            = SPI_DEVICE_NO_DUMMY;
#else
  devcfg.flags            = 0;
#endif
  devcfg.queue_size       = QUEUE_DEPTH;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USE, &devcfg, &s->dev));

  // Transactions
  for (int i = 0; i < NUM_DMA_BUFS; ++i) {
    spi_transaction_t* t = &s->trans[i];
    memset(t, 0, sizeof(*t));
    t->length    = CHUNK_BYTES * 8;
    t->rxlength  = 0;
    t->tx_buffer = s->txdummy;
    t->rx_buffer = s->rxbuf[i];
  }

  for (int i = 0; i < NUM_DMA_BUFS; ++i)
    ESP_ERROR_CHECK(spi_device_queue_trans(s->dev, &s->trans[i], portMAX_DELAY));
}

// Byte-step CIC over one finished RX buffer; also update measured bit-rate
static inline void cic_process_rx_bytes_and_bitrate(const uint8_t* buf, size_t nbytes, uint64_t dt_us) {
  cic3_state_t& c = g_cic;
  const uint16_t BYTES_PER_OSR = (uint16_t)(OSR / 8);

  for (size_t i = 0; i < nbytes; ++i) {
    const bytecic_t bc = g_lut[buf[i]];
    long long p1 = c.i1, p2 = c.i2;

    c.i1 = p1 + (long long)bc.S1;
    c.i2 = p2 + 8LL * p1 + (long long)bc.PS1;
    c.i3 = c.i3 + 8LL * p2 + 36LL * p1 + (long long)bc.SS;

    if (++c.osr_byte_phase >= BYTES_PER_OSR) {
      c.osr_byte_phase = 0;
      long long y0 = c.i3;
      long long d1 = y0 - c.c1; c.c1 = y0;
      long long d2 = d1 - c.c2; c.c2 = d1;
      long long d3 = d2 - c.c3; c.c3 = d2;
      push_decim(cic_to_mV(d3));
    }
  }

  update_bit_rate_bps(dt_us);
}

// --- tolerant fractional rising ZC with hysteresis ---
static inline double frac_rising_zc_hyst(float a, float b, float h) {
  const double A = (double)a + (double)h;
  const double B = (double)b - (double)h;
  const double denom = B - A;
  if (denom == 0.0) return 0.5;        // flat line between samples
  double f = -(A) / denom;              // ideal [0,1]
  // Nudge off exact endpoints so partial dt isn't 0
  if (f <= 0.0) f = 1e-6;
  else if (f >= 1.0) f = 1.0 - 1e-6;
  return f;
}

// --- exact RMS over one period using piecewise-linear y(t) (tolerant to edge cases) ---
static float compute_rms_one_period_exact(const float* win, size_t N,
                                          double t0, double t1, double fs_decim)
{
  if (!(fs_decim > 0.0) || !(t1 > t0)) return NAN;

  const double Ts = 1.0 / fs_decim;
  int i0 = (int)floor(t0);
  int i1 = (int)floor(t1);

  if (i0 < 0) i0 = 0;
  if (i1 < 0) return NAN;
  if (i0 + 1 >= (int)N || i1 >= (int)N) return NAN;

  auto seg_int_y2 = [](double y0, double y1, double dt) -> double {
    return (dt > 0.0) ? dt * ((y0*y0 + y0*y1 + y1*y1) / 3.0) : 0.0;
  };

  double integral = 0.0;

  // first partial: t0 -> (i0+1)
  {
    const double y_end = (double)win[i0 + 1];
    const double dt    = ((double)(i0 + 1) - t0) * Ts;
    integral += seg_int_y2(0.0, y_end, dt);
  }

  // full segments
  for (int k = i0 + 1; k <= i1 - 1; ++k) {
    if (k + 1 >= (int)N) break;
    integral += seg_int_y2((double)win[k], (double)win[k + 1], Ts);
  }

  // last partial: i1 -> t1
  {
    const double y_start = (double)win[i1];
    const double dt      = (t1 - (double)i1) * Ts;
    integral += seg_int_y2(y_start, 0.0, dt);
  }

  const double T = (t1 - t0) * Ts;
  if (!(T > 0.0)) return NAN;

  double ms = integral / T;
  if (ms < 0.0) ms = 0.0;
  return (float)sqrt(ms);
}



// Sampler task: process finished buffer then immediately re-queue it
static void spi_sampler_task(void* arg) {
  sampler_ctx_t* s = (sampler_ctx_t*)arg;
  uint64_t last_us = esp_timer_get_time();

  while (true) {
    spi_transaction_t* rtrans = nullptr;
    esp_err_t e = spi_device_get_trans_result(s->dev, &rtrans, portMAX_DELAY);
    if (e != ESP_OK) { ESP_LOGE(TAG, "get_trans_result err=%d", e); continue; }

    uint64_t now_us = esp_timer_get_time();
    uint64_t dt_us  = now_us - last_us;
    last_us = now_us;

    cic_process_rx_bytes_and_bitrate((const uint8_t*)rtrans->rx_buffer, CHUNK_BYTES, dt_us);
    ESP_ERROR_CHECK(spi_device_queue_trans(s->dev, rtrans, portMAX_DELAY));
    taskYIELD();
  }
}

// ===== Analyzer task: compute freq/RMS & capture last period (no printing) =====
static float* g_win = nullptr;  // size MAX_SCAN

static void analyzer_task(void* arg) {
  (void)arg;
  uint32_t last_ms = 0;
  while (true) {
    uint32_t now = millis();
    if ((uint32_t)(now - last_ms) >= ANALYZE_EVERY_MS) {
      last_ms = now;

      if (!g_win || g_count < 32) { vTaskDelay(1); continue; }

      double mean = 0.0; float vmin = 0.0f, vmax = 0.0f;
      size_t N = copy_recent_window_with_stats(g_win, mean, vmin, vmax);
      if (N < 32) { vTaskDelay(1); continue; }

      // Mean removal
      for (size_t i = 0; i < N; ++i) g_win[i] = (float)((double)g_win[i] - mean);

      // Hysteresis: 2% pp, clamped
      float pp = vmax - vmin;
      float hyst = max(2.0f, min(0.02f * pp, 50.0f));

      double t0, t1; int zcCount = 0;
      meas_snapshot_t snap = {NAN, NAN, 0};

      if (find_last_two_rising_zc_hyst(g_win, N, t0, t1, zcCount, hyst)) {
        int s = (int)floor(t0);
        int e = (int)floor(t1);
        if (s < 0) s = 0;
        if (e >= (int)N) e = (int)N - 1;

        if (e > s) {
          const double fs_decim = g_bit_rate_bps / (double)OSR;
          const double spp = (t1 - t0);
          snap.freq_hz = (float)((spp > 0.0) ? (fs_decim / spp) : NAN);

          // Copy integer samples for CSV (visualization only)
        uint16_t count = (uint16_t)min((int)MAX_PERIOD_BUF, e - s + 1);
        for (uint16_t i = 0; i < count; ++i) {
        g_period_buf[i] = g_win[s + i];
        }
        snap.n_period = count;

        // Compute RMS over the exact one-period [t0, t1] with linearized endpoints
        const double fs_decim_temp = g_bit_rate_bps / (double)OSR;   // measured Fs
        snap.rms_mV = compute_rms_one_period_exact(g_win, N, t0, t1, fs_decim_temp);

        }
      }

      // Publish snapshot
      if (xSemaphoreTake(g_snap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_snap = snap; // struct copy
        xSemaphoreGive(g_snap_mutex);
      }
    }
    vTaskDelay(1); // let others run
  }
}

// ===== Printer task: prints every PRINT_EVERY_MS =====
static void printer_task(void* arg) {
  (void)arg;
  uint32_t last_ms = 0;
  while (true) {
    uint32_t now = millis();
    if ((uint32_t)(now - last_ms) >= PRINT_EVERY_MS) {
      last_ms = now;

      meas_snapshot_t snap;
      if (xSemaphoreTake(g_snap_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        snap = g_snap;
        xSemaphoreGive(g_snap_mutex);
      } else {
        snap.n_period = 0;
        snap.freq_hz = NAN;
        snap.rms_mV = NAN;
      }

      // CSV of last period
      Serial.print("period_csv=");
      if (snap.n_period == 0) {
        Serial.println();
      } else {
        for (uint16_t i = 0; i < snap.n_period; ++i) {
          Serial.print(g_period_buf[i], 3);
          Serial.print(',');
          if ((i & 0x1FF) == 0x1FF) vTaskDelay(1); // keep WDT happy on long prints
        }
        Serial.println();
      }

      Serial.print("freq_hz=");
      if (isfinite(snap.freq_hz)) Serial.println(snap.freq_hz, 2);
      else                        Serial.println("nan");

      Serial.print("rms_mV=");
      if (snap.n_period) Serial.println(snap.rms_mV, 3);
      else               Serial.println("nan");
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // chill
  }
}

// ===== Arduino entry =====
static sampler_ctx_t g_sampler;

static void alloc_buffers_or_die() {
  uint32_t caps = MALLOC_CAP_8BIT;
#if defined(BOARD_HAS_PSRAM) || defined(CONFIG_SPIRAM_SUPPORT)
  if (psramFound()) caps |= MALLOC_CAP_SPIRAM;
#endif

  // Decimated ring + windows + period buf (all heap)
  size_t ring = DECIM_RING, scan = MAX_SCAN;
  while (true) {
    g_decim_ring = (float*)heap_caps_malloc(ring * sizeof(float), caps);
    g_win        = (float*)heap_caps_malloc(scan * sizeof(float), caps);
    g_period_buf = (float*)heap_caps_malloc(MAX_PERIOD_BUF * sizeof(float), caps);
    if (g_decim_ring && g_win && g_period_buf) {
      DECIM_RING = ring; MAX_SCAN = scan;
      memset(g_decim_ring, 0, ring * sizeof(float));
      memset(g_win,        0, scan * sizeof(float));
      memset(g_period_buf, 0, MAX_PERIOD_BUF * sizeof(float));
      ESP_LOGI(TAG, "Buffers: ring=%u (%.1f KB), win=%u (%.1f KB), period=%u (%.1f KB), caps=0x%x",
               (unsigned)DECIM_RING, (DECIM_RING*sizeof(float))/1024.0f,
               (unsigned)MAX_SCAN,   (MAX_SCAN*sizeof(float))/1024.0f,
               (unsigned)MAX_PERIOD_BUF, (MAX_PERIOD_BUF*sizeof(float))/1024.0f, caps);
      break;
    }
    if (g_decim_ring) { heap_caps_free(g_decim_ring); g_decim_ring = nullptr; }
    if (g_win)        { heap_caps_free(g_win);        g_win = nullptr; }
    if (g_period_buf) { heap_caps_free(g_period_buf); g_period_buf = nullptr; }
    if (ring <= 2048 || scan <= 1024) {
      Serial.println("FATAL: buffer alloc failed");
      abort();
    }
    ring >>= 1; scan >>= 1;
  }

  g_snap_mutex = xSemaphoreCreateMutex();
  if (!g_snap_mutex) { Serial.println("FATAL: snap mutex alloc failed"); abort(); }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  ESP_LOGI(TAG, "AMC1306: DMA triple-buffer + byte-step CIC + analyzer/print split");

  alloc_buffers_or_die();
  spi_init_and_start(&g_sampler);

  // Sampler: high prio on Core 1
  xTaskCreatePinnedToCore(
      spi_sampler_task, "spi_sampler",
      4096, &g_sampler, configMAX_PRIORITIES - 2, nullptr, 1);

  // Analyzer: medium prio on Core 0
  xTaskCreatePinnedToCore(
      analyzer_task, "analyzer",
      6144, nullptr, 2, nullptr, 0);

  // Printer: low prio on Core 0
  xTaskCreatePinnedToCore(
      printer_task, "printer",
      4096, nullptr, 1, nullptr, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
