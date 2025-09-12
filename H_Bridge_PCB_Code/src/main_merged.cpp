// // ==== AMC1306 continuous capture via SPI DMA (Arduino-ESP32) ====
// // - Dual-channel: HSPI (CH1) + VSPI (CH2), each with triple-buffer DMA
// // - CIC3 per BYTE (fast), continuous state
// // - Analyzer tasks (per channel): mean-removal, rising-ZC with hysteresis, freq & RMS
// // - Printer task (10 s): prints last full period CSV + freq_hz + rms_mV per channel
// // - Watchdog-friendly; no large stacks; big buffers on heap/PSRAM

// #include <Arduino.h>
// #include <math.h>
// #include <string.h>
// #include <float.h>

// extern "C" {
//   #include "freertos/FreeRTOS.h"
//   #include "freertos/task.h"
//   #include "freertos/semphr.h"
//   #include "driver/spi_master.h"
//   #include "driver/gpio.h"
//   #include "esp_system.h"
//   #include "esp_heap_caps.h"
//   #include "esp_timer.h"
//   #include "esp_log.h"
// }

// #if __has_include(<esp32-hal-psram.h>)
//   #include <esp32-hal-psram.h>
// #endif

// // ---------- Pins (Channel 1: your existing wiring) ----------
// #define CH1_PIN_SCLK   17
// #define CH1_PIN_MISO   18
// #define CH1_PIN_MOSI   -1
// #define CH1_PIN_CS     -1

// // ---------- Pins (Channel 2: your second AMC1306) ----------
// #define CH2_PIN_SCLK   25   // AMC1306 #2 CLKIN -> ESP32 SCLK (VSPI)
// #define CH2_PIN_MISO   26   // AMC1306 #2 DOUT  -> ESP32 MISO (VSPI)
// #define CH2_PIN_MOSI   -1
// #define CH2_PIN_CS     -1

// // ---------- SPI / DMA ----------
// #define CH1_SPI_HOST      HSPI_HOST
// #define CH1_DMA_CHAN      1

// #define CH2_SPI_HOST      VSPI_HOST
// #define CH2_DMA_CHAN      2

// #define F_CLK_HZ          5000000      // per host
// #define SPI_MODE          1

// #define CHUNK_BYTES       4096         // per DMA transaction
// #define QUEUE_DEPTH       3            // triple-buffer
// #define NUM_DMA_BUFS      3

// // ---------- Modulator polarity ----------
// #define BIT_ONE_IS_POSITIVE  1

// // ---------- CIC / Decimation ----------
// #define OSR               2000u        // MUST be multiple of 8
// #define FS_MV             320.0f

// // ---------- Analyzer cadence ----------
// #define ANALYZE_EVERY_MS  5
// #define PRINT_EVERY_MS    10000

// // ---------- Decimated ring (heap) ----------
// #define DECIM_RING_DEFAULT   8192
// #define MAX_SCAN_DEFAULT     4096
// #define MAX_PERIOD_BUF       1024

// #ifndef SPICOMMON_BUSFLAG_MASTER
// #define SPICOMMON_BUSFLAG_MASTER 0
// #endif

// static const char* TAG = "AMC1306-DUAL";

// // ===== CIC3 continuous state =====
// typedef struct {
//   long long i1, i2, i3;   // integrators
//   long long c1, c2, c3;   // comb delays (at decim instants)
//   uint16_t  osr_byte_phase; // 0..(OSR/8 - 1)
// } cic3_state_t;

// // ===== Byte-step CIC LUT (shared) =====
// typedef struct { int8_t S1; int16_t PS1; int16_t SS; } bytecic_t;
// static bytecic_t g_lut[256];

// static void build_byte_cic_lut() {
//   for (int b = 0; b < 256; ++b) {
//     int s[8];
// #if BIT_ONE_IS_POSITIVE
//     for (int k = 0; k < 8; ++k) s[k] = ((b >> (7 - k)) & 1) ? +1 : -1;
// #else
//     for (int k = 0; k < 8; ++k) s[k] = ((b >> (7 - k)) & 1) ? -1 : +1;
// #endif
//     int S1 = 0, prefix[8];
//     for (int k = 0; k < 8; ++k) { S1 += s[k]; prefix[k] = (k ? prefix[k-1] : 0) + s[k]; }
//     int PS1 = 0; for (int j = 0; j < 8; ++j) PS1 += prefix[j];
//     int SS  = 0; for (int m = 0; m < 8; ++m) SS  += (9 - (m + 1)) * prefix[m];
//     g_lut[b].S1 = (int8_t)S1; g_lut[b].PS1 = (int16_t)PS1; g_lut[b].SS = (int16_t)SS;
//   }
// }

// static inline float cic_to_mV(long long y) {
//   const float G = (float)((double)OSR * (double)OSR * (double)OSR);
//   return ((float)y / G) * FS_MV;
// }

// // ===== Channel-scoped state =====
// typedef struct {
//   // CIC and bitrate
//   cic3_state_t cic;
//   float bit_rate_bps;

//   // Decimated ring
//   size_t DECIM_RING = DECIM_RING_DEFAULT;
//   size_t MAX_SCAN   = MAX_SCAN_DEFAULT;
//   float* decim_ring = nullptr;
//   size_t widx = 0;
//   size_t count = 0;

//   // Windows / period buffers
//   float* win         = nullptr;  // size MAX_SCAN
//   float* period_buf  = nullptr;  // size MAX_PERIOD_BUF

//   // Snapshots
//   typedef struct {
//     float   freq_hz;
//     float   rms_mV;
//     uint16_t n_period;
//   } meas_snapshot_t;
//   meas_snapshot_t snap = {NAN, NAN, 0};
//   SemaphoreHandle_t snap_mutex = nullptr;

//   // Perf timing
//   volatile int t_us_start = 0;
//   volatile int t_us_end   = 0;
// } chan_t;

// static chan_t ch1, ch2;

// // ===== Sampler (per channel) =====
// typedef struct {
//   spi_device_handle_t dev;
//   spi_transaction_t   trans[NUM_DMA_BUFS];
//   uint8_t*            rxbuf[NUM_DMA_BUFS];
//   uint8_t*            txdummy;
//   // pinning
//   int pin_sclk, pin_miso, pin_mosi, pin_cs;
//   spi_host_device_t host;
//   int dma_chan;
// } sampler_ctx_t;

// static sampler_ctx_t g_sampler1 = { .pin_sclk=CH1_PIN_SCLK, .pin_miso=CH1_PIN_MISO,
//   .pin_mosi=CH1_PIN_MOSI, .pin_cs=CH1_PIN_CS, .host=CH1_SPI_HOST, .dma_chan=CH1_DMA_CHAN };

// static sampler_ctx_t g_sampler2 = { .pin_sclk=CH2_PIN_SCLK, .pin_miso=CH2_PIN_MISO,
//   .pin_mosi=CH2_PIN_MOSI, .pin_cs=CH2_PIN_CS, .host=CH2_SPI_HOST, .dma_chan=CH2_DMA_CHAN };

// // ===== Measured bit-rate update (per channel) =====
// static inline void update_bit_rate_bps(float& smoothed_bps, uint64_t dt_us) {
//   if (dt_us == 0) return;
//   float inst = ((float)CHUNK_BYTES * 8.0f) / ((float)dt_us * 1e-6f);
//   smoothed_bps = 0.9f * smoothed_bps + 0.1f * inst;
// }

// // ===== Ring push (per channel) =====
// static inline void push_decim(chan_t& C, float v) {
//   C.decim_ring[C.widx] = v;
//   C.widx = (C.widx + 1) % C.DECIM_RING;
//   if (C.count < C.DECIM_RING) C.count++;
// }

// // ===== Window copy + stats (per channel) =====
// static size_t copy_recent_window_with_stats(chan_t& C, float* dst, float& mean, float& vmin, float& vmax) {
//   size_t avail = C.count;
//   if (avail == 0) return 0;
//   size_t N = (C.MAX_SCAN > avail) ? avail : C.MAX_SCAN;

//   size_t widx = C.widx;
//   size_t start = (widx + C.DECIM_RING - N) % C.DECIM_RING;

//   float acc = 0.0f;
//   vmin = +FLT_MAX; vmax = -FLT_MAX;
//   for (size_t i = 0; i < N; ++i) {
//     size_t idx = (start + i) % C.DECIM_RING;
//     float v = C.decim_ring[idx];
//     dst[i] = v;
//     acc += v;
//     if (v < vmin) vmin = v;
//     if (v > vmax) vmax = v;
//   }
//   mean = acc / (float)N;
//   return N;
// }

// // ===== Rising ZC with hysteresis (same as your original) =====
// static bool find_last_two_rising_zc_hyst(const float* win, size_t N,
//                                          float& t0, float& t1, int& zcCount,
//                                          float hyst_mV) {
//   t0 = t1 = NAN; zcCount = 0;
//   if (N < 4) return false;
//   bool below = (win[0] <= -hyst_mV);
//   float last2 = NAN, last1 = NAN;
//   for (size_t i = 1; i < N; ++i) {
//     float v = win[i];
//     if (below && v >= +hyst_mV) {
//       float a = win[i - 1], b = win[i];
//       float A = a + hyst_mV, B = b - hyst_mV;
//       float denom = B - A;
//       float frac  = (denom != 0.0f) ? (-(A)) / denom : 0.0f;
//       if (frac < 0.0f) frac = 0.0f; if (frac > 1.0f) frac = 1.0f;
//       float idx = (float)(i - 1) + frac;
//       last2 = last1; last1 = idx; ++zcCount; below = false;
//     } else if (v <= -hyst_mV) {
//       below = true;
//     }
//   }
//   if (zcCount >= 2 && last2 < last1) { t0 = last2; t1 = last1; return true; }
//   return false;
// }

// // --- tolerant fractional rising ZC with hysteresis ---
// static inline float frac_rising_zc_hyst(float a, float b, float h) {
//   const float A = a + h;
//   const float B = b - h;
//   const float denom = B - A;
//   if (denom == 0.0f) return 0.5f;
//   float f = -(A) / denom;
//   if (f <= 0.0f) f = 1e-6f;
//   else if (f >= 1.0f) f = 1.0f - 1e-6f;
//   return f;
// }

// // --- exact RMS over one period using piecewise-linear y(t) ---
// static float compute_rms_one_period_exact(const float* win, size_t N,
//                                           float t0, float t1, float fs_decim)
// {
//   if (!(fs_decim > 0.0f) || !(t1 > t0)) return NAN;
//   const float Ts = 1.0f / fs_decim;
//   int i0 = (int)floorf(t0);
//   int i1 = (int)floorf(t1);
//   if (i0 < 0) i0 = 0;
//   if (i1 < 0) return NAN;
//   if (i0 + 1 >= (int)N || i1 >= (int)N) return NAN;

//   auto seg_int_y2 = [](float y0, float y1, float dt) -> float {
//     return (dt > 0.0f) ? dt * ((y0*y0 + y0*y1 + y1*y1) / 3.0f) : 0.0f;
//   };

//   float integral = 0.0f;
//   { // first partial
//     const float y_end = win[i0 + 1];
//     const float dt    = ((float)(i0 + 1) - t0) * Ts;
//     integral += seg_int_y2(0.0f, y_end, dt);
//   }
//   for (int k = i0 + 1; k <= i1 - 1; ++k) {
//     if (k + 1 >= (int)N) break;
//     integral += seg_int_y2(win[k], win[k + 1], Ts);
//   }
//   { // last partial
//     const float y_start = win[i1];
//     const float dt      = (t1 - (float)i1) * Ts;
//     integral += seg_int_y2(y_start, 0.0f, dt);
//   }

//   const float T = (t1 - t0) * Ts;
//   if (!(T > 0.0f)) return NAN;
//   float ms = integral / T;
//   if (ms < 0.0f) ms = 0.0f;
//   return sqrtf(ms);
// }

// // ===== SPI init/start (per sampler) =====
// static void spi_init_and_start(sampler_ctx_t* s) {
//   // DMA buffers
//   for (int i = 0; i < NUM_DMA_BUFS; ++i) {
//     s->rxbuf[i] = (uint8_t*)heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
//     if (!s->rxbuf[i]) { ESP_LOGE(TAG, "RX DMA alloc fail"); abort(); }
//   }
//   s->txdummy = (uint8_t*)heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
//   if (!s->txdummy) { ESP_LOGE(TAG, "TX dummy alloc fail"); abort(); }
//   memset(s->txdummy, 0xFF, CHUNK_BYTES);

//   // SPI bus
//   spi_bus_config_t buscfg = {};
//   buscfg.mosi_io_num     = s->pin_mosi;
//   buscfg.miso_io_num     = s->pin_miso;
//   buscfg.sclk_io_num     = s->pin_sclk;
//   buscfg.quadwp_io_num   = -1;
//   buscfg.quadhd_io_num   = -1;
//   buscfg.max_transfer_sz = CHUNK_BYTES;
//   buscfg.flags           = SPICOMMON_BUSFLAG_MASTER;
//   buscfg.intr_flags      = 0;
//   ESP_ERROR_CHECK(spi_bus_initialize(s->host, &buscfg, s->dma_chan));

//   // Device
//   spi_device_interface_config_t devcfg = {};
//   devcfg.mode             = SPI_MODE;
//   devcfg.clock_speed_hz   = F_CLK_HZ;
//   devcfg.spics_io_num     = s->pin_cs;
// #ifdef SPI_DEVICE_NO_DUMMY
//   devcfg.flags            = SPI_DEVICE_NO_DUMMY;
// #else
//   devcfg.flags            = 0;
// #endif
//   devcfg.queue_size       = QUEUE_DEPTH;
//   ESP_ERROR_CHECK(spi_bus_add_device(s->host, &devcfg, &s->dev));

//   // Transactions
//   for (int i = 0; i < NUM_DMA_BUFS; ++i) {
//     spi_transaction_t* t = &s->trans[i];
//     memset(t, 0, sizeof(*t));
//     t->length    = CHUNK_BYTES * 8;
//     t->rxlength  = 0;
//     t->tx_buffer = s->txdummy;
//     t->rx_buffer = s->rxbuf[i];
//   }

//   for (int i = 0; i < NUM_DMA_BUFS; ++i)
//     ESP_ERROR_CHECK(spi_device_queue_trans(s->dev, &s->trans[i], portMAX_DELAY));
// }

// // ===== CIC process one RX buffer (per channel) =====
// static inline void cic_process_rx_bytes_and_bitrate(chan_t& C, const uint8_t* buf, size_t nbytes, uint64_t dt_us) {
//   cic3_state_t& c = C.cic;
//   const uint16_t BYTES_PER_OSR = (uint16_t)(OSR / 8);

//   for (size_t i = 0; i < nbytes; ++i) {
//     const bytecic_t bc = g_lut[buf[i]];
//     long long p1 = c.i1, p2 = c.i2;

//     c.i1 = p1 + (long long)bc.S1;
//     c.i2 = p2 + 8LL * p1 + (long long)bc.PS1;
//     c.i3 = c.i3 + 8LL * p2 + 36LL * p1 + (long long)bc.SS;

//     if (++c.osr_byte_phase >= BYTES_PER_OSR) {
//       c.osr_byte_phase = 0;
//       long long y0 = c.i3;
//       long long d1 = y0 - c.c1; c.c1 = y0;
//       long long d2 = d1 - c.c2; c.c2 = d1;
//       long long d3 = d2 - c.c3; c.c3 = d2;
//       push_decim(C, cic_to_mV(d3));
//     }
//   }
//   update_bit_rate_bps(C.bit_rate_bps, dt_us);
// }

// // ===== Sampler tasks (one per SPI host) =====
// static void spi_sampler_task(void* arg) {
//   auto* pack = (std::pair<sampler_ctx_t*, chan_t*>*)arg;
//   sampler_ctx_t* s = pack->first;
//   chan_t* C = pack->second;

//   uint64_t last_us = esp_timer_get_time();
//   while (true) {
//     spi_transaction_t* rtrans = nullptr;
//     esp_err_t e = spi_device_get_trans_result(s->dev, &rtrans, portMAX_DELAY);
//     if (e != ESP_OK) { ESP_LOGE(TAG, "get_trans_result err=%d", e); continue; }

//     uint64_t now_us = esp_timer_get_time();
//     uint64_t dt_us  = now_us - last_us;
//     last_us = now_us;

//     cic_process_rx_bytes_and_bitrate(*C, (const uint8_t*)rtrans->rx_buffer, CHUNK_BYTES, dt_us);
//     ESP_ERROR_CHECK(spi_device_queue_trans(s->dev, rtrans, portMAX_DELAY));
//     taskYIELD();
//   }
// }

// // ===== Analyzer task (per channel) =====
// static void analyzer_task(void* arg) {
//   chan_t* C = (chan_t*)arg;
//   uint32_t last_ms = 0;
//   while (true) {
//     uint32_t now = millis();
//     if ((uint32_t)(now - last_ms) >= ANALYZE_EVERY_MS) {
//       last_ms = now;
//       C->t_us_start = micros();

//       if (!C->win || C->count < 32) { vTaskDelay(1); continue; }

//       float mean = 0.0f; float vmin = 0.0f, vmax = 0.0f;
//       size_t N = copy_recent_window_with_stats(*C, C->win, mean, vmin, vmax);
//       if (N < 32) { vTaskDelay(1); continue; }

//       for (size_t i = 0; i < N; ++i) C->win[i] = C->win[i] - mean;

//       float pp = vmax - vmin;
//       float hyst = max(2.0f, min(0.02f * pp, 50.0f));

//       float t0, t1; int zcCount = 0;
//       chan_t::meas_snapshot_t snap = {NAN, NAN, 0};

//       if (find_last_two_rising_zc_hyst(C->win, N, t0, t1, zcCount, hyst)) {
//         int s = (int)floorf(t0);
//         int e = (int)floorf(t1);
//         if (s < 0) s = 0;
//         if (e >= (int)N) e = (int)N - 1;

//         if (e > s) {
//           const float fs_decim = C->bit_rate_bps / (float)OSR;
//           const float spp = (t1 - t0);
//           snap.freq_hz = (spp > 0.0f) ? (fs_decim / spp) : NAN;

//           uint16_t count = (uint16_t)min((int)MAX_PERIOD_BUF, e - s + 1);
//           for (uint16_t i = 0; i < count; ++i) C->period_buf[i] = C->win[s + i];
//           snap.n_period = count;

//           snap.rms_mV = compute_rms_one_period_exact(C->win, N, t0, t1, fs_decim);
//         }
//       }

//       if (xSemaphoreTake(C->snap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
//         C->snap = snap;
//         xSemaphoreGive(C->snap_mutex);
//       }
//       C->t_us_end = micros();
//     }
//     vTaskDelay(1);
//   }
// }

// // ===== Printer task (prints both channels) =====
// static void printer_task(void* arg) {
//   (void)arg;
//   uint32_t last_ms = 0;
//   while (true) {
//     uint32_t now = millis();
//     if ((uint32_t)(now - last_ms) >= PRINT_EVERY_MS) {
//       last_ms = now;

//       chan_t::meas_snapshot_t s1, s2;
//       if (xSemaphoreTake(ch1.snap_mutex, pdMS_TO_TICKS(50)) == pdTRUE) { s1 = ch1.snap; xSemaphoreGive(ch1.snap_mutex); }
//       else { s1 = {NAN, NAN, 0}; }
//       if (xSemaphoreTake(ch2.snap_mutex, pdMS_TO_TICKS(50)) == pdTRUE) { s2 = ch2.snap; xSemaphoreGive(ch2.snap_mutex); }
//       else { s2 = {NAN, NAN, 0}; }

//       // CH1
//       Serial.println("ch1_analyzer_dt_us=" + String(ch1.t_us_end - ch1.t_us_start));
//       Serial.print("ch1_period_csv=");
//       if (s1.n_period == 0) Serial.println();
//       else {
//         for (uint16_t i = 0; i < s1.n_period; ++i) {
//           Serial.print(ch1.period_buf[i], 3); Serial.print(',');
//           if ((i & 0x1FF) == 0x1FF) vTaskDelay(1);
//         }
//         Serial.println();
//       }
//       Serial.print("ch1_freq_hz="); if (isfinite(s1.freq_hz)) Serial.println(s1.freq_hz, 2); else Serial.println("nan");
//       Serial.print("ch1_rms_mV="); if (s1.n_period) Serial.println(s1.rms_mV, 3); else Serial.println("nan");

//       // CH2
//       Serial.println("ch2_analyzer_dt_us=" + String(ch2.t_us_end - ch2.t_us_start));
//       Serial.print("ch2_period_csv=");
//       if (s2.n_period == 0) Serial.println();
//       else {
//         for (uint16_t i = 0; i < s2.n_period; ++i) {
//           Serial.print(ch2.period_buf[i], 3); Serial.print(',');
//           if ((i & 0x1FF) == 0x1FF) vTaskDelay(1);
//         }
//         Serial.println();
//       }
//       Serial.print("ch2_freq_hz="); if (isfinite(s2.freq_hz)) Serial.println(s2.freq_hz, 2); else Serial.println("nan");
//       Serial.print("ch2_rms_mV="); if (s2.n_period) Serial.println(s2.rms_mV, 3); else Serial.println("nan");
//     }
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// // ===== Buffer allocation (per channel) =====
// static void alloc_buffers_or_die(chan_t& C, const char* name) {
//   uint32_t caps = MALLOC_CAP_8BIT;
// #if defined(BOARD_HAS_PSRAM) || defined(CONFIG_SPIRAM_SUPPORT)
//   if (psramFound()) caps |= MALLOC_CAP_SPIRAM;
// #endif

//   size_t ring = C.DECIM_RING, scan = C.MAX_SCAN;
//   while (true) {
//     C.decim_ring = (float*)heap_caps_malloc(ring * sizeof(float), caps);
//     C.win        = (float*)heap_caps_malloc(scan * sizeof(float), caps);
//     C.period_buf = (float*)heap_caps_malloc(MAX_PERIOD_BUF * sizeof(float), caps);
//     if (C.decim_ring && C.win && C.period_buf) {
//       C.DECIM_RING = ring; C.MAX_SCAN = scan;
//       memset(C.decim_ring, 0, ring * sizeof(float));
//       memset(C.win,        0, scan * sizeof(float));
//       memset(C.period_buf, 0, MAX_PERIOD_BUF * sizeof(float));
//       ESP_LOGI(TAG, "%s buffers: ring=%u (%.1f KB), win=%u (%.1f KB), period=%u (%.1f KB), caps=0x%x",
//                name,
//                (unsigned)C.DECIM_RING, (C.DECIM_RING*sizeof(float))/1024.0f,
//                (unsigned)C.MAX_SCAN,   (C.MAX_SCAN*sizeof(float))/1024.0f,
//                (unsigned)MAX_PERIOD_BUF, (MAX_PERIOD_BUF*sizeof(float))/1024.0f, caps);
//       break;
//     }
//     if (C.decim_ring) { heap_caps_free(C.decim_ring); C.decim_ring = nullptr; }
//     if (C.win)        { heap_caps_free(C.win);        C.win = nullptr; }
//     if (C.period_buf) { heap_caps_free(C.period_buf); C.period_buf = nullptr; }
//     if (ring <= 2048 || scan <= 1024) {
//       Serial.println("FATAL: buffer alloc failed");
//       abort();
//     }
//     ring >>= 1; scan >>= 1;
//   }

//   C.snap_mutex = xSemaphoreCreateMutex();
//   if (!C.snap_mutex) { Serial.println("FATAL: snap mutex alloc failed"); abort(); }

//   // init CIC & bitrate
//   memset(&C.cic, 0, sizeof(C.cic));
//   C.bit_rate_bps = (float)F_CLK_HZ;
// }

// // ===== Arduino entry =====
// void setup() {
//   Serial.begin(115200);
//   delay(100);
//   ESP_LOGI(TAG, "AMC1306 dual: DMA triple-buffer per host + byte-step CIC + per-channel analyzers");

//   build_byte_cic_lut();

//   alloc_buffers_or_die(ch1, "CH1");
//   alloc_buffers_or_die(ch2, "CH2");

//   spi_init_and_start(&g_sampler1);
//   spi_init_and_start(&g_sampler2);

//   // Samplers: high prio on separate cores (one each is fine)
//   static std::pair<sampler_ctx_t*, chan_t*> pack1 = { &g_sampler1, &ch1 };
//   static std::pair<sampler_ctx_t*, chan_t*> pack2 = { &g_sampler2, &ch2 };

//   xTaskCreatePinnedToCore(
//       spi_sampler_task, "spi_sampler_ch1",
//       4096, &pack1, configMAX_PRIORITIES - 2, nullptr, 1);

//   xTaskCreatePinnedToCore(
//       spi_sampler_task, "spi_sampler_ch2",
//       4096, &pack2, configMAX_PRIORITIES - 2, nullptr, 1);

//   // Analyzers: medium prio (one per channel)
//   xTaskCreatePinnedToCore(
//       analyzer_task, "analyzer_ch1",
//       6144, &ch1, 2, nullptr, 1);

//   xTaskCreatePinnedToCore(
//       analyzer_task, "analyzer_ch2",
//       6144, &ch2, 2, nullptr, 1);

//   // Printer: low prio
//   xTaskCreatePinnedToCore(
//       printer_task, "printer",
//       4096, nullptr, 1, nullptr, 0);
// }

// void loop() {
//   vTaskDelay(pdMS_TO_TICKS(1000));
// }
