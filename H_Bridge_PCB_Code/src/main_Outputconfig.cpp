// // ==== AMC1306 continuous capture via SPI DMA (Arduino-ESP32) ====
// // - Dual-channel: HSPI (CH1) + VSPI (CH2), each with triple-buffer DMA
// // - CIC3 per BYTE (fast), continuous state
// // - Analyzer tasks (per channel): mean-removal, rising-ZC with hysteresis, freq & RMS
// // - Printer task (10 s): prints last full period CSV + freq_hz + rms_mV per channel
// // - Watchdog-friendly; no large stacks; big buffers on heap/PSRAM

// #include <Arduino.h>
// #include <math.h>
// #include <string.h>

// extern "C" {
//   #include "freertos/FreeRTOS.h"
//   #include "freertos/task.h"
//   #include "driver/spi_master.h"
//   #include "driver/gpio.h"
//   #include "esp_system.h"
//   #include "esp_heap_caps.h"
//   #include "esp_timer.h"
//   #include "esp_log.h"
// }

// #include "Output_meas.h"

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

// // ---------- Analyzer cadence ----------
// #define ANALYZE_EVERY_MS  5
// #define PRINT_EVERY_MS    10000

// #ifndef SPICOMMON_BUSFLAG_MASTER
// #define SPICOMMON_BUSFLAG_MASTER 0
// #endif

// static const char* TAG = "AMC1306-DUAL";

// // ===== Channel-scoped measurement engines =====
// static OutputMeasurements ch1_om(/*osr*/ OM_OSR, /*fs_mv*/ OM_FS_MV, /*init bitrate*/ (float)F_CLK_HZ);
// static OutputMeasurements ch2_om(/*osr*/ OM_OSR, /*fs_mv*/ OM_FS_MV, /*init bitrate*/ (float)F_CLK_HZ);

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

// // ===== Sampler tasks (one per SPI host) =====
// static void spi_sampler_task(void* arg) {
//   auto* pack = (std::pair<sampler_ctx_t*, OutputMeasurements*>*)arg;
//   sampler_ctx_t* s = pack->first;
//   OutputMeasurements* OM = pack->second;

//   uint64_t last_us = esp_timer_get_time();
//   while (true) {
//     spi_transaction_t* rtrans = nullptr;
//     esp_err_t e = spi_device_get_trans_result(s->dev, &rtrans, portMAX_DELAY);
//     if (e != ESP_OK) { ESP_LOGE(TAG, "get_trans_result err=%d", e); continue; }

//     uint64_t now_us = esp_timer_get_time();
//     uint64_t dt_us  = now_us - last_us;
//     last_us = now_us;

//     OM->processRxBytesAndUpdateBitrate((const uint8_t*)rtrans->rx_buffer, CHUNK_BYTES, dt_us);
//     ESP_ERROR_CHECK(spi_device_queue_trans(s->dev, rtrans, portMAX_DELAY));
//     taskYIELD();
//   }
// }

// // ===== Analyzer task (per channel) =====
// static void analyzer_task(void* arg) {
//   OutputMeasurements* OM = (OutputMeasurements*)arg;
//   uint32_t last_ms = 0;
//   while (true) {
//     uint32_t now = millis();
//     if ((uint32_t)(now - last_ms) >= ANALYZE_EVERY_MS) {
//       last_ms = now;
//       OM->analyzeStep();
//     }
//     vTaskDelay(1);
//   }
// }

// // ===== Printer task (prints both channels) =====
// static void printer_task(void* arg) {
//   (void)arg;
//   uint32_t last_ms = 0;

//   // temp buffers to print period CSV without holding mutexes too long
//   static float tmp_period1[OM_MAX_PERIOD_BUF];
//   static float tmp_period2[OM_MAX_PERIOD_BUF];

//   while (true) {
//     uint32_t now = millis();
//     if ((uint32_t)(now - last_ms) >= PRINT_EVERY_MS) {
//       last_ms = now;

//       OutputMeasurements::Snapshot s1{}, s2{};
//       bool ok1 = ch1_om.getSnapshot(s1);
//       bool ok2 = ch2_om.getSnapshot(s2);
//       const int dt1 = ch1_om.analyzeEndUs() - ch1_om.analyzeStartUs();
//       const int dt2 = ch2_om.analyzeEndUs() - ch2_om.analyzeStartUs();

//       // CH1
//       Serial.println("ch1_analyzer_dt_us=" + String(dt1));
//       Serial.print("ch1_period_csv=");
//       if (!ok1 || s1.n_period == 0) {
//         Serial.println();
//       } else {
//         uint16_t n = ch1_om.copyLastPeriod(tmp_period1, OM_MAX_PERIOD_BUF);
//         for (uint16_t i = 0; i < n; ++i) {
//           Serial.print(tmp_period1[i], 3); Serial.print(',');
//           if ((i & 0x1FF) == 0x1FF) vTaskDelay(1);
//         }
//         Serial.println();
//       }
//       Serial.print("ch1_freq_hz="); if (ok1 && isfinite(s1.freq_hz)) Serial.println(s1.freq_hz, 2); else Serial.println("nan");
//       Serial.print("ch1_rms_mV=");  if (ok1 && s1.n_period)         Serial.println(s1.rms_mV, 3);   else Serial.println("nan");

//       // CH2
//       Serial.println("ch2_analyzer_dt_us=" + String(dt2));
//       Serial.print("ch2_period_csv=");
//       if (!ok2 || s2.n_period == 0) {
//         Serial.println();
//       } else {
//         uint16_t n = ch2_om.copyLastPeriod(tmp_period2, OM_MAX_PERIOD_BUF);
//         for (uint16_t i = 0; i < n; ++i) {
//           Serial.print(tmp_period2[i], 3); Serial.print(',');
//           if ((i & 0x1FF) == 0x1FF) vTaskDelay(1);
//         }
//         Serial.println();
//       }
//       Serial.print("ch2_freq_hz="); if (ok2 && isfinite(s2.freq_hz)) Serial.println(s2.freq_hz, 2); else Serial.println("nan");
//       Serial.print("ch2_rms_mV=");  if (ok2 && s2.n_period)         Serial.println(s2.rms_mV, 3);   else Serial.println("nan");
//     }
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// // ===== Arduino entry =====
// void setup() {
//   Serial.begin(115200);
//   delay(100);
//   ESP_LOGI(TAG, "AMC1306 dual: DMA triple-buffer per host + byte-step CIC + per-channel analyzers (refactored)");

//   // Init measurement engines
//   if (!ch1_om.init("CH1") || !ch2_om.init("CH2")) {
//     Serial.println("FATAL: OutputMeasurements init failed");
//     abort();
//   }

//   // Start SPI engines
//   spi_init_and_start(&g_sampler1);
//   spi_init_and_start(&g_sampler2);

//   // Samplers: high prio on separate cores (one each is fine)
//   static std::pair<sampler_ctx_t*, OutputMeasurements*> pack1 = { &g_sampler1, &ch1_om };
//   static std::pair<sampler_ctx_t*, OutputMeasurements*> pack2 = { &g_sampler2, &ch2_om };

//   xTaskCreatePinnedToCore(
//       spi_sampler_task, "spi_sampler_ch1",
//       4096, &pack1, configMAX_PRIORITIES - 2, nullptr, 1);

//   xTaskCreatePinnedToCore(
//       spi_sampler_task, "spi_sampler_ch2",
//       4096, &pack2, configMAX_PRIORITIES - 2, nullptr, 1);

//   // Analyzers: medium prio (one per channel)
//   xTaskCreatePinnedToCore(
//       analyzer_task, "analyzer_ch1",
//       6144, &ch1_om, 2, nullptr, 1);

//   xTaskCreatePinnedToCore(
//       analyzer_task, "analyzer_ch2",
//       6144, &ch2_om, 2, nullptr, 1);

//   // Printer: low prio
//   xTaskCreatePinnedToCore(
//       printer_task, "printer",
//       4096, nullptr, 1, nullptr, 0);
// }

// void loop() {
//   vTaskDelay(pdMS_TO_TICKS(1000));
// }
