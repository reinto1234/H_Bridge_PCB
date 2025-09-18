#include "spi_sampler.h"

static const char* TAG_SPI = "SPI-SAMPLER";

// ===== Channel-scoped measurement engines (dual AMC1306) =====
// Tune FS multipliers to your hardware scaling.
OutputMeasurements g_ch1_om(/*osr*/ OM_OSR, /*fs_mv*/ OM_FS_MV*121.0f, /*init bitrate*/ (float)F_CLK_HZ);
OutputMeasurements g_ch2_om(/*osr*/ OM_OSR, /*fs_mv*/ OM_FS_MV*10.0f,  /*init bitrate*/ (float)F_CLK_HZ);

// ===== Sampler contexts =====
sampler_ctx_t g_sampler1 = {
  .dev=nullptr, .trans={}, .rxbuf={nullptr,nullptr,nullptr}, .txdummy=nullptr,
  .pin_sclk=CH1_PIN_SCLK, .pin_miso=CH1_PIN_MISO, .pin_mosi=CH1_PIN_MOSI, .pin_cs=CH1_PIN_CS,
  .host=CH1_SPI_HOST, .dma_chan=CH1_DMA_CHAN
};
sampler_ctx_t g_sampler2 = {
  .dev=nullptr, .trans={}, .rxbuf={nullptr,nullptr,nullptr}, .txdummy=nullptr,
  .pin_sclk=CH2_PIN_SCLK, .pin_miso=CH2_PIN_MISO, .pin_mosi=CH2_PIN_MOSI, .pin_cs=CH2_PIN_CS,
  .host=CH2_SPI_HOST, .dma_chan=CH2_DMA_CHAN
};

// ===== Packs used by sampler tasks =====
std::pair<sampler_ctx_t*, OutputMeasurements*> g_pack1 = { &g_sampler1, &g_ch1_om };
std::pair<sampler_ctx_t*, OutputMeasurements*> g_pack2 = { &g_sampler2, &g_ch2_om };

// ----- Measurements init -----
bool spiSamplerInitMeasurements() {
  bool ok1 = g_ch1_om.init("CH1");
  bool ok2 = g_ch2_om.init("CH2");
  if (!ok1 || !ok2) {
    ESP_LOGE(TAG_SPI, "FATAL: OutputMeasurements init failed (ok1=%d ok2=%d)", ok1, ok2);
    return false;
  }
  return true;
}

// ----- SPI init + queue seed -----
void spiInitAndStart(sampler_ctx_t* s) {
  // DMA buffers
  for (int i = 0; i < NUM_DMA_BUFS; ++i) {
    s->rxbuf[i] = (uint8_t*)heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!s->rxbuf[i]) { ESP_LOGE(TAG_SPI, "RX DMA alloc fail"); abort(); }
  }
  s->txdummy = (uint8_t*)heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!s->txdummy) { ESP_LOGE(TAG_SPI, "TX dummy alloc fail"); abort(); }
  memset(s->txdummy, 0xFF, CHUNK_BYTES);

  // SPI bus
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num     = s->pin_mosi;
  buscfg.miso_io_num     = s->pin_miso;
  buscfg.sclk_io_num     = s->pin_sclk;
  buscfg.quadwp_io_num   = -1;
  buscfg.quadhd_io_num   = -1;
  buscfg.max_transfer_sz = CHUNK_BYTES;
  buscfg.flags           = SPICOMMON_BUSFLAG_MASTER;
  buscfg.intr_flags      = 0;
  ESP_ERROR_CHECK(spi_bus_initialize(s->host, &buscfg, s->dma_chan));

  // Device
  spi_device_interface_config_t devcfg = {};
  devcfg.mode             = SPI_MODE;
  devcfg.clock_speed_hz   = F_CLK_HZ;
  devcfg.spics_io_num     = s->pin_cs;
#ifdef SPI_DEVICE_NO_DUMMY
  devcfg.flags            = SPI_DEVICE_NO_DUMMY;
#else
  devcfg.flags            = 0;
#endif
  devcfg.queue_size       = QUEUE_DEPTH;
  ESP_ERROR_CHECK(spi_bus_add_device(s->host, &devcfg, &s->dev));

  // Transactions
  for (int i = 0; i < NUM_DMA_BUFS; ++i) {
    spi_transaction_t* t = &s->trans[i];
    memset(t, 0, sizeof(*t));
    t->length    = CHUNK_BYTES * 8;
    t->rxlength  = 0;
    t->tx_buffer = s->txdummy;
    t->rx_buffer = s->rxbuf[i];
  }

  // Seed the queue
  for (int i = 0; i < NUM_DMA_BUFS; ++i)
    ESP_ERROR_CHECK(spi_device_queue_trans(s->dev, &s->trans[i], portMAX_DELAY));
}

// ----- Sampler task (per SPI host) -----
void spiSamplerTask(void* arg) {
  auto* pack = (std::pair<sampler_ctx_t*, OutputMeasurements*>*)arg;
  sampler_ctx_t* s = pack->first;
  OutputMeasurements* OM = pack->second;

  uint64_t last_us = esp_timer_get_time();
  while (true) {
    spi_transaction_t* rtrans = nullptr;
    esp_err_t e = spi_device_get_trans_result(s->dev, &rtrans, portMAX_DELAY);
    if (e != ESP_OK) { ESP_LOGE(TAG_SPI, "get_trans_result err=%d", e); continue; }

    uint64_t now_us = esp_timer_get_time();
    uint64_t dt_us  = now_us - last_us;
    last_us = now_us;

    OM->processRxBytesAndUpdateBitrate((const uint8_t*)rtrans->rx_buffer, CHUNK_BYTES, dt_us);
    ESP_ERROR_CHECK(spi_device_queue_trans(s->dev, rtrans, portMAX_DELAY));
    taskYIELD();
  }
}

// ----- Analyzer task (per channel) -----
void analyzerTask(void* arg) {
  OutputMeasurements* OM = (OutputMeasurements*)arg;
  uint32_t last_ms = 0;
  while (true) {
    uint32_t now = millis();
    if ((uint32_t)(now - last_ms) >= OM_ANALYZE_EVERY_MS) { // cadence from Output_meas.h
      last_ms = now;
      OM->analyzeStep();
    }
    vTaskDelay(1);
  }
}

// ----- Optional: console printer (diagnostic) -----
void printerTask(void* arg) {
  (void)arg;
  uint32_t last_ms = 0;

  static float tmp_period1[OM_MAX_PERIOD_BUF];
  static float tmp_period2[OM_MAX_PERIOD_BUF];

  while (true) {
    uint32_t now = millis();
    if ((uint32_t)(now - last_ms) >= PRINT_EVERY_MS) {
      last_ms = now;

      OutputMeasurements::Snapshot s1{}, s2{};
      bool ok1 = g_ch1_om.getSnapshot(s1);
      bool ok2 = g_ch2_om.getSnapshot(s2);
      const int dt1 = g_ch1_om.analyzeEndUs() - g_ch1_om.analyzeStartUs();
      const int dt2 = g_ch2_om.analyzeEndUs() - g_ch2_om.analyzeStartUs();

      // CH1
      Serial.println("ch1_analyzer_dt_us=" + String(dt1));
      Serial.print("ch1_period_csv=");
      if (!ok1 || s1.n_period == 0) {
        Serial.println();
      } else {
        uint16_t n = g_ch1_om.copyLastPeriod(tmp_period1, OM_MAX_PERIOD_BUF);
        for (uint16_t i = 0; i < n; ++i) {
          Serial.print(tmp_period1[i], 3); Serial.print(',');
          if ((i & 0x1FF) == 0x1FF) vTaskDelay(1);
        }
        Serial.println();
      }
      Serial.print("ch1_freq_hz="); if (ok1 && isfinite(s1.freq_hz)) Serial.println(s1.freq_hz, 2); else Serial.println("nan");
      Serial.print("ch1_rms_mV=");  if (ok1 && s1.n_period)         Serial.println(s1.rms_mV, 3);   else Serial.println("nan");

      // CH2
      Serial.println("ch2_analyzer_dt_us=" + String(dt2));
      Serial.print("ch2_period_csv=");
      if (!ok2 || s2.n_period == 0) {
        Serial.println();
      } else {
        uint16_t n = g_ch2_om.copyLastPeriod(tmp_period2, OM_MAX_PERIOD_BUF);
        for (uint16_t i = 0; i < n; ++i) {
          Serial.print(tmp_period2[i], 3); Serial.print(',');
          if ((i & 0x1FF) == 0x1FF) vTaskDelay(1);
        }
        Serial.println();
      }
      Serial.print("ch2_freq_hz="); if (ok2 && isfinite(s2.freq_hz)) Serial.println(s2.freq_hz, 2); else Serial.println("nan");
      Serial.print("ch2_rms_mV=");  if (ok2 && s2.n_period)         Serial.println(s2.rms_mV, 3);   else Serial.println("nan");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ----- WebSocket loop task -----
void webSocketTask(void *pvParameters) {
  (void)pvParameters;
  while (true) {
    HBridgeWebServer::loopWebSocket();
    vTaskDelay(pdMS_TO_TICKS(10)); // keep responsive
  }
}

// ----- Periodic WebSocket update task -----
// Pulls Input measurements and AMC1306 OutputMeasurements (freq/RMS) and sends to clients.
void webSocketUpdate(void *pvParameters) {
  (void)pvParameters;

  // Adjust size/layout to whatever your UI expects.
  static float outBuf[7]; // [0]=ch1_rms_mV, [1]=ch2_rms_mV, [6]=ch1_freq_hz

  while (true) {
    float* inBuf = InputMeasurement::measurementall();

    // Build output array from AMC1306 analyzers
    OutputMeasurements::Snapshot s1{}, s2{};
    g_ch1_om.getSnapshot(s1);
    g_ch2_om.getSnapshot(s2);

    outBuf[0] = (s1.n_period > 0)    ? s1.rms_mV  : NAN;
    outBuf[1] = (s2.n_period > 0)    ? s2.rms_mV  : NAN;
    outBuf[2] = 0.0f;
    outBuf[3] = 0.0f;
    outBuf[4] = 0.0f;
    outBuf[5] = 0.0f;
    outBuf[6] = isfinite(s1.freq_hz) ? s1.freq_hz : NAN;

    HBridgeWebServer::updateMeasurements(inBuf, outBuf);
    vTaskDelay(pdMS_TO_TICKS(500)); // match your previous CYCLETIME_WEBSOCKET_UPDATE
  }
}
