#include "Tasks.h"


extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_timer.h"
  #include "esp_log.h"
}

#include "Output_meas.h"
#include "webserver.h"
#include "safety.h"
#include "Input_meas.h"
#include "PWM.h"   // for inverter (declared elsewhere)


static const char* TAG_TASKS = "TASKS";

// ----- Sampler task (per SPI host) -----
void spiSamplerTask(void* arg) {
  auto* pack = (std::pair<sampler_ctx_t*, OutputMeasurements*>*)arg;
  sampler_ctx_t* s = pack->first;
  OutputMeasurements* OM = pack->second;

  uint64_t last_us = esp_timer_get_time();
  while (true) {
    spi_transaction_t* rtrans = nullptr;
    esp_err_t e = spi_device_get_trans_result(s->dev, &rtrans, portMAX_DELAY);
    if (e != ESP_OK) { ESP_LOGE(TAG_TASKS, "spi get_trans_result err=%d", e); continue; }

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
    if ((uint32_t)(now - last_ms) >= OM_ANALYZE_EVERY_MS) {
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
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ----- Periodic WebSocket update task -----
void webSocketUpdate(void *pvParameters) {
  (void)pvParameters;

  // Adjust size/layout to whatever your UI expects.
  static float outBuf[7]; // [0]=ch1_rms_mV, [1]=ch2_rms_mV, [6]=ch1_freq_hz

  while (true) {
    float* inBuf = InputMeasurement::measurementall();

    OutputMeasurements::Snapshot s1{}, s2{};
    g_ch1_om.getSnapshot(s1);
    g_ch2_om.getSnapshot(s2);

    outBuf[0] = (s1.n_period > 0)    ? s1.rms_mV  : NAN;
    outBuf[1] = (s2.n_period > 0)    ? s2.rms_mV  : NAN;
    outBuf[2] = (s1.n_period > 0 && s2.n_period > 0)    ? s1.rms_mV*s2.rms_mV  : NAN;
    outBuf[3] = 0.0f;
    outBuf[4] = 0.0f;
    outBuf[5] = 0.0f;
    outBuf[6] = isfinite(s1.freq_hz) ? s1.freq_hz : NAN;

    HBridgeWebServer::updateMeasurements(inBuf, outBuf);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ----- Controller loop -----
void ControllerTask(void* pvParameters) {
  (void)pvParameters;

  while (true) {
    // NOTE: this should be &&, not || (otherwise you'll deref a null inverter)
    if (inverter != nullptr && &(inverter->controller) != nullptr) {
      OutputMeasurements::Snapshot s1{}, s2{};
      g_ch1_om.getSnapshot(s1);
      //g_ch2_om.getSnapshot(s2);
      //inverter->controller.ControlSPWM(g_ch1_om, 14.0f);
      inverter->controller.ControlRMS(&s1.rms_mV);
    }
    vTaskDelay(pdMS_TO_TICKS(Controller_Timecycle));
  }
}

void EmergencyStopTask(void*) {
    for (;;) {
        if (g_emergency_stop) {
            g_emergency_stop = false;
            stopInverter();
            HBridgeWebServer::broadcastTrip("INA228 Overcurrent detected \n Emergency Stop Activated \n Change Load and click OK to continue");
            digitalWrite(ESTOP_OUTPUT_PIN, HIGH);
            Serial.println("Emergency Stop Triggered!");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
