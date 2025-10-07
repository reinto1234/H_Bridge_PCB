#pragma once
#include <Arduino.h>
#include <utility>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "driver/spi_master.h"
  #include "driver/gpio.h"
  #include "esp_system.h"
  #include "esp_heap_caps.h"
  #include "esp_timer.h"
  #include "esp_log.h"
}

#include "Output_meas.h"
#include "webserver.h"
#include "Input_meas.h"
#include "PWM.h"

#ifndef SPICOMMON_BUSFLAG_MASTER
#define SPICOMMON_BUSFLAG_MASTER 0
#endif

// ---------- SPI / DMA & Pins for two AMC1306 channels ----------
#ifndef CH1_PIN_SCLK
#define CH1_PIN_SCLK   17
#endif
#ifndef CH1_PIN_MISO
#define CH1_PIN_MISO   18
#endif
#ifndef CH1_PIN_MOSI
#define CH1_PIN_MOSI   -1
#endif
#ifndef CH1_PIN_CS
#define CH1_PIN_CS     -1
#endif

#ifndef CH2_PIN_SCLK
#define CH2_PIN_SCLK   25   // AMC1306 #2 CLKIN -> ESP32 SCLK (VSPI)
#endif
#ifndef CH2_PIN_MISO
#define CH2_PIN_MISO   26   // AMC1306 #2 DOUT  -> ESP32 MISO (VSPI)
#endif
#ifndef CH2_PIN_MOSI
#define CH2_PIN_MOSI   -1
#endif
#ifndef CH2_PIN_CS
#define CH2_PIN_CS     -1
#endif

#ifndef CH1_SPI_HOST
#define CH1_SPI_HOST      VSPI_HOST
#endif
#ifndef CH1_DMA_CHAN
#define CH1_DMA_CHAN      1
#endif
#ifndef CH2_SPI_HOST
#define CH2_SPI_HOST      HSPI_HOST
#endif
#ifndef CH2_DMA_CHAN
#define CH2_DMA_CHAN      2
#endif

#ifndef F_CLK_HZ
#define F_CLK_HZ          5000000      // SPI clk per host
#endif
#ifndef SPI_MODE
#define SPI_MODE          1
#endif

#ifndef CHUNK_BYTES
#define CHUNK_BYTES       2048         // per DMA transaction
#endif
#ifndef QUEUE_DEPTH
#define QUEUE_DEPTH       3            // triple-buffer
#endif
#ifndef NUM_DMA_BUFS
#define NUM_DMA_BUFS      3
#endif

// Optional console printer cadence (diagnostic)
#ifndef PRINT_EVERY_MS
#define PRINT_EVERY_MS    10000
#endif

// ===== Sampler context (per SPI host) =====
typedef struct {
  spi_device_handle_t dev;
  spi_transaction_t   trans[NUM_DMA_BUFS];
  uint8_t*            rxbuf[NUM_DMA_BUFS];
  uint8_t*            txdummy;
  int pin_sclk, pin_miso, pin_mosi, pin_cs;
  spi_host_device_t host;
  int dma_chan;
} sampler_ctx_t;

// Public globals
extern sampler_ctx_t g_sampler1;
extern sampler_ctx_t g_sampler2;

extern std::pair<sampler_ctx_t*, OutputMeasurements*> g_pack1;
extern std::pair<sampler_ctx_t*, OutputMeasurements*> g_pack2;

extern OutputMeasurements g_ch1_om;
extern OutputMeasurements g_ch2_om;

// Init helpers
bool spiSamplerInitMeasurements();
void spiInitAndStart(sampler_ctx_t* s);


