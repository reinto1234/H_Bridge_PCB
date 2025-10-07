#include "spi_sampler.h"

static const char* TAG_SPI = "SPI-SAMPLER";

// ===== Channel-scoped measurement engines (dual AMC1306) =====
// Tune FS multipliers to your hardware scaling.
OutputMeasurements g_ch1_om(/*osr*/ OM_OSR, /*fs_mv*/ OM_FS_MV*121.0f, /*init bitrate*/ (float)F_CLK_HZ);
OutputMeasurements g_ch2_om(/*osr*/ OM_OSR, /*fs_mv*/ OM_FS_MV*(-10.0f),  /*init bitrate*/ (float)F_CLK_HZ);

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
  buscfg.flags           = SPICOMMON_BUSFLAG_MASTERMODE;
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

