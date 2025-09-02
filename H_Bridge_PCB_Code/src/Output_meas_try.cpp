// #include "Output_meas_try.h"
// #include <math.h>
// #include <string.h>

// AMC1306Measurement::AMC1306Measurement(
//   SPIClass& spi,
//   int pin_sclk,
//   int pin_miso,
//   uint32_t f_clk_hz,
//   uint16_t osr,
//   uint16_t history_seconds,
//   size_t   bytes_per_service,
//   float    fs_mV
// )
// : spi_(spi),
//   pin_sclk_(pin_sclk),
//   pin_miso_(pin_miso),
//   f_clk_hz_(f_clk_hz),
//   osr_(osr ? osr : 256),
//   history_seconds_(history_seconds ? history_seconds : 2),
//   bytes_per_service_(bytes_per_service ? bytes_per_service : 4092),
//   fs_mV_(fs_mV)
// {
//   fs_decim_hz_ = (float)f_clk_hz_ / (float)osr_;
//   // CIC(3) Gain = OSR^3  → Normierung auf [-1..+1], danach auf ±fs_mV_
//   double gain = (double)osr_ * (double)osr_ * (double)osr_;
//   bit_to_mV_ = (float)(fs_mV_ / gain);

//   // Ringpuffer: Sekunden * fs_decim  (+ etwas headroom)
//   size_t need = (size_t)ceilf(fs_decim_hz_ * history_seconds_) + 256;
//   ringInit_(need);
// }

// bool AMC1306Measurement::begin() {
// #if defined(ARDUINO_ARCH_ESP32)
//   bool ok = initSpiDma_();
//   Serial.printf("[SPI] begin() %s\n", ok ? "OK" : "FAILED");
//   return ok;
// #else
//   return false;
// #endif
// }


// bool AMC1306Measurement::queue_(spi_transaction_t& t) {
//   esp_err_t err = spi_device_queue_trans(dev_, &t, 0);   // 0 timeout: non-blocking
//   if (err != ESP_OK) {
//     Serial.printf("[SPI] queue_ failed: %d\n", (int)err);
//     return false;
//   }
//   return true;
// }

// void AMC1306Measurement::service() {
// #if defined(ARDUINO_ARCH_ESP32)
//   if (!spi_ready_) {
//     static bool once=false; if(!once){ once=true; Serial.println("[SPI] service(): not ready"); }
//     return;
//   }

//   spi_transaction_t* r = nullptr;
//   bool got_any = false;

//   // pull *all* completed txns; 0 timeout => non-blocking
//   while (spi_device_get_trans_result(dev_, &r, 0) == ESP_OK) {
//     got_any = true;
//     const uint8_t* rx = (const uint8_t*)r->rx_buffer;
//     processBlock_(rx, bytes_per_service_);

//     // immediately requeue same buffer; if it fails, mark it so queueBoth_() can retry
//     if (!queue_(*r)) {
//       if (&tA_ == r) queuedA_ = false;
//       if (&tB_ == r) queuedB_ = false;
//     }
//   }

//   // if nothing in flight, try (re)start and warn once per second
//   if (!queuedA_ || !queuedB_) {
//     queueBoth_();
//     static uint32_t lastWarn = 0;
//     uint32_t now = millis();
//     if (now - lastWarn > 1000) {
//       lastWarn = now;
//       Serial.printf("[SPI] warn: queue empty (A=%d, B=%d)\n", (int)queuedA_, (int)queuedB_);
//     }
//   }
// #endif
// }


// bool AMC1306Measurement::compute(uint32_t window_ms) {
//   // Mindestens ~1 Periode (50/60 Hz) nötig
//   if (ring_count_ < (size_t)(fs_decim_hz_ * 0.5f)) return false;

//   // Suchfenster bestimmen
//   size_t want = window_ms ? (size_t)((window_ms / 1000.0f) * fs_decim_hz_)
//                           : (size_t)(2.5f * fs_decim_hz_ / 50.0f * 50.0f); // ~2,5 Perioden @50Hz
//   if (want > ring_count_) want = ring_count_;

//   // In den letzten "want" Samples zwei aufeinanderfolgende steigende Nulldurchgänge finden
//   // Wir holen Samples vom Ring-Ende rückwärts
//   size_t tail = want;
//   auto samp = [&](size_t i_from_tail) -> float { return ringAt_(i_from_tail); };

//   // Suche von hinten nach vorn
//   int idx_last = -1, idx_prev = -1;
//   float s_prev = samp( tail >= 1 ? (tail-1) : 0 );
//   for (size_t k = 2; k <= tail; ++k) {
//     float s_cur = samp(tail - k);
//     // steigender Nulldurchgang: s_prev < 0 && s_cur >= 0 (beachte Richtung; wir iterieren rückwärts)
//     if (s_cur >= 0.0f && s_prev < 0.0f) {
//       // "k-1" ist der Index (von hinten gezählt), an dem die Steigung passiert
//       int idx = (int)(tail - k);
//       if (idx_last < 0) {
//         idx_last = idx;
//       } else {
//         idx_prev = idx;
//         break;
//       }
//     }
//     s_prev = s_cur;
//   }

//   if (idx_last < 0 || idx_prev < 0) return false;

//   // Periodenlänge in Samples
//   int period_samples = idx_last - idx_prev;
//   if (period_samples <= 1) return false;

//   // Frequenz
//   freq_hz_ = fs_decim_hz_ / (float)period_samples;

//   // Letzte Periode extrahieren (von idx_prev bis idx_last-1)
//   last_period_.clear();
//   last_period_.reserve((size_t)period_samples);
//   for (int i = idx_prev; i < idx_last; ++i) {
//     last_period_.push_back( samp((size_t)i) );
//   }

//   // RMS (mV)
//   double acc2 = 0.0;
//   for (float v : last_period_) acc2 += (double)v * (double)v;
//   rms_mV_ = (float)sqrt(acc2 / (double)last_period_.size());

//   return true;
// }

// /* =====================  Private: CIC / Ring  ===================== */

// inline void AMC1306Measurement::cicPushBit_(int bit01) {
//   // Map 0/1 -> -1/+1
//   int s = bit01 ? +1 : -1;

//   // 3 Integratoren
//   i1_ += s;
//   i2_ += i1_;
//   i3_ += i2_;

//   // Decimation
//   if (++decim_cnt_ >= osr_) {
//     decim_cnt_ = 0;

//     // 3 Combs
//     int64_t c1 = i3_ - z1_; z1_ = i3_;
//     int64_t c2 = c1 - z2_;  z2_ = c1;
//     int64_t c3 = c2 - z3_;  z3_ = c2;

//     // Skalieren -> mV
//     float v_mV = (float)c3 * bit_to_mV_;
//     ringPush_(v_mV);
//   }
// }

// void AMC1306Measurement::ringInit_(size_t samples) {
//   if (ring_) { free(ring_); ring_ = nullptr; }
//   ring_size_ = samples ? samples : 4096;
//   ring_ = (float*)heap_caps_calloc(ring_size_, sizeof(float), MALLOC_CAP_DEFAULT);
//   ring_write_ = 0;
//   ring_count_ = 0;
// }

// void AMC1306Measurement::ringPush_(float v) {
//   if (!ring_) return;
//   ring_[ring_write_] = v;
//   ring_write_ = (ring_write_ + 1) % ring_size_;
//   if (ring_count_ < ring_size_) ++ring_count_;
// }

// float AMC1306Measurement::ringAt_(size_t idx_from_tail) const {
//   if (!ring_ || idx_from_tail >= ring_count_) return 0.0f;
//   // idx_from_tail = 0 -> letztes geschriebenes (write-1)
//   size_t pos = (ring_write_ + ring_size_ - 1 - idx_from_tail) % ring_size_;
//   return ring_[pos];
// }

// /* =====================  Private: SPI / DMA  ===================== */

// #if defined(ARDUINO_ARCH_ESP32)

// bool AMC1306Measurement::initSpiDma_() {
//   // SPI-Bus initialisieren (nur MISO & SCLK; kein MOSI, kein CS)
//   spi_bus_config_t bus{};
//   bus.miso_io_num = pin_miso_;
//   bus.mosi_io_num = -1;
//   bus.sclk_io_num = pin_sclk_;
//   bus.quadwp_io_num = -1;
//   bus.quadhd_io_num = -1;
//   bus.max_transfer_sz = bytes_per_service_ + 2;
//   bus.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_SCLK;

//   // VSPI Host benutzen (entspricht Arduino VSPI)
//   spi_host_device_t host = VSPI_HOST;

//   esp_err_t err = spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO);
// if (err == ESP_ERR_INVALID_STATE) {
//   // Bus war schon initialisiert (vermutlich mit anderen Pins) -> freigeben und mit unseren Pins neu starten
//   spi_bus_free(host);
//   err = spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO);
// }
// if (err != ESP_OK) {
//   Serial.printf("spi_bus_initialize failed: %d\n", err);
//   return false;
// }


//   if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
//     Serial.printf("spi_bus_initialize failed: %d\n", err);
//     return false;
//   }

//   spi_device_interface_config_t dev{};
//   dev.mode = 0;                      // CPOL=0, CPHA=0 -> Sample mittig der High-Phase
//   dev.clock_speed_hz = (int)f_clk_hz_;
//   dev.spics_io_num = -1;             // kein CS
//   dev.queue_size = 4;                // Ping-Pong
//   dev.flags = SPI_DEVICE_NO_DUMMY;
//   dev.input_delay_ns = 0;            // bei 5 MHz unkritisch; DOUT valid ~15 ns nach rising CLK

//   err = spi_bus_add_device(host, &dev, &dev_);
//   if (err != ESP_OK) {
//     Serial.printf("spi_bus_add_device failed: %d\n", err);
//     return false;
//   }

//   // DMA-Puffer
//   rxA_ = (uint8_t*)heap_caps_malloc(bytes_per_service_, MALLOC_CAP_DMA);
//   rxB_ = (uint8_t*)heap_caps_malloc(bytes_per_service_, MALLOC_CAP_DMA);
//   txA_ = (uint8_t*)heap_caps_malloc(bytes_per_service_, MALLOC_CAP_DMA);
//   txB_ = (uint8_t*)heap_caps_malloc(bytes_per_service_, MALLOC_CAP_DMA);
//   if (!rxA_ || !rxB_ || !txA_ || !txB_) {
//     Serial.println("DMA buffer alloc failed");
//     return false;
//   }
//   memset(txA_, 0x00, bytes_per_service_);
//   memset(txB_, 0x00, bytes_per_service_);

//   // Transaktionen vorbereiten
//   memset(&tA_, 0, sizeof(tA_));
//   memset(&tB_, 0, sizeof(tB_));
//   tA_.length   = bytes_per_service_ * 8;  // Bits
//   tA_.rxlength = 0;                       // = length
//   tA_.rx_buffer = rxA_;
//   tA_.tx_buffer = txA_;

//   tB_.length   = bytes_per_service_ * 8;
//   tB_.rxlength = 0;
//   tB_.rx_buffer = rxB_;
//   tB_.tx_buffer = txB_;

//   queuedA_ = queuedB_ = false;
//   queueBoth_();

//   spi_ready_ = true;
//   return true;
// }

// void AMC1306Measurement::queueBoth_() {
//   if (!queuedA_) queuedA_ = queue_(tA_);
//   if (!queuedB_) queuedB_ = queue_(tB_);
// }


// void AMC1306Measurement::processBlock_(const uint8_t* rx, size_t nbytes) {
//   // Bitstrom durchlaufen (MSB->LSB); Byte-Grenzen sind beliebig, da kein Framing
//   rx_bytes_total_  += nbytes;
//   rx_blocks_total_ += 1;
//   for (size_t i = 0; i < nbytes; ++i) {
//     uint8_t b = rx[i];
//     // MSB first
//     cicPushBit_((b >> 7) & 1);
//     cicPushBit_((b >> 6) & 1);
//     cicPushBit_((b >> 5) & 1);
//     cicPushBit_((b >> 4) & 1);
//     cicPushBit_((b >> 3) & 1);
//     cicPushBit_((b >> 2) & 1);
//     cicPushBit_((b >> 1) & 1);
//     cicPushBit_( b       & 1);
//   }
// }

// #endif // ESP32
