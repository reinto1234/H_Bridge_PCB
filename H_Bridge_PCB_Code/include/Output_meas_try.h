// #pragma once

// #include <Arduino.h>
// #include <SPI.h>
// #include <vector>

// #if defined(ARDUINO_ARCH_ESP32)
// extern "C" {
//   #include "driver/spi_master.h"
//   #include "esp_system.h"
// }
// #endif

// // --- AMC1306Measurement ---
// // Kontinuierlicher SPI-DMA-Empfang des 1-Bit-Modulatorstroms (AMC1306M25),
// // sinc^3-Decimation, Skalierung auf mV, Frequenz- & RMS-Berechnung.
// class AMC1306Measurement {
// public:
//   AMC1306Measurement(
//     SPIClass& spi,
//     int pin_sclk,
//     int pin_miso,
//     uint32_t f_clk_hz,
//     uint16_t osr,
//     uint16_t history_seconds,
//     size_t   bytes_per_service,
//     float    fs_mV
//   );

//   // SPI + DMA initialisieren, Ping-Pong-Transfers anwerfen
//   bool begin();

//   // Non-blocking: holt fertige DMA-Blöcke ab, decimiert und füllt den Ringpuffer
//   void service();

//   // Berechnet aus der Historie Frequenz, RMS und extrahiert die letzte Voll-Periode.
//   // window_ms: gewünschtes Auswertefenster (z.B. 20 ms bei 50 Hz). 0 => auto (~1-2 Perioden)
//   bool compute(uint32_t window_ms = 0);

//   // Ergebnisse
//   float frequencyHz() const { return freq_hz_; }
//   float rmsLastPeriod() const { return rms_mV_; }

//   // Liefert die letzte volle Periode als mV-Samples (Pointer bleibt bis zum nächsten compute() gültig)
//   const float* lastPeriod(size_t& n) const {
//     n = last_period_.size();
//     return last_period_.empty() ? nullptr : last_period_.data();
//   }

//   // Für Debug/Info
//   float fsDecimatedHz() const { return fs_decim_hz_; }

//   // --- am Ende von public: ---
// uint64_t rxBytesTotal() const { return rx_bytes_total_; }
// uint64_t rxBlocksTotal() const { return rx_blocks_total_; }
// size_t   ringSamplesAvailable() const { return ring_count_; }

// // --- am Ende von private: ---
// volatile uint64_t rx_bytes_total_ = 0;
// volatile uint64_t rx_blocks_total_ = 0;

// private:
// #if defined(ARDUINO_ARCH_ESP32)
//   // SPI/ DMA
//   bool              initSpiDma_();
//   void              queueBoth_();
//   bool              queue_(spi_transaction_t& t);
//   void              processBlock_(const uint8_t* rx, size_t nbytes);
// #endif

//   // CIC / sinc^3 Stufen
//   inline void cicPushBit_(int bit01);

//   // Ringpuffer für decimierte mV-Samples
//   void   ringInit_(size_t samples);
//   void   ringPush_(float v);
//   float  ringAt_(size_t idx_from_tail) const;      // idx 0 = letztes Sample
//   size_t ringCount_() const { return ring_count_; }

// private:
//   // Konstruktor-Parameter
//   SPIClass& spi_;
//   const int pin_sclk_;
//   const int pin_miso_;
//   const uint32_t f_clk_hz_;
//   const uint16_t osr_;
//   const uint16_t history_seconds_;
//   const size_t   bytes_per_service_;
//   const float    fs_mV_;

//   // Abgeleitet
//   float    fs_decim_hz_ = 0.0f;
//   float    bit_to_mV_   = 0.0f;  // Skalfaktor aus CIC-Ausgang -> mV

//   // CIC Zustände (64-bit für große OSR)
//   int64_t  i1_ = 0, i2_ = 0, i3_ = 0;
//   int64_t  z1_ = 0, z2_ = 0, z3_ = 0;  // comb delays
//   uint32_t decim_cnt_ = 0;

//   // Ringpuffer
//   float*   ring_ = nullptr;
//   size_t   ring_size_ = 0;
//   size_t   ring_write_ = 0;
//   size_t   ring_count_ = 0;

//   // Ergebnisse
//   float           freq_hz_ = NAN;
//   float           rms_mV_  = NAN;
//   std::vector<float> last_period_;

// #if defined(ARDUINO_ARCH_ESP32)
//   // ESP-IDF SPI
//   spi_device_handle_t dev_ = nullptr;
//   spi_transaction_t   tA_{}, tB_{};
//   uint8_t* rxA_ = nullptr;
//   uint8_t* rxB_ = nullptr;
//   uint8_t* txA_ = nullptr;
//   uint8_t* txB_ = nullptr;
//   bool     queuedA_ = false;
//   bool     queuedB_ = false;
//   bool     spi_ready_ = false;
// #endif
// };
