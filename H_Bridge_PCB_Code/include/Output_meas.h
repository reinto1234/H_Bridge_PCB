#pragma once
/*
  Output_meas.h
  ESP32 <-> TI AMC1306M25 measurement helper
*/

#include <Arduino.h>
#include <SPI.h>
#include "mutexdefinitions.h"

class AMC1306Measurement {
public:
  AMC1306Measurement(
      SPIClass& spi,
      int pinSclk,
      int pinMiso,
      uint32_t f_clk_hz = 5000000UL,
      uint16_t osr      = 2000,
      float fs_mV       = 320.0f,
      size_t ringBytes  = 32768,
      size_t bytesPerCall = 4092
  );

  void begin();

  // Thread-safe:
  // - service(): locks measurementSpiMutex (SPI clocking & raw ring writes)
  // - compute(): locks measurementoutMutex (decimation & results)
  void service();
  bool compute();

  float frequencyHz() const { return _freqHz; }
  float rmsLastPeriod() const { return _rms_mV; }
  const float* lastPeriod(size_t& n) const {
    n = _lastPeriodCount;
    return (_lastPeriodCount ? _lastPeriodBuf : nullptr);
  }

  double measuredBitRate() const { return _bit_rate_bps; }
  double measuredFsDecim() const { return _bit_rate_bps / (double)_OSR; }
  float calcFreqFast();

private:
  // configuration
  SPIClass&  _spi;
  int        _pinSclk;
  int        _pinMiso;
  const uint32_t _F_CLK_HZ;
  const uint16_t _OSR;
  const float    _FS_mV;
  const size_t   _RING_BYTES;
  const size_t   _BYTES_PER_CALL;

  

  // ring
  uint8_t*       _ringbuf = nullptr;
  volatile uint32_t _ring_write = 0;

  // timing / bit-rate
  volatile uint32_t _last_wrap_us = 0;
  volatile double   _bit_rate_bps;

  // decimated buffer
  static constexpr uint32_t DECIM_MAX = 2048;
  float     _decim[DECIM_MAX];
  uint32_t  _decim_count = 0;

  // results
  float     _lastPeriodBuf[DECIM_MAX];
  size_t    _lastPeriodCount = 0;
  float     _rms_mV = NAN;
  float     _freqHz = NAN;

  //RMS
  bool  _findLastTwoRisingZC(double& t0, double& t1) const;
    float _computeRmsOnePeriodExact() const;


  // CIC sinc^3
  struct CIC3 {
    long long i1=0,i2=0,i3=0, d1=0,d2=0,d3=0;
    inline void reset(){ i1=i2=i3=d1=d2=d3=0; }
    inline void push(int sgn){ i1+=sgn; i2+=i1; i3+=i2; }
    inline long long decimate(){
      long long c1=i3-d1; d1=i3;
      long long c2=c1-d2; d2=c1;
      long long c3=c2-d3; d3=c2;
      return c3;
    }
  } _cic;

  // helpers
  void   _spiInitIfNeeded();
  void   _clockAndFillRing();
  void   _decimateFromRing();
  float  _cicToMilliVolts(long long y) const;
  float  _estimateFreqHzPrecise(double fs_decim_meas) const;
  bool   _extractLastFullPeriod();
  void   _computeRms();
  static inline uint32_t _usdiff(uint32_t now, uint32_t then){
    return (uint32_t)(now - then);
  }
};
