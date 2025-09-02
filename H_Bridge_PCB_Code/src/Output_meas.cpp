#include "Output_meas.h"
#include <math.h>

AMC1306Measurement::AMC1306Measurement(
    SPIClass& spi, int pinSclk, int pinMiso,
    uint32_t f_clk_hz, uint16_t osr, float fs_mV,
    size_t ringBytes, size_t bytesPerCall)
  : _spi(spi),
    _pinSclk(pinSclk),
    _pinMiso(pinMiso),
    _F_CLK_HZ(f_clk_hz),
    _OSR(osr),
    _FS_mV(fs_mV),
    _RING_BYTES(ringBytes),
    _BYTES_PER_CALL(bytesPerCall),
    _bit_rate_bps((double)f_clk_hz)
{ }

void AMC1306Measurement::begin() {
  if (!_ringbuf) {
    _ringbuf = (uint8_t*)heap_caps_malloc(_RING_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!_ringbuf) _ringbuf = (uint8_t*)malloc(_RING_BYTES);
  }
  _ring_write = 0;
  _last_wrap_us = 0;
  _bit_rate_bps = (double)_F_CLK_HZ;
  _decim_count = 0;
  _lastPeriodCount = 0;
  _rms_mV = NAN;
  _freqHz = NAN;
  _cic.reset();

  _spiInitIfNeeded();

  for (int i = 0; i < 32; ++i) service();
}

void AMC1306Measurement::service() {
  // SPI protection dedicated to AMC1306 path
  if (measurementSpiMutex && xSemaphoreTake(measurementSpiMutex, portMAX_DELAY) == pdTRUE) {
    _clockAndFillRing();
    xSemaphoreGive(measurementSpiMutex);
  } else {
    _clockAndFillRing();
  }
}

bool AMC1306Measurement::compute() {
  bool ok = false;


  
  // Post-processing protection
  if (measurementoutMutex && xSemaphoreTake(measurementoutMutex, portMAX_DELAY) == pdTRUE) {
    if (measurementSpiMutex && xSemaphoreTake(measurementSpiMutex, portMAX_DELAY) == pdTRUE) {
      _decimateFromRing();
      xSemaphoreGive(measurementSpiMutex);
    } else {
      _decimateFromRing();
    }
    const double fs_decim_meas = _bit_rate_bps / (double)_OSR;
    _freqHz = _estimateFreqHzPrecise(fs_decim_meas);

    if (_extractLastFullPeriod()) {
      //_computeRms();
      _rms_mV = _computeRmsOnePeriodExact();
      ok = true;
    } else {
      _rms_mV = NAN;
    }
    xSemaphoreGive(measurementoutMutex);
  } else {
    _decimateFromRing();
    const double fs_decim_meas = _bit_rate_bps / (double)_OSR;
    _freqHz = _estimateFreqHzPrecise(fs_decim_meas);
    if (_extractLastFullPeriod()) {
      //_computeRms();
      _rms_mV = _computeRmsOnePeriodExact();
      ok = true;
    } else {
      _rms_mV = NAN;
    }
  }
  return ok;
}

void AMC1306Measurement::_spiInitIfNeeded() {
  _spi.begin(_pinSclk, _pinMiso, -1, -1);
  _spi.beginTransaction(SPISettings(_F_CLK_HZ, MSBFIRST, SPI_MODE1));
}

void AMC1306Measurement::_clockAndFillRing() {
  uint32_t w = _ring_write;
  uint32_t spaceToEnd = _RING_BYTES - w;

  if (spaceToEnd > _BYTES_PER_CALL) {
    _spi.transferBytes(nullptr, &_ringbuf[w], _BYTES_PER_CALL);
    w += _BYTES_PER_CALL;

  } else if (spaceToEnd == _BYTES_PER_CALL) {
    _spi.transferBytes(nullptr, &_ringbuf[w], _BYTES_PER_CALL);
    w += _BYTES_PER_CALL; // == _RING_BYTES
    const uint32_t t = micros();
    if (_last_wrap_us) {
      const uint32_t dt = _usdiff(t, _last_wrap_us);
      if (dt) _bit_rate_bps = ((double)_RING_BYTES * 8.0) / ((double)dt * 1e-6);
    }
    _last_wrap_us = t;
    w = 0;

  } else {
    const size_t n1 = spaceToEnd;
    const size_t n2 = _BYTES_PER_CALL - n1;

    _spi.transferBytes(nullptr, &_ringbuf[w], n1);

    const uint32_t t = micros();
    if (_last_wrap_us) {
      const uint32_t dt = _usdiff(t, _last_wrap_us);
      if (dt) _bit_rate_bps = ((double)_RING_BYTES * 8.0) / ((double)dt * 1e-6);
    }
    _last_wrap_us = t;

    _spi.transferBytes(nullptr, &_ringbuf[0], n2);
    w = n2;
  }

  _ring_write = w;
}

float AMC1306Measurement::_cicToMilliVolts(long long y) const {
  const double G = (double)_OSR * (double)_OSR * (double)_OSR;
  return (float)((double)y / G * (double)_FS_mV);
}

void AMC1306Measurement::_decimateFromRing() {
  _cic.reset();
  _decim_count = 0;

  const uint32_t w_end = _ring_write;
  uint32_t bit_mod = 0;

  auto process_byte = [&](uint8_t b) {
    for (int k = 7; k >= 0; --k) {
      const int sgn = ((b >> k) & 1) ? +1 : -1;
      _cic.push(sgn);
      if (++bit_mod == _OSR) {
        bit_mod = 0;
        if (_decim_count < DECIM_MAX) {
          _decim[_decim_count++] = _cicToMilliVolts(_cic.decimate());
        }
      }
    }
  };

  const uint32_t first_len = _RING_BYTES - w_end;
  const uint8_t* p = &_ringbuf[w_end];
  for (uint32_t i = 0; i < first_len; ++i) process_byte(p[i]);

  p = &_ringbuf[0];
  for (uint32_t i = 0; i < w_end; ++i) process_byte(p[i]);
}

// Output_meas.cpp

float AMC1306Measurement::_estimateFreqHzPrecise(double fs_decim) const {
  if (_decim_count < 4) return NAN;
  const int MAXZ = 64;
  double zc[MAXZ]; int zcN = 0;
  for (uint32_t i=1; i<_decim_count && zcN<MAXZ; ++i) {
    float a=_decim[i-1], b=_decim[i];
    if (a < 0.0f && b >= 0.0f) {
      double frac = (b!=a) ? (-a)/(b-a) : 0.0;
      zc[zcN++] = (double)(i-1) + frac;
    }
  }
  if (zcN < 2) return NAN;
  int use = min(10, zcN-1);
  double sum=0.0;
  for (int k=zcN-1-use; k<zcN-1; ++k) sum += (zc[k+1]-zc[k]);
  double spp = sum / use;
  if (51.0f < (float)(2420 / spp) || (float)(2420 / spp) < 49.0f) {
    // outlier clamp
    Serial.println((float)(2420 / spp));
  }
  return (float)(2420 / spp);
}

bool AMC1306Measurement::_extractLastFullPeriod() {
  _lastPeriodCount = 0;
  if (_decim_count < 8) return false;

  const float  HYST = 2.0f;
  const double fs_decim_meas = _bit_rate_bps / (double)_OSR;
  const double fTarget = isfinite(_freqHz) ? (double)_freqHz : 50.0;
  const uint16_t EXP  = (uint16_t)(fs_decim_meas / fTarget + 0.5);
  const uint16_t MINP = (uint16_t)((double)EXP * 0.60);
  const uint16_t MAXP = (uint16_t)((double)EXP * 1.40);

  // Collect rising zero-crossings (with hysteresis)
  int zc[32]; int zcN = 0;
  bool below = (_decim[0] <= -HYST);
  for (uint32_t i = 1; i < _decim_count && zcN < 32; ++i) {
    const float v = _decim[i];
    if (below && v >= +HYST) { zc[zcN++] = (int)i; below = false; }
    else if (v <= -HYST) { below = true; }
  }

  // Choose the last plausible period between two rising crossings
  int i0 = -1, i1 = -1;
  for (int k = zcN - 2; k >= 0; --k) {
    const int d = zc[k + 1] - zc[k];
    if (d >= (int)MINP && d <= (int)MAXP) { i0 = zc[k]; i1 = zc[k + 1]; break; }
  }
  if (i0 < 0 && zcN >= 2) {
    int bestK = zcN - 2, bestErr = 0x7FFFFFFF;
    for (int k = zcN - 2; k >= 0; --k) {
      const int d = zc[k + 1] - zc[k];
      const int err = abs(d - (int)EXP);
      if (err < bestErr) { bestErr = err; bestK = k; }
    }
    i0 = zc[bestK]; i1 = zc[bestK + 1];
  }

  if (i0 >= 0 && i1 > i0) {
    // Shift start left to the last non-positive sample BEFORE i0
    int s = i0 - 1;
    while (s > 0 && _decim[s] > 0.0f) --s;   // walk left until ≤ 0 or index 0

    // End at i1 (already ≥ +HYST, i.e., positive)
    const int e = i1;

    const int N = e - s + 1;
    if (s < 0 || N <= 0 || N > (int)DECIM_MAX) return false;

    for (int i = 0; i < N; ++i) _lastPeriodBuf[i] = _decim[s + i];
    _lastPeriodCount = (size_t)N;
    return true;

  } else {
    // Fallback: use ~one period from the tail, but try to start on a negative sample.
    const uint32_t exp = (uint32_t)EXP ? (uint32_t)EXP : 50U;
    uint32_t start = (_decim_count > exp) ? (_decim_count - exp) : 0;

    // Nudge 'start' forward to a non-positive sample if possible
    uint32_t s = start;
    while (s + 1 < _decim_count && _decim[s] > 0.0f) ++s;

    const uint32_t N = _decim_count - s;
    if (N == 0 || N > DECIM_MAX) return false;

    for (uint32_t i = 0; i < N; ++i) _lastPeriodBuf[i] = _decim[s + i];
    _lastPeriodCount = (size_t)N;
    return true;
  }
}


void AMC1306Measurement::_computeRms() {
  if (_lastPeriodCount == 0) { _rms_mV = NAN; return; }
  double acc = 0.0;
  for (size_t i = 0; i < _lastPeriodCount; ++i) {
    const double x = (double)_lastPeriodBuf[i];
    acc += x * x;
  }
  _rms_mV = (float)sqrt(acc / (double)_lastPeriodCount);
}

// Find the last two *rising* zero-crossings (y crosses from <=0 to >0)
// Returns true on success and sets t0 and t1 as *fractional sample indices*.
bool AMC1306Measurement::_findLastTwoRisingZC(double& t0, double& t1) const {
  t0 = t1 = NAN;
  if (_decim_count < 4) return false;

  // Collect fractional rising zero-crossings across the current decimated buffer
  const int MAXZ = 64;
  double zc[MAXZ]; int zcN = 0;
  for (uint32_t i = 1; i < _decim_count && zcN < MAXZ; ++i) {
    const float a = _decim[i-1], b = _decim[i];
    if (a <= 0.0f && b > 0.0f) {
      const double denom = (double)b - (double)a;
      const double frac  = (denom != 0.0) ? (-(double)a) / denom : 0.0; // in [0,1)
      zc[zcN++] = (double)(i - 1) + frac; // fractional sample index
    }
  }
  if (zcN < 2) return false;

  t1 = zc[zcN - 1];
  t0 = zc[zcN - 2];
  return (t1 > t0);
}

// Compute exact RMS over *one period* between last two rising zero-crossings.
// Uses piecewise-linear y(t) between decimated samples and integrates y^2 exactly.
float AMC1306Measurement::_computeRmsOnePeriodExact() const {
  double t0, t1;
  if (!_findLastTwoRisingZC(t0, t1)) return NAN;

  // Sampling rate of the decimated stream
  const double fs = _bit_rate_bps / (double)_OSR;
  if (!(fs > 0.0)) return NAN;

  // If the period is too short, bail
  if (t1 <= t0 + 1e-9) return NAN;

  // Helper: integral of y^2 over a linear segment y0->y1 during duration dt
  auto seg_int_y2 = [](double y0, double y1, double dt) -> double {
    // Exact: dt * (y0^2 + y0*y1 + y1^2) / 3
    return dt * ( (y0*y0 + y0*y1 + y1*y1) / 3.0 );
  };

  // Build integral from t0 to t1
  const int i0 = (int)floor(t0);
  const int i1 = (int)floor(t1);

  double integral = 0.0;          // ∫ y^2 dt
  const double Ts = 1.0 / fs;

  // FIRST partial segment: from t0 (y=0) to (i0+1)
  if (i0 + 1 < (int)_decim_count) {
    const double y_end = (double)_decim[i0 + 1];
    const double dt    = ((double)(i0 + 1) - t0) * Ts;   // seconds
    integral += seg_int_y2(0.0, y_end, dt);
  } else {
    return NAN; // not enough samples to the right
  }

  // FULL segments: from k to k+1, for k = i0+1 .. i1-1
  for (int k = i0 + 1; k <= i1 - 1; ++k) {
    if (k + 1 >= (int)_decim_count) return NAN;
    const double y0 = (double)_decim[k];
    const double y1 = (double)_decim[k + 1];
    integral += seg_int_y2(y0, y1, Ts);
  }

  // LAST partial segment: from i1 to t1 (ending at y=0)
  if (i1 >= 0 && i1 < (int)_decim_count) {
    const double y_start = (double)_decim[i1];
    const double dt      = (t1 - (double)i1) * Ts;       // seconds
    integral += seg_int_y2(y_start, 0.0, dt);
  } else {
    return NAN;
  }

  // Period duration
  const double T = (t1 - t0) * Ts;  // seconds
  if (!(T > 0.0)) return NAN;

  // RMS over exactly one period
  const double mean_square = integral / T;
  return (mean_square > 0.0) ? (float)sqrt(mean_square) : 0.0f;
}


float AMC1306Measurement::calcFreqFast() {
  // Fast frequency estimate based on last period length.
  // Less accurate than _estimateFreqHzPrecise(), but very cheap.
  if (_lastPeriodCount < 8) return NAN;
  const double fs_decim_meas = _bit_rate_bps / (double)_OSR;
  const double spp = (double)_lastPeriodCount; // samples per period
  float zerotozero=0.0f;
  bool inperiod=false;
  for (size_t i = 1; i < _lastPeriodCount; ++i) {
    if (inperiod==true){
      zerotozero += 1.0f;
    }
    if ((_lastPeriodBuf[i-1] <= 0.0f && _lastPeriodBuf[i] > 0.0f) && inperiod==false) {
      zerotozero += _lastPeriodBuf[i]/(_lastPeriodBuf[i] - _lastPeriodBuf[i-1]);
      inperiod = true;
    }
    else if ((_lastPeriodBuf[i-1] <= 0.0f && _lastPeriodBuf[i] > 0.0f) && inperiod==true) {
      zerotozero += _lastPeriodBuf[i-1]/(_lastPeriodBuf[i-1] - _lastPeriodBuf[i]);
      inperiod = false;
    }

  }
  Serial.println(zerotozero);
  Serial.println(spp);
  Serial.println(fs_decim_meas);
  return (float)( fs_decim_meas / (double)zerotozero);
}


