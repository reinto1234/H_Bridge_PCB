#include "Output_meas.h"

static const char* OM_TAG = "OutputMeasurements";

// ----- Static LUT -----
OutputMeasurements::bytecic_t OutputMeasurements::s_lut[256];
bool OutputMeasurements::s_lut_built = false;

void OutputMeasurements::buildByteCicLut() {
  if (s_lut_built) return;
  for (int b = 0; b < 256; ++b) {
    int s[8];
#if OM_BIT_ONE_IS_POSITIVE
    for (int k = 0; k < 8; ++k) s[k] = ((b >> (7 - k)) & 1) ? +1 : -1;
#else
    for (int k = 0; k < 8; ++k) s[k] = ((b >> (7 - k)) & 1) ? -1 : +1;
#endif
    int S1 = 0, prefix[8];
    for (int k = 0; k < 8; ++k) { S1 += s[k]; prefix[k] = (k ? prefix[k-1] : 0) + s[k]; }
    int PS1 = 0; for (int j = 0; j < 8; ++j) PS1 += prefix[j];
    int SS  = 0; for (int m = 0; m < 8; ++m) SS  += (9 - (m + 1)) * prefix[m];
    s_lut[b].S1  = (int8_t)S1;
    s_lut[b].PS1 = (int16_t)PS1;
    s_lut[b].SS  = (int16_t)SS;
  }
  s_lut_built = true;
}

// ----- ctor -----
OutputMeasurements::OutputMeasurements(uint32_t osr,
                                       float fs_mv,
                                       float initial_bit_rate_bps,
                                       size_t decim_ring_len,
                                       size_t max_scan_len,
                                       size_t max_period_len)
: OSR(osr),
  FS_mV(fs_mv),
  bit_rate_bps(initial_bit_rate_bps),
  DECIM_RING(decim_ring_len),
  MAX_SCAN(max_scan_len),
  MAX_PERIOD(max_period_len) {}

// ----- init -----
bool OutputMeasurements::init(const char* log_name) {
  log_tag = log_name ? log_name : "OM";
  buildByteCicLut();

  uint32_t caps = MALLOC_CAP_8BIT;
#if defined(BOARD_HAS_PSRAM) || defined(CONFIG_SPIRAM_SUPPORT)
  if (psramFound()) caps |= MALLOC_CAP_SPIRAM;
#endif

  size_t ring = DECIM_RING, scan = MAX_SCAN, per = MAX_PERIOD;
  while (true) {
    decim_ring = (float*)heap_caps_malloc(ring * sizeof(float), caps);
    win        = (float*)heap_caps_malloc(scan * sizeof(float), caps);
    period_buf = (float*)heap_caps_malloc(per  * sizeof(float), caps);
    if (decim_ring && win && period_buf) {
      DECIM_RING = ring; MAX_SCAN = scan; MAX_PERIOD = per;
      memset(decim_ring, 0, ring * sizeof(float));
      memset(win,        0, scan * sizeof(float));
      memset(period_buf, 0, per  * sizeof(float));
      ESP_LOGI(OM_TAG, "%s buffers: ring=%u (%.1f KB), win=%u (%.1f KB), period=%u (%.1f KB), caps=0x%x",
               log_tag,
               (unsigned)DECIM_RING, (DECIM_RING*sizeof(float))/1024.0f,
               (unsigned)MAX_SCAN,   (MAX_SCAN*sizeof(float))/1024.0f,
               (unsigned)MAX_PERIOD, (MAX_PERIOD*sizeof(float))/1024.0f, caps);
      break;
    }
    if (decim_ring) { heap_caps_free(decim_ring); decim_ring = nullptr; }
    if (win)        { heap_caps_free(win);        win = nullptr; }
    if (period_buf) { heap_caps_free(period_buf); period_buf = nullptr; }
    if (ring <= 2048 || scan <= 1024 || per <= 512) {
      ESP_LOGE(OM_TAG, "FATAL: buffer alloc failed");
      return false;
    }
    ring >>= 1; scan >>= 1; per >>= 1;
  }

  snap_mutex = xSemaphoreCreateMutex();
  if (!snap_mutex) {
    ESP_LOGE(OM_TAG, "FATAL: snapshot mutex alloc failed");
    return false;
  }

  memset(&cic, 0, sizeof(cic));
  // keep bit_rate_bps as provided
  widx = 0; count = 0;
  t_us_start = t_us_end = 0;

  return true;
}

// ----- ring window copy & stats -----
size_t OutputMeasurements::copyRecentWindowWithStats(
    const float* ring, size_t ring_len, size_t widx, size_t count, size_t max_scan,
    float* dst, float& mean, float& vmin, float& vmax)
{
  size_t avail = count;
  if (avail == 0) return 0;
  size_t N = (max_scan > avail) ? avail : max_scan;

  size_t start = (widx + ring_len - N) % ring_len;

  float acc = 0.0f;
  vmin = +FLT_MAX; vmax = -FLT_MAX;
  for (size_t i = 0; i < N; ++i) {
    size_t idx = (start + i) % ring_len;
    float v = ring[idx];
    dst[i] = v;
    acc += v;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }
  mean = acc / (float)N;
  return N;
}

// ----- rising ZC with hysteresis -----
bool OutputMeasurements::findLastTwoRisingZcHyst(const float* win, size_t N,
                                                  float& t0, float& t1, int& zcCount,
                                                  float hyst_mV)
{
  t0 = t1 = NAN; zcCount = 0;
  if (N < 4) return false;
  bool below = (win[0] <= -hyst_mV);
  float last2 = NAN, last1 = NAN;
  for (size_t i = 1; i < N; ++i) {
    float v = win[i];
    if (below && v >= +hyst_mV) {
      float a = win[i - 1], b = win[i];
      float A = a + hyst_mV, B = b - hyst_mV;
      float denom = B - A;
      float frac  = (denom != 0.0f) ? (-(A)) / denom : 0.0f;
      if (frac < 0.0f) frac = 0.0f; if (frac > 1.0f) frac = 1.0f;
      float idx = (float)(i - 1) + frac;
      last2 = last1; last1 = idx; ++zcCount; below = false;
    } else if (v <= -hyst_mV) {
      below = true;
    }
  }
  if (zcCount >= 2 && last2 < last1) { t0 = last2; t1 = last1; return true; }
  return false;
}

// ----- exact RMS over one period (piecewise linear) -----
float OutputMeasurements::computeRmsOnePeriodExact(const float* win, size_t N,
                                                   float t0, float t1, float fs_decim)
{
  if (!(fs_decim > 0.0f) || !(t1 > t0)) return NAN;
  const float Ts = 1.0f / fs_decim;
  int i0 = (int)floorf(t0);
  int i1 = (int)floorf(t1);
  if (i0 < 0) i0 = 0;
  if (i1 < 0) return NAN;
  if (i0 + 1 >= (int)N || i1 >= (int)N) return NAN;

  auto seg_int_y2 = [](float y0, float y1, float dt) -> float {
    return (dt > 0.0f) ? dt * ((y0*y0 + y0*y1 + y1*y1) / 3.0f) : 0.0f;
  };

  float integral = 0.0f;
  { // first partial (assume y(t0)=0 at rising ZC)
    const float y_end = win[i0 + 1];
    const float dt    = ((float)(i0 + 1) - t0) * Ts;
    integral += seg_int_y2(0.0f, y_end, dt);
  }
  for (int k = i0 + 1; k <= i1 - 1; ++k) {
    if (k + 1 >= (int)N) break;
    integral += seg_int_y2(win[k], win[k + 1], Ts);
  }
  { // last partial (assume y(t1)=0 at rising ZC)
    const float y_start = win[i1];
    const float dt      = (t1 - (float)i1) * Ts;
    integral += seg_int_y2(y_start, 0.0f, dt);
  }

  const float T = (t1 - t0) * Ts;
  if (!(T > 0.0f)) return NAN;
  float ms = integral / T;
  if (ms < 0.0f) ms = 0.0f;
  return sqrtf(ms);
}

// ----- processing -----
void OutputMeasurements::processRxBytesAndUpdateBitrate(const uint8_t* buf, size_t nbytes, uint64_t dt_us) {
  const uint16_t BYTES_PER_OSR = (uint16_t)(OSR / 8);

  for (size_t i = 0; i < nbytes; ++i) {
    const bytecic_t bc = s_lut[buf[i]];
    long long p1 = cic.i1, p2 = cic.i2;

    cic.i1 = p1 + (long long)bc.S1;
    cic.i2 = p2 + 8LL * p1 + (long long)bc.PS1;
    cic.i3 = cic.i3 + 8LL * p2 + 36LL * p1 + (long long)bc.SS;

    if (++cic.osr_byte_phase >= BYTES_PER_OSR) {
      cic.osr_byte_phase = 0;
      long long y0 = cic.i3;
      long long d1 = y0 - cic.c1; cic.c1 = y0;
      long long d2 = d1 - cic.c2; cic.c2 = d1;
      long long d3 = d2 - cic.c3; cic.c3 = d2;
      pushDecim(cicToVolts(d3, OSR, FS_mV));
    }
  }
  updateBitRate(bit_rate_bps, dt_us, nbytes);
}

// ----- analyze -----
void OutputMeasurements::analyzeStep() {
  t_us_start = micros();

  if (!win || count < 32) { t_us_end = micros(); return; }

  float mean = 0.0f, vmin = 0.0f, vmax = 0.0f;
  size_t N = copyRecentWindowWithStats(decim_ring, DECIM_RING, widx, count, MAX_SCAN, win, mean, vmin, vmax);
  if (N < 32) { t_us_end = micros(); return; }

  for (size_t i = 0; i < N; ++i) win[i] = win[i] - mean;

  const float pp = vmax - vmin;
  const float hyst = max(0.002f, min(0.02f * pp, 50.0f));

  float t0, t1; int zcCount = 0;
  Snapshot newsnap {NAN, NAN, 0};

  if (findLastTwoRisingZcHyst(win, N, t0, t1, zcCount, hyst)) {
    int s = (int)floorf(t0);
    int e = (int)floorf(t1);
    if (s < 0) s = 0;
    if (e >= (int)N) e = (int)N - 1;

    if (e > s) {
      const float fs_decim = bit_rate_bps / (float)OSR;
      const float spp = (t1 - t0);
      newsnap.freq_hz = (spp > 0.0f) ? (fs_decim / spp) : NAN;

      uint16_t countCopy = (uint16_t)min((int)MAX_PERIOD, e - s + 1);
      for (uint16_t i = 0; i < countCopy; ++i) period_buf[i] = win[s + i];
      newsnap.n_period = countCopy;

      newsnap.rms_mV = computeRmsOnePeriodExact(win, N, t0, t1, fs_decim);
    }
  }

  if (xSemaphoreTake(snap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    snap = newsnap;
    xSemaphoreGive(snap_mutex);
  }

  t_us_end = micros();
}

// ----- snapshot -----
bool OutputMeasurements::getSnapshot(Snapshot& out) const {
  bool ok = false;
  if (xSemaphoreTake(snap_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    out = snap;
    ok = (isfinite(out.freq_hz) || out.n_period > 0);
    xSemaphoreGive(snap_mutex);
  }
  return ok;
}

uint16_t OutputMeasurements::copyLastPeriod(float* dst, uint16_t dst_len) const {
  if (!dst || dst_len == 0) return 0;
  uint16_t copied = 0;
  if (xSemaphoreTake(snap_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    const uint16_t n = min((uint16_t)dst_len, snap.n_period);
    for (uint16_t i = 0; i < n; ++i) dst[i] = period_buf[i];
    copied = n;
    xSemaphoreGive(snap_mutex);
  }
  return copied;
}
