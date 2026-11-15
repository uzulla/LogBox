#include "magcal.h"

#include <math.h>

namespace imu {
namespace {

// Samples are taken relative to the first one (so the coordinates stay within about two field radii
// of zero even with a 200 uT offset) and in units of kUnit uT: this keeps the normal equations well
// conditioned.
constexpr double kUnit = 50;

// Accepted results.
constexpr float kMinField = 15, kMaxField = 120;  // uT; the Earth's field is 25..65 uT
constexpr float kMinScale = 0.6f, kMaxScale = 1.6f;
constexpr double kMaxRadialRms = 0.08;     // RMS of (distance from the centre / radius - 1)
constexpr float kMinCoverageToSolve = 0.5f;

// Solves a x = b (6x6) by Gaussian elimination with partial pivoting. False when (nearly) singular.
bool solve6(double a[6][6], double b[6], double x[6]) {
  double maxDiag = 0;
  for (int i = 0; i < 6; i++) maxDiag = fmax(maxDiag, fabs(a[i][i]));
  if (maxDiag == 0) return false;
  for (int c = 0; c < 6; c++) {
    int p = c;
    for (int r = c + 1; r < 6; r++) {
      if (fabs(a[r][c]) > fabs(a[p][c])) p = r;
    }
    if (fabs(a[p][c]) < 1e-12 * maxDiag) return false;
    if (p != c) {
      for (int k = 0; k < 6; k++) {
        const double t = a[c][k];
        a[c][k] = a[p][k];
        a[p][k] = t;
      }
      const double t = b[c];
      b[c] = b[p];
      b[p] = t;
    }
    for (int r = c + 1; r < 6; r++) {
      const double f = a[r][c] / a[c][c];
      for (int k = c; k < 6; k++) a[r][k] -= f * a[c][k];
      b[r] -= f * b[c];
    }
  }
  for (int r = 5; r >= 0; r--) {
    double s = b[r];
    for (int k = r + 1; k < 6; k++) s -= a[r][k] * x[k];
    x[r] = s / a[r][r];
  }
  return true;
}

}  // namespace

void MagCalFit::reset() {
  for (int i = 0; i < 6; i++) {
    b_[i] = 0;
    for (int j = 0; j < 6; j++) m_[i][j] = 0;
  }
  tt_ = 0;
  n_ = 0;
}

void MagCalFit::add(const float raw[3]) {
  if (!n_) {
    for (int k = 0; k < 3; k++) ref_[k] = min_[k] = max_[k] = raw[k];
  }
  double u[3];
  for (int k = 0; k < 3; k++) {
    if (raw[k] < min_[k]) min_[k] = raw[k];
    if (raw[k] > max_[k]) max_[k] = raw[k];
    u[k] = (raw[k] - ref_[k]) / kUnit;
  }
  const double v[6] = {u[1] * u[1], u[2] * u[2], u[0], u[1], u[2], 1};
  const double t = -u[0] * u[0];
  tt_ += t * t;
  for (int i = 0; i < 6; i++) {
    b_[i] += v[i] * t;
    for (int j = i; j < 6; j++) m_[i][j] += v[i] * v[j];
  }
  n_++;
}

float MagCalFit::coverage(int axis) const {
  if (!n_) return 0;
  const float c = (max_[axis] - min_[axis]) / kFullSpanUt;
  return c < 1 ? c : 1;
}

bool MagCalFit::complete() const {
  return n_ >= kMinSamples && coverage(0) >= 1 && coverage(1) >= 1 && coverage(2) >= 1;
}

MagCalError MagCalFit::solve(MagCal& out) const {
  if (n_ < kMinSamples) return MagCalError::kTooFewSamples;
  for (int k = 0; k < 3; k++) {
    if (coverage(k) < kMinCoverageToSolve) return MagCalError::kNotRotated;
  }
  double a[6][6], b[6], p[6];
  for (int i = 0; i < 6; i++) {
    b[i] = b_[i];
    for (int j = 0; j < 6; j++) a[i][j] = j >= i ? m_[i][j] : m_[j][i];
  }
  if (!solve6(a, b, p)) return MagCalError::kNotRotated;

  // x^2 + B y^2 + C z^2 + D x + E y + F z + G = 0, i.e. (x - cx)^2 + B (y - cy)^2 + C (z - cz)^2 = R
  // with cx = -D / 2, cy = -E / 2B, cz = -F / 2C, R = cx^2 + B cy^2 + C cz^2 - G. An ellipsoid has B, C, R > 0.
  const double k[3] = {1, p[0], p[1]};
  double c[3], rr = -p[5];
  for (int i = 0; i < 3; i++) {
    if (!(k[i] > 0)) return MagCalError::kBadFit;
    c[i] = -p[2 + i] / (2 * k[i]);
    rr += k[i] * c[i] * c[i];
  }
  if (!(rr > 0)) return MagCalError::kBadFit;
  double r[3];
  for (int i = 0; i < 3; i++) r[i] = sqrt(rr / k[i]);
  const double field = cbrt(r[0] * r[1] * r[2]);

  // Residual from the normal equations: sum (v.p - t)^2 = p'Mp - 2 p'b + sum t^2. A sample at (1 + d) times
  // the radius has an algebraic error of about 2 R d.
  double pmp = 0, pb = 0;
  for (int i = 0; i < 6; i++) {
    pb += p[i] * b_[i];
    for (int j = 0; j < 6; j++) pmp += p[i] * (j >= i ? m_[i][j] : m_[j][i]) * p[j];
  }
  const double sse = fmax(0, pmp - 2 * pb + tt_);
  const double radialRms = sqrt(sse / n_) / (2 * rr);
  if (!(radialRms <= kMaxRadialRms)) return MagCalError::kBadFit;

  MagCal cal;
  cal.field = static_cast<float>(field * kUnit);
  if (cal.field < kMinField || cal.field > kMaxField) return MagCalError::kBadFit;
  for (int k = 0; k < 3; k++) {
    cal.offset[k] = static_cast<float>(ref_[k] + c[k] * kUnit);
    cal.scale[k] = static_cast<float>(field / r[k]);
    if (cal.scale[k] < kMinScale || cal.scale[k] > kMaxScale) return MagCalError::kBadFit;
  }
  out = cal;
  return MagCalError::kNone;
}

}  // namespace imu
