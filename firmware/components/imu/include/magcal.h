// Magnetometer calibration (hard iron + axis-aligned soft iron), independent of the hardware so it can be
// tested on the host.
//
// The M5Stack's base holds magnets that move with the sensor, so the field the BMM150 sees is
// offset by a constant vector (hard iron, ~200 uT here) and the axes may be scaled differently (soft
// iron). While the unit is rotated in every direction the raw readings lie on an ellipsoid around that offset;
// we fit an axis-aligned ellipsoid
//     x^2 + B y^2 + C z^2 + D x + E y + F z + G = 0
// by linear least squares (streamed into 6x6 normal equations, no sample buffer), take its centre as the
// offset and scale each axis so the ellipsoid becomes a sphere whose radius is the geometric mean of the
// semi-axes (the local field strength).
#pragma once

#include <stdint.h>

namespace imu {

struct MagCal {
  float offset[3];  // uT, subtracted from the raw reading
  float scale[3];   // multiplied after the offset is subtracted
  float field;      // uT, radius of the fitted sphere (the local field strength)

  void apply(const float raw[3], float out[3]) const {
    for (int k = 0; k < 3; k++) out[k] = (raw[k] - offset[k]) * scale[k];
  }
};

enum class MagCalError : uint8_t {
  kNone = 0,
  kTooFewSamples,  // fewer than kMinSamples
  kNotRotated,     // some axis did not swing enough, or the fit is degenerate
  kBadFit,         // not an ellipsoid, implausible field / scale, or too noisy
  kNotCollecting,  // finish without a running collection (cancelled, or finished elsewhere)
  kNotSaved,       // fitted and applied, but NVS failed: it is lost at the next reboot
};

class MagCalFit {
 public:
  static constexpr uint32_t kMinSamples = 100;
  // Each axis must swing at least this much (peak to peak, uT) for a sample set to count as complete.
  // The field is ~45 uT in Japan, so turning an axis from pointing along it to against it gives ~90 uT.
  static constexpr float kFullSpanUt = 70;

  void reset();
  void add(const float raw[3]);
  uint32_t samples() const { return n_; }
  // 0..1 per axis: swing so far / kFullSpanUt.
  float coverage(int axis) const;
  bool complete() const;  // enough samples and every axis swung kFullSpanUt
  // Fits the samples so far. On success fills `out` and returns kNone.
  MagCalError solve(MagCal& out) const;

 private:
  double m_[6][6] = {};  // upper triangle of sum v v^T, v = (y^2, z^2, x, y, z, 1) (see magcal.cpp)
  double b_[6] = {};     // sum of v t, t = -x^2
  double tt_ = 0;        // sum of t^2
  uint32_t n_ = 0;
  float ref_[3];  // first sample: the origin of the fit's coordinates
  float min_[3], max_[3];
};

}  // namespace imu
