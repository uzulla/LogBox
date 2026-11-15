#include "carframe.h"

#include <math.h>

namespace carframe {
namespace {
constexpr int kStillSamples = 25;   // records (~1 s at 25 Hz) of calm before learning gravity
constexpr float kStillRotDps = 5;   // any axis
constexpr float kStillJerkG = 0.03f;
constexpr float kRefAlpha = 0.05f;  // low-pass of the gravity reference while still

float dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
}  // namespace

void Estimator::update(const float a[3], const float gyro[3], int gnssStill) {
  float jerk = 0;
  for (int i = 0; i < 3; i++) {
    jerk = fmaxf(jerk, fabsf(a[i] - prev_[i]));
    prev_[i] = a[i];
  }
  const float rot = fmaxf(fabsf(gyro[0]), fmaxf(fabsf(gyro[1]), fabsf(gyro[2])));
  const bool calm = rot < kStillRotDps && (gnssStill >= 0 ? gnssStill == 1 : jerk < kStillJerkG);
  still_ = calm ? (still_ < 1000 ? still_ + 1 : still_) : 0;
  if (still_ < kStillSamples) return;
  for (int i = 0; i < 3; i++) ref_[i] = haveRef_ ? ref_[i] + (a[i] - ref_[i]) * kRefAlpha : a[i];
  haveRef_ = true;

  const float n = sqrtf(dot(ref_, ref_));
  if (n < 0.5f) return;
  for (int i = 0; i < 3; i++) up_[i] = ref_[i] / n;
  // Right: the screen's right edge projected onto the horizontal plane. Forward = up x right, left = -right.
  const float k = dot(kScreenRight, up_);
  float r[3] = {kScreenRight[0] - k * up_[0], kScreenRight[1] - k * up_[1], kScreenRight[2] - k * up_[2]};
  const float rn = sqrtf(dot(r, r));
  if (rn < 0.3f) {  // screen-right points up or down: lateral is undefined
    ready_ = false;
    return;
  }
  for (float& v : r) v /= rn;
  fwd_[0] = up_[1] * r[2] - up_[2] * r[1];
  fwd_[1] = up_[2] * r[0] - up_[0] * r[2];
  fwd_[2] = up_[0] * r[1] - up_[1] * r[0];
  for (int i = 0; i < 3; i++) left_[i] = -r[i];
  ready_ = true;
}

void Estimator::toCar(const float v[3], float out[3]) const {
  out[0] = dot(v, fwd_);
  out[1] = dot(v, left_);
  out[2] = dot(v, up_);
}

}  // namespace carframe
