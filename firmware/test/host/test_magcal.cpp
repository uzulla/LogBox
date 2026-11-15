// Host tests for the magnetometer calibration fit (components/imu/magcal.cpp).
#include <math.h>

#include "magcal.h"
#include "unity.h"

namespace {

constexpr float kOffset[3] = {205, -150, 80};  // like the M5Stack base magnets
constexpr float kRadius[3] = {50, 41, 45};     // soft iron: the sphere of 45 uT squashed per axis

// Deterministic noise in [-1, 1].
float noise(uint32_t& seed) {
  seed = seed * 1664525u + 1013904223u;
  return static_cast<float>(seed >> 8) / (1u << 23) - 1.0f;
}

// n directions spread evenly over the sphere (Fibonacci lattice) or, with maxPolar < pi, over a cap.
void direction(int i, int n, float maxPolar, float d[3]) {
  const float z = 1 - (1 - cosf(maxPolar)) * (i + 0.5f) / n;
  const float r = sqrtf(fmaxf(0, 1 - z * z));
  const float phi = i * 2.399963f;
  d[0] = r * cosf(phi), d[1] = r * sinf(phi), d[2] = z;
}

void reading(const float d[3], float noiseUt, uint32_t& seed, float out[3]) {
  for (int k = 0; k < 3; k++) out[k] = kOffset[k] + kRadius[k] * d[k] + noiseUt * noise(seed);
}

}  // namespace

void test_magcal_recovers_offset_and_scale() {
  imu::MagCalFit fit;
  fit.reset();
  uint32_t seed = 1;
  constexpr int kN = 400;
  for (int i = 0; i < kN; i++) {
    float d[3], m[3];
    direction(i, kN, static_cast<float>(M_PI), d);
    reading(d, 1.0f, seed, m);
    fit.add(m);
  }
  TEST_ASSERT_TRUE(fit.complete());
  imu::MagCal cal;
  TEST_ASSERT_EQUAL(static_cast<int>(imu::MagCalError::kNone), static_cast<int>(fit.solve(cal)));
  const float field = cbrtf(kRadius[0] * kRadius[1] * kRadius[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, field, cal.field);
  for (int k = 0; k < 3; k++) {
    TEST_ASSERT_FLOAT_WITHIN(0.5f, kOffset[k], cal.offset[k]);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, field / kRadius[k], cal.scale[k]);
  }
  // Calibrated readings lie on a sphere of radius `field` around zero.
  for (int i = 0; i < 50; i++) {
    float d[3], m[3], c[3];
    direction(i * 7, kN, static_cast<float>(M_PI), d);
    reading(d, 0, seed, m);
    cal.apply(m, c);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, field, sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]));
  }
}

void test_magcal_rejects_too_few_and_no_rotation() {
  imu::MagCalFit fit;
  fit.reset();
  uint32_t seed = 2;
  imu::MagCal cal;
  float m[3];
  float d[3] = {0, 0, 1};
  for (int i = 0; i < 50; i++) {
    reading(d, 0.5f, seed, m);
    fit.add(m);
  }
  TEST_ASSERT_EQUAL(static_cast<int>(imu::MagCalError::kTooFewSamples), static_cast<int>(fit.solve(cal)));
  // Held still for long enough: still not rotated.
  for (int i = 0; i < 200; i++) {
    reading(d, 0.5f, seed, m);
    fit.add(m);
  }
  TEST_ASSERT_FALSE(fit.complete());
  TEST_ASSERT_EQUAL(static_cast<int>(imu::MagCalError::kNotRotated), static_cast<int>(fit.solve(cal)));
  // Only a small tilt (cap of 20 degrees): not rotated.
  fit.reset();
  for (int i = 0; i < 300; i++) {
    direction(i, 300, 0.35f, d);
    reading(d, 0.5f, seed, m);
    fit.add(m);
  }
  TEST_ASSERT_EQUAL(static_cast<int>(imu::MagCalError::kNotRotated), static_cast<int>(fit.solve(cal)));
}

void test_magcal_rejects_noise() {
  imu::MagCalFit fit;
  fit.reset();
  uint32_t seed = 3;
  for (int i = 0; i < 400; i++) {
    float m[3];
    for (int k = 0; k < 3; k++) m[k] = kOffset[k] + 60 * noise(seed);  // a cloud, not a shell
    fit.add(m);
  }
  imu::MagCal cal;
  TEST_ASSERT_NOT_EQUAL(static_cast<int>(imu::MagCalError::kNone), static_cast<int>(fit.solve(cal)));
}

void runMagCalTests() {
  RUN_TEST(test_magcal_recovers_offset_and_scale);
  RUN_TEST(test_magcal_rejects_too_few_and_no_rotation);
  RUN_TEST(test_magcal_rejects_noise);
}
