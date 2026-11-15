// Host tests for components/carframe: the car axes follow from gravity and the screen's right edge.
#include <math.h>

#include "carframe.h"
#include "unity.h"

namespace {
void feed(carframe::Estimator& e, const float a[3], int n, int gnssStill) {
  const float g[3] = {0, 0, 0};
  for (int i = 0; i < n; i++) e.update(a, g, gnssStill);
}
}  // namespace

// Screen upright facing the driver: BMI270 +Y up, -Z towards the driver (= backwards), -X right.
void test_carframe_upright() {
  carframe::Estimator e;
  const float rest[3] = {0, 1, 0};
  feed(e, rest, 10, 1);
  TEST_ASSERT_FALSE(e.ready());  // not still long enough
  feed(e, rest, 30, 1);
  TEST_ASSERT_TRUE(e.ready());
  float c[3];
  e.toCar(rest, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0, c[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0, c[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 1, c[2]);
  const float accel[3] = {0, 1, 0.4f};  // pushed forward (+Z = into the screen = forward)
  e.toCar(accel, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.4f, c[0]);
  const float left[3] = {0.3f, 1, 0};  // +X = screen left = car left
  e.toCar(left, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.3f, c[1]);
  const float yaw[3] = {0, 20, 0};  // rotation about up, counter-clockwise from above
  e.toCar(yaw, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 20, c[2]);
}

// Moving (GNSS says so): the reference is not learnt even if the IMU is calm.
void test_carframe_moving_does_not_learn() {
  carframe::Estimator e;
  const float a[3] = {0, 1, 0};
  feed(e, a, 100, 0);
  TEST_ASSERT_FALSE(e.ready());
  feed(e, a, 30, -1);  // no fix: judged from the IMU alone
  TEST_ASSERT_TRUE(e.ready());
}

// Lying flat, screen up: BMI270 -Z up. Right is still the screen's right, forward = the screen's top.
void test_carframe_flat() {
  carframe::Estimator e;
  const float rest[3] = {0, 0, -1};
  feed(e, rest, 30, 1);
  TEST_ASSERT_TRUE(e.ready());
  float c[3];
  const float top[3] = {0, 0.5f, -1};  // towards the screen's top
  e.toCar(top, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.5f, c[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 1, c[2]);
}

// GNSS already below 1 km/h while still braking: the braking must not be learnt as gravity.
void test_carframe_braking_not_learnt() {
  carframe::Estimator e;
  const float g[3] = {0, 0, 0};
  // Braking 0.3 G (backwards = -Z), with the deceleration easing off a little each record
  for (int i = 0; i < 40; i++) {
    const float a[3] = {0, 1, -0.3f + 0.1f * (i % 2)};  // jerky: the car is still pitching / braking
    e.update(a, g, 1);
  }
  TEST_ASSERT_FALSE(e.ready());
  const float rest[3] = {0, 1, 0};
  for (int i = 0; i < 30; i++) e.update(rest, g, 1);
  TEST_ASSERT_TRUE(e.ready());
  float c[3];
  e.toCar(rest, c);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0, c[0]);
}

void runCarframeTests() {
  RUN_TEST(test_carframe_braking_not_learnt);
  RUN_TEST(test_carframe_upright);
  RUN_TEST(test_carframe_moving_does_not_learn);
  RUN_TEST(test_carframe_flat);
}
