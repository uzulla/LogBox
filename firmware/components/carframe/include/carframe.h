// Vehicle axes from the IMU: "up" is the gravity vector measured while standing still, "right" is the
// screen's right-hand edge laid into the horizontal plane, "forward" = up x right. With the screen facing the
// driver (upright or tilted back) this is the car's frame. Shared by the display (G circle) and the UDP
// output, which uses the RaceBox convention (x forward, y left, z up) so receivers written for
// esp32-racebox-bridge work unchanged.
#pragma once

namespace carframe {

// Screen axes in BMI270 axes, checked on the board: +Y points to the top of the screen, -Z out of the screen
// (towards the viewer) and -X to the right (tilting the screen to the right makes acc_x positive).
constexpr float kScreenRight[3] = {-1, 0, 0};

class Estimator {
 public:
  // a: G, gyro: deg/s, both in BMI270 axes (mean over the epoch). gnssStill: 1 = GNSS says standing (fix,
  // < 1 km/h), 0 = moving, -1 = unknown (no fix): then "still" is judged from the IMU alone (no rotation,
  // steady acceleration). After ~1 s still the gravity reference is (re)learnt.
  void update(const float a[3], const float gyro[3], int gnssStill);
  bool ready() const { return ready_; }
  // v (BMI270 axes) -> RaceBox axes: x forward, y left, z up. Only meaningful when ready().
  void toCar(const float v[3], float out[3]) const;

 private:
  float ref_[3] = {};
  bool haveRef_ = false, ready_ = false;
  float prev_[3] = {};
  int still_ = 0;
  float fwd_[3] = {}, left_[3] = {}, up_[3] = {};
};

}  // namespace carframe
