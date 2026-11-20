// IMU and barometer of the M5Stack GNSS Module (on an M5Stack Gray): BMI270 accel/gyro, BMM150
// magnetometer behind the BMI270's auxiliary I2C interface, BMP280 barometer. All of them are read by
// one sampling task over the internal I2C bus (port 0, SDA GPIO21, SCL GPIO22, 400 kHz).
//
// Accel/gyro: BMI270 FIFO, 200 Hz, normal filter, +-8 g and +-2000 deg/s (a spin or a crash can exceed
// 1000 deg/s; even at 2000 deg/s one LSB is 0.06 deg/s, below the sensor noise). Every FIFO sample is
// used exactly once by takeAverage().
// Mag: BMM150 in forced mode (enhanced preset), triggered and read by the BMI270 at 25 Hz.
// Baro: BMP280 normal mode, pressure x16 / temperature x2 oversampling, IIR filter 4, about 26 Hz.
//
// Axes: accel and gyro are the BMI270's own axes. The mag axes are the BMM150's own (raw) axes: the
// module's documentation does not say how the BMM150 is placed relative to the BMI270, so they are NOT
// remapped and may differ from the accel/gyro axes by a rotation.
#pragma once

#include <stdint.h>

#include "magcal.h"

namespace imu {

struct Sample {
  float ax, ay, az;          // G, BMI270 sensor axes
  float gx, gy, gz;          // deg/s, BMI270 sensor axes
  float mx, my, mz;          // uT, BMM150 axes (not remapped, see above)
  bool magValid;
  bool magCal;               // mx..mz are calibrated (hard / soft iron); false = raw
  float pressureHpa, tempC;  // BMP280
  bool baroValid;
  uint32_t n;                // accel/gyro samples averaged into this Sample (0 = none since last call)
};

struct Status {
  bool bmi270, bmm150, bmp280;
};

// Creates the I2C master bus on port 0 (GPIO21/22), initialises the three sensors and starts a sampling
// task. Sensors that fail are reported in Status and skipped; never aborts. Call once.
// Missing sensors are looked for again: once more at once after freeing the bus (a sensor may hold SDA low
// when the ESP32 was reset in the middle of a transfer), then every 5 s by the sampling task.
Status init();
// Sensors running now (they may show up after init(), see above). Thread-safe.
Status status();
// Times the I2C bus was freed since boot.
uint32_t busResets();

// Mean of the accel/gyro samples since the previous call, plus the latest mag and baro values.
// Thread-safe, non-blocking. When n is 0 the accel/gyro fields hold the latest single sample (zeros if
// there never was one).
void takeAverage(Sample& out);

// Latest single values (for the display), thread-safe. n is 1 once there is an accel/gyro sample, else 0.
void latest(Sample& out);

// Magnetometer calibration (see magcal.h). The calibration is kept in NVS (namespace "imu"; nvs_flash_init()
// must have run before init()) and applied to every mag reading; Sample::magCal tells whether it was.
// Collecting: rotate the unit slowly in every direction, then magCalFinish() (or wait for complete).
struct MagCalInfo {
  bool calibrated;     // a calibration is applied
  MagCal cal;          // valid when calibrated
  bool collecting;
  uint32_t session;    // changes on every start (a restart resets samples / coverage)
  uint32_t samples;    // collected so far
  float coverage[3];   // 0..1 per raw axis
  bool complete;       // enough samples and rotation: finishing will very likely succeed
};
uint32_t magCalStart();           // starts (or restarts) collecting; the current calibration stays applied.
                                  // Returns the new session number (MagCalInfo::session)
MagCalError magCalFinish();       // fits, and on success applies and saves the result; stops collecting.
                                  // kNotCollecting when nothing is being collected, kNotSaved when applied
                                  // but NVS failed
void magCalCancel();
bool magCalClear();               // stops any collection and forgets the calibration (raw readings from now
                                  // on); false when NVS failed
                                  // (the old one comes back at the next reboot)
MagCalInfo magCalInfo();

// Battery, from (in this order) the M5Stack Dual 16340 Battery Base's INA226 (I2C 0x45 on the same bus:
// pack voltage and current, the level estimated from the voltage) or the Gray's power IC (IP5306, I2C 0x75: the
// level in 4 steps, 25/50/75/100 %; 0 = nearly empty). The voltage-based levels jump while charging or under
// load. Units with neither report nothing.
struct Battery {
  bool valid;          // false: no battery source (or not read yet)
  uint8_t level;       // 0..100 % (IP5306: 0, 25, 50, 75, 100)
  bool charging;       // USB power is in and the battery is charging
  bool full;           // USB power is in and the battery is full
  bool hasVoltage;     // voltageMv / currentMa are valid (battery base)
  uint16_t voltageMv;  // pack voltage
  int16_t currentMa;   // into the pack (charging) positive, out of it negative
};
// Reads the source at most every 2 s (cached in between); call after init(). Thread-safe.
Battery battery();

}  // namespace imu
