// Interface between the logger (main/: GNSS + IMU -> SD / Wi-Fi UDP) and the display. The logger knows
// nothing about the screen: it fills a Status and hands it over; the display calls back on button presses.
#pragma once

#include <stdint.h>

namespace logger {

struct Status {
  // Links
  bool configWindow;     // BLE config service is being advertised
  bool wifiConfigured;
  bool wifiUp;
  bool apOn, staOn;      // radio switches of this boot (off = not started)
  int8_t rssi;
  char ip[16];
  char ssid[33];
  int apClients;         // stations joined to our soft AP
  uint32_t udpErrors;
  uint8_t rec;           // SD log: 0 none, 1 recording, 2 saving (do not remove), 3 safe to remove, 4 failed
  char logName[16];      // current / last log file, e.g. "log_0012_01.csv"

  // Sensors
  bool imuOk, magOk, baroOk;
  bool gnssConfigured;   // receiver answered and runs our configuration (UBX, rate)
  uint32_t gnssBaud;
  uint32_t gnssMessages; // NAV-PVT received since boot
  uint32_t records;      // records made since boot (GNSS epochs, or IMU-only while GNSS is silent)

  // Latest record; meaningful when dataAtUs != 0.
  int64_t dataAtUs;      // esp_timer_get_time() of the last record, 0 = none yet
  int64_t gnssAtUs;      // same, of the last GNSS message
  int64_t unixUs;        // UTC (microseconds since the Unix epoch), 0 = not valid
  uint8_t fix;           // 0 none, 2 2D, 3 3D
  bool fixOk;
  uint8_t sats;
  int32_t lat, lon;      // deg * 1e7
  int32_t altMm;         // MSL
  uint32_t hAccMm;
  int32_t speedMms;
  int32_t headingE5;     // deg * 1e5
  bool headingValid;
  float ax, ay, az;      // G, BMI270 axes, mean over the epoch
  float gx, gy, gz;      // deg/s
  float mx, my, mz;      // uT (calibrated when magCalibrated)
  bool carValid;         // carG is known (gravity learnt while standing still)
  float carG[3];         // G in the car's axes: x forward, y left, z up (see carframe.h)

  // Magnetometer calibration
  bool magCalibrated;
  float magField;        // uT, field strength of the calibration
  bool magCollecting;    // calibration running: rotate the unit
  uint32_t magCalSamples;
  float magCoverage[3];  // 0..1 per axis
  bool magCalComplete;   // enough rotation: finishing now will very likely succeed
  uint32_t magCalSession;  // changes on every start of a collection (also from the serial console)
  uint8_t magCalResult;  // last finish: 0 none yet, 1 ok, 2 too few samples, 3 not rotated, 4 bad fit,
                         // 5 not running, 6 applied but not saved
  int64_t magCalResultAtUs;
  float pressureHpa, tempC;
};

// Called once from app_main (after sdlog::init(): the LCD shares the SD card's SPI bus).
void displayStart();
// Called whenever the state changes (cheap; may be called 25+ times/s).
void displayUpdate(const Status& s);

// Implemented by the logger, called by the display on a long press: stops the SD recording and makes the
// card safe to remove; otherwise tries to start recording (after SAFE, SD ERR, or a card inserted late).
void onLongPress();

// Implemented by the logger, called by the display: start collecting for the magnetometer calibration,
// finish it (fit, apply, save; the result shows up in Status::magCalResult), or cancel it.
uint32_t onMagCalStart();  // returns the new session (Status::magCalSession once the logger reports it)
void onMagCalFinish();
void onMagCalCancel();

}  // namespace logger
