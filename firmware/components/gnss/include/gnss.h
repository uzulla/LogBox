// u-blox M9 (NEO-M9N) GNSS receiver on a UART: detection, configuration via
// UBX-CFG-VALSET and NAV-PVT delivery. This header has no ESP-IDF dependencies
// so the Pvt type can be used by host tests.
#pragma once

#include <stdint.h>

namespace gnss {

// Minimum ground speed for headingValid() on receivers without sensor fusion.
// Below this the u-blox motion heading is unreliable (and frozen by the
// receiver's static hold), so 0.5 m/s (1.8 km/h) is used.
constexpr int32_t kHeadingMinSpeedMmps = 500;

struct Pvt {  // UBX-NAV-PVT, raw protocol units
  uint32_t iTOW;        // ms of GPS week
  uint16_t year;
  uint8_t month, day, hour, min, sec;
  uint8_t valid;        // bit0 validDate, bit1 validTime, bit2 fullyResolved, bit3 validMag
  uint32_t tAcc;        // ns
  int32_t nano;         // ns, -1e9..1e9
  uint8_t fixType;      // 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
  uint8_t flags;        // bit0 gnssFixOK, bit1 diffSoln, bit5 headVehValid, bit6-7 carrSoln
  uint8_t flags2;       // bit5 confirmedAvai, bit6 confirmedDate, bit7 confirmedTime
  uint8_t numSV;
  int32_t lon, lat;     // deg * 1e7
  int32_t height, hMSL; // mm
  uint32_t hAcc, vAcc;  // mm
  int32_t velN, velE, velD, gSpeed;  // mm/s
  int32_t headMot;      // deg * 1e5
  uint32_t sAcc;        // mm/s
  uint32_t headAcc;     // deg * 1e5
  uint16_t pDOP;        // * 0.01
  uint8_t flags3;       // bit0 invalidLlh

  bool dateTimeValid() const { return (valid & 0x07) == 0x07; }
  bool fixOk() const { return (fixType == 3 || fixType == 4) && (flags & 0x01); }
  // Valid horizontal position: also a 2D fix (whose altitude is only the last one held).
  bool posOk() const { return fixType >= 2 && fixType <= 4 && (flags & 0x01); }
  // headVehValid is only ever set by sensor-fusion (ADR/UDR) receivers; a plain
  // NEO-M9N never sets it, so headMot is also accepted when there is a good fix
  // and the ground speed exceeds kHeadingMinSpeedMmps.
  bool headingValid() const {
    return (flags & 0x20) || (fixOk() && gSpeed > kHeadingMinSpeedMmps);
  }
};

struct Stats {
  uint32_t pvt, checksumErrors, otherMsgs, bytes;
  uint32_t baud;     // current ESP-side UART baud (0 until detected)
  bool configured;   // receiver configured and NAV-PVT flowing
  uint16_t rateHz;   // measurement rate in effect (after any fallback), 0 if not configured
};

// Satellites in view (UBX-NAV-SAT) and the dilutions of precision (UBX-NAV-DOP), both requested about
// once a second, for the display.
struct Sat {
  uint8_t gnssId;   // 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS
  uint8_t svId;
  uint8_t cno;      // dBHz, 0 = no signal
  int8_t elev;      // deg, -90..90 (out of range: unknown)
  int16_t azim;     // deg, 0..360 (unknown when the elevation is)
  bool used;        // used in the navigation solution
};
constexpr int kMaxSats = 64;

struct Sky {
  uint32_t iTOW;
  int numSvs;            // entries in sats (the receiver may report more than kMaxSats; the rest are dropped)
  Sat sats[kMaxSats];
  // Counts over every satellite of the message (not only the kept ones), per gnssId (0..7) and in total.
  int totalSvs, totalUsed, totalSignal;
  uint8_t viewBy[8], usedBy[8];
  int64_t satAtUs;       // esp_timer time of the last NAV-SAT, 0 = none yet
  uint16_t gDOP, pDOP, tDOP, vDOP, hDOP;  // * 0.01
  int64_t dopAtUs;       // same, NAV-DOP; 0 again whenever a NAV-PVT has no valid fix (DOP of an earlier fix)
};

using Callback = void (*)(const Pvt&);

// Installs the UART driver (UART_NUM_2) and starts the GNSS task (priority 10).
// rateHz is clamped to 1..25. cb is called from the GNSS task for every
// NAV-PVT; keep it fast. Call once.
void start(int rxPin, int txPin, int rateHz, Callback cb);
Stats stats();
// Copies the latest satellites / DOP into out (about 1 KB: keep it static).
void sky(Sky& out);

}  // namespace gnss
