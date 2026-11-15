// One logged sample: the GNSS solution of one epoch (UBX-NAV-PVT) plus the IMU / magnetometer /
// barometer values averaged over that epoch. Formatting to the CSV log row and the UDP JSON lives
// here so both stay in one place and can be unit-tested on the host.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "gnss.h"
#include "imu.h"

namespace record {

// Upper bound for one UDP datagram (unfragmented on a 1500-byte MTU link, with headroom).
constexpr size_t kMaxUdpPayload = 1000;

struct Record {
  int64_t atUs;      // esp_timer_get_time() when the record was made
  bool hasPvt;       // false: no GNSS message this epoch (receiver silent), pvt is zeroed
  gnss::Pvt pvt;
  imu::Sample imu;
};

// UTC of the PVT as microseconds since the Unix epoch (nano field applied), 0 if not valid.
int64_t unixMicros(const gnss::Pvt& p);

// CSV: `#` comment lines and the header row (ends with '\n'), then one row per record.
// The columns up to `charging` are the same as the RaceBox bridge logs, so log-viewer reads both.
size_t csvHeader(char* buf, size_t size, unsigned session, unsigned seq);
// Returns the length written (row ends with '\n'), 0 if it did not fit.
size_t csvRow(const Record& r, int64_t startUs, char* buf, size_t size);

// One JSON object (no trailing newline), type "sample". Returns the length, 0 if it did not fit.
size_t toJson(const Record& r, uint32_t seq, char* buf, size_t size);

// Copies s into out escaping it for a JSON string (each input byte takes at most 6 output bytes).
void jsonEscape(const char* s, char* out, size_t size);

}  // namespace record
