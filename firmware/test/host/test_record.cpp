// Host tests for components/record: the CSV row must stay readable by log-viewer (same leading columns as the
// RaceBox logs), and the JSON must fit one UDP datagram.
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "record.h"
#include "unity.h"

namespace {

record::Record sample() {
  record::Record r = {};
  r.atUs = 1'234'567;
  r.hasPvt = true;
  gnss::Pvt& p = r.pvt;
  p.year = 2026, p.month = 11, p.day = 15, p.hour = 5, p.min = 6, p.sec = 7;
  p.valid = 0x07;  // date, time, fully resolved
  p.nano = -1'000'000;  // -1 ms: the second rolls back
  p.fixType = 3, p.flags = 0x01, p.numSV = 14;
  p.lat = 356'812'360, p.lon = 1'397'671'250;
  p.hMSL = 40'500, p.hAcc = 1'200, p.gSpeed = 27'778, p.headMot = 9'000'000;
  r.imu.n = 8;
  r.imu.ax = 0.1f, r.imu.ay = -0.25f, r.imu.az = 0.98f;
  r.imu.gx = 1.5f, r.imu.gy = -2.0f, r.imu.gz = 30.25f;
  r.imu.magValid = true, r.imu.magCal = true, r.imu.mx = 20, r.imu.my = -5, r.imu.mz = 40;
  r.imu.baroValid = true, r.imu.pressureHpa = 1013.25f, r.imu.tempC = 21.5f;
  return r;
}

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  size_t start = 0;
  for (;;) {
    const size_t c = s.find(',', start);
    out.push_back(s.substr(start, c == std::string::npos ? std::string::npos : c - start));
    if (c == std::string::npos) return out;
    start = c + 1;
  }
}

std::string headerRow() {
  char buf[1024];
  const size_t n = record::csvHeader(buf, sizeof(buf), 12, 1);
  TEST_ASSERT_GREATER_THAN(0, n);
  std::string h(buf, n);
  const size_t at = h.find("t_ms,");
  TEST_ASSERT_TRUE(at != std::string::npos);
  return h.substr(at, h.size() - at - 1);  // without '\n'
}

}  // namespace

void test_csv_header_keeps_racebox_columns() {
  const std::string h = headerRow();
  TEST_ASSERT_EQUAL(0, h.rfind("t_ms,utc,fix,sats,lat,lon,alt_m,speed_kmh,heading_deg,hacc_m,acc_x,acc_y,acc_z,"
                               "gyro_x,gyro_y,gyro_z,batt,charging,", 0));
}

void test_csv_row() {
  char buf[512];
  const size_t n = record::csvRow(sample(), 234'567, buf, sizeof(buf));
  TEST_ASSERT_GREATER_THAN(0, n);
  TEST_ASSERT_EQUAL_CHAR('\n', buf[n - 1]);
  const std::string row(buf, n - 1);
  const auto f = split(row);
  TEST_ASSERT_EQUAL(split(headerRow()).size(), f.size());
  TEST_ASSERT_EQUAL_STRING("1000", f[0].c_str());
  TEST_ASSERT_EQUAL_STRING("2026-11-15T05:06:06.999Z", f[1].c_str());
  TEST_ASSERT_EQUAL_STRING("3", f[2].c_str());
  TEST_ASSERT_EQUAL_STRING("35.6812360", f[4].c_str());
  TEST_ASSERT_EQUAL_STRING("100.00", f[7].c_str());  // 27.778 m/s
  TEST_ASSERT_EQUAL_STRING("0.980", f[12].c_str());
  TEST_ASSERT_EQUAL_STRING("30.25", f[15].c_str());
  TEST_ASSERT_EQUAL_STRING("", f[16].c_str());  // batt
  TEST_ASSERT_EQUAL_STRING("1013.25", f[21].c_str());
  TEST_ASSERT_EQUAL_STRING("1", f[23].c_str());  // mag_cal
}

void test_csv_row_without_gnss_and_sensors() {
  record::Record r = {};
  r.atUs = 5000;
  char buf[512];
  const size_t n = record::csvRow(r, 0, buf, sizeof(buf));
  TEST_ASSERT_GREATER_THAN(0, n);
  const auto f = split(std::string(buf, n - 1));
  TEST_ASSERT_EQUAL(split(headerRow()).size(), f.size());
  TEST_ASSERT_EQUAL_STRING("", f[1].c_str());   // no UTC
  TEST_ASSERT_EQUAL_STRING("0", f[2].c_str());  // no fix
  TEST_ASSERT_EQUAL_STRING("", f[10].c_str());  // no IMU
  TEST_ASSERT_EQUAL_STRING("", f[23].c_str());  // no mag
}

void test_json_fits_and_fields() {
  char buf[record::kMaxUdpPayload + 1];
  const size_t n = record::toJson(sample(), 7, buf, sizeof(buf));
  TEST_ASSERT_GREATER_THAN(0, n);
  TEST_ASSERT_TRUE(strstr(buf, "\"type\":\"sample\",\"seq\":7,\"gnss\":true") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"time\":\"2026-11-15T05:06:06.999000Z\"") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"press_hpa\":1013.25") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"mag_cal\":true") != nullptr);
  // Widest values still fit one datagram.
  record::Record w = sample();
  w.pvt.lat = -900'000'000, w.pvt.lon = -1'800'000'000;
  w.pvt.height = w.pvt.hMSL = -2'000'000'000;
  w.pvt.hAcc = w.pvt.vAcc = w.pvt.sAcc = w.pvt.headAcc = w.pvt.tAcc = 4'000'000'000u;
  w.pvt.velN = w.pvt.velE = w.pvt.velD = w.pvt.gSpeed = w.pvt.headMot = -2'000'000'000;
  w.pvt.iTOW = 4'000'000'000u;
  w.imu.ax = w.imu.ay = w.imu.az = -16.0f;
  w.imu.gx = w.imu.gy = w.imu.gz = -2000.0f;
  w.imu.mx = w.imu.my = w.imu.mz = -2500.0f;
  TEST_ASSERT_GREATER_THAN(0, record::toJson(w, 4'000'000'000u, buf, sizeof(buf)));
}

void runRecordTests() {
  RUN_TEST(test_csv_header_keeps_racebox_columns);
  RUN_TEST(test_csv_row);
  RUN_TEST(test_csv_row_without_gnss_and_sensors);
  RUN_TEST(test_json_fits_and_fields);
}
