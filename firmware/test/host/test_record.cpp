// Host tests for components/record: the CSV row must stay readable by log-viewer (same leading columns as the
// RaceBox logs), and the JSON must fit one UDP datagram.
#include <math.h>
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
  p.headAcc = 123'000, p.sAcc = 150;  // 1.23 deg, 0.15 m/s
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
  char buf[record::kMaxCsvHeader];
  const size_t n = record::csvHeader(buf, sizeof(buf), 12, 1, 3);
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

void test_csv_header_fits() {
  // sdlog writes the header from a kMaxCsvHeader buffer: a longer header means no log file at all.
  char buf[record::kMaxCsvHeader];
  TEST_ASSERT_GREATER_THAN(0, record::csvHeader(buf, sizeof(buf), 9999, 99, 999));
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
  TEST_ASSERT_EQUAL_STRING("1.23", f[24].c_str());  // head_acc_deg
  TEST_ASSERT_EQUAL_STRING("0.54", f[25].c_str());  // speed_acc_kmh
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
  TEST_ASSERT_EQUAL_STRING("", f[24].c_str());  // no GNSS
}

void test_json_fits_and_fields() {
  char buf[record::kMaxUdpPayload + 1];
  record::Record r = sample();
  r.carValid = true;
  r.carG[0] = -0.3f, r.carG[1] = 0.25f, r.carG[2] = 0.98f;
  r.carGyro[0] = 1, r.carGyro[1] = 2, r.carGyro[2] = -15.5f;
  r.pvt.flags2 = 0xE0, r.pvt.nano = 123'456'789;
  const size_t n = record::toJson(r, 7, "logbo-06FA", buf, sizeof(buf));
  TEST_ASSERT_GREATER_THAN(0, n);
  // Same leading members as esp32-racebox-bridge, so lapcam's osd-feed (match type=racebox, g_x / g_y,
  // speed_mps, lat / lon / latlon_valid) and other receivers written for it work unchanged.
  TEST_ASSERT_TRUE(strstr(buf, "{\"type\":\"racebox\",\"seq\":7,\"device\":\"logbo-06FA\",\"itow\":") == buf);
  TEST_ASSERT_TRUE(strstr(buf, "\"time\":\"2026-11-15T05:06:07.123456789Z\",\"time_valid\":true,\"time_confirmed\":true") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"lat\":35.6812360,\"lon\":139.7671250,\"latlon_valid\":true") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"speed_mps\":27.778,") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"g\":{\"x\":-0.300,\"y\":0.250,\"z\":0.980},\"gyro_dps\":{\"x\":1.00,\"y\":2.00,\"z\":-15.50}") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"gnss\":true,\"vel_ned_mps\":{") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"g_sensor\":{\"x\":0.100,") != nullptr);
  // Car axes unknown yet: null (osd-feed hides the ball)
  r.carValid = false;
  TEST_ASSERT_GREATER_THAN(0, record::toJson(r, 7, "logbo-06FA", buf, sizeof(buf)));
  TEST_ASSERT_TRUE(strstr(buf, "\"g\":null,\"gyro_dps\":null") != nullptr);
  // NaN never reaches the JSON (it would make the datagram invalid)
  r.carValid = true;
  r.carG[1] = NAN;
  r.imu.mx = INFINITY;
  TEST_ASSERT_GREATER_THAN(0, record::toJson(r, 7, "logbo-06FA", buf, sizeof(buf)));
  TEST_ASSERT_TRUE(strstr(buf, "\"g\":null,\"gyro_dps\":null") != nullptr);
  TEST_ASSERT_TRUE(strstr(buf, "\"mag_ut\":null") != nullptr);
  TEST_ASSERT_NULL(strstr(buf, "nan"));
  TEST_ASSERT_NULL(strstr(buf, "inf"));
  // Negative nano rolls the second back
  r = sample();
  TEST_ASSERT_GREATER_THAN(0, record::toJson(r, 7, "x", buf, sizeof(buf)));
  TEST_ASSERT_TRUE(strstr(buf, "\"time\":\"2026-11-15T05:06:06.999000000Z\"") != nullptr);
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
  w.carValid = true;
  for (int k = 0; k < 3; k++) w.carG[k] = -16.0f, w.carGyro[k] = -2000.0f;
  w.pvt.pDOP = 65535, w.pvt.fixType = 255, w.pvt.numSV = 255, w.pvt.tAcc = 4'000'000'000u;
  w.imu.baroValid = true, w.imu.pressureHpa = -1000.0f, w.imu.tempC = -100.0f;
  const size_t wn = record::toJson(w, 4'000'000'000u, "logbo-FFFF", buf, sizeof(buf));
  printf("widest JSON: %zu bytes\n", wn);
  TEST_ASSERT_GREATER_THAN(0, wn);
}

void runRecordTests() {
  RUN_TEST(test_csv_header_keeps_racebox_columns);
  RUN_TEST(test_csv_header_fits);
  RUN_TEST(test_csv_row);
  RUN_TEST(test_csv_row_without_gnss_and_sensors);
  RUN_TEST(test_json_fits_and_fields);
}
