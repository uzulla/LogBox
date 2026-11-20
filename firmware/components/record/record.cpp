#include "record.h"

#include <math.h>
#include <stdio.h>

namespace record {
namespace {

// Howard Hinnant's days_from_civil / civil_from_days.
int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

void civilFromDays(int64_t z, int& y, unsigned& m, unsigned& d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = static_cast<unsigned>(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y = static_cast<int>(static_cast<int64_t>(yoe) + era * 400 + (m <= 2));
}

// "2026-11-15T05:12:34.120Z" with `digits` fractional digits (3 or 6); "" when us == 0.
void formatUtc(int64_t us, int digits, char* out, size_t size) {
  if (!us) {
    if (size) out[0] = '\0';
    return;
  }
  int64_t secs = us / 1000000;
  int64_t frac = us % 1000000;
  if (frac < 0) {
    frac += 1000000;
    secs -= 1;
  }
  int64_t days = secs / 86400;
  int64_t sod = secs % 86400;
  if (sod < 0) {
    sod += 86400;
    days -= 1;
  }
  int y;
  unsigned m, d;
  civilFromDays(days, y, m, d);
  snprintf(out, size, "%04d-%02u-%02uT%02d:%02d:%02d.%0*ldZ", y, m, d, static_cast<int>(sod / 3600),
           static_cast<int>(sod / 60 % 60), static_cast<int>(sod % 60), digits,
           static_cast<long>(digits == 3 ? frac / 1000 : frac));
}

}  // namespace

int64_t unixMicros(const gnss::Pvt& p) {
  if (!p.dateTimeValid()) return 0;
  const int64_t days = daysFromCivil(p.year, p.month, p.day);
  const int64_t secs = days * 86400 + p.hour * 3600 + p.min * 60 + p.sec;
  return secs * 1000000 + p.nano / 1000;
}

size_t csvHeader(char* buf, size_t size, unsigned session, unsigned seq, unsigned part) {
  const int n = snprintf(
      buf, size,
      "# session %u seq %u part %u\n"
      "# logbo (M5Stack + GNSS module: NEO-M9N / BMI270 / BMM150 / BMP280). One row per GNSS epoch, "
      "t_ms = ms since the recording started (continues across the 5-minute files of one recording)\n"
      "# lat/lon: deg, alt: m (MSL), speed: km/h, heading: deg (of motion), hacc: m, "
      "acc: G and gyro: deg/s (BMI270 axes, mean over the epoch), batt/charging: not available (empty), "
      "mag: uT (BMM150 axes; mag_cal 1 = hard / soft iron calibrated, 0 = raw), press: hPa, temp: C (BMP280), "
      "head_acc: deg and speed_acc: km/h (receiver's 1-sigma estimates)\n"
      "t_ms,utc,fix,sats,lat,lon,alt_m,speed_kmh,heading_deg,hacc_m,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,"
      "batt,charging,mag_x,mag_y,mag_z,press_hpa,temp_c,mag_cal,head_acc_deg,speed_acc_kmh\n",
      session, seq, part);
  return n > 0 && static_cast<size_t>(n) < size ? static_cast<size_t>(n) : 0;
}

size_t csvRow(const Record& r, int64_t startUs, char* buf, size_t size) {
  const gnss::Pvt& p = r.pvt;
  const imu::Sample& s = r.imu;
  char utc[32];
  formatUtc(r.hasPvt ? unixMicros(p) : 0, 3, utc, sizeof(utc));
  char acc[64] = ",,,,,";
  if (s.n) snprintf(acc, sizeof(acc), "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f", s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
  char mag[32] = ",,";
  if (s.magValid) snprintf(mag, sizeof(mag), "%.1f,%.1f,%.1f", s.mx, s.my, s.mz);
  char baro[24] = ",";
  const char* magCal = s.magValid ? (s.magCal ? "1" : "0") : "";
  char acc2[32] = ",";
  if (r.hasPvt) snprintf(acc2, sizeof(acc2), "%.2f,%.2f", p.headAcc / 1e5, p.sAcc * 0.0036);
  if (s.baroValid) snprintf(baro, sizeof(baro), "%.2f,%.2f", s.pressureHpa, s.tempC);
  const int n = snprintf(buf, size, "%ld,%s,%u,%u,%.7f,%.7f,%.1f,%.2f,%.2f,%.1f,%s,,,%s,%s,%s,%s\n",
                         static_cast<long>((r.atUs - startUs) / 1000), utc, p.fixType, p.numSV, p.lat / 1e7,
                         p.lon / 1e7, p.hMSL / 1000.0, p.gSpeed * 0.0036, p.headMot / 1e5, p.hAcc / 1000.0, acc,
                         mag, baro, magCal, acc2);
  return n > 0 && static_cast<size_t>(n) < size ? static_cast<size_t>(n) : 0;
}

void jsonEscape(const char* s, char* out, size_t size) {
  size_t o = 0;
  for (; *s && o + 7 < size; s++) {
    const unsigned char c = static_cast<unsigned char>(*s);
    if (c == '"' || c == '\\') {
      out[o++] = '\\';
      out[o++] = c;
    } else if (c < 0x20) {
      o += snprintf(out + o, size - o, "\\u%04x", c);
    } else {
      out[o++] = c;
    }
  }
  if (size) out[o] = '\0';
}

// "2022-01-10T08:51:08.239971626Z" (nanoseconds, as esp32-racebox-bridge) from the PVT date / time / nano.
static void formatPvtTime(const gnss::Pvt& p, char* out, size_t size) {
  int64_t secs = daysFromCivil(p.year, p.month, p.day) * 86400 + p.hour * 3600 + p.min * 60 + p.sec;
  int64_t nano = p.nano;
  if (nano < 0) {
    nano += 1000000000;
    secs -= 1;
  }
  char head[32];
  formatUtc(secs * 1000000, 6, head, sizeof(head));  // "....T..:..:..." + ".000000Z"
  head[19] = '\0';                                     // keep "YYYY-MM-DDTHH:MM:SS"
  snprintf(out, size, "\"%s.%09ldZ\"", head, static_cast<long>(nano));
}

size_t toJson(const Record& r, uint32_t seq, const char* device, char* buf, size_t size) {
  const gnss::Pvt& p = r.pvt;
  const imu::Sample& s = r.imu;
  const char* t = "true";
  const char* f = "false";
  const bool timeOk = r.hasPvt && p.dateTimeValid();
  char time[48] = "null";
  if (timeOk) formatPvtTime(p, time, sizeof(time));
  char dev[64];
  jsonEscape(device ? device : "", dev, sizeof(dev));
  // A NaN / inf would print as "nan" / "inf" and make the whole datagram invalid JSON: such values become null.
  const auto finite3 = [](float a, float b, float c) { return isfinite(a) && isfinite(b) && isfinite(c); };
  char car[128] = "\"g\":null,\"gyro_dps\":null";
  if (r.carValid && finite3(r.carG[0], r.carG[1], r.carG[2]) && finite3(r.carGyro[0], r.carGyro[1], r.carGyro[2])) {
    snprintf(car, sizeof(car), "\"g\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"gyro_dps\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
             r.carG[0], r.carG[1], r.carG[2], r.carGyro[0], r.carGyro[1], r.carGyro[2]);
  }
  char raw[128] = "\"g_sensor\":null,\"gyro_sensor\":null";
  if (s.n && finite3(s.ax, s.ay, s.az) && finite3(s.gx, s.gy, s.gz)) {
    snprintf(raw, sizeof(raw),
             "\"g_sensor\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"gyro_sensor\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}", s.ax,
             s.ay, s.az, s.gx, s.gy, s.gz);
  }
  char mag[64] = "null";
  if (s.magValid && finite3(s.mx, s.my, s.mz)) snprintf(mag, sizeof(mag), "{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f}", s.mx, s.my, s.mz);
  char press[16] = "null", temp[16] = "null";
  if (s.baroValid && isfinite(s.pressureHpa) && isfinite(s.tempC)) {
    snprintf(press, sizeof(press), "%.2f", s.pressureHpa);
    snprintf(temp, sizeof(temp), "%.2f", s.tempC);
  }
  const int n = snprintf(
      buf, size,
      "{\"type\":\"racebox\",\"seq\":%lu,\"device\":\"%s\","
      "\"itow\":%lu,\"time\":%s,\"time_valid\":%s,\"time_confirmed\":%s,\"time_acc_ns\":%lu,"
      "\"fix\":%u,\"fix_ok\":%s,\"sats\":%u,"
      "\"lat\":%.7f,\"lon\":%.7f,\"latlon_valid\":%s,"
      "\"alt_wgs_m\":%.3f,\"alt_msl_m\":%.3f,\"h_acc_m\":%.3f,\"v_acc_m\":%.3f,"
      "\"speed_mps\":%.3f,\"speed_acc_mps\":%.3f,"
      "\"heading_deg\":%.5f,\"heading_acc_deg\":%.5f,\"heading_valid\":%s,"
      "\"pdop\":%.2f,\"battery\":null,%s,"
      "\"gnss\":%s,\"vel_ned_mps\":{\"n\":%.3f,\"e\":%.3f,\"d\":%.3f},%s,\"mag_ut\":%s,\"mag_cal\":%s,\"press_hpa\":%s,\"temp_c\":%s}",
      static_cast<unsigned long>(seq), dev, static_cast<unsigned long>(p.iTOW), time, timeOk ? t : f,
      timeOk && (p.flags2 & 0xC0) == 0xC0 ? t : f, static_cast<unsigned long>(p.tAcc), p.fixType, p.fixOk() ? t : f,
      p.numSV, p.lat / 1e7, p.lon / 1e7, r.hasPvt && !(p.flags3 & 0x01) && p.fixType >= 2 ? t : f, p.height / 1e3,
      p.hMSL / 1e3, p.hAcc / 1e3, p.vAcc / 1e3, p.gSpeed / 1e3, p.sAcc / 1e3, p.headMot / 1e5, p.headAcc / 1e5,
      p.headingValid() ? t : f, p.pDOP / 100.0, car, r.hasPvt ? t : f, p.velN / 1e3, p.velE / 1e3, p.velD / 1e3, raw, mag, s.magValid && s.magCal ? t : f,
      press, temp);
  return n > 0 && static_cast<size_t>(n) < size ? static_cast<size_t>(n) : 0;
}

}  // namespace record
