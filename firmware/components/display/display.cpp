// M5Stack (Basic / Gray) display: ILI9342C 320x240 on the SPI bus shared with the TF card (initialised
// by components/sdlog), and the three buttons under the screen.
//
// There is no PSRAM, so a full RGB565 frame (150 KB) does not fit next to Wi-Fi and BLE. The frame is
// drawn in horizontal bands instead: the whole scene is drawn once per band, clipped to it, into one of
// two band buffers, and sent by DMA while the next band is drawn.
//
// Buttons: A (left) / C (right) switch pages, B (middle) held for 1.5 s stops / starts the SD recording;
// a short press of B on the G page resets the peak values, on the STATUS page it starts the magnetometer
// calibration (a page of its own until it ends: B finishes, A / C cancel).
//
// Pages: 0 speed + G, 1 G circle, 2 status.

#include "logger_status.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <algorithm>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace logger {
namespace {

const char* TAG = "display";

constexpr spi_host_device_t kHost = SPI3_HOST;  // initialised by sdlog (shared with the TF card)
constexpr gpio_num_t kCs = GPIO_NUM_14, kDc = GPIO_NUM_27, kRst = GPIO_NUM_33, kBl = GPIO_NUM_32;
constexpr gpio_num_t kBtnA = GPIO_NUM_39, kBtnB = GPIO_NUM_38, kBtnC = GPIO_NUM_37;  // low = pressed
constexpr int W = 320, H = 240;
constexpr int kBandH = 24;  // 10 bands per frame, 15 KB per band buffer
constexpr int kFrameMs = 100;
constexpr int kPages = 3;

// The panel takes big-endian RGB565, so the buffers hold byte-swapped values.
constexpr uint16_t rgb(int r, int g, int b) {
  const uint16_t v = static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
  return static_cast<uint16_t>((v << 8) | (v >> 8));
}

constexpr uint16_t kBg = rgb(5, 8, 16), kBar = rgb(12, 28, 52), kCyan = rgb(0, 205, 255),
                   kWhite = rgb(235, 240, 250), kDim = rgb(105, 118, 140), kTrack = rgb(26, 33, 48),
                   kGreen = rgb(50, 230, 120), kAmber = rgb(255, 176, 0), kRed = rgb(255, 70, 70),
                   kSpeed = rgb(255, 255, 255), kGray = rgb(150, 156, 168), kGrid = rgb(40, 52, 72);

#include "font5x7.inc"

// ---------------------------------------------------------------------------
// Band canvas: every primitive is clipped to the band [bandY, bandY + kBandH).

uint16_t* bufs[2];
uint16_t* cv;  // band being drawn
int bandY;
SemaphoreHandle_t bufFree;  // counts free band buffers
esp_lcd_panel_handle_t panel;

inline void px(int x, int y, uint16_t c) {
  if (x >= 0 && x < W && y >= bandY && y < bandY + kBandH) cv[(y - bandY) * W + x] = c;
}

void fillRect(int x, int y, int w, int h, uint16_t c) {
  const int x0 = std::max(x, 0), x1 = std::min(x + w, W);
  const int y0 = std::max(y, bandY), y1 = std::min(y + h, bandY + kBandH);
  for (int j = y0; j < y1; j++) {
    uint16_t* row = cv + (j - bandY) * W;
    for (int i = x0; i < x1; i++) row[i] = c;
  }
}

void disc(int cx, int cy, int r, uint16_t c) {
  for (int j = std::max(-r, bandY - cy); j <= std::min(r, bandY + kBandH - 1 - cy); j++) {
    const int half = static_cast<int>(sqrtf(static_cast<float>(r * r + r - j * j)));
    fillRect(cx - half, cy + j, 2 * half + 1, 1, c);
  }
}

void circle(int cx, int cy, int r, uint16_t c) {
  // Midpoint circle, 1 px.
  int x = r, y = 0, err = 1 - r;
  while (x >= y) {
    const int pts[8][2] = {{x, y}, {y, x}, {-y, x}, {-x, y}, {-x, -y}, {-y, -x}, {y, -x}, {x, -y}};
    for (const auto& p : pts) px(cx + p[0], cy + p[1], c);
    y++;
    if (err < 0) {
      err += 2 * y + 1;
    } else {
      x--;
      err += 2 * (y - x) + 1;
    }
  }
}

constexpr int kGlyphW = 6;  // 5 columns + 1 gap

// Draws s with the 5x7 font enlarged `scale` times. Returns the x after the last character.
int text(int x, int y, const char* s, int scale, uint16_t c) {
  if (y > bandY + kBandH || y + 7 * scale < bandY) return x + static_cast<int>(strlen(s)) * kGlyphW * scale;
  for (; *s; s++, x += kGlyphW * scale) {
    const char ch = (*s < 0x20 || *s > 0x7E) ? '?' : *s;
    const uint8_t* g = FONT5X7[ch - 0x20];
    for (int cx = 0; cx < 5; cx++)
      for (int cy = 0; cy < 7; cy++)
        if (g[cx] & (1 << cy)) fillRect(x + cx * scale, y + cy * scale, scale, scale, c);
  }
  return x;
}

int textWidth(const char* s, int scale) { return static_cast<int>(strlen(s)) * kGlyphW * scale - scale; }
int textRight(int xRight, int y, const char* s, int scale, uint16_t c) {
  return text(xRight - textWidth(s, scale), y, s, scale, c);
}
int textCenter(int cx, int y, const char* s, int scale, uint16_t c) {
  return text(cx - textWidth(s, scale) / 2, y, s, scale, c);
}

// Seven-segment digit, w x h, segment thickness t. d < 0: only the middle segment ("-").
void segDigit(int x, int y, int w, int h, int t, int d, uint16_t c) {
  static const uint8_t kSeg[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};  // gfedcba
  const uint8_t m = d < 0 ? 0x40 : kSeg[d % 10];
  const int hh = (h - t) / 2;
  if (m & 0x01) fillRect(x + t, y, w - 2 * t, t, c);                   // a
  if (m & 0x02) fillRect(x + w - t, y + t, t, hh - t, c);              // b
  if (m & 0x04) fillRect(x + w - t, y + hh + t, t, hh - t, c);         // c
  if (m & 0x08) fillRect(x + t, y + h - t, w - 2 * t, t, c);           // d
  if (m & 0x10) fillRect(x, y + hh + t, t, hh - t, c);                 // e
  if (m & 0x20) fillRect(x, y + t, t, hh - t, c);                      // f
  if (m & 0x40) fillRect(x + t, y + hh, w - 2 * t, t, c);              // g
}

// ---------------------------------------------------------------------------
// State (written by the logger through displayUpdate, read by the display task)

Status st;
portMUX_TYPE stLock = portMUX_INITIALIZER_UNLOCKED;
int page = 0;

constexpr int64_t kMagResultUs = 5000000;        // the calibration result stays on the STATUS page this long
constexpr int64_t kMagTimeoutUs = 120000000;     // collecting is cancelled after this long
constexpr uint32_t kMagAutoFinishSamples = 375;  // 15 s at 25 Hz: finish by itself once complete

int kmh10(int32_t mms) { return static_cast<int>(static_cast<int64_t>(mms) * 36 / 1000); }

// G circle: the car-axis G from the logger (carframe: gravity learnt while standing still, lateral along the
// screen's left-right axis), shown as the G the driver feels.
struct GState {
  bool haveRef = false;
  float lat = 0, lon = 0, total = 0;  // G
  static constexpr int kTrail = 50;   // 2 s at 25 Hz
  float trail[kTrail][2] = {};
  int trailN = 0, trailPos = 0;
  float peakLat = 0, peakLon = 0;
} gs;
GState gv;  // display task's copy of gs

void updateG(const Status& s) {
  if (!s.carValid) return;  // until the unit has stood still once
  gs.haveRef = true;
  // Shown as the G the driver feels (like a ball in a bowl): opposite to the acceleration. Braking moves it
  // forward (up), a left turn to the right, and tilting the unit moves it towards the side that goes down.
  // carG: x forward, y left (z up), with gravity only on z.
  gs.lat = s.carG[1];
  gs.lon = -s.carG[0];
  gs.total = sqrtf(gs.lat * gs.lat + gs.lon * gs.lon);
  gs.trail[gs.trailPos][0] = gs.lat;
  gs.trail[gs.trailPos][1] = gs.lon;
  gs.trailPos = (gs.trailPos + 1) % GState::kTrail;
  gs.trailN = std::min(gs.trailN + 1, GState::kTrail);
  gs.peakLat = fmaxf(gs.peakLat, fabsf(gs.lat));
  gs.peakLon = fmaxf(gs.peakLon, fabsf(gs.lon));
}

// ---------------------------------------------------------------------------
// Widgets

void fmtCoord(char* out, size_t n, int32_t v) {  // 6 decimals, 1e-7 degree input
  const int64_t a = v < 0 ? -static_cast<int64_t>(v) : v;
  snprintf(out, n, "%s%d.%06d", v < 0 ? "-" : "", static_cast<int>(a / 10000000), static_cast<int>(a % 10000000 / 10));
}

void drawSignal(int x, int y, int bars, uint16_t on) {  // 4 bars, 3 px wide, up to 13 px high
  for (int i = 0; i < 4; i++) {
    const int h = 4 + i * 3;
    fillRect(x + i * 4, y + 13 - h, 3, h, i < bars ? on : kTrack);
  }
}

// SD state word with a dot; the dot blinks at 1 Hz while recording.
void drawRec(int x, int y, int rec, int scale, int64_t now) {
  static const struct { const char* word; uint16_t col; } kRec[] = {
      {"NO SD", kDim}, {"REC", kRed}, {"SAVING", kAmber}, {"SAFE", kGreen}, {"SD ERR", kRed}};
  if (rec > 4) rec = 0;
  if (rec != 1 || (now / 500000) % 2 == 0) disc(x + 3 * scale, y + 3 * scale + scale / 2, 3 * scale, kRec[rec].col);
  text(x + 8 * scale, y, kRec[rec].word, scale, kRec[rec].col);
}

bool gnssFresh(const Status& s, int64_t now) { return s.gnssAtUs && now - s.gnssAtUs < 2000000; }

// Top bar on every page: SD state, GNSS, Wi-Fi, clock (JST).
void drawTopBar(const Status& s, int64_t now) {
  if (bandY >= 22) return;
  fillRect(0, 0, W, 21, kBar);
  fillRect(0, 21, W, 1, kCyan);
  drawRec(4, 4, s.rec, 2, now);

  char b[24];
  const bool fresh = gnssFresh(s, now);
  const uint16_t gc = fresh && s.fixOk ? kGreen : fresh ? kAmber : kRed;
  snprintf(b, sizeof(b), "%s%d", !fresh ? "GPS -" : s.fixOk ? "3D " : s.fix == 2 ? "2D " : "NO ", fresh ? s.sats : 0);
  if (!fresh) snprintf(b, sizeof(b), "GPS --");
  text(110, 4, b, 2, gc);

  const int bars = !s.wifiUp ? 0 : s.rssi > -55 ? 4 : s.rssi > -65 ? 3 : s.rssi > -75 ? 2 : 1;
  drawSignal(204, 4, bars, s.wifiUp ? kGreen : s.apClients ? kCyan : kDim);
  if (s.configWindow) text(222, 11, "BT", 1, kAmber);

  if (s.unixUs) {
    const time_t t = static_cast<time_t>((s.unixUs + (now - s.gnssAtUs)) / 1000000) + 9 * 3600;
    tm jst;
    gmtime_r(&t, &jst);
    snprintf(b, sizeof(b), "%02d:%02d", jst.tm_hour, jst.tm_min);
    textRight(W - 4, 4, b, 2, fresh ? kWhite : kDim);
  } else {
    textRight(W - 4, 4, "--:--", 2, kDim);
  }
}

// Labels of the three buttons, at the bottom edge above them (A x=68, B x=160, C x=252 on the case).
void drawButtonBar(const Status& s) {
  if (bandY + kBandH < H - 14) return;
  fillRect(0, H - 14, W, 14, kBar);
  static const char* const kNames[kPages] = {"SPEED", "G", "STATUS"};
  textCenter(68, H - 11, "<", 1, kGray);
  textCenter(252, H - 11, ">", 1, kGray);
  textCenter(160, H - 11, s.rec == 1 ? "HOLD: STOP REC" : "HOLD: START REC", 1, s.rec == 1 ? kRed : kGray);
  if (s.magCollecting) {
    textCenter(68, H - 11, "CANCEL", 1, kGray);
    textCenter(252, H - 11, "CANCEL", 1, kGray);
    textCenter(160, H - 22, "PRESS: FINISH", 1, s.magCalComplete ? kGreen : kDim);
    return;
  }
  if (page == 1) textCenter(160, H - 22, "PRESS: RESET PEAK", 1, kDim);
  if (page == 2) textCenter(160, H - 22, "PRESS: MAG CAL", 1, kDim);
  for (int i = 0; i < kPages; i++) fillRect(4 + i * 8, H - 9, 5, 5, i == page ? kCyan : kTrack);
  textRight(W - 4, H - 11, kNames[page], 1, kDim);
}

// ---------------------------------------------------------------------------
// Page 0: speed (seven-segment, 3 digits) and G

void drawSpeedPage(const Status& s, int64_t now) {
  const bool fresh = gnssFresh(s, now);
  const bool good = fresh && s.fixOk;
  const int kmh = good ? (kmh10(s.speedMms) + 5) / 10 : -1;

  // Speed: 3 digits 64 x 112, right aligned.
  constexpr int dw = 64, dh = 112, dt = 12, gap = 14, y = 34;
  const int x0 = W - 12 - 3 * dw - 2 * gap;
  for (int i = 0; i < 3; i++) {
    const int x = x0 + i * (dw + gap);
    const int place = 2 - i;
    int d = -1;
    if (kmh >= 0) {
      int p = 1;
      for (int k = 0; k < place; k++) p *= 10;
      d = kmh / p % 10;
      if (kmh < p && place > 0) d = -2;  // leading blank
    }
    if (d == -2) continue;
    segDigit(x, y, dw, dh, dt, d, kmh >= 0 ? kSpeed : kDim);
  }
  textRight(W - 12, y + dh + 6, "KM/H", 2, kGray);

  // Heading under the digits, left.
  char b[24];
  if (good && s.headingValid) {
    static const char* const dirs[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    const int deg = (s.headingE5 / 100000 % 360 + 360) % 360;
    snprintf(b, sizeof(b), "%03d %s", deg, dirs[((deg + 22) / 45) % 8]);
    text(x0, y + dh + 6, b, 2, kWhite);
  } else {
    text(x0, y + dh + 6, "---", 2, kDim);
  }

  // G bar, left edge: total horizontal G, 0..1.5 G.
  constexpr int bx = 8, bw = 22, by = 30, bh = 160, kMaxMg = 1500;
  fillRect(bx, by, bw, bh, kTrack);
  const int mg = gv.haveRef ? static_cast<int>(gv.total * 1000) : -1;
  if (mg >= 0) {
    const int lit = std::min(mg, kMaxMg) * bh / kMaxMg;
    const uint16_t c = mg < 500 ? kGreen : mg < 1000 ? kAmber : kRed;
    fillRect(bx, by + bh - lit, bw, lit, c);
  }
  for (int g = 1; g < 3; g++) fillRect(bx, by + bh - bh * g / 3, bw, 1, kBg);
  if (mg >= 0) {
    snprintf(b, sizeof(b), "%d.%02dG", mg / 1000, mg % 1000 / 10);
    text(bx, by + bh + 6, b, 2, kWhite);
  } else {
    text(bx, by + bh + 6, "-.--G", 2, kDim);
  }
}

// ---------------------------------------------------------------------------
// Page 1: G circle (felt G: braking up, left turn right) with a 2 s trail

void drawGPage(const Status& s, int64_t now) {
  (void)now;
  constexpr int cx = 112, cy = 128, R = 96;  // R = 1.5 G
  constexpr float kPxPerG = R / 1.5f;
  for (int g = 1; g <= 3; g++) circle(cx, cy, R * g / 3, g == 2 ? kDim : kGrid);
  fillRect(cx - R, cy, 2 * R + 1, 1, kGrid);
  fillRect(cx, cy - R, 1, 2 * R + 1, kGrid);
  text(cx + R * 2 / 3 + 3, cy + 3, "1G", 1, kDim);

  char b[24];
  if (gv.haveRef) {
    for (int i = 0; i < gv.trailN; i++) {
      const int k = (gv.trailPos - gv.trailN + i + GState::kTrail) % GState::kTrail;
      const int x = cx + static_cast<int>(gv.trail[k][0] * kPxPerG);
      const int y = cy - static_cast<int>(gv.trail[k][1] * kPxPerG);
      fillRect(x - 1, y - 1, 3, 3, i * 3 > gv.trailN * 2 ? kCyan : kTrack);
    }
    const int x = cx + static_cast<int>(std::clamp(gv.lat, -1.6f, 1.6f) * kPxPerG);
    const int y = cy - static_cast<int>(std::clamp(gv.lon, -1.6f, 1.6f) * kPxPerG);
    disc(x, y, 7, kRed);

    constexpr int tx = 222;
    text(tx, 34, "LAT", 2, kDim);
    snprintf(b, sizeof(b), "%+.2f", gv.lat);
    text(tx, 54, b, 2, kWhite);
    text(tx, 82, "LON", 2, kDim);
    snprintf(b, sizeof(b), "%+.2f", gv.lon);
    text(tx, 102, b, 2, kWhite);
    text(tx, 130, "PEAK", 2, kDim);
    snprintf(b, sizeof(b), "L %.2f", gv.peakLat);
    text(tx, 150, b, 2, kAmber);
    snprintf(b, sizeof(b), "F %.2f", gv.peakLon);
    text(tx, 170, b, 2, kAmber);
  } else {
    textCenter(cx, cy - 20, "HOLD STILL", 2, kAmber);
    textCenter(cx, cy + 4, "CALIBRATING", 1, kDim);
  }
  if (!s.imuOk) textCenter(cx, cy + 30, "NO IMU", 2, kRed);
}

// ---------------------------------------------------------------------------
// Page 2: status

void drawStatusPage(const Status& s, int64_t now, int rate) {
  char b[48];
  constexpr int lx = 6, vx = 70, dy = 18;
  int y = 28;
  auto row = [&](const char* label, const char* value, uint16_t c) {
    text(lx, y, label, 2, kDim);
    text(vx, y, value, 2, c);
    y += dy;
  };

  // GNSS
  const bool fresh = gnssFresh(s, now);
  if (!fresh) {
    snprintf(b, sizeof(b), s.gnssConfigured ? "SILENT" : "NOT FOUND");
    row("GNSS", b, kRed);
  } else {
    snprintf(b, sizeof(b), "%s %2dSV %2dHZ", s.fixOk ? "3D" : s.fix == 2 ? "2D" : "NO FIX", s.sats, rate);
    row("GNSS", b, s.fixOk ? kGreen : kAmber);
  }
  if (fresh && s.fix >= 2) {
    fmtCoord(b, sizeof(b), s.lat);
    row("LAT", b, kWhite);
    fmtCoord(b, sizeof(b), s.lon);
    row("LON", b, kWhite);
    const uint32_t acc = std::min<uint32_t>(s.hAccMm, 99900);
    snprintf(b, sizeof(b), "%dM  ACC %d.%dM", static_cast<int>(s.altMm / 1000), static_cast<int>(acc / 1000),
             static_cast<int>(acc % 1000 / 100));
    row("ALT", b, kWhite);
  } else {
    row("LAT", "--", kDim);
    row("LON", "--", kDim);
    row("ALT", "--", kDim);
  }

  // Sensors
  if (s.imuOk) {
    snprintf(b, sizeof(b), "%+.2f %+.2f %+.2f", s.ax, s.ay, s.az);
    row("ACC", b, kWhite);
  } else {
    row("ACC", "NO IMU", kRed);
  }
  if (s.baroOk) {
    snprintf(b, sizeof(b), "%.1fHPA %.1fC", s.pressureHpa, s.tempC);
    row("BARO", b, kWhite);
  } else {
    row("BARO", "--", kRed);
  }
  if (s.magOk) {
    snprintf(b, sizeof(b), "%.0f %.0f %.0f UT", s.mx, s.my, s.mz);
    row("MAG", b, kWhite);
    const float now3 = sqrtf(s.mx * s.mx + s.my * s.my + s.mz * s.mz);
    const bool showResult = s.magCalResult && now - s.magCalResultAtUs < kMagResultUs;
    if (showResult && s.magCalResult != 1) {
      static const char* const kFail[] = {"",          "",           "FAILED: TOO SHORT", "FAILED: ROTATE MORE",
                                          "FAILED: TRY AGAIN", "NOT RUNNING", "NOT SAVED"};
      row("CAL", kFail[s.magCalResult < 7 ? s.magCalResult : 4], kRed);
    } else if (s.magCalibrated) {
      snprintf(b, sizeof(b), "%s %.0fUT NOW %.0f", showResult ? "DONE" : "ON", s.magField, now3);
      row("CAL", b, kGreen);
    } else {
      row("CAL", "NONE (RAW)", kAmber);
    }
  } else {
    row("MAG", "--", kRed);
  }

  // SD
  static const char* const kRec[] = {"NO CARD", "REC", "SAVING", "SAFE", "ERROR"};
  snprintf(b, sizeof(b), "%s %s", kRec[s.rec < 5 ? s.rec : 0], s.rec ? s.logName : "");
  row("SD", b, s.rec == 1 ? kGreen : s.rec == 4 ? kRed : s.rec == 3 ? kCyan : kAmber);

  // Wi-Fi
  if (!s.wifiConfigured) {
    snprintf(b, sizeof(b), "NOT SET  AP %d", s.apClients);
    row("WIFI", b, kAmber);
  } else if (!s.wifiUp) {
    snprintf(b, sizeof(b), "JOINING  AP %d", s.apClients);
    row("WIFI", b, kAmber);
  } else {
    snprintf(b, sizeof(b), "%s %d", s.ip, s.rssi);
    row("WIFI", b, kWhite);
  }
}

// Magnetometer calibration (replaces the page while collecting)

void drawMagCal(const Status& s) {
  textCenter(160, 30, "MAG CALIBRATION", 2, kCyan);
  textCenter(160, 56, "ROTATE THE UNIT SLOWLY", 1, kWhite);
  textCenter(160, 68, "IN EVERY DIRECTION (TURN IT OVER TOO)", 1, kWhite);
  static const char* const kAxis[3] = {"X", "Y", "Z"};
  for (int k = 0; k < 3; k++) {
    const int y = 92 + k * 30;
    text(24, y, kAxis[k], 2, kDim);
    constexpr int bx = 50, bw = 220, bh = 16;
    fillRect(bx, y, bw, bh, kTrack);
    const float c = std::clamp(s.magCoverage[k], 0.0f, 1.0f);
    fillRect(bx, y, static_cast<int>(bw * c), bh, c >= 1 ? kGreen : kAmber);
    char b[8];
    snprintf(b, sizeof(b), "%d%%", static_cast<int>(c * 100));
    text(bx + bw + 8, y + 1, b, 2, c >= 1 ? kGreen : kWhite);
  }
  char b[32];
  snprintf(b, sizeof(b), "%lu SAMPLES", static_cast<unsigned long>(s.magCalSamples));
  textCenter(160, 186, b, 1, kDim);
  if (s.magCalComplete) textCenter(160, 200, "OK - KEEP TURNING OR PRESS B", 1, kGreen);
}

// ---------------------------------------------------------------------------
// LCD, buttons, task

bool onTxDone(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(bufFree, &woken);
  return woken == pdTRUE;
}

void initLcd() {
  gpio_config_t bl = {};
  bl.pin_bit_mask = 1ULL << kBl;
  bl.mode = GPIO_MODE_OUTPUT;
  gpio_config(&bl);
  gpio_set_level(kBl, 0);
  gpio_config_t btn = {};  // GPIO37-39 are input only, the board has pull-ups
  btn.pin_bit_mask = (1ULL << kBtnA) | (1ULL << kBtnB) | (1ULL << kBtnC);
  btn.mode = GPIO_MODE_INPUT;
  gpio_config(&btn);

  for (auto& b : bufs) {
    b = static_cast<uint16_t*>(heap_caps_malloc(W * kBandH * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    assert(b);
  }
  bufFree = xSemaphoreCreateCounting(2, 2);

  esp_lcd_panel_io_handle_t io;
  esp_lcd_panel_io_spi_config_t ioc = {};
  ioc.dc_gpio_num = kDc;
  ioc.cs_gpio_num = kCs;
  ioc.pclk_hz = 40 * 1000 * 1000;
  ioc.lcd_cmd_bits = 8;
  ioc.lcd_param_bits = 8;
  ioc.spi_mode = 0;
  ioc.trans_queue_depth = 4;
  ioc.on_color_trans_done = onTxDone;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(kHost), &ioc, &io));

  // The ILI9342C understands the commands the ST7789 driver sends (SLPOUT, MADCTL, COLMOD, CASET/RASET, ...).
  esp_lcd_panel_dev_config_t pc = {};
  pc.reset_gpio_num = kRst;
  pc.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
  pc.bits_per_pixel = 16;
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &pc, &panel));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
}

bool resetPeaks = false;
// Calibration state as the display sees it. The logger's Status lags a button press by up to one update, so
// for a moment after we start / end collecting our own idea wins; after that, the Status does (the serial
// console can start and end it too).
bool magCollecting = false;
int64_t magStartAt = 0, magLocalAt = 0;
// Collection session the display follows: the number our own start returned, or one started from the serial
// console. Progress in a Status counts only when it carries this number (an older Status may still hold the
// previous collection's counts, even while saying it is collecting).
uint32_t magSession = 0;
bool magSessionKnown = false;

void magCalEnd(bool finish, int64_t now) {
  if (finish) {
    onMagCalFinish();
  } else {
    onMagCalCancel();
  }
  magCollecting = false;
  magLocalAt = now;
}

// A / C: previous / next page on release. B held 1.5 s: SD stop / start; B short on the G page: reset peaks.
void pollButtons(int64_t now) {
  static bool prevA, prevC;
  const bool a = gpio_get_level(kBtnA) == 0, c = gpio_get_level(kBtnC) == 0;
  if ((prevA && !a) || (prevC && !c)) {
    if (magCollecting) {
      magCalEnd(false, now);
    } else if (prevA && !a) {
      page = (page + kPages - 1) % kPages;
    } else {
      page = (page + 1) % kPages;
    }
  }
  prevA = a;
  prevC = c;

  static int64_t downAt = 0;
  static bool longDone = false;
  const bool b = gpio_get_level(kBtnB) == 0;
  if (b && !downAt) {
    downAt = now;
    longDone = false;
  } else if (b && !longDone && now - downAt >= 1500000) {
    longDone = true;
    onLongPress();
  } else if (!b && downAt) {
    if (!longDone) {  // short press
      if (magCollecting) {
        magCalEnd(true, now);
      } else if (page == 1) {
        resetPeaks = true;
      } else if (page == 2) {
        magSession = onMagCalStart();
        magSessionKnown = true;
        magCollecting = true;
        magStartAt = magLocalAt = now;
      }
    }
    downAt = 0;
  }
}

void displayTask(void*) {
  Status s;
  int64_t rateAt = esp_timer_get_time();
  uint32_t lastMsgs = 0;
  int rate = 0;
  bool backlight = false;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const int64_t now = esp_timer_get_time();
    pollButtons(now);
    portENTER_CRITICAL(&stLock);
    s = st;
    if (resetPeaks) {
      gs.peakLat = gs.peakLon = 0;
      resetPeaks = false;
    }
    gv = gs;
    portEXIT_CRITICAL(&stLock);
    // Calibration: finish by itself once there is plenty of rotation, give up after a while.
    // Session numbers only grow: a Status with a smaller number than ours is older than our own start (the logger
    // can go up to ~1 s between updates while the GNSS is silent) and is ignored, however long it lingers.
    const bool stale = magSessionKnown && static_cast<int32_t>(s.magCalSession - magSession) < 0;
    if (now - magLocalAt > 500000 && !stale) {
      magCollecting = s.magCollecting;
      // A newer session we did not start (a (re)start from the serial console): follow it, 2 minutes from now.
      if (s.magCollecting && (!magSessionKnown || s.magCalSession != magSession)) {
        magStartAt = now;
        magSession = s.magCalSession;
        magSessionKnown = true;
      }
    }
    if (magCollecting) {
      page = 2;
      // Only on the counts of the session we follow (not the previous collection's, see magSession).
      const bool current = magSessionKnown && s.magCalSession == magSession;
      if (current && s.magCalComplete && s.magCalSamples >= kMagAutoFinishSamples) {
        magCalEnd(true, now);
      } else if (now - magStartAt > kMagTimeoutUs) {
        magCalEnd(false, now);
      }
    }
    s.magCollecting = magCollecting;
    if (now - rateAt >= 1000000) {
      rate = static_cast<int>((s.gnssMessages - lastMsgs) * 1000000LL / (now - rateAt));
      lastMsgs = s.gnssMessages;
      rateAt = now;
    }

    for (bandY = 0; bandY < H; bandY += kBandH) {
      xSemaphoreTake(bufFree, portMAX_DELAY);
      static int which = 0;
      cv = bufs[which];
      which ^= 1;
      std::fill(cv, cv + W * kBandH, kBg);
      drawTopBar(s, now);
      if (s.magCollecting) {
        drawMagCal(s);
      } else if (page == 0) {
        drawSpeedPage(s, now);
      } else if (page == 1) {
        drawGPage(s, now);
      } else {
        drawStatusPage(s, now, rate);
      }
      drawButtonBar(s);
      esp_lcd_panel_draw_bitmap(panel, 0, bandY, W, bandY + kBandH, cv);
    }
    if (!backlight) {
      gpio_set_level(kBl, 1);
      backlight = true;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(kFrameMs));
  }
}

}  // namespace

void displayStart() {
  initLcd();
  xTaskCreate(displayTask, "display", 6144, nullptr, 3, nullptr);
  ESP_LOGI(TAG, "LCD ready");
}

void displayUpdate(const Status& s) {
  portENTER_CRITICAL(&stLock);
  const bool newData = s.dataAtUs != st.dataAtUs;
  st = s;
  if (newData && s.imuOk) updateG(s);
  portEXIT_CRITICAL(&stLock);
}

}  // namespace logger
