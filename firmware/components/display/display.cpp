// M5Stack (Basic / Gray) display: ILI9342C 320x240 on the SPI bus shared with the TF card (initialised
// by components/sdlog), and the three buttons under the screen.
//
// There is no PSRAM, so a full RGB565 frame (150 KB) does not fit next to Wi-Fi and BLE. The frame is
// drawn in horizontal bands instead: the whole scene is drawn once per band, clipped to it, into one of
// two band buffers, and sent by DMA while the next band is drawn.
//
// Buttons: A (left) / C (right) switch pages (held 0.8 s: backlight darker / brighter, repeating while held),
// B (middle) held for 1.5 s stops / starts the SD recording;
// a short press of B on the G page resets the peak values, on the STATUS page it starts the magnetometer
// calibration (a page of its own until it ends: B finishes, A / C cancel).
//
// Pages: 0 speed + G, 1 G circle, 2 status, 3 GNSS details (satellites in the sky).

#include "logger_status.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <algorithm>
#include <atomic>

#include "driver/gpio.h"
#include "driver/ledc.h"
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
#include "nvs.h"
#include "gnss.h"

namespace logger {
namespace {

const char* TAG = "display";

constexpr spi_host_device_t kHost = SPI3_HOST;  // initialised by sdlog (shared with the TF card)
constexpr gpio_num_t kCs = GPIO_NUM_14, kDc = GPIO_NUM_27, kRst = GPIO_NUM_33, kBl = GPIO_NUM_32;
constexpr gpio_num_t kBtnA = GPIO_NUM_39, kBtnB = GPIO_NUM_38, kBtnC = GPIO_NUM_37;  // low = pressed
constexpr int W = 320, H = 240;
constexpr int kBandH = 24;  // 10 bands per frame, 15 KB per band buffer
constexpr int kFrameMs = 100;  // buttons and state; the screen is redrawn every kRedrawFrames[page] of these
constexpr int kPages = 4;
// Redraw every n-th frame per page: G 10 fps, SPEED 5 fps, STATUS / GNSS 2 fps. Drawing is most of the CPU
// time, and with less of it the CPU spends more time at the lower DFS frequency.
constexpr int kRedrawFrames[] = {2, 1, 5, 5};
static_assert(sizeof(kRedrawFrames) / sizeof(kRedrawFrames[0]) == kPages);

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

// Battery: 4 segments (the IC reports 25 % steps). Cyan while charging, green when full on USB.
void drawBattery(int x, int y, const Status& s) {
  const uint16_t c = s.batteryFull ? kGreen : s.batteryCharging ? kCyan
                     : s.batteryLevel <= 25 ? kRed : s.batteryLevel <= 50 ? kAmber : kWhite;
  fillRect(x, y, 22, 12, c);          // outline
  fillRect(x + 1, y + 1, 20, 10, kBar);
  fillRect(x + 22, y + 3, 2, 6, c);   // terminal
  const int seg = s.batteryLevel / 25;
  for (int i = 0; i < 4; i++) fillRect(x + 2 + i * 5, y + 2, 4, 8, i < seg ? c : kTrack);
}

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
  drawSignal(190, 4, bars, s.wifiUp ? kGreen : s.apClients ? kCyan : kDim);
  if (s.bleClients) text(208, 11, "BT", 1, kCyan);  // only while a config client is connected
  if (s.batteryValid) drawBattery(226, 5, s);

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
  static const char* const kNames[kPages] = {"SPEED", "G", "STATUS", "GNSS"};
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
  constexpr int lx = 6, vx = 70, dy = 17;
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
  if (!s.staOn) {
    if (s.apOn) snprintf(b, sizeof(b), "STA OFF  AP %d", s.apClients);
    else snprintf(b, sizeof(b), "OFF");
    row("WIFI", b, kDim);
  } else if (!s.wifiConfigured) {
    snprintf(b, sizeof(b), "NOT SET  AP %d", s.apClients);
    row("WIFI", b, kAmber);
  } else if (!s.wifiUp) {
    snprintf(b, sizeof(b), "JOINING  AP %d", s.apClients);
    row("WIFI", b, kAmber);
  } else {
    snprintf(b, sizeof(b), "%s %d", s.ip, s.rssi);
    row("WIFI", b, kWhite);
  }

  // Battery (only with the I2C power IC)
  if (s.batteryValid) {
    snprintf(b, sizeof(b), "%d%%%s", s.batteryLevel, s.batteryFull ? " FULL" : s.batteryCharging ? " CHARGING" : "");
    row("BAT", b, s.batteryCharging || s.batteryFull ? kCyan : s.batteryLevel <= 25 ? kRed : kWhite);
  }
}

// ---------------------------------------------------------------------------
// Page 3: GNSS details, with the satellites on a sky plot (from NAV-SAT / NAV-DOP, about once a second)

gnss::Sky sky;  // display task's copy

uint16_t cnoColor(uint8_t cno) { return cno >= 35 ? kGreen : cno >= 25 ? kAmber : cno > 0 ? kRed : kDim; }

void drawGnssPage(const Status& s, int64_t now, int rate) {
  char b[32];
  const bool fresh = gnssFresh(s, now);
  const bool pos = fresh && s.posOk;
  constexpr int lx = 4, vx = 34, dy = 19;
  int y = 28;
  auto row = [&](const char* label, const char* value, uint16_t c) {
    text(lx, y + 4, label, 1, kDim);
    text(vx, y, value, 2, c);
    y += dy;
  };
  auto dash = [&](const char* label) { row(label, "--", kDim); };

  if (pos) {
    fmtCoord(b, sizeof(b), s.lat);
    row("LAT", b, kWhite);
    fmtCoord(b, sizeof(b), s.lon);
    row("LON", b, kWhite);
    if (s.fixOk) {  // a 2D fix only holds the last altitude
      snprintf(b, sizeof(b), "%.1fM", s.altMm / 1000.0);
      row("ALT", b, kWhite);
    } else {
      row("ALT", "2D --", kDim);
    }
    snprintf(b, sizeof(b), "%.1fKM/H", kmh10(s.speedMms) / 10.0);
    row("SPD", b, kWhite);
    if (s.headingValid) {
      snprintf(b, sizeof(b), "%.1f", (s.headingE5 % 36000000 + 36000000) % 36000000 / 1e5);
      row("CRS", b, kWhite);
    } else {
      dash("CRS");
    }
  } else {
    dash("LAT");
    dash("LON");
    dash("ALT");
    dash("SPD");
    dash("CRS");
  }
  if (s.unixUs) {
    const time_t t = static_cast<time_t>((s.unixUs + (now - s.gnssAtUs)) / 1000000) + 9 * 3600;
    tm jst;
    gmtime_r(&t, &jst);
    snprintf(b, sizeof(b), "%04d-%02d-%02d", jst.tm_year + 1900, jst.tm_mon + 1, jst.tm_mday);
    row("DATE", b, fresh ? kWhite : kDim);
    snprintf(b, sizeof(b), "%02d:%02d:%02d", jst.tm_hour, jst.tm_min, jst.tm_sec);
    row("JST", b, fresh ? kWhite : kDim);
  } else {
    dash("DATE");
    dash("JST");
  }
  const bool dop = sky.dopAtUs && now - sky.dopAtUs < 3000000;
  if (dop && pos) {
    snprintf(b, sizeof(b), "%d.%d P%d.%d", sky.hDOP / 100, sky.hDOP % 100 / 10, sky.pDOP / 100, sky.pDOP % 100 / 10);
    row("HDOP", b, sky.hDOP <= 200 ? kGreen : sky.hDOP <= 500 ? kAmber : kRed);
  } else {
    dash("HDOP");
  }
  if (pos) {
    const uint32_t acc = std::min<uint32_t>(s.hAccMm, 999900);
    snprintf(b, sizeof(b), "%d.%dM", static_cast<int>(acc / 1000), static_cast<int>(acc % 1000 / 100));
    row("ACC", b, acc <= 5000 ? kGreen : acc <= 20000 ? kAmber : kRed);
  } else {
    dash("ACC");
  }

  // Sky plot: zenith in the centre, horizon on the outer ring, north up. Filled = used in the fix, ring =
  // tracked but not used; the colour is the signal strength.
  constexpr int cx = 250, cy = 98, R = 66;
  const bool sats = sky.satAtUs && now - sky.satAtUs < 3000000;
  if (bandY <= cy + R + 8 && bandY + kBandH >= cy - R - 8) {
    for (int k = 1; k <= 3; k++) circle(cx, cy, R * k / 3, k == 3 ? kDim : kGrid);
    fillRect(cx - R, cy, 2 * R + 1, 1, kGrid);
    fillRect(cx, cy - R, 1, 2 * R + 1, kGrid);
    textCenter(cx, cy - R + 3, "N", 1, kGray);
    textCenter(cx, cy + R - 9, "S", 1, kGray);
    text(cx + R - 8, cy - 9, "E", 1, kGray);
    text(cx - R + 3, cy - 9, "W", 1, kGray);
    // Update rate in the plot's top corners: measured NAV-PVT per second (right), configured (left).
    const int set = gnss::stats().rateHz;
    snprintf(b, sizeof(b), "%d", fresh ? rate : 0);
    textRight(W - 4, 26, b, 2, !fresh || !set ? kDim : rate * 10 >= set * 9 ? kGreen : rate * 2 >= set ? kAmber : kRed);
    textRight(W - 4, 42, "HZ", 1, kDim);
    text(cx - R - 2, 26, "SET", 1, kDim);
    snprintf(b, sizeof(b), set ? "%dHZ" : "--", set);
    text(cx - R - 2, 36, b, 1, kGray);
    if (sats) {
      for (int pass = 0; pass < 2; pass++) {  // unused first, so the used ones are drawn on top
        for (int i = 0; i < sky.numSvs; i++) {
          const gnss::Sat& v = sky.sats[i];
          if (v.used != (pass == 1) || v.elev < 0 || v.elev > 90 || v.azim < 0 || v.azim > 360) continue;
          const float r = R * (90 - v.elev) / 90.0f, a = v.azim * static_cast<float>(M_PI) / 180;
          const int x = cx + static_cast<int>(lroundf(r * sinf(a))), yy = cy - static_cast<int>(lroundf(r * cosf(a)));
          if (v.used) {
            disc(x, yy, 4, cnoColor(v.cno));
          } else {
            circle(x, yy, 4, cnoColor(v.cno));
          }
        }
      }
    } else {
      textCenter(cx, cy + 12, "NO SAT DATA", 1, kDim);
    }
  }

  // Counts per constellation: used / tracked (with a signal) / in view.
  int ty = cy + R + 8;
  if (!sats) return;
  static const struct { uint8_t id; const char* name; } kSys[] = {
      {0, "GPS"}, {6, "GLO"}, {2, "GAL"}, {3, "BDS"}, {5, "QZS"}, {1, "SBS"}};
  snprintf(b, sizeof(b), "USED %d SIG %d VIEW %d", sky.totalUsed, sky.totalSignal, sky.totalSvs);
  textCenter(cx, ty, b, 1, kWhite);
  ty += 12;
  for (int k = 0; k < 6; k++) {
    const int u = sky.usedBy[kSys[k].id], n = sky.viewBy[kSys[k].id];
    snprintf(b, sizeof(b), "%s %2d/%-2d", kSys[k].name, u, n);
    text(cx - 68 + (k % 2) * 72, ty + (k / 2) * 11, b, 1, n ? kGray : kTrack);
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
// Backlight: PWM on the BL pin (LEDC from the 1 MHz REF_TICK, so DFS does not change it), level in NVS

constexpr uint8_t kBlDuty[kBacklightLevels] = {8, 20, 50, 110, 255};  // of 255, roughly even steps to the eye
constexpr int64_t kBlShowUs = 1500000;  // the level stays on the screen this long after a change
std::atomic<int> blLevel{kBacklightLevels};
std::atomic<int64_t> blShownAt{0};
bool blOn = false;  // off until the first frame is drawn

void blApply() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, blOn ? kBlDuty[blLevel - 1] : 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

bool blSave(int level) {
  nvs_handle_t h;
  if (nvs_open("disp", NVS_READWRITE, &h) != ESP_OK) return false;
  const bool ok = nvs_set_u8(h, "bl", static_cast<uint8_t>(level)) == ESP_OK && nvs_commit(h) == ESP_OK;
  nvs_close(h);
  return ok;
}

void blInit() {
  nvs_handle_t h;
  uint8_t v;
  if (nvs_open("disp", NVS_READONLY, &h) == ESP_OK) {
    if (nvs_get_u8(h, "bl", &v) == ESP_OK && v >= 1 && v <= kBacklightLevels) blLevel = v;
    nvs_close(h);
  }
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_8_BIT;
  t.timer_num = LEDC_TIMER_0;
  t.freq_hz = 2000;
  t.clk_cfg = LEDC_USE_REF_TICK;
  ESP_ERROR_CHECK(ledc_timer_config(&t));
  ledc_channel_config_t c = {};
  c.gpio_num = kBl;
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.channel = LEDC_CHANNEL_0;
  c.timer_sel = LEDC_TIMER_0;
  c.duty = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&c));
}

// The level while it is being changed: a box in the middle of the screen.
void drawBacklightLevel(int64_t now) {
  if (now - blShownAt >= kBlShowUs) return;
  constexpr int bw = 200, bh = 60, bx = (W - bw) / 2, by = (H - bh) / 2;
  fillRect(bx, by, bw, bh, kCyan);
  fillRect(bx + 2, by + 2, bw - 4, bh - 4, kBar);
  textCenter(W / 2, by + 8, "BRIGHTNESS", 2, kWhite);
  const int lv = blLevel;
  for (int i = 0; i < kBacklightLevels; i++) fillRect(bx + 20 + i * 33, by + 32, 28, 18, i < lv ? kCyan : kTrack);
}

// ---------------------------------------------------------------------------
// LCD, buttons, task

bool onTxDone(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(bufFree, &woken);
  return woken == pdTRUE;
}

void initLcd() {
  blInit();
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

// A / C: previous / next page on release; held 0.8 s: backlight darker / brighter (again every 0.5 s while
// held, saved on release). B held 1.5 s: SD stop / start; B short on the G page: reset peaks.
void pollButtons(int64_t now) {
  static bool prevA, prevC, held;
  static int64_t acDownAt, stepAt;
  const bool a = gpio_get_level(kBtnA) == 0, c = gpio_get_level(kBtnC) == 0;
  if ((a && !prevA) || (c && !prevC)) {
    acDownAt = now;
    held = false;
  }
  if ((a || c) && !(a && c) && !magCollecting && now - acDownAt >= 800000 && (!held || now - stepAt >= 500000)) {
    const int lv = std::clamp(blLevel + (c ? 1 : -1), 1, kBacklightLevels);
    if (lv != blLevel) {
      blLevel = lv;
      blApply();
    }
    blShownAt = now;
    held = true;
    stepAt = now;
  }
  if ((prevA && !a) || (prevC && !c)) {
    if (magCollecting) {
      magCalEnd(false, now);
    } else if (held) {
      if (!a && !c) {
        if (!blSave(blLevel)) ESP_LOGW(TAG, "backlight level not saved");
      }
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
    if (page == 3) gnss::sky(sky);
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

    // At once on a page change or when the calibration starts / ends, otherwise at the page's rate.
    static int drawnPage = -1, sinceDraw = 0;
    const int drawPage = s.magCollecting ? -2 : page;  // the calibration page: every frame
    // While the backlight level shows (and once after, to remove it): every frame.
    static bool blShowing = false;
    const bool blShow = now - blShownAt < kBlShowUs, blChanged = blShow || blShowing;
    blShowing = blShow;
    const bool redraw = drawPage != drawnPage || blChanged ||
                        ++sinceDraw >= (s.magCollecting ? 1 : kRedrawFrames[page]);
    if (redraw) {
      drawnPage = drawPage;
      sinceDraw = 0;
    }
    for (bandY = 0; redraw && bandY < H; bandY += kBandH) {
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
      } else if (page == 3) {
        drawGnssPage(s, now, rate);
      } else {
        drawStatusPage(s, now, rate);
      }
      drawButtonBar(s);
      drawBacklightLevel(now);
      esp_lcd_panel_draw_bitmap(panel, 0, bandY, W, bandY + kBandH, cv);
    }
    if (!backlight) {  // after the first frame
      blOn = backlight = true;
      blApply();
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

int displayBacklight() { return blLevel; }

bool displaySetBacklight(int level) {
  if (level < 1 || level > kBacklightLevels) return false;
  blLevel = level;
  blApply();
  blShownAt = esp_timer_get_time();
  return blSave(level);
}

void displayUpdate(const Status& s) {
  portENTER_CRITICAL(&stLock);
  const bool newData = s.dataAtUs != st.dataAtUs;
  st = s;
  if (newData && s.imuOk) updateG(s);
  portEXIT_CRITICAL(&stLock);
}

}  // namespace logger
