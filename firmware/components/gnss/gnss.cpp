// GNSS task: baud detection, M9 configuration through UBX-CFG-VALSET, NAV-PVT
// dispatch and automatic recovery when the receiver loses its configuration.
#include "gnss.h"

#include <atomic>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ubx.h"

namespace gnss {
namespace {

constexpr const char* TAG = "gnss";
constexpr uart_port_t kUart = UART_NUM_2;
constexpr uint32_t kTargetBaud = 115200;
constexpr uint32_t kCandidateBauds[] = {38400, 115200, 9600, 230400, 57600, 460800, 921600};
constexpr int kMaxRateHz = 25;  // NEO-M9N maximum navigation update rate
constexpr int kFallbackRateHz = 10;
constexpr int64_t kPvtTimeoutUs = 2000000;  // no NAV-PVT this long -> redetect + reconfigure
constexpr uint8_t kLayers = ubx::kLayerRam | ubx::kLayerBbr;

enum class Ack { None, Ack, Nak };

Callback g_cb = nullptr;
int g_requestedRate = 10;

ubx::Parser g_parser;
ubx::NmeaDetector g_nmea;
uint32_t g_ubxFrames = 0;  // valid UBX frames of any kind (for detection)
Ack g_ack = Ack::None;
uint8_t g_ackCls = 0, g_ackId = 0;
int64_t g_lastPvtUs = 0;

std::atomic<uint32_t> g_pvt{0}, g_other{0}, g_bytes{0}, g_ckErr{0}, g_baud{0};
std::atomic<uint16_t> g_rate{0};
std::atomic<bool> g_configured{false};

Sky g_sky;  // decoded in place by the GNSS task under g_skyLock, copied out by sky()
portMUX_TYPE g_skyLock = portMUX_INITIALIZER_UNLOCKED;

void handleFrame() {
  g_ubxFrames++;
  const uint8_t cls = g_parser.msgClass(), id = g_parser.msgId();
  if (cls == ubx::kClsNav && id == ubx::kIdNavPvt) {
    Pvt pvt;
    if (ubx::decodeNavPvt(g_parser.payload(), g_parser.payloadLen(), pvt)) {
      g_pvt++;
      g_lastPvtUs = esp_timer_get_time();
      if (!pvt.posOk()) {  // a DOP from before the fix was lost must not be shown with the next fix
        taskENTER_CRITICAL(&g_skyLock);
        g_sky.dopAtUs = 0;
        taskEXIT_CRITICAL(&g_skyLock);
      }
      if (g_cb) g_cb(pvt);
      return;
    }
  } else if (cls == ubx::kClsNav && id == ubx::kIdNavSat) {
    taskENTER_CRITICAL(&g_skyLock);  // decoded in place: a few microseconds for 64 satellites
    const bool ok = ubx::decodeNavSat(g_parser.payload(), g_parser.payloadLen(), g_parser.frameLen(), g_sky);
    if (ok) g_sky.satAtUs = esp_timer_get_time();
    taskEXIT_CRITICAL(&g_skyLock);
    if (ok) return;
  } else if (cls == ubx::kClsNav && id == ubx::kIdNavDop) {
    taskENTER_CRITICAL(&g_skyLock);
    const bool ok = ubx::decodeNavDop(g_parser.payload(), g_parser.payloadLen(), g_sky);
    if (ok) g_sky.dopAtUs = esp_timer_get_time();
    taskEXIT_CRITICAL(&g_skyLock);
    if (ok) return;
  } else if (cls == ubx::kClsAck && g_parser.payloadLen() >= 2) {
    g_ackCls = g_parser.payload()[0];
    g_ackId = g_parser.payload()[1];
    g_ack = id == ubx::kIdAckAck ? Ack::Ack : Ack::Nak;
    return;
  }
  g_other++;
}

// Reads and dispatches everything that arrives within ms milliseconds, or until
// stop() returns true.
template <typename F>
void pump(int ms, F stop) {
  uint8_t buf[256];
  const int64_t deadline = esp_timer_get_time() + int64_t(ms) * 1000;
  while (!stop()) {
    const int64_t left = deadline - esp_timer_get_time();
    if (left <= 0) break;
    const int waitMs = left > 20000 ? 20 : int(left / 1000) + 1;
    const int n = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(waitMs));
    for (int i = 0; i < n; i++) {
      g_nmea.feed(buf[i]);
      if (g_parser.feed(buf[i])) handleFrame();
    }
    if (n > 0) {
      g_bytes += n;
      g_ckErr = g_parser.checksumErrors();
    }
  }
}
void pump(int ms) {
  pump(ms, [] { return false; });
}

void setBaud(uint32_t baud) {
  uart_wait_tx_done(kUart, pdMS_TO_TICKS(100));
  uart_set_baudrate(kUart, baud);
  uart_flush_input(kUart);
  g_parser.reset();
  g_nmea.reset();
  g_baud = baud;
}

void send(const uint8_t* frame, size_t len) { uart_write_bytes(kUart, frame, len); }

// Sends a CFG-VALSET and waits for its ACK/NAK, retrying on timeout.
Ack valset(const ubx::KeyValue* kv, size_t n, int tries = 3) {
  uint8_t frame[128];
  const size_t len = ubx::buildValset(kLayers, kv, n, frame, sizeof(frame));
  if (len == 0) return Ack::Nak;
  for (int t = 0; t < tries; t++) {
    g_ack = Ack::None;
    send(frame, len);
    pump(500, [] {
      return g_ack != Ack::None && g_ackCls == ubx::kClsCfg && g_ackId == ubx::kIdCfgValset;
    });
    if (g_ack != Ack::None && g_ackCls == ubx::kClsCfg && g_ackId == ubx::kIdCfgValset) return g_ack;
  }
  return Ack::None;
}

const char* ackName(Ack a) { return a == Ack::Ack ? "ACK" : a == Ack::Nak ? "NAK" : "timeout"; }

// Finds the receiver's baud rate: listens for valid NMEA sentences or UBX frames
// (a MON-VER poll provokes one even if all periodic output is off).
bool detect() {
  static uint32_t lastGood = 0;
  uint32_t order[1 + sizeof(kCandidateBauds) / sizeof(kCandidateBauds[0])];
  size_t n = 0;
  if (lastGood) order[n++] = lastGood;
  for (uint32_t b : kCandidateBauds)
    if (b != lastGood) order[n++] = b;

  uint8_t poll[ubx::kFrameOverhead];
  const size_t pollLen = ubx::buildFrame(ubx::kClsMon, ubx::kIdMonVer, nullptr, 0, poll, sizeof(poll));
  for (size_t i = 0; i < n; i++) {
    setBaud(order[i]);
    const uint32_t ubx0 = g_ubxFrames, nmea0 = g_nmea.sentences();
    auto found = [&] { return g_ubxFrames > ubx0 || g_nmea.sentences() >= nmea0 + 2; };
    send(poll, pollLen);
    pump(1200, found);
    if (found()) {
      ESP_LOGI(TAG, "receiver found at %lu baud (%s)", (unsigned long)order[i],
               g_ubxFrames > ubx0 ? "UBX" : "NMEA");
      lastGood = order[i];
      return true;
    }
  }
  g_baud = 0;
  return false;
}

// Also sets NAV-SAT / NAV-DOP to every hz-th epoch, i.e. about once a second at any rate.
bool setRate(int hz) {
  const ubx::KeyValue kv[] = {{ubx::kCfgRateMeas, uint64_t(1000 / hz)},
                              {ubx::kCfgRateNav, 1},
                              {ubx::kCfgMsgoutNavSatUart1, uint64_t(hz)},
                              {ubx::kCfgMsgoutNavDopUart1, uint64_t(hz)}};
  const Ack a = valset(kv, 4);
  ESP_LOGI(TAG, "rate %d Hz (%d ms): %s", hz, 1000 / hz, ackName(a));
  if (a == Ack::Ack) g_rate = uint16_t(hz);
  return a == Ack::Ack;
}

bool configure() {
  // 1. UBX only on UART1 (at the detected baud).
  const ubx::KeyValue proto[] = {{ubx::kCfgUart1InprotUbx, 1},
                                 {ubx::kCfgUart1OutprotUbx, 1},
                                 {ubx::kCfgUart1OutprotNmea, 0}};
  Ack a = valset(proto, 3);
  ESP_LOGI(TAG, "UART1 UBX-only: %s", ackName(a));
  if (a != Ack::Ack) return false;

  // 2. Baud change. The ACK may be sent at either baud or be lost, so it is not
  //    required; step 3's ACK proves the link at the new baud.
  if (g_baud != kTargetBaud) {
    const ubx::KeyValue baud[] = {{ubx::kCfgUart1Baudrate, kTargetBaud}};
    uint8_t frame[32];
    const size_t len = ubx::buildValset(kLayers, baud, 1, frame, sizeof(frame));
    g_ack = Ack::None;
    send(frame, len);
    pump(100);
    ESP_LOGI(TAG, "baud %lu: %s at old baud", (unsigned long)kTargetBaud, ackName(g_ack));
    setBaud(kTargetBaud);
    pump(50);
  }

  // 3. Navigation settings and NAV-PVT output.
  const ubx::KeyValue nav[] = {{ubx::kCfgNavspgDynmodel, ubx::kDynModelAutomotive},
                               {ubx::kCfgMsgoutNavPvtUart1, 1}};
  a = valset(nav, 2);
  ESP_LOGI(TAG, "dynmodel automotive + NAV-PVT every epoch: %s", ackName(a));
  if (a != Ack::Ack) return false;

  // 4. Measurement rate, falling back to 10 Hz (then 1 Hz) if NAKed.
  if (setRate(g_requestedRate)) return true;
  if (g_requestedRate > kFallbackRateHz && setRate(kFallbackRateHz)) return true;
  return setRate(1);
}

void task(void*) {
  int failures = 0;
  for (;;) {
    g_configured = false;
    g_rate = 0;
    if (!detect()) {
      if (failures++ % 10 == 0) ESP_LOGW(TAG, "no receiver found on UART (attempt %d)", failures);
      continue;
    }
    if (!configure()) {
      ESP_LOGW(TAG, "configuration failed, retrying");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    failures = 0;
    g_lastPvtUs = esp_timer_get_time();
    g_configured = true;
    ESP_LOGI(TAG, "configured: %lu baud, %u Hz", (unsigned long)g_baud.load(), g_rate.load());

    // Run: watch for the receiver going silent, and check the achieved rate in
    // consecutive 5 s windows (a receiver that ACKs 25 Hz but cannot sustain it
    // once it tracks many satellites is dropped to 10 Hz).
    constexpr int64_t kWindowUs = 5000000;
    int64_t windowStartUs = esp_timer_get_time();
    uint32_t windowPvt0 = g_pvt;
    for (;;) {
      pump(100);
      const int64_t now = esp_timer_get_time();
      if (now - g_lastPvtUs > kPvtTimeoutUs) {
        ESP_LOGW(TAG, "no NAV-PVT for %lld ms, reconfiguring", (now - g_lastPvtUs) / 1000);
        break;
      }
      if (now - windowStartUs >= kWindowUs) {
        const uint32_t got = g_pvt - windowPvt0;
        const uint32_t want = uint32_t((now - windowStartUs) * g_rate / 1000000);
        if (g_rate > kFallbackRateHz && got * 10 < want * 7) {
          ESP_LOGW(TAG, "rate not sustained (%lu of %lu NAV-PVT), falling back to %d Hz",
                   (unsigned long)got, (unsigned long)want, kFallbackRateHz);
          setRate(kFallbackRateHz);
        }
        windowStartUs = now;
        windowPvt0 = g_pvt;
      }
    }
  }
}

}  // namespace

void sky(Sky& out) {
  taskENTER_CRITICAL(&g_skyLock);
  out = g_sky;
  taskEXIT_CRITICAL(&g_skyLock);
}

void start(int rxPin, int txPin, int rateHz, Callback cb) {
  static bool started = false;
  if (started) return;
  started = true;
  g_cb = cb;
  g_requestedRate = rateHz < 1 ? 1 : rateHz > kMaxRateHz ? kMaxRateHz : rateHz;

  uart_config_t cfg = {};
  cfg.baud_rate = int(kCandidateBauds[0]);
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  // The UART driver holds the APB frequency only while sending. On the ESP32 a DFS switch between 240 MHz
  // and 80 MHz turns the PLL off for a moment, which garbles what is being received (measured: ~7 instead
  // of 25 NAV-PVT/s, with checksum errors). The receiver talks all the time, so keep APB at 80 MHz for good.
  // With a 240 MHz maximum this keeps the CPU at 240 MHz; with 160 MHz (the default) DFS still goes down to
  // 80 MHz, as both use the same PLL.
  esp_pm_lock_handle_t pm;
  if (esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "gnss_uart", &pm) == ESP_OK) esp_pm_lock_acquire(pm);
  // ISR in IRAM (CONFIG_UART_ISR_IN_IRAM): it keeps emptying the 128-byte FIFO (~11 ms at 115200) while a
  // flash write / erase (NVS) has the cache off.
  ESP_ERROR_CHECK(uart_driver_install(kUart, 4096, 0, 0, nullptr, ESP_INTR_FLAG_IRAM));
  ESP_ERROR_CHECK(uart_param_config(kUart, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(kUart, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  xTaskCreate(task, "gnss", 4096, nullptr, 10, nullptr);
}

Stats stats() {
  Stats s;
  s.pvt = g_pvt;
  s.checksumErrors = g_ckErr;
  s.otherMsgs = g_other;
  s.bytes = g_bytes;
  s.baud = g_baud;
  s.configured = g_configured;
  s.rateHz = g_rate;
  return s;
}

}  // namespace gnss
