// logbo: stand-alone GNSS / IMU logger on an M5Stack (Gray / Basic) with the M5Stack GNSS module
// (u-blox NEO-M9N, BMI270 + BMM150, BMP280).
//
//   GNSS (UART, NAV-PVT every epoch) --+--> record (PVT + IMU mean over the epoch) --> SD card CSV
//   IMU / mag / baro (I2C)  -----------+                                          \--> Wi-Fi UDP broadcast (JSON)
//
// Wi-Fi credentials are set over the serial console or the BLE config service (advertised for a while after boot).

#include <stdio.h>
#include <string.h>

#include <atomic>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "lwip/sockets.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "ap_credentials.h"  // generated per build, see CMakeLists.txt
#include "gnss.h"
#include "carframe.h"
#include "imu.h"
#include "logger_status.h"
#include "record.h"
#include "sdlog.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char* TAG = "logbo";

// GNSS module UART on the M-Bus, selected with its DIP switches (RX block 1 = G16, TX block 1 = G17).
static constexpr int kGnssRx = 16, kGnssTx = 17;
// M5Stack speaker (DAC on GPIO25): held low, otherwise it hisses.
static constexpr gpio_num_t kSpeaker = GPIO_NUM_25;
// LCD chip select: held high (deselected) while the SD card is initialised on the shared bus.
static constexpr gpio_num_t kLcdCs = GPIO_NUM_14;

// ---------------------------------------------------------------------------
// Shared state

static int udpSock = -1;
// Subnet broadcast address (network byte order); 0 while Wi-Fi is down.
static std::atomic<uint32_t> broadcastAddr{0};
// Same for the soft-AP subnet, and the number of stations joined to it.
static uint32_t apBroadcastAddr = 0;
static std::atomic<int> apClients{0};
static std::atomic<uint32_t> udpErrors{0};
static bool wifiConfigured = false;
static char wifiSsid[33];  // configured SSID, "" if none
static char deviceName[16];  // "logbo-XXXX" (BLE name, UDP "device")

static imu::Status imuStatus;
static std::atomic<uint32_t> records{0};
static std::atomic<int64_t> lastPvtUs{0};
static std::atomic<uint32_t> pvtCount{0};

// ---------------------------------------------------------------------------
// UDP

static void sendUdpTo(uint32_t addr, const char* buf, size_t len) {
  sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(CONFIG_LOGBO_UDP_PORT);
  to.sin_addr.s_addr = addr;
  // Never block: a datagram that cannot go out now is stale anyway.
  if (sendto(udpSock, buf, len, MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&to), sizeof(to)) < 0) {
    udpErrors++;
  }
}

// To the joined Wi-Fi network (STA) and, while someone is connected to our own AP, to the AP subnet.
static void sendUdp(const char* buf, size_t len) {
  if (const uint32_t addr = broadcastAddr) sendUdpTo(addr, buf, len);
  if (apClients > 0 && apBroadcastAddr) sendUdpTo(apBroadcastAddr, buf, len);
}

// ---------------------------------------------------------------------------
// Wi-Fi (credentials in NVS namespace "wifi", keys "ssid"/"pass")

// Validates and stores credentials. Returns an error message, or nullptr.
static const char* saveWifiCredentials(const char* ssid, const char* pass) {
  const size_t ssidLen = strlen(ssid);
  const size_t passLen = strlen(pass);
  if (ssidLen == 0 || ssidLen > 32) return "SSID must be 1-32 bytes";
  if (passLen > 64) return "password must be at most 64 bytes";
  nvs_handle_t h;
  if (nvs_open("wifi", NVS_READWRITE, &h) != ESP_OK) return "nvs open failed";
  const bool ok = nvs_set_str(h, "ssid", ssid) == ESP_OK && nvs_set_str(h, "pass", pass) == ESP_OK &&
                  nvs_commit(h) == ESP_OK;
  nvs_close(h);
  return ok ? nullptr : "nvs write failed";
}

static void clearWifiCredentials() {
  nvs_handle_t h;
  if (nvs_open("wifi", NVS_READWRITE, &h) == ESP_OK) {
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
  }
}

static void applyWifiConfig() {
  wifi_config_t cfg = {};
  nvs_handle_t h;
  size_t ssidLen = sizeof(cfg.sta.ssid);
  size_t passLen = sizeof(cfg.sta.password);
  wifiConfigured = false;
  if (nvs_open("wifi", NVS_READONLY, &h) == ESP_OK) {
    wifiConfigured =
        nvs_get_str(h, "ssid", reinterpret_cast<char*>(cfg.sta.ssid), &ssidLen) == ESP_OK && cfg.sta.ssid[0];
    if (nvs_get_str(h, "pass", reinterpret_cast<char*>(cfg.sta.password), &passLen) != ESP_OK) {
      cfg.sta.password[0] = '\0';
    }
    nvs_close(h);
  }
  snprintf(wifiSsid, sizeof(wifiSsid), "%s", wifiConfigured ? (const char*)cfg.sta.ssid : "");
  esp_wifi_disconnect();
  if (!wifiConfigured) {
    ESP_LOGW(TAG, "Wi-Fi not configured; run tools/set-wifi.py or tools/ble-config.py");
    return;
  }
  ESP_LOGI(TAG, "Wi-Fi connecting to \"%s\"", cfg.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
  esp_wifi_connect();
}

static void refreshApClients() {
  wifi_sta_list_t list;
  apClients = esp_wifi_ap_get_sta_list(&list) == ESP_OK ? list.num : 0;
}

static void onWifiEvent(void*, esp_event_base_t base, int32_t id, void* data) {
  if (base == WIFI_EVENT && (id == WIFI_EVENT_AP_STACONNECTED || id == WIFI_EVENT_AP_STADISCONNECTED)) {
    refreshApClients();
    ESP_LOGI(TAG, "AP: %d station(s) connected", apClients.load());
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    broadcastAddr = 0;
    if (wifiConfigured) esp_wifi_connect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    const auto* ev = static_cast<ip_event_got_ip_t*>(data);
    const uint32_t bcast = ev->ip_info.ip.addr | ~ev->ip_info.netmask.addr;
    broadcastAddr = bcast;
    ESP_LOGI(TAG, "Wi-Fi connected: " IPSTR ", broadcast " IPSTR ":%d", IP2STR(&ev->ip_info.ip),
             IP2STR(reinterpret_cast<const esp_ip4_addr_t*>(&bcast)), CONFIG_LOGBO_UDP_PORT);
  }
}

static void initWifi() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  esp_netif_t* apNetif = esp_netif_create_default_wifi_ap();
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init));
  // The credentials already live in our own NVS namespace; don't copy them.
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiEvent, nullptr));
  // AP + STA: the STA joins the configured network, the AP lets a phone or PC connect directly
  // (no router at the track). The radio has one channel, so the AP follows the STA's channel.
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  wifi_config_t ap = {};
  strlcpy(reinterpret_cast<char*>(ap.ap.ssid), LOGBO_AP_SSID, sizeof(ap.ap.ssid));
  strlcpy(reinterpret_cast<char*>(ap.ap.password), LOGBO_AP_PASSWORD, sizeof(ap.ap.password));
  ap.ap.ssid_len = strlen(LOGBO_AP_SSID);
  ap.ap.channel = 1;
  ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
  ap.ap.max_connection = 4;
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
  esp_netif_ip_info_t apIp = {};
  if (esp_netif_get_ip_info(apNetif, &apIp) == ESP_OK) apBroadcastAddr = apIp.ip.addr | ~apIp.netmask.addr;
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG, "AP \"%s\" password \"%s\" at " IPSTR, LOGBO_AP_SSID, LOGBO_AP_PASSWORD, IP2STR(&apIp.ip));
  applyWifiConfig();

  udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  const int yes = 1;
  setsockopt(udpSock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
}

// ---------------------------------------------------------------------------
// BLE peripheral: config service. Advertised for CONFIG_LOGBO_CONFIG_WINDOW_S seconds after boot; a client
// that connects within the window may stay until it disconnects. Both characteristics need an encrypted
// link (Just Works pairing, no bonding), so the password is not sent in the clear.
//   status (read):  JSON, see statusJson()
//   wifi   (write): "<ssid>\t<password>", saved to NVS and applied at once

// 128-bit UUIDs are stored little-endian.
static constexpr ble_uuid128_t uuid128(const uint8_t (&be)[16]) {
  ble_uuid128_t u{};
  u.u.type = BLE_UUID_TYPE_128;
  for (int i = 0; i < 16; i++) u.value[i] = be[15 - i];
  return u;
}

// 9b3c0001-4e5f-4a2b-8c6d-2f1e0a7b5c3d and ...0002 / ...0003
static constexpr ble_uuid128_t kConfigService = uuid128(
    {0x9B, 0x3C, 0x00, 0x01, 0x4E, 0x5F, 0x4A, 0x2B, 0x8C, 0x6D, 0x2F, 0x1E, 0x0A, 0x7B, 0x5C, 0x3D});
static constexpr ble_uuid128_t kConfigStatus = uuid128(
    {0x9B, 0x3C, 0x00, 0x02, 0x4E, 0x5F, 0x4A, 0x2B, 0x8C, 0x6D, 0x2F, 0x1E, 0x0A, 0x7B, 0x5C, 0x3D});
static constexpr ble_uuid128_t kConfigWifi = uuid128(
    {0x9B, 0x3C, 0x00, 0x03, 0x4E, 0x5F, 0x4A, 0x2B, 0x8C, 0x6D, 0x2F, 0x1E, 0x0A, 0x7B, 0x5C, 0x3D});

static uint8_t ownAddrType;
static std::atomic<bool> configWindowOpen{true};
static std::atomic<uint16_t> configConnHandle{BLE_HS_CONN_HANDLE_NONE};

// {"gnss":{"configured":true,"fix":3,"sats":14},"rec":"rec","log":"log_0012_01.csv",
//  "wifi_ssid":"...","wifi_connected":true,"ip":"192.168.11.38"}
static size_t statusJson(char* buf, size_t size) {
  static const char* const kRec[] = {"off", "rec", "saving", "safe", "failed"};
  char ssid[sizeof(wifiSsid) * 6];
  record::jsonEscape(wifiSsid, ssid, sizeof(ssid));
  const bool connected = broadcastAddr != 0;
  char ip[24] = "null";
  esp_netif_ip_info_t info = {};
  if (connected && esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &info) == ESP_OK) {
    snprintf(ip, sizeof(ip), "\"" IPSTR "\"", IP2STR(&info.ip));
  }
  const gnss::Stats gs = gnss::stats();
  const int n = snprintf(buf, size,
                         "{\"gnss\":{\"configured\":%s,\"messages\":%lu},\"rec\":\"%s\",\"log\":\"%s\","
                         "\"wifi_ssid\":\"%s\",\"wifi_connected\":%s,\"ip\":%s}",
                         gs.configured ? "true" : "false", static_cast<unsigned long>(gs.pvt),
                         kRec[static_cast<int>(sdlog::state())], sdlog::fileName(), ssid,
                         connected ? "true" : "false", ip);
  return n > 0 && static_cast<size_t>(n) < size ? n : 0;
}

static int onConfigAccess(uint16_t, uint16_t, ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    char json[400];
    const size_t n = statusJson(json, sizeof(json));
    return n && os_mbuf_append(ctxt->om, json, n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    char buf[32 + 1 + 64 + 1];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len) != 0) {
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    buf[len] = '\0';
    char* tab = strchr(buf, '\t');
    if (!tab) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    *tab = '\0';
    if (const char* err = saveWifiCredentials(buf, tab + 1)) {
      ESP_LOGW(TAG, "Wi-Fi write over BLE rejected: %s", err);
      return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    ESP_LOGI(TAG, "Wi-Fi credentials updated over BLE");
    applyWifiConfig();
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static const ble_gatt_chr_def kConfigChrs[] = {
    {.uuid = &kConfigStatus.u,
     .access_cb = onConfigAccess,
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC},
    {.uuid = &kConfigWifi.u,
     .access_cb = onConfigAccess,
     .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC},
    {},
};

static const ble_gatt_svc_def kConfigSvcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &kConfigService.u, .characteristics = kConfigChrs},
    {},
};

static int onConfigGapEvent(ble_gap_event* ev, void*);

static void startConfigAdvertising() {
  if (!configWindowOpen || configConnHandle != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active()) {
    return;
  }
  ble_hs_adv_fields adv = {};
  adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv.uuids128 = &kConfigService;
  adv.num_uuids128 = 1;
  adv.uuids128_is_complete = 1;
  ble_hs_adv_fields rsp = {};
  const char* name = ble_svc_gap_device_name();
  rsp.name = reinterpret_cast<const uint8_t*>(name);
  rsp.name_len = strlen(name);
  rsp.name_is_complete = 1;
  ble_gap_adv_params params = {};
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  params.itvl_min = BLE_GAP_ADV_ITVL_MS(200);
  params.itvl_max = BLE_GAP_ADV_ITVL_MS(300);
  int rc = ble_gap_adv_set_fields(&adv);
  if (rc == 0) rc = ble_gap_adv_rsp_set_fields(&rsp);
  if (rc == 0) {
    rc = ble_gap_adv_start(ownAddrType, nullptr, BLE_HS_FOREVER, &params, onConfigGapEvent, nullptr);
  }
  if (rc != 0) ESP_LOGE(TAG, "Config advertising failed: %d", rc);
}

static void closeConfigWindow(void*) {
  configWindowOpen = false;
  ble_gap_adv_stop();
  ESP_LOGI(TAG, "Config window closed%s",
           configConnHandle != BLE_HS_CONN_HANDLE_NONE ? " (client stays until it disconnects)" : "");
}

static int onConfigGapEvent(ble_gap_event* ev, void*) {
  switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status == 0) {
        configConnHandle = ev->connect.conn_handle;
        ESP_LOGI(TAG, "Config client connected");
        // Ask the client to pair right away; macOS does not start pairing on its
        // own when a read fails with "insufficient encryption".
        ble_gap_security_initiate(ev->connect.conn_handle);
      } else {
        startConfigAdvertising();
      }
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      configConnHandle = BLE_HS_CONN_HANDLE_NONE;
      ESP_LOGI(TAG, "Config client disconnected, reason=0x%x", ev->disconnect.reason);
      startConfigAdvertising();
      break;
    case BLE_GAP_EVENT_ENC_CHANGE:
      ESP_LOGI(TAG, "Config link encryption: %s", ev->enc_change.status == 0 ? "on" : "failed");
      break;
    default:
      break;
  }
  return 0;
}

static void onBleSync() {
  ESP_ERROR_CHECK(ble_hs_util_ensure_addr(0));
  ESP_ERROR_CHECK(ble_hs_id_infer_auto(0, &ownAddrType));
  ble_svc_gap_device_name_set(deviceName);
  startConfigAdvertising();
}

static void onBleReset(int reason) { ESP_LOGE(TAG, "NimBLE reset, reason=%d", reason); }

static void bleHostTask(void*) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void initBle() {
  ESP_ERROR_CHECK(nimble_port_init());
  ble_hs_cfg.sync_cb = onBleSync;
  ble_hs_cfg.reset_cb = onBleReset;
  // Encrypted link for the config service without stored bonds (Just Works).
  ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
  ble_hs_cfg.sm_bonding = 0;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;
  ble_svc_gap_init();
  ble_svc_gatt_init();
  ESP_ERROR_CHECK(ble_gatts_count_cfg(kConfigSvcs));
  ESP_ERROR_CHECK(ble_gatts_add_svcs(kConfigSvcs));
  nimble_port_freertos_init(bleHostTask);

  const esp_timer_create_args_t window = {.callback = closeConfigWindow, .name = "config_window"};
  esp_timer_handle_t windowTimer;
  ESP_ERROR_CHECK(esp_timer_create(&window, &windowTimer));
  ESP_ERROR_CHECK(esp_timer_start_once(windowTimer, CONFIG_LOGBO_CONFIG_WINDOW_S * 1000000LL));
}

// ---------------------------------------------------------------------------
// Records: one per GNSS epoch (from the GNSS task), or from the watchdog task at the configured rate while
// the receiver is silent, so the IMU is still logged without GNSS.

// Held while a record is built and sent: the GNSS task and the watchdog may meet when the GNSS resumes.
static SemaphoreHandle_t recordMutex;

// Result of the last magnetometer calibration, for the display (see logger::Status::magCalResult).
static std::atomic<uint8_t> magCalResult{0};
static std::atomic<int64_t> magCalResultAtUs{0};

static void publishStatus(const record::Record* r) {
  static logger::Status ds = {};
  static int64_t slowAt = 0;
  static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
  const imu::MagCalInfo mc = imu::magCalInfo();
  taskENTER_CRITICAL(&lock);  // called from the GNSS task and the watchdog task
  ds.magCalibrated = mc.calibrated;
  ds.magField = mc.calibrated ? mc.cal.field : 0;
  ds.magCollecting = mc.collecting;
  ds.magCalSession = mc.session;
  ds.magCalSamples = mc.samples;
  for (int k = 0; k < 3; k++) ds.magCoverage[k] = mc.coverage[k];
  ds.magCalComplete = mc.complete;
  ds.magCalResult = magCalResult;
  ds.magCalResultAtUs = magCalResultAtUs;
  ds.configWindow = configWindowOpen;
  ds.wifiConfigured = wifiConfigured;
  ds.wifiUp = broadcastAddr != 0;
  ds.apClients = apClients;
  ds.udpErrors = udpErrors;
  ds.rec = static_cast<uint8_t>(sdlog::state());
  ds.imuOk = imuStatus.bmi270;
  ds.magOk = imuStatus.bmm150;
  ds.baroOk = imuStatus.bmp280;
  ds.records = records;
  ds.gnssMessages = pvtCount;
  const int64_t now = esp_timer_get_time();
  const bool slow = now - slowAt >= 1000000;  // RSSI, IP, ... change slowly
  if (slow) slowAt = now;
  if (r) {
    const gnss::Pvt& p = r->pvt;
    ds.dataAtUs = r->atUs;
    if (r->hasPvt) ds.gnssAtUs = r->atUs;
    ds.unixUs = r->hasPvt ? record::unixMicros(p) : 0;
    ds.fix = p.fixType;
    ds.fixOk = p.fixOk();
    ds.sats = p.numSV;
    ds.lat = p.lat;
    ds.lon = p.lon;
    ds.altMm = p.hMSL;
    ds.hAccMm = p.hAcc;
    ds.speedMms = p.gSpeed;
    ds.headingE5 = p.headMot;
    ds.headingValid = p.headingValid();
    const imu::Sample& s = r->imu;
    if (s.n) {
      ds.ax = s.ax, ds.ay = s.ay, ds.az = s.az;
      ds.gx = s.gx, ds.gy = s.gy, ds.gz = s.gz;
    }
    if (s.magValid) ds.mx = s.mx, ds.my = s.my, ds.mz = s.mz;
    ds.carValid = r->carValid;
    if (r->carValid) for (int k = 0; k < 3; k++) ds.carG[k] = r->carG[k];
    if (s.baroValid) ds.pressureHpa = s.pressureHpa, ds.tempC = s.tempC;
  }
  logger::Status copy = ds;
  taskEXIT_CRITICAL(&lock);

  if (slow) {  // outside the critical section: these calls may block
    const gnss::Stats gs = gnss::stats();
    wifi_ap_record_t ap;
    copy.rssi = copy.wifiUp && esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
    esp_netif_ip_info_t ip = {};
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip);
    snprintf(copy.ip, sizeof(copy.ip), IPSTR, IP2STR(&ip.ip));
    snprintf(copy.ssid, sizeof(copy.ssid), "%s", wifiSsid);
    snprintf(copy.logName, sizeof(copy.logName), "%s", sdlog::fileName());
    taskENTER_CRITICAL(&lock);
    ds.rssi = copy.rssi;
    memcpy(ds.ip, copy.ip, sizeof(ds.ip));
    memcpy(ds.ssid, copy.ssid, sizeof(ds.ssid));
    memcpy(ds.logName, copy.logName, sizeof(ds.logName));
    ds.gnssConfigured = copy.gnssConfigured = gs.configured;
    ds.gnssBaud = copy.gnssBaud = gs.baud;
    taskEXIT_CRITICAL(&lock);
  }
  logger::displayUpdate(copy);
}

static void emitRecord(const gnss::Pvt* pvt) {
  xSemaphoreTake(recordMutex, portMAX_DELAY);
  static record::Record r;
  r = {};
  r.atUs = esp_timer_get_time();
  r.hasPvt = pvt != nullptr;
  if (pvt) r.pvt = *pvt;
  imu::takeAverage(r.imu);
  // Car axes (gravity learnt while standing still): for the UDP "g" / "gyro_dps" and the display's G circle
  static carframe::Estimator car;
  if (r.imu.n) {
    const float a[3] = {r.imu.ax, r.imu.ay, r.imu.az}, g[3] = {r.imu.gx, r.imu.gy, r.imu.gz};
    const int gnssStill = pvt && pvt->fixOk() ? (pvt->gSpeed < 278 ? 1 : 0) : -1;  // 278 mm/s = 1 km/h
    car.update(a, g, gnssStill);
    r.carValid = car.ready();
    if (r.carValid) {
      car.toCar(a, r.carG);
      car.toCar(g, r.carGyro);
    }
  }
  static uint32_t seq = 0;
  records++;
  sdlog::write(r);
  publishStatus(&r);

  static char json[record::kMaxUdpPayload + 1];
  const size_t n = record::toJson(r, seq++, deviceName, json, sizeof(json));
  if (n) sendUdp(json, n);
  xSemaphoreGive(recordMutex);
}

static void onPvt(const gnss::Pvt& p) {
  lastPvtUs = esp_timer_get_time();
  pvtCount++;
  emitRecord(&p);
}

static void sendStatus() {
  const gnss::Stats gs = gnss::stats();
  int rssi = 0;
  wifi_ap_record_t ap;
  if (broadcastAddr && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
  esp_netif_ip_info_t ip = {};
  esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip);

  char json[record::kMaxUdpPayload + 1];
  const int n = snprintf(
      json, sizeof(json),
      "{\"type\":\"status\",\"uptime_ms\":%lu,\"records\":%lu,"
      "\"gnss\":{\"configured\":%s,\"baud\":%lu,\"pvt\":%lu,\"checksum_errors\":%lu,\"other\":%lu,\"bytes\":%lu},"
      "\"imu\":%s,\"mag\":%s,\"mag_cal\":%s,\"baro\":%s,\"rec\":%d,\"log\":\"%s\",\"udp_errors\":%lu,"
      "\"wifi_configured\":%s,\"wifi_rssi\":%d,\"ip\":\"" IPSTR "\",\"free_heap\":%lu}",
      static_cast<unsigned long>(esp_timer_get_time() / 1000), static_cast<unsigned long>(records.load()),
      gs.configured ? "true" : "false", static_cast<unsigned long>(gs.baud), static_cast<unsigned long>(gs.pvt),
      static_cast<unsigned long>(gs.checksumErrors), static_cast<unsigned long>(gs.otherMsgs),
      static_cast<unsigned long>(gs.bytes), imuStatus.bmi270 ? "true" : "false",
      imuStatus.bmm150 ? "true" : "false", imu::magCalInfo().calibrated ? "true" : "false",
      imuStatus.bmp280 ? "true" : "false",
      static_cast<int>(sdlog::state()), sdlog::fileName(), static_cast<unsigned long>(udpErrors.load()),
      wifiConfigured ? "true" : "false", rssi, IP2STR(&ip.ip), static_cast<unsigned long>(esp_get_free_heap_size()));
  if (n > 0 && static_cast<size_t>(n) < sizeof(json)) sendUdp(json, n);
  ESP_LOGI(TAG, "%s", json);
}

// Runs at the epoch rate: makes IMU-only records while the GNSS is silent, refreshes the display state and
// sends the status datagram once per second.
static void watchdogTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_LOGBO_GNSS_RATE_HZ) ?: 1;
  TickType_t wake = xTaskGetTickCount();
  int64_t lastStatus = 0;
  for (;;) {
    vTaskDelayUntil(&wake, period);
    const int64_t now = esp_timer_get_time();
    if (now - lastPvtUs > 1000000) {
      emitRecord(nullptr);
    } else {
      publishStatus(nullptr);
    }
    if (now - lastStatus >= 1000000) {
      lastStatus = now;
      sendStatus();
    }
  }
}

// ---------------------------------------------------------------------------
// Serial console (UART0, the M5Stack's USB port), one command per line:
//   wifi-set<TAB><ssid><TAB><password>   save credentials to NVS and reconnect
//   wifi-clear                           erase credentials
//   mag-cal                              start the magnetometer calibration (rotate the unit in every
//                                        direction), mag-cal-done to finish it
//   mag-clear                            forget the magnetometer calibration
//   mag                                  show the calibration and the latest reading

static void handleCommand(char* line) {
  if (strncmp(line, "wifi-set\t", 9) == 0) {
    char* ssid = line + 9;
    char* tab = strchr(ssid, '\t');
    if (!tab || tab == ssid) {
      printf("ERR usage: wifi-set<TAB>ssid<TAB>password\n");
      return;
    }
    *tab = '\0';
    if (const char* err = saveWifiCredentials(ssid, tab + 1)) {
      printf("ERR %s\n", err);
      return;
    }
    printf("OK wifi saved\n");
    applyWifiConfig();
  } else if (strcmp(line, "wifi-clear") == 0) {
    clearWifiCredentials();
    printf("OK wifi cleared\n");
    applyWifiConfig();
  } else if (strcmp(line, "mag-cal") == 0) {
    logger::onMagCalStart();
    printf("OK rotate the unit slowly in every direction, then mag-cal-done\n");
  } else if (strcmp(line, "mag-cal-done") == 0) {
    logger::onMagCalFinish();
    static const char* const kResult[] = {"",
                                          "OK",
                                          "ERR too few samples",
                                          "ERR not rotated enough",
                                          "ERR bad fit",
                                          "ERR not collecting (run mag-cal first)",
                                          "ERR applied but not saved (lost at the next reboot)"};
    const uint8_t res = magCalResult;
    printf("%s\n", res < 7 ? kResult[res] : "ERR");
  } else if (strcmp(line, "mag-clear") == 0) {
    if (imu::magCalClear()) {
      printf("OK mag calibration cleared\n");
    } else {
      printf("ERR cleared until the next reboot (NVS failed)\n");
    }
  } else if (strcmp(line, "mag") == 0) {
    const imu::MagCalInfo mc = imu::magCalInfo();
    imu::Sample s;
    imu::latest(s);
    if (mc.calibrated) {
      printf("cal offset %.1f %.1f %.1f uT scale %.3f %.3f %.3f field %.1f uT\n", mc.cal.offset[0],
             mc.cal.offset[1], mc.cal.offset[2], mc.cal.scale[0], mc.cal.scale[1], mc.cal.scale[2], mc.cal.field);
    } else {
      printf("cal none\n");
    }
    printf("mag %.1f %.1f %.1f uT (%s)%s\n", s.mx, s.my, s.mz, s.magCal ? "calibrated" : "raw",
           mc.collecting ? " collecting" : "");
  } else if (line[0]) {
    printf("ERR unknown command\n");
  }
}

static void consoleTask(void*) {
  char line[200];
  size_t len = 0;
  for (;;) {
    uint8_t c;
    if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) <= 0) continue;
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = '\0';
      handleCommand(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = static_cast<char>(c);
    }
  }
}

static void initConsole() {
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, nullptr, 0));
  uart_vfs_dev_use_driver(UART_NUM_0);
}

// ---------------------------------------------------------------------------

extern "C" void app_main() {
  gpio_set_direction(kSpeaker, GPIO_MODE_OUTPUT);
  gpio_set_level(kSpeaker, 0);
  gpio_set_direction(kLcdCs, GPIO_MODE_OUTPUT);
  gpio_set_level(kLcdCs, 1);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  snprintf(deviceName, sizeof(deviceName), "logbo-%02X%02X", mac[4], mac[5]);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  esp_pm_config_t pm = {};
  pm.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  pm.min_freq_mhz = CONFIG_LOGBO_CPU_MIN_MHZ;
  pm.light_sleep_enable = false;
  ESP_ERROR_CHECK(esp_pm_configure(&pm));

  initConsole();
  sdlog::init();  // before the display: the LCD shares its SPI bus
  logger::displayStart();
  imuStatus = imu::init();
  ESP_LOGI(TAG, "IMU: BMI270 %s, BMM150 %s, BMP280 %s", imuStatus.bmi270 ? "ok" : "MISSING",
           imuStatus.bmm150 ? "ok" : "MISSING", imuStatus.bmp280 ? "ok" : "MISSING");
  initWifi();
  initBle();

  recordMutex = xSemaphoreCreateMutex();
  gnss::start(kGnssRx, kGnssTx, CONFIG_LOGBO_GNSS_RATE_HZ, onPvt);
  xTaskCreate(watchdogTask, "watchdog", 6144, nullptr, 9, nullptr);
  xTaskCreate(consoleTask, "console", 4096, nullptr, 2, nullptr);
}

uint32_t logger::onMagCalStart() {
  magCalResult = 0;
  return imu::magCalStart();
}

void logger::onMagCalFinish() {
  magCalResult = static_cast<uint8_t>(imu::magCalFinish()) + 1;  // MagCalError::kNone -> 1 (ok)
  magCalResultAtUs = esp_timer_get_time();
}

void logger::onMagCalCancel() { imu::magCalCancel(); }

void logger::onLongPress() {
  if (sdlog::state() == sdlog::State::Rec) {
    sdlog::requestStop();
  } else {
    sdlog::requestStart();  // Safe, Failed or no card at boot; ignored while saving
  }
}
