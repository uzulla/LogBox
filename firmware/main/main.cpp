// logbo: stand-alone GNSS / IMU logger on an M5Stack (Gray / Basic) with the M5Stack GNSS module
// (u-blox NEO-M9N, BMI270 + BMM150, BMP280).
//
//   GNSS (UART, NAV-PVT every epoch) --+--> record (PVT + IMU mean over the epoch) --> SD card CSV
//   IMU / mag / baro (I2C)  -----------+                                          \--> Wi-Fi UDP broadcast (JSON)
//
// Wi-Fi credentials are set over the serial console or the BLE config service (advertised for a while after boot).

#include <stdio.h>
#include <stdlib.h>
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
#include "freertos/event_groups.h"
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
#include "wifinets.h"
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
static char deviceName[16];  // "logbo-XXXX" (BLE name, UDP "device")

static imu::Status imuStatus;
static std::atomic<uint32_t> records{0};
static std::atomic<int64_t> lastPvtUs{0};
static std::atomic<uint32_t> pvtCount{0};
static std::atomic<uint8_t> lastFix{0}, lastSats{0};  // of the latest NAV-PVT, for the BLE status
// Result of the last magnetometer calibration, for the display (see logger::Status::magCalResult).
static std::atomic<uint8_t> magCalResult{0};
static std::atomic<int64_t> magCalResultAtUs{0};

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
// Wi-Fi STA: the networks in wifinets (NVS), tried from the highest priority down until one gives an IP.
// While joined, a disconnect or a change of the list starts over from the top.

static EventGroupHandle_t wifiEvents;
constexpr EventBits_t kWifiGotIp = 1 << 0, kWifiDisconnected = 1 << 1, kWifiChanged = 1 << 2;
constexpr TickType_t kWifiJoinTimeout = pdMS_TO_TICKS(30000);  // association + DHCP, per network
constexpr TickType_t kWifiRetryIdle = pdMS_TO_TICKS(10000);    // after every network failed
static std::atomic<int> wifiCount{0};                           // networks in NVS
static bool radioApOn = true, radioStaOn = true;                // this boot (see the radio switches below)
// BLE clients of the config service: up to CONFIG_BT_NIMBLE_MAX_CONNECTIONS at once (advertising goes on while there
// is room). Each has its own read position in the Wi-Fi list, which is read in pages of at most 512 bytes (the ATT
// limit): a client writes "list=<next>" and reads again. Back to the top on a change of the list (from anywhere).
struct BleClient {
  uint16_t handle;
  int listStart;
};
static BleClient bleClients[CONFIG_BT_NIMBLE_MAX_CONNECTIONS];
static std::atomic<uint32_t> wifiListGen{1};  // bumped on every change of the list (shown as "gen")
static int bleClientCount = 0;
static portMUX_TYPE bleClientsLock = portMUX_INITIALIZER_UNLOCKED;

static void bleClientAdd(uint16_t handle) {
  taskENTER_CRITICAL(&bleClientsLock);
  if (bleClientCount < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) bleClients[bleClientCount++] = {handle, 0};
  taskEXIT_CRITICAL(&bleClientsLock);
}

static void bleClientRemove(uint16_t handle) {
  taskENTER_CRITICAL(&bleClientsLock);
  for (int i = 0; i < bleClientCount; i++) {
    if (bleClients[i].handle == handle) {
      bleClients[i] = bleClients[--bleClientCount];
      break;
    }
  }
  taskEXIT_CRITICAL(&bleClientsLock);
}

static int bleClientsConnected() {
  taskENTER_CRITICAL(&bleClientsLock);
  const int n = bleClientCount;
  taskEXIT_CRITICAL(&bleClientsLock);
  return n;
}

static int wifiListStartOf(uint16_t handle) {
  int start = 0;
  taskENTER_CRITICAL(&bleClientsLock);
  for (int i = 0; i < bleClientCount; i++) {
    if (bleClients[i].handle == handle) start = bleClients[i].listStart;
  }
  taskEXIT_CRITICAL(&bleClientsLock);
  return start;
}

static void setWifiListStart(uint16_t handle, int start) {
  taskENTER_CRITICAL(&bleClientsLock);
  for (int i = 0; i < bleClientCount; i++) {
    if (bleClients[i].handle == handle) bleClients[i].listStart = start;
  }
  taskEXIT_CRITICAL(&bleClientsLock);
}

static void resetWifiListStarts() {
  taskENTER_CRITICAL(&bleClientsLock);
  for (int i = 0; i < bleClientCount; i++) bleClients[i].listStart = 0;
  taskEXIT_CRITICAL(&bleClientsLock);
}
static std::atomic<bool> staAssociated{false};                  // between STA_CONNECTED and STA_DISCONNECTED
static portMUX_TYPE wifiLock = portMUX_INITIALIZER_UNLOCKED;
static char wifiTarget[33];  // network being tried or joined, "" if none (guarded by wifiLock)

static void setWifiTarget(const char* ssid) {
  taskENTER_CRITICAL(&wifiLock);
  strlcpy(wifiTarget, ssid, sizeof(wifiTarget));
  taskEXIT_CRITICAL(&wifiLock);
}

static void wifiTargetCopy(char (&out)[33]) {
  taskENTER_CRITICAL(&wifiLock);
  memcpy(out, wifiTarget, sizeof(out));
  taskEXIT_CRITICAL(&wifiLock);
}

// The list changed (BLE, console): count it (also while the STA is off), read it from the top again, and reconnect from
// the top of the new list.
static void wifiListChanged() {
  wifiCount = wifinets::count();
  wifiListGen++;
  resetWifiListStarts();
  if (wifiEvents) xEventGroupSetBits(wifiEvents, kWifiChanged);
}

// Adds / updates / deletes a network from one of the wifinets text formats. Returns nullptr on success, else
// an error message; *full / *notFound tell the BLE side which ATT error to give.
static const char* applyWifiText(const char* text, bool* full = nullptr, bool* notFound = nullptr) {
  wifinets::Command cmd;
  switch (wifinets::parse(text, cmd)) {
    case wifinets::ParseError::kNoSsid: return "no SSID";
    case wifinets::ParseError::kTooLong: return "SSID must be 1-32 bytes, password at most 64";
    case wifinets::ParseError::kInvalid: return "invalid value (NUL byte, or priority not a number in +-1000000)";
    default: break;
  }
  const wifinets::StoreError err =
      cmd.op == wifinets::Op::kDelete ? wifinets::remove(cmd.entry.ssid) : wifinets::set(cmd.entry);
  if (full) *full = err == wifinets::StoreError::kFull;
  if (notFound) *notFound = err == wifinets::StoreError::kNotFound;
  switch (err) {
    case wifinets::StoreError::kFull: return "already 8 networks; delete one first";
    case wifinets::StoreError::kNotFound: return "no such network";
    case wifinets::StoreError::kNvs: return "NVS write failed";
    default: break;
  }
  ESP_LOGI(TAG, "Wi-Fi network \"%s\" %s", cmd.entry.ssid, cmd.op == wifinets::Op::kDelete ? "deleted" : "saved");
  wifiListChanged();
  return nullptr;
}

// Leaves the current network and waits for its disconnect event, so that event is not taken for the failure of
// the next attempt.
static void staLeave() {
  if (!staAssociated) {
    esp_wifi_disconnect();
    return;
  }
  xEventGroupClearBits(wifiEvents, kWifiDisconnected);
  esp_wifi_disconnect();
  xEventGroupWaitBits(wifiEvents, kWifiDisconnected, pdFALSE, pdFALSE, pdMS_TO_TICKS(2000));
}

static void staTask(void*) {
  static wifinets::Entry list[wifinets::kMax];
  for (;;) {
    xEventGroupClearBits(wifiEvents, kWifiChanged);
    const int n = wifinets::load(list);
    wifiCount = n;
    if (!n) {
      staLeave();  // before clearing the target: the disconnect event is matched against it
      setWifiTarget("");
      ESP_LOGW(TAG, "Wi-Fi not configured; set a network over BLE (docs/index.html, tools/ble-config.py) or "
                    "tools/set-wifi.py");
      xEventGroupWaitBits(wifiEvents, kWifiChanged, pdFALSE, pdFALSE, portMAX_DELAY);
      continue;
    }
    wifinets::sortByPriority(list, n);
    bool joined = false, changed = false;
    for (int i = 0; i < n && !joined && !changed; i++) {
      ESP_LOGI(TAG, "Wi-Fi trying \"%s\" (priority %ld)", list[i].ssid, static_cast<long>(list[i].prio));
      staLeave();  // leave the previous network while it is still the target (its disconnect event is matched)
      setWifiTarget(list[i].ssid);
      wifi_config_t cfg = {};
      // Fixed-size fields: a 32-byte SSID / 64-character PSK fills them with no terminator (strlcpy would cut
      // the last byte off). The entries are validated to fit.
      memcpy(cfg.sta.ssid, list[i].ssid, strnlen(list[i].ssid, sizeof(cfg.sta.ssid)));
      memcpy(cfg.sta.password, list[i].pass, strnlen(list[i].pass, sizeof(cfg.sta.password)));
      cfg.sta.threshold.authmode = list[i].pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
      xEventGroupClearBits(wifiEvents, kWifiGotIp | kWifiDisconnected);
      if (esp_wifi_set_config(WIFI_IF_STA, &cfg) != ESP_OK || esp_wifi_connect() != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi \"%s\": could not start connecting", list[i].ssid);
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      // An IP, a failure (disconnect event: wrong password, not found, ...), a new list, or the timeout
      const EventBits_t bits = xEventGroupWaitBits(wifiEvents, kWifiGotIp | kWifiDisconnected | kWifiChanged,
                                                   pdFALSE, pdFALSE, kWifiJoinTimeout);
      joined = bits & kWifiGotIp;
      changed = bits & kWifiChanged;
      if (!joined) ESP_LOGW(TAG, "Wi-Fi \"%s\" failed", list[i].ssid);
    }
    if (changed) continue;
    if (!joined) {
      staLeave();
      ESP_LOGW(TAG, "Wi-Fi: no network joined; retrying in %lu s",
               static_cast<unsigned long>(pdTICKS_TO_MS(kWifiRetryIdle) / 1000));
      xEventGroupWaitBits(wifiEvents, kWifiChanged, pdFALSE, pdFALSE, kWifiRetryIdle);
      continue;
    }
    // Joined: a disconnect starts over from the highest priority. A change of the list only does when it touches
    // the joined network (deleted, password or priority changed) or adds one with a higher priority: leaving the
    // network also moves the soft AP's channel and drops the phones on it.
    char joinedSsid[33];
    wifiTargetCopy(joinedSsid);
    int32_t joinedPrio = 0;
    char joinedPass[65] = "";
    for (int i = 0; i < n; i++) {
      if (!strcmp(list[i].ssid, joinedSsid)) {
        joinedPrio = list[i].prio;
        memcpy(joinedPass, list[i].pass, sizeof(joinedPass));
      }
    }
    for (;;) {
      const EventBits_t bits =
          xEventGroupWaitBits(wifiEvents, kWifiDisconnected | kWifiChanged, pdFALSE, pdFALSE, portMAX_DELAY);
      if (bits & kWifiDisconnected) break;
      xEventGroupClearBits(wifiEvents, kWifiChanged);
      const int m = wifinets::load(list);
      wifiCount = m;
      bool stay = false, better = false;
      for (int i = 0; i < m; i++) {
        if (!strcmp(list[i].ssid, joinedSsid)) {
          stay = list[i].prio == joinedPrio && !strcmp(list[i].pass, joinedPass);
        } else if (list[i].prio > joinedPrio) {
          better = true;
        }
      }
      if (!stay || better) {
        xEventGroupSetBits(wifiEvents, kWifiChanged);  // handled at the top of the loop
        break;
      }
      ESP_LOGI(TAG, "Wi-Fi list changed; staying on \"%s\"", joinedSsid);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// The STA's address; false (and 0.0.0.0) when the STA is off or not joined. The STA netif only exists when the
// STA was started, and none exists when Wi-Fi is off.
static bool staIpInfo(esp_netif_ip_info_t* ip) {
  *ip = {};
  if (!radioStaOn || !broadcastAddr) return false;
  esp_netif_t* sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  return sta && esp_netif_get_ip_info(sta, ip) == ESP_OK;
}

static void refreshApClients() {
  wifi_sta_list_t list;
  apClients = esp_wifi_ap_get_sta_list(&list) == ESP_OK ? list.num : 0;
}

static void onWifiEvent(void*, esp_event_base_t base, int32_t id, void* data) {
  if (base == WIFI_EVENT && (id == WIFI_EVENT_AP_STACONNECTED || id == WIFI_EVENT_AP_STADISCONNECTED)) {
    refreshApClients();
    ESP_LOGI(TAG, "AP: %d station(s) connected", apClients.load());
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
    staAssociated = true;
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    staAssociated = false;
    broadcastAddr = 0;
    xEventGroupClearBits(wifiEvents, kWifiGotIp);
    // Only a disconnect from the network being tried counts: a failed attempt can report more than once, and
    // the late reports must not fail the next network.
    const auto* ev = static_cast<wifi_event_sta_disconnected_t*>(data);
    char target[33];
    wifiTargetCopy(target);
    const size_t len = strnlen(reinterpret_cast<const char*>(ev->ssid), sizeof(ev->ssid));
    if (len == strlen(target) && !memcmp(ev->ssid, target, len)) {
      xEventGroupSetBits(wifiEvents, kWifiDisconnected);  // staTask decides what to try next
    }
  } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
    // The DHCP lease ran out without a disconnect: stop sending to the old subnet and join again
    broadcastAddr = 0;
    xEventGroupClearBits(wifiEvents, kWifiGotIp);
    xEventGroupSetBits(wifiEvents, kWifiDisconnected);
    ESP_LOGW(TAG, "Wi-Fi lost the IP address");
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    const auto* ev = static_cast<ip_event_got_ip_t*>(data);
    const uint32_t bcast = ev->ip_info.ip.addr | ~ev->ip_info.netmask.addr;
    broadcastAddr = bcast;
    xEventGroupSetBits(wifiEvents, kWifiGotIp);
    ESP_LOGI(TAG, "Wi-Fi connected: " IPSTR ", broadcast " IPSTR ":%d", IP2STR(&ev->ip_info.ip),
             IP2STR(reinterpret_cast<const esp_ip4_addr_t*>(&bcast)), CONFIG_LOGBO_UDP_PORT);
  }
}

// ---------------------------------------------------------------------------
// Radio switches: the soft AP and the STA can each be turned off. Saved in NVS namespace "radio" (u8 "ap" /
// "sta", missing = on) and applied at the next boot. Both off: Wi-Fi is not started at all; BLE stays, so the
// Web page / tools/ble-config.py can turn them back on.


static void radioSaved(bool* ap, bool* sta) {
  *ap = *sta = true;
  nvs_handle_t h;
  if (nvs_open("radio", NVS_READONLY, &h) != ESP_OK) return;
  uint8_t v;
  if (nvs_get_u8(h, "ap", &v) == ESP_OK) *ap = v != 0;
  if (nvs_get_u8(h, "sta", &v) == ESP_OK) *sta = v != 0;
  nvs_close(h);
}

static bool radioSave(const char* key, bool on) {
  nvs_handle_t h;
  if (nvs_open("radio", NVS_READWRITE, &h) != ESP_OK) return false;
  const bool ok = nvs_set_u8(h, key, on) == ESP_OK && nvs_commit(h) == ESP_OK;
  nvs_close(h);
  if (ok) ESP_LOGI(TAG, "radio: %s %s from the next boot", key, on ? "on" : "off");
  return ok;
}

// {"ap":true,"sta":false,"active":{"ap":true,"sta":true}}: saved (next boot) and this boot
static size_t radioJson(char* buf, size_t size) {
  bool ap, sta;
  radioSaved(&ap, &sta);
  const int n = snprintf(buf, size, "{\"ap\":%s,\"sta\":%s,\"active\":{\"ap\":%s,\"sta\":%s}}",
                         ap ? "true" : "false", sta ? "true" : "false", radioApOn ? "true" : "false",
                         radioStaOn ? "true" : "false");
  return n > 0 && static_cast<size_t>(n) < size ? n : 0;
}

static void initWifi() {
  wifiEvents = xEventGroupCreate();
  wifiCount = wifinets::count();
  radioSaved(&radioApOn, &radioStaOn);
  if (!radioApOn && !radioStaOn) {
    ESP_LOGW(TAG, "Wi-Fi off (AP and STA both turned off); BLE only");
    return;
  }
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  if (radioStaOn) esp_netif_create_default_wifi_sta();
  esp_netif_t* apNetif = radioApOn ? esp_netif_create_default_wifi_ap() : nullptr;
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init));
  // The credentials already live in our own NVS namespace; don't copy them.
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiEvent, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, onWifiEvent, nullptr));
  // AP + STA: the STA joins the configured network, the AP lets a phone or PC connect directly
  // (no router at the track). The radio has one channel, so the AP follows the STA's channel.
  ESP_ERROR_CHECK(esp_wifi_set_mode(radioApOn && radioStaOn ? WIFI_MODE_APSTA : radioApOn ? WIFI_MODE_AP : WIFI_MODE_STA));
  if (radioApOn) {
    wifi_config_t ap = {};
    strlcpy(reinterpret_cast<char*>(ap.ap.ssid), LOGBO_AP_SSID, sizeof(ap.ap.ssid));
    strlcpy(reinterpret_cast<char*>(ap.ap.password), LOGBO_AP_PASSWORD, sizeof(ap.ap.password));
    ap.ap.ssid_len = strlen(LOGBO_AP_SSID);
    ap.ap.channel = 1;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.max_connection = 4;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
  }
  esp_netif_ip_info_t apIp = {};
  if (apNetif && esp_netif_get_ip_info(apNetif, &apIp) == ESP_OK) apBroadcastAddr = apIp.ip.addr | ~apIp.netmask.addr;
  ESP_ERROR_CHECK(esp_wifi_start());
  if (radioApOn) {
    ESP_LOGI(TAG, "AP \"%s\" password \"%s\" at " IPSTR, LOGBO_AP_SSID, LOGBO_AP_PASSWORD, IP2STR(&apIp.ip));
  } else {
    ESP_LOGI(TAG, "AP off");
  }
  if (radioStaOn) {
    xTaskCreate(staTask, "wifi_sta", 4096, nullptr, 4, nullptr);
  } else {
    ESP_LOGI(TAG, "STA off");
  }

  udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  const int yes = 1;
  setsockopt(udpSock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
}

// ---------------------------------------------------------------------------
// BLE peripheral: config service. Advertised all the time (or, when CONFIG_LOGBO_CONFIG_WINDOW_S > 0, for that
// many seconds after boot; a client that connects within the window may stay until it disconnects). All characteristics need an encrypted
// link (Just Works pairing, no bonding), so the password is not sent in the clear. Used by tools/ble-config.py
// and the Web Bluetooth page docs/index.html.
//   status (read):  JSON, see statusJson()
//   wifi   (read):  the Wi-Fi networks (no passwords); (write): add / update / delete one, see wifinets.h
//   ctrl   (read):  magnetometer calibration state and the radio switches, see ctrlJson()
//          (write): one command, see onCtrlWrite()

// 128-bit UUIDs are stored little-endian.
static constexpr ble_uuid128_t uuid128(const uint8_t (&be)[16]) {
  ble_uuid128_t u{};
  u.u.type = BLE_UUID_TYPE_128;
  for (int i = 0; i < 16; i++) u.value[i] = be[15 - i];
  return u;
}

// 9b3c0001-4e5f-4a2b-8c6d-2f1e0a7b5c3d and ...0002 / ...0003 / ...0004
static constexpr ble_uuid128_t kConfigService = uuid128(
    {0x9B, 0x3C, 0x00, 0x01, 0x4E, 0x5F, 0x4A, 0x2B, 0x8C, 0x6D, 0x2F, 0x1E, 0x0A, 0x7B, 0x5C, 0x3D});
static constexpr ble_uuid128_t kConfigStatus = uuid128(
    {0x9B, 0x3C, 0x00, 0x02, 0x4E, 0x5F, 0x4A, 0x2B, 0x8C, 0x6D, 0x2F, 0x1E, 0x0A, 0x7B, 0x5C, 0x3D});
static constexpr ble_uuid128_t kConfigWifi = uuid128(
    {0x9B, 0x3C, 0x00, 0x03, 0x4E, 0x5F, 0x4A, 0x2B, 0x8C, 0x6D, 0x2F, 0x1E, 0x0A, 0x7B, 0x5C, 0x3D});
static constexpr ble_uuid128_t kConfigCtrl = uuid128(
    {0x9B, 0x3C, 0x00, 0x04, 0x4E, 0x5F, 0x4A, 0x2B, 0x8C, 0x6D, 0x2F, 0x1E, 0x0A, 0x7B, 0x5C, 0x3D});

static uint8_t ownAddrType;
static std::atomic<bool> configWindowOpen{true};

// {"device":"logbo-CD0E","uptime_s":120,"free_heap":45000,
//  "gnss":{"configured":true,"messages":1234,"baud":115200,"fix":3,"sats":14},"rec":"rec","log":"log_0012_01.csv",
//  "records":3000,"imu":true,"mag":true,"baro":true,"wifi_ssid":"...","wifi_connected":true,"ip":"192.168.11.38",
//  "rssi":-40,"ap_ssid":"LOGBO-XXXXXX","ap_clients":0}
// The earlier members keep their names and meaning (tools/ble-config.py). At most ~470 bytes even with an
// SSID of 32 characters that all need escaping, below the 512 bytes an attribute may have.
static size_t statusJson(char* buf, size_t size) {
  static const char* const kRec[] = {"off", "rec", "saving", "safe", "failed"};
  char target[33];
  wifiTargetCopy(target);
  char ssid[100];  // an SSID full of characters that need escaping is cut short: the reply must stay <= 512 bytes
  record::jsonEscape(target, ssid, sizeof(ssid));
  const bool connected = broadcastAddr != 0;
  char ip[24] = "null";
  esp_netif_ip_info_t info = {};
  if (connected && staIpInfo(&info)) {
    snprintf(ip, sizeof(ip), "\"" IPSTR "\"", IP2STR(&info.ip));
  }
  wifi_ap_record_t ap;
  const int rssi = connected && esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
  const gnss::Stats gs = gnss::stats();
  const bool fresh = lastPvtUs && esp_timer_get_time() - lastPvtUs < 2000000;
  const char* t = "true";
  const char* f = "false";
  const int n = snprintf(
      buf, size,
      "{\"device\":\"%s\",\"uptime_s\":%lu,\"free_heap\":%lu,"
      "\"gnss\":{\"configured\":%s,\"messages\":%lu,\"baud\":%lu,\"fix\":%u,\"sats\":%u},"
      "\"rec\":\"%s\",\"log\":\"%s\",\"records\":%lu,\"imu\":%s,\"mag\":%s,\"baro\":%s,"
      "\"wifi_ssid\":\"%s\",\"wifi_connected\":%s,\"wifi_networks\":%d,\"ip\":%s,\"rssi\":%d,"
      "\"ap_ssid\":\"%s\",\"ap_clients\":%d}",
      deviceName, static_cast<unsigned long>(esp_timer_get_time() / 1000000),
      static_cast<unsigned long>(esp_get_free_heap_size()), gs.configured ? t : f,
      static_cast<unsigned long>(gs.pvt), static_cast<unsigned long>(gs.baud), fresh ? lastFix.load() : 0,
      fresh ? lastSats.load() : 0, kRec[static_cast<int>(sdlog::state())], sdlog::fileName(),
      static_cast<unsigned long>(records.load()), imuStatus.bmi270 ? t : f, imuStatus.bmm150 ? t : f,
      imuStatus.bmp280 ? t : f, ssid, connected ? t : f, wifiCount.load(), ip, rssi, LOGBO_AP_SSID,
      apClients.load());
  return n > 0 && static_cast<size_t>(n) < size ? n : 0;
}

// {"calibrated":true,"field":34.4,"collecting":false,"session":3,"samples":0,"coverage":[0.00,0.00,0.00],
//  "complete":false,"result":1}   result: as logger::Status::magCalResult (0 none, 1 ok, 2.. failures)
static size_t magCalJsonOnly(char* buf, size_t size) {
  const imu::MagCalInfo mc = imu::magCalInfo();
  const char* t = "true";
  const char* f = "false";
  const int n = snprintf(buf, size,
                         "{\"calibrated\":%s,\"field\":%.1f,\"collecting\":%s,\"session\":%lu,\"samples\":%lu,"
                         "\"coverage\":[%.2f,%.2f,%.2f],\"complete\":%s,\"result\":%u}",
                         mc.calibrated ? t : f, mc.calibrated ? mc.cal.field : 0.0f, mc.collecting ? t : f,
                         static_cast<unsigned long>(mc.session), static_cast<unsigned long>(mc.samples),
                         mc.coverage[0], mc.coverage[1], mc.coverage[2], mc.complete ? t : f, magCalResult.load());
  return n > 0 && static_cast<size_t>(n) < size ? n : 0;
}

// ctrl read: {"magcal":{...magCalJsonOnly},"radio":{...radioJson}}
static size_t ctrlJson(char* buf, size_t size) {
  char mag[256], radio[96];
  if (!magCalJsonOnly(mag, sizeof(mag)) || !radioJson(radio, sizeof(radio))) return 0;
  const int n = snprintf(buf, size, "{\"magcal\":%s,\"radio\":%s}", mag, radio);
  return n > 0 && static_cast<size_t>(n) < size ? n : 0;
}

static void rebootSoon() {
  // Let the write response go out first
  static esp_timer_handle_t timer;
  if (!timer) {
    const esp_timer_create_args_t args = {.callback = [](void*) { esp_restart(); }, .name = "reboot"};
    if (esp_timer_create(&args, &timer) != ESP_OK) return;
  }
  esp_timer_start_once(timer, 500000);
}

// ctrl commands (ASCII, no newline):
//   rec=1 / rec=0                      start / stop the SD recording (as holding B)
//   mag=start / done / cancel / clear  magnetometer calibration (as the STATUS page and the serial console)
//   wifi=clear                         forget all Wi-Fi networks
//   ap=0 / ap=1, sta=0 / sta=1         turn the soft AP / the STA off or on from the next boot
//   reboot=1
// Unknown commands, and a calibration finish that fails, are refused with ATT error 0x13 (value not allowed);
// the calibration's result is in the ctrl read (ctrlJson) either way.
static int onCtrlWrite(const char* cmd) {
  if (strcmp(cmd, "rec=1") == 0) {
    sdlog::requestStart();
  } else if (strcmp(cmd, "rec=0") == 0) {
    sdlog::requestStop();
  } else if (strcmp(cmd, "mag=start") == 0) {
    logger::onMagCalStart();
  } else if (strcmp(cmd, "mag=done") == 0) {
    logger::onMagCalFinish();
    if (magCalResult != 1) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
  } else if (strcmp(cmd, "mag=cancel") == 0) {
    logger::onMagCalCancel();
  } else if (strcmp(cmd, "mag=clear") == 0) {
    if (!imu::magCalClear()) return BLE_ATT_ERR_UNLIKELY;
  } else if (strcmp(cmd, "wifi=clear") == 0) {
    if (wifinets::clear() != wifinets::StoreError::kNone) return BLE_ATT_ERR_UNLIKELY;
    wifiListChanged();
  } else if (!strncmp(cmd, "ap=", 3) || !strncmp(cmd, "sta=", 4)) {
    const char* v = strchr(cmd, '=') + 1;
    if (strcmp(v, "0") && strcmp(v, "1")) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    if (!radioSave(cmd[0] == 'a' ? "ap" : "sta", v[0] == '1')) return BLE_ATT_ERR_UNLIKELY;
  } else if (strcmp(cmd, "reboot=1") == 0) {
    rebootSoon();
  } else {
    return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
  }
  ESP_LOGI(TAG, "BLE ctrl: %s", cmd);
  return 0;
}

static int onCtrlAccess(uint16_t, uint16_t, ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    char json[384];
    const size_t n = ctrlJson(json, sizeof(json));
    return n && os_mbuf_append(ctxt->om, json, n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    char cmd[32];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, cmd, sizeof(cmd) - 1, &len) != 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    cmd[len] = '\0';
    return onCtrlWrite(cmd);
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static int onConfigAccess(uint16_t, uint16_t, ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    char json[512];
    const size_t n = statusJson(json, sizeof(json));
    return n && os_mbuf_append(ctxt->om, json, n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

// wifi characteristic. read: one page of the list JSON (no passwords). write: one wifinets text command, or
// "list=<index>" to read the list from that entry on; refused with ATT error 0x80 (invalid), 0x81 (8 networks
// already), 0x82 (no such network to delete).
static int onWifiAccess(uint16_t connHandle, uint16_t, ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    static wifinets::Entry list[wifinets::kMax];
    static char json[512];  // BLE host task only
    const size_t n = wifinets::listJson(list, wifinets::load(list), wifiListStartOf(connHandle), wifiListGen.load(),
                                        json, sizeof(json));
    return n && os_mbuf_append(ctxt->om, json, n) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    char buf[352];  // "ssid=...&pass=...&prio=..." with every byte URL-encoded
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len) != 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    buf[len] = '\0';
    int start;
    uint32_t gen;
    bool hasGen;
    if (wifinets::parseListRequest(buf, &start, &gen, &hasGen)) {
      // A request for a page of an older list: refused (the client starts over); "list=0" always works
      if (hasGen && gen != wifiListGen.load() && start != 0) return 0x83;
      setWifiListStart(connHandle, start);
      return 0;
    }
    bool full = false, notFound = false;
    if (const char* err = applyWifiText(buf, &full, &notFound)) {
      ESP_LOGW(TAG, "Wi-Fi write over BLE rejected: %s", err);
      return full ? 0x81 : notFound ? 0x82 : 0x80;
    }
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static const ble_gatt_chr_def kConfigChrs[] = {
    {.uuid = &kConfigStatus.u,
     .access_cb = onConfigAccess,
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC},
    {.uuid = &kConfigWifi.u,
     .access_cb = onWifiAccess,
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC},
    {.uuid = &kConfigCtrl.u,
     .access_cb = onCtrlAccess,
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC},
    {},
};

static const ble_gatt_svc_def kConfigSvcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &kConfigService.u, .characteristics = kConfigChrs},
    {},
};

static int onConfigGapEvent(ble_gap_event* ev, void*);

static void startConfigAdvertising() {
  if (!configWindowOpen || bleClientsConnected() >= CONFIG_BT_NIMBLE_MAX_CONNECTIONS || ble_gap_adv_active()) {
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
           bleClientsConnected() ? " (clients stay until they disconnect)" : "");
}

static int onConfigGapEvent(ble_gap_event* ev, void*) {
  switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status == 0) {
        bleClientAdd(ev->connect.conn_handle);
        ESP_LOGI(TAG, "Config client connected (%d of %d)", bleClientsConnected(), CONFIG_BT_NIMBLE_MAX_CONNECTIONS);
        // Ask the client to pair right away; macOS does not start pairing on its
        // own when a read fails with "insufficient encryption".
        ble_gap_security_initiate(ev->connect.conn_handle);
      }
      startConfigAdvertising();  // advertising stops on a connection: go on while there is room for another client
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      bleClientRemove(ev->disconnect.conn.conn_handle);
      ESP_LOGI(TAG, "Config client disconnected, reason=0x%x (%d left)", ev->disconnect.reason, bleClientsConnected());
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
  if (CONFIG_LOGBO_CONFIG_WINDOW_S > 0) {
    ESP_ERROR_CHECK(esp_timer_start_once(windowTimer, CONFIG_LOGBO_CONFIG_WINDOW_S * 1000000LL));
  }
}

// ---------------------------------------------------------------------------
// Records: one per GNSS epoch (from the GNSS task), or from the watchdog task at the configured rate while
// the receiver is silent, so the IMU is still logged without GNSS.

// Held while a record is built and sent: the GNSS task and the watchdog may meet when the GNSS resumes.
static SemaphoreHandle_t recordMutex;


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
  ds.wifiConfigured = wifiCount > 0;
  ds.apOn = radioApOn;
  ds.staOn = radioStaOn;
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
    staIpInfo(&ip);
    snprintf(copy.ip, sizeof(copy.ip), IPSTR, IP2STR(&ip.ip));
    char target[33];
    wifiTargetCopy(target);
    snprintf(copy.ssid, sizeof(copy.ssid), "%s", target);
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
  lastFix = p.fixOk() ? p.fixType : 0;
  lastSats = p.numSV;
  emitRecord(&p);
}

static void sendStatus() {
  const gnss::Stats gs = gnss::stats();
  int rssi = 0;
  wifi_ap_record_t ap;
  if (broadcastAddr && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
  esp_netif_ip_info_t ip = {};
  staIpInfo(&ip);

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
      wifiCount > 0 ? "true" : "false", rssi, IP2STR(&ip.ip), static_cast<unsigned long>(esp_get_free_heap_size()));
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
//   wifi-set<TAB><ssid><TAB><password>[<TAB><priority>]   add / update a Wi-Fi network (priority: higher first)
//   wifi-del<TAB><ssid>                  delete a network
//   wifi-list                            the networks (JSON, no passwords)
//   wifi-clear                           delete all networks
//   radio                                the AP / STA switches (JSON); radio ap=0, radio sta=1, ... to change
//                                        them (from the next boot), reboot to restart
//   mag-cal                              start the magnetometer calibration (rotate the unit in every
//                                        direction), mag-cal-done to finish it
//   mag-clear                            forget the magnetometer calibration
//   mag                                  show the calibration and the latest reading

static void handleCommand(char* line) {
  if (strncmp(line, "wifi-set\t", 9) == 0) {
    char* ssid = line + 9;
    char* tab = strchr(ssid, '\t');
    if (!tab || tab == ssid) {
      printf("ERR usage: wifi-set<TAB>ssid<TAB>password[<TAB>priority]\n");
      return;
    }
    // Priority as an optional third field; the rest goes through the "<ssid>\t<password>" format
    char* prioField = strchr(tab + 1, '\t');
    int prio = 0;
    if (prioField) {
      *prioField = '\0';
      prio = atoi(prioField + 1);
    }
    wifinets::Command cmd;
    if (wifinets::parse(ssid, cmd) != wifinets::ParseError::kNone) {
      printf("ERR SSID must be 1-32 bytes, password at most 64\n");
      return;
    }
    cmd.entry.prio = prio;
    const wifinets::StoreError err = wifinets::set(cmd.entry);
    if (err != wifinets::StoreError::kNone) {
      printf("ERR %s\n", err == wifinets::StoreError::kFull ? "already 8 networks; wifi-del one first" : "NVS write failed");
      return;
    }
    wifiListChanged();
    printf("OK wifi saved\n");
  } else if (strncmp(line, "wifi-del\t", 9) == 0) {
    const wifinets::StoreError err = wifinets::remove(line + 9);
    if (err == wifinets::StoreError::kNone) wifiListChanged();
    printf("%s\n", err == wifinets::StoreError::kNone       ? "OK wifi deleted"
                   : err == wifinets::StoreError::kNotFound ? "ERR no such network"
                                                             : "ERR NVS write failed");
  } else if (strcmp(line, "wifi-list") == 0) {
    static wifinets::Entry list[wifinets::kMax];
    static char json[wifinets::kMax * (33 * 6 + 48) + 64];  // one page holds them all; static: ~2 KB
    if (wifinets::listJson(list, wifinets::load(list), 0, wifiListGen.load(), json, sizeof(json))) printf("%s\n", json);
  } else if (strcmp(line, "wifi-clear") == 0) {
    if (wifinets::clear() != wifinets::StoreError::kNone) {
      printf("ERR NVS write failed\n");
    } else {
      wifiListChanged();
      printf("OK wifi cleared\n");
    }
  } else if (strcmp(line, "radio") == 0) {
    char json[96];
    if (radioJson(json, sizeof(json))) printf("%s\n", json);
  } else if (!strncmp(line, "radio ap=", 9) || !strncmp(line, "radio sta=", 10)) {
    const char* cmd = line + 6;
    const char* v = strchr(cmd, '=') + 1;
    if (strcmp(v, "0") && strcmp(v, "1")) {
      printf("ERR usage: radio ap=0|1 / radio sta=0|1\n");
    } else {
      printf(radioSave(cmd[0] == 'a' ? "ap" : "sta", v[0] == '1') ? "OK saved; takes effect after reboot\n"
                                                                  : "ERR NVS write failed\n");
    }
  } else if (strcmp(line, "reboot") == 0) {
    printf("OK rebooting\n");
    rebootSoon();
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
