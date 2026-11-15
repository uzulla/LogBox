// NVS side of wifinets (not built for the host tests).
#include <stdio.h>
#include <string.h>

#include "nvs.h"
#include "wifinets.h"

namespace wifinets {
namespace {

constexpr char kNs[] = "wifi";

void key(char* k, const char* name, int i) { snprintf(k, 12, "%s%d", name, i); }

// Older firmware kept one network as "ssid" / "pass": move it into slot 0 (priority 0).
void migrate(nvs_handle_t h) {
  char ssid[33];
  size_t len = sizeof(ssid);
  if (nvs_get_str(h, "ssid", ssid, &len) != ESP_OK) return;
  char pass[65] = "";
  len = sizeof(pass);
  nvs_get_str(h, "pass", pass, &len);
  char k[12];
  key(k, "ssid", 0);
  char existing[33];
  len = sizeof(existing);
  if (ssid[0] && nvs_get_str(h, k, existing, &len) != ESP_OK) {
    nvs_set_str(h, k, ssid);
    key(k, "pass", 0);
    nvs_set_str(h, k, pass);
    key(k, "prio", 0);
    nvs_set_i32(h, k, 0);
  }
  nvs_erase_key(h, "ssid");
  nvs_erase_key(h, "pass");
  nvs_commit(h);
}

}  // namespace

int load(Entry* out) {
  nvs_handle_t h;
  if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return 0;
  migrate(h);
  int n = 0;
  for (int i = 0; i < kMax; i++) {
    Entry& e = out[n];
    e = {};
    char k[12];
    size_t len = sizeof(e.ssid);
    key(k, "ssid", i);
    if (nvs_get_str(h, k, e.ssid, &len) != ESP_OK || !e.ssid[0]) continue;
    len = sizeof(e.pass);
    key(k, "pass", i);
    if (nvs_get_str(h, k, e.pass, &len) != ESP_OK) e.pass[0] = '\0';
    key(k, "prio", i);
    nvs_get_i32(h, k, &e.prio);
    n++;
  }
  nvs_close(h);
  return n;
}

StoreError set(const Entry& e) {
  nvs_handle_t h;
  if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return StoreError::kNvs;
  migrate(h);
  int slot = -1, freeSlot = -1;
  for (int i = 0; i < kMax; i++) {
    char k[12], v[33];
    size_t len = sizeof(v);
    key(k, "ssid", i);
    if (nvs_get_str(h, k, v, &len) != ESP_OK || !v[0]) {
      if (freeSlot < 0) freeSlot = i;
    } else if (!strcmp(v, e.ssid)) {
      slot = i;
    }
  }
  if (slot < 0) slot = freeSlot;
  if (slot < 0) {
    nvs_close(h);
    return StoreError::kFull;
  }
  char k[12];
  key(k, "ssid", slot);
  esp_err_t err = nvs_set_str(h, k, e.ssid);
  key(k, "pass", slot);
  if (err == ESP_OK) err = nvs_set_str(h, k, e.pass);
  key(k, "prio", slot);
  if (err == ESP_OK) err = nvs_set_i32(h, k, e.prio);
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  return err == ESP_OK ? StoreError::kNone : StoreError::kNvs;
}

StoreError remove(const char* ssid) {
  nvs_handle_t h;
  if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return StoreError::kNvs;
  migrate(h);
  StoreError result = StoreError::kNotFound;
  for (int i = 0; i < kMax; i++) {
    char k[12], v[33];
    size_t len = sizeof(v);
    key(k, "ssid", i);
    if (nvs_get_str(h, k, v, &len) == ESP_OK && !strcmp(v, ssid)) {
      nvs_erase_key(h, k);
      key(k, "pass", i);
      nvs_erase_key(h, k);
      key(k, "prio", i);
      nvs_erase_key(h, k);
      result = nvs_commit(h) == ESP_OK ? StoreError::kNone : StoreError::kNvs;
      break;
    }
  }
  nvs_close(h);
  return result;
}

StoreError clear() {
  nvs_handle_t h;
  if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return StoreError::kNvs;
  const bool ok = nvs_erase_all(h) == ESP_OK && nvs_commit(h) == ESP_OK;
  nvs_close(h);
  return ok ? StoreError::kNone : StoreError::kNvs;
}

}  // namespace wifinets
