// NVS side of wifinets (not built for the host tests).
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "wifinets.h"

namespace wifinets {
namespace {

constexpr char kNs[] = "wifi";

void key(char* k, const char* name, int i) { snprintf(k, 12, "%s%d", name, i); }

// Every store operation runs under this mutex (created on first use).
class Lock {
 public:
  Lock() {
    static StaticSemaphore_t storage;
    static SemaphoreHandle_t mutex;
    static portMUX_TYPE init = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&init);
    if (!mutex) mutex = xSemaphoreCreateMutexStatic(&storage);
    taskEXIT_CRITICAL(&init);
    mutex_ = mutex;
    xSemaphoreTake(mutex_, portMAX_DELAY);
  }
  ~Lock() { xSemaphoreGive(mutex_); }

 private:
  SemaphoreHandle_t mutex_;
};

// Older firmware kept one network as "ssid" / "pass": copy it into slot 0 (priority 0) and only then delete the old
// keys, in one commit. If anything fails the old keys stay, and the next call tries again (slot 0 written by an
// interrupted earlier attempt is simply written again).
void migrate(nvs_handle_t h) {
  char ssid[33];
  size_t len = sizeof(ssid);
  if (nvs_get_str(h, "ssid", ssid, &len) != ESP_OK) return;
  char pass[65] = "";
  len = sizeof(pass);
  const esp_err_t passErr = nvs_get_str(h, "pass", pass, &len);
  if (passErr != ESP_OK && passErr != ESP_ERR_NVS_NOT_FOUND) return;
  if (ssid[0]) {
    char k[12], existing[33];
    key(k, "ssid", 0);
    len = sizeof(existing);
    const esp_err_t slotErr = nvs_get_str(h, k, existing, &len);
    // Slot 0 free, or already holding this network (an earlier attempt): (re)write it
    if (slotErr == ESP_ERR_NVS_NOT_FOUND || (slotErr == ESP_OK && (!existing[0] || !strcmp(existing, ssid)))) {
      esp_err_t err = nvs_set_str(h, k, ssid);
      key(k, "pass", 0);
      if (err == ESP_OK) err = nvs_set_str(h, k, pass);
      key(k, "prio", 0);
      if (err == ESP_OK) err = nvs_set_i32(h, k, 0);
      if (err != ESP_OK) return;  // keep the old keys
    } else if (slotErr != ESP_OK) {
      return;
    }
    // else slot 0 holds another network (new firmware already in use): the old pair is just dropped
  }
  esp_err_t err = nvs_erase_key(h, "ssid");
  if (err == ESP_OK) {
    const esp_err_t e2 = nvs_erase_key(h, "pass");
    if (e2 != ESP_OK && e2 != ESP_ERR_NVS_NOT_FOUND) err = e2;
  }
  if (err == ESP_OK) nvs_commit(h);
}

}  // namespace

int load(Entry* out) {
  Lock lock;
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

int count() {
  Lock lock;
  nvs_handle_t h;
  if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return 0;
  migrate(h);
  int n = 0;
  for (int i = 0; i < kMax; i++) {
    char k[12], v[33];
    size_t len = sizeof(v);
    key(k, "ssid", i);
    if (nvs_get_str(h, k, v, &len) == ESP_OK && v[0]) n++;
  }
  nvs_close(h);
  return n;
}

StoreError set(const Entry& e) {
  Lock lock;
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
  Lock lock;
  nvs_handle_t h;
  if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return StoreError::kNvs;
  migrate(h);
  StoreError result = StoreError::kNotFound;
  for (int i = 0; i < kMax; i++) {
    char k[12], v[33];
    size_t len = sizeof(v);
    key(k, "ssid", i);
    if (nvs_get_str(h, k, v, &len) == ESP_OK && !strcmp(v, ssid)) {
      // The ssid key is what makes the slot used: its erase must succeed. pass / prio may be missing.
      esp_err_t err = nvs_erase_key(h, k);
      key(k, "pass", i);
      nvs_erase_key(h, k);
      key(k, "prio", i);
      nvs_erase_key(h, k);
      if (err == ESP_OK) err = nvs_commit(h);
      result = err == ESP_OK ? StoreError::kNone : StoreError::kNvs;
      break;
    }
  }
  nvs_close(h);
  return result;
}

StoreError clear() {
  Lock lock;
  nvs_handle_t h;
  if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return StoreError::kNvs;
  const bool ok = nvs_erase_all(h) == ESP_OK && nvs_commit(h) == ESP_OK;
  nvs_close(h);
  return ok ? StoreError::kNone : StoreError::kNvs;
}

}  // namespace wifinets
