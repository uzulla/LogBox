#include "sdlog.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "sdmmc_cmd.h"

namespace sdlog {
namespace {

const char* TAG = "sdlog";

// M5Stack (Basic / Gray): the TF slot and the LCD share VSPI (SPI3: SCLK 18, MOSI 23, MISO 19). The card's CS
// is GPIO4, the LCD's GPIO14; the bus is initialised here and the display adds itself as a second device.
constexpr spi_host_device_t kHost = SPI3_HOST;
constexpr gpio_num_t kSdClk = GPIO_NUM_18, kSdMosi = GPIO_NUM_23, kSdMiso = GPIO_NUM_19, kSdCs = GPIO_NUM_4;
constexpr int kBusMaxTransfer = 320 * 48 * 2;  // one LCD band (the display draws the screen in bands)

constexpr const char* kMount = "/sdcard";
constexpr int kQueueLen = 64;  // about 2.5 s at 25 Hz
constexpr int64_t kFlushUs = 1000000;

using Item = record::Record;

std::atomic<State> gState{State::Off};
std::atomic<bool> gStop{false};
QueueHandle_t gQueue;
sdmmc_card_t* gCard;
FILE* gFile;
char gPath[40];
uint32_t gSession;
uint32_t gSeq;
std::atomic<bool> gStarting{false};
uint32_t gDropped;

// boot_log.csv: lines from event(), written by the log task (the only one that touches the card).
constexpr const char* kEventFile = "boot_log.csv";
constexpr const char* kEventHeader = "boot,utc,uptime_s,event,detail\n";
constexpr int kEventMax = 16, kEventLen = 160;
char gEvents[kEventMax][kEventLen];
int gEventHead, gEventCount;
portMUX_TYPE gEventLock = portMUX_INITIALIZER_UNLOCKED;

// Appends the waiting lines to boot_log.csv. Returns false on a write error (the lines stay for later).
bool writeEvents() {
  static char lines[kEventMax][kEventLen];  // the log task only
  taskENTER_CRITICAL(&gEventLock);
  const int n = gEventCount;
  for (int i = 0; i < n; i++) memcpy(lines[i], gEvents[(gEventHead + i) % kEventMax], kEventLen);
  taskEXIT_CRITICAL(&gEventLock);
  if (!n) return true;
  char path[40];
  snprintf(path, sizeof(path), "%s/%s", kMount, kEventFile);
  struct stat st;
  const bool fresh = stat(path, &st) != 0 || st.st_size == 0;
  FILE* f = fopen(path, "a");
  if (!f) return false;
  bool ok = !fresh || fputs(kEventHeader, f) >= 0;
  for (int i = 0; ok && i < n; i++) ok = fputs(lines[i], f) >= 0 && fputc('\n', f) != EOF;
  ok = fflush(f) == 0 && fsync(fileno(f)) == 0 && ok;
  fclose(f);
  if (!ok) return false;
  taskENTER_CRITICAL(&gEventLock);
  const int done = n <= gEventCount ? n : gEventCount;  // event() may have dropped some meanwhile
  gEventHead = (gEventHead + done) % kEventMax;
  gEventCount -= done;
  taskEXIT_CRITICAL(&gEventLock);
  return true;
}

bool initBus() {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = kSdMosi;
  bus.miso_io_num = kSdMiso;
  bus.sclk_io_num = kSdClk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = kBusMaxTransfer;
  return spi_bus_initialize(kHost, &bus, SPI_DMA_CH_AUTO) == ESP_OK;
}

bool mountCard() {
  esp_vfs_fat_mount_config_t mc = {};
  mc.format_if_mount_failed = false;  // never touch a card we cannot read
  mc.max_files = 2;
  mc.allocation_unit_size = 0;
  sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot.gpio_cs = kSdCs;
  slot.host_id = kHost;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = kHost;
  const esp_err_t err = esp_vfs_fat_sdspi_mount(kMount, &host, &slot, &mc, &gCard);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "no usable FAT32 card (%s)", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "card %s, %llu MB", gCard->cid.name,
           static_cast<unsigned long long>(gCard->csd.capacity) * gCard->csd.sector_size / (1024 * 1024));
  return true;
}

void unmountCard() {
  esp_vfs_fat_sdcard_unmount(kMount, gCard);
  gCard = nullptr;
}

// Like gmeter: the session number counts up per boot (kept in NVS) and skips numbers already on the card.
uint32_t pickSession() {
  uint32_t sess = 0;
  nvs_handle_t h;
  if (nvs_open("log", NVS_READWRITE, &h) == ESP_OK) {
    nvs_get_u32(h, "session", &sess);
    char path[48];
    for (int tries = 0; tries < 1000; tries++) {
      sess = sess % 9999 + 1;
      snprintf(path, sizeof(path), "%s/log_%04u_01.csv", kMount, static_cast<unsigned>(sess));
      struct stat st;
      if (stat(path, &st) != 0) break;
    }
    nvs_set_u32(h, "session", sess);
    nvs_commit(h);
    nvs_close(h);
  }
  return sess ? sess : 1;
}

bool openLog() {
  // Like gmeter: the file name is log_<session>_<seq>; seq counts up each time recording is restarted.
  struct stat st;
  do {
    gSeq++;
    snprintf(gPath, sizeof(gPath), "%s/log_%04u_%02u.csv", kMount, static_cast<unsigned>(gSession),
             static_cast<unsigned>(gSeq));
  } while (stat(gPath, &st) == 0 && gSeq < 99);
  gFile = fopen(gPath, "w");
  if (!gFile) return false;
  static char buf[4096];
  setvbuf(gFile, buf, _IOFBF, sizeof(buf));
  char head[768];
  const size_t n = record::csvHeader(head, sizeof(head), gSession, gSeq);
  return n && fwrite(head, 1, n, gFile) == n;
}

// Returns false on a write error (disk full, card removed, ...).
bool writeRow(const Item& it, int64_t startUs) {
  char row[320];
  const size_t n = record::csvRow(it, startUs, row, sizeof(row));
  return n && fwrite(row, 1, n, gFile) == n;
}

bool flushFile() { return fflush(gFile) == 0 && fsync(fileno(gFile)) == 0; }

void closeAndUnmount() {
  if (gFile) {
    fclose(gFile);  // errors here were already counted by the last flush
    gFile = nullptr;
  }
  unmountCard();
}

void logTask(void*) {
  const int64_t startUs = esp_timer_get_time();
  int64_t flushedAt = startUs;
  bool failed = false;
  Item it;
  while (!gStop) {
    if (xQueueReceive(gQueue, &it, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (!writeRow(it, startUs)) {
        ESP_LOGE(TAG, "write failed (errno %d), recording stopped", errno);
        failed = true;
        break;
      }
    }
    const int64_t now = esp_timer_get_time();
    if (now - flushedAt >= kFlushUs) {
      flushedAt = now;
      if (!writeEvents()) ESP_LOGW(TAG, "cannot write %s (errno %d)", kEventFile, errno);
      if (!flushFile()) {
        ESP_LOGE(TAG, "flush failed (errno %d), recording stopped", errno);
        failed = true;
        break;
      }
    }
  }
  if (failed) {
    gState = State::Failed;
  } else {
    while (xQueueReceive(gQueue, &it, 0) == pdTRUE) {  // what was queued before the stop
      if (!writeRow(it, startUs)) break;
    }
    flushFile();
    writeEvents();
  }
  closeAndUnmount();
  if (!failed) gState = State::Safe;
  ESP_LOGI(TAG, "%s %s (%lu dropped)", failed ? "failed:" : "saved", gPath, static_cast<unsigned long>(gDropped));
  vTaskDelete(nullptr);
}

// Creates the next log file on the mounted card and starts the writer task.
bool startLogging() {
  if (!openLog()) {
    ESP_LOGE(TAG, "cannot create %s", gPath);
    closeAndUnmount();
    gState = State::Failed;
    return false;
  }
  gStop = false;
  Item stale;
  while (xQueueReceive(gQueue, &stale, 0) == pdTRUE) {}
  gState = State::Rec;
  xTaskCreate(logTask, "sdlog", 6144, nullptr, 4, nullptr);
  ESP_LOGI(TAG, "recording to %s", gPath);
  return true;
}

// Mounting takes a while, so restarting runs in its own task (the caller is the LCD task).
void restartTask(void*) {
  if (mountCard()) {
    if (!gSession) gSession = pickSession();
    startLogging();
  } else {
    gState = State::Failed;
  }
  gStarting = false;
  vTaskDelete(nullptr);
}

}  // namespace

void init() {
  if (!initBus()) return;
  gQueue = xQueueCreate(kQueueLen, sizeof(Item));
  if (!mountCard()) return;  // no card now: a long press tries again later
  gSession = pickSession();
  startLogging();
}

void write(const record::Record& r) {
  if (gState != State::Rec) return;
  if (xQueueSend(gQueue, &r, 0) != pdTRUE) gDropped++;
}

void requestStop() {
  State rec = State::Rec;
  if (gState.compare_exchange_strong(rec, State::Saving)) gStop = true;
}

void requestStart() {
  const State st = gState;
  if (st == State::Rec || st == State::Saving || !gQueue) return;
  bool idle = false;
  if (gStarting.compare_exchange_strong(idle, true)) {
    xTaskCreate(restartTask, "sdlog_start", 4096, nullptr, 4, nullptr);
  }
}

State state() { return gState; }

void event(const char* line) {
  taskENTER_CRITICAL(&gEventLock);
  if (gEventCount == kEventMax) {  // full: drop the oldest
    gEventHead = (gEventHead + 1) % kEventMax;
    gEventCount--;
  }
  char* slot = gEvents[(gEventHead + gEventCount) % kEventMax];
  strlcpy(slot, line, kEventLen);
  gEventCount++;
  taskEXIT_CRITICAL(&gEventLock);
}

const char* fileName() {
  const char* slash = strrchr(gPath, '/');
  return slash ? slash + 1 : gPath;
}

}  // namespace sdlog
