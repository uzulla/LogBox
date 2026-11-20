#include "sdlog.h"

#include <errno.h>
#include <fcntl.h>
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
// A new file every 5 minutes: a power cut or a card error can then only damage the file being written.
constexpr int64_t kPartUs = 5LL * 60 * 1000000;
constexpr unsigned kMaxPart = 999;

using Item = record::Record;

std::atomic<State> gState{State::Off};
std::atomic<bool> gStop{false};
QueueHandle_t gQueue;
sdmmc_card_t* gCard;
FILE* gFile;
// The current path, double buffered: fileName() is read by other tasks while the log task names the next
// part, so the new name goes to the other buffer and is published by switching gPathIdx.
char gPaths[2][40];
std::atomic<int> gPathIdx{0};
uint32_t gSession;
uint32_t gSeq;
uint32_t gPart;
std::atomic<bool> gStarting{false};
uint32_t gDropped;

// boot_log.csv: lines from event(), written by the log task (the only one that touches the card).
constexpr const char* kEventFile = "boot_log.csv";
constexpr const char* kEventHeader = "boot,utc,uptime_s,event,detail\n";
constexpr int kEventMax = 16, kEventLen = 160;
char gEvents[kEventMax][kEventLen];
uint32_t gEventFirst, gEventCount;  // sequence number of the oldest waiting line (slot: seq % kEventMax), count
portMUX_TYPE gEventLock = portMUX_INITIALIZER_UNLOCKED;

// Appends the waiting lines to boot_log.csv. Returns false on a write error (the lines stay for later).
bool writeEvents() {
  static char lines[kEventMax][kEventLen];  // the log task only
  taskENTER_CRITICAL(&gEventLock);
  const uint32_t first = gEventFirst;
  const int n = static_cast<int>(gEventCount);
  for (int i = 0; i < n; i++) memcpy(lines[i], gEvents[(first + i) % kEventMax], kEventLen);
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
  // Remove exactly the lines written (sequence numbers first .. first + n - 1); event() may have dropped some
  // of them (queue full) and added new ones meanwhile.
  taskENTER_CRITICAL(&gEventLock);
  const uint32_t end = first + n;
  if (static_cast<int32_t>(end - gEventFirst) > 0) {
    const uint32_t done = end - gEventFirst;
    gEventFirst = end;
    gEventCount -= done;
  }
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
      snprintf(path, sizeof(path), "%s/log_%04u_01_001.csv", kMount, static_cast<unsigned>(sess));
      struct stat st;
      if (stat(path, &st) != 0) break;
    }
    nvs_set_u32(h, "session", sess);
    nvs_commit(h);
    nvs_close(h);
  }
  return sess ? sess : 1;
}

const char* curPath() { return gPaths[gPathIdx.load()]; }

// Opens log_<session>_<seq>_<part>.csv (part counts the 5-minute files of one recording) and writes the header.
bool openPart(unsigned part) {
  char* path = gPaths[gPathIdx.load() ^ 1];
  snprintf(path, sizeof(gPaths[0]), "%s/log_%04u_%02u_%03u.csv", kMount, static_cast<unsigned>(gSession),
           static_cast<unsigned>(gSeq), part);
  gPart = part;
  // Never over an existing file (O_EXCL): e.g. after NVS was erased, a session and seq whose _001 is gone may
  // still have later parts on the card. errno EEXIST then.
  const int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0666);
  gFile = fd >= 0 ? fdopen(fd, "w") : nullptr;
  if (!gFile) {
    const int e = errno;
    if (fd >= 0) close(fd);
    ESP_LOGE(TAG, "cannot create %s (errno %d)", path, e);
    errno = e;
    return false;  // fileName() keeps the last file that was really written
  }
  gPathIdx.store(gPathIdx.load() ^ 1);
  static char buf[4096];
  setvbuf(gFile, buf, _IOFBF, sizeof(buf));
  static char head[record::kMaxCsvHeader];  // the log task / the start task, never both at once
  const size_t n = record::csvHeader(head, sizeof(head), gSession, gSeq, part);
  if (!n) {
    ESP_LOGE(TAG, "CSV header does not fit");
    errno = EINVAL;
    return false;
  }
  return fwrite(head, 1, n, gFile) == n;
}

bool openLog() {
  // Like gmeter: the file name is log_<session>_<seq>; seq counts up each time recording is restarted.
  char path[40];
  struct stat st;
  // seq is two digits: past 99 nothing is overwritten, recording stops (the next boot has a new session).
  for (;;) {
    if (gSeq >= 99) {
      ESP_LOGE(TAG, "no free seq left in session %u", static_cast<unsigned>(gSession));
      return false;
    }
    gSeq++;
    snprintf(path, sizeof(path), "%s/log_%04u_%02u_001.csv", kMount, static_cast<unsigned>(gSession),
             static_cast<unsigned>(gSeq));
    if (stat(path, &st) == 0) continue;
    if (openPart(1)) return true;
    if (errno != EEXIST) return false;
  }
}

// Closes the current part and continues in the next one. t_ms keeps counting from the start of the recording.
bool nextPart() {
  const bool closed = fclose(gFile) == 0;
  gFile = nullptr;
  if (!closed) return false;
  if (gPart >= kMaxPart) {
    // Out of part numbers: continue in a new recording (seq) rather than stop.
    return openLog();
  }
  if (openPart(gPart + 1)) return true;
  // That part already exists (left from an earlier recording with the same name): continue in a new seq.
  return errno == EEXIST && openLog();
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
  int64_t flushedAt = startUs, partAt = startUs;
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
      if (now - partAt >= kPartUs) {
        partAt = now;
        if (!nextPart()) {
          ESP_LOGE(TAG, "cannot start the next part after %s, recording stopped", curPath());
          failed = true;
          break;
        }
        ESP_LOGI(TAG, "recording to %s", curPath());
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
  ESP_LOGI(TAG, "%s %s (%lu dropped)", failed ? "failed:" : "saved", curPath(), static_cast<unsigned long>(gDropped));
  vTaskDelete(nullptr);
}

// Creates the next log file on the mounted card and starts the writer task.
bool startLogging() {
  if (!openLog()) {
    ESP_LOGE(TAG, "cannot start recording");
    closeAndUnmount();
    gState = State::Failed;
    return false;
  }
  gStop = false;
  Item stale;
  while (xQueueReceive(gQueue, &stale, 0) == pdTRUE) {}
  gState = State::Rec;
  xTaskCreate(logTask, "sdlog", 6144, nullptr, 4, nullptr);
  ESP_LOGI(TAG, "recording to %s", curPath());
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
    gEventFirst++;
    gEventCount--;
  }
  char* slot = gEvents[(gEventFirst + gEventCount) % kEventMax];
  strlcpy(slot, line, kEventLen);
  gEventCount++;
  taskEXIT_CRITICAL(&gEventLock);
}

const char* fileName() {
  const char* path = curPath();
  const char* slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

}  // namespace sdlog
