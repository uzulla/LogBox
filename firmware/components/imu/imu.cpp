#include "imu.h"

#include <string.h>

#include <algorithm>

#include "bmi270_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

namespace imu {
namespace {

const char* TAG = "imu";

constexpr i2c_port_num_t kPort = I2C_NUM_0;
constexpr gpio_num_t kSda = GPIO_NUM_21, kScl = GPIO_NUM_22;
constexpr uint32_t kSclHz = 400000;
constexpr int kTimeoutMs = 100;  // a full 2 KB FIFO read takes about 50 ms at 400 kHz

constexpr uint8_t kBmiAddr = 0x69, kBmpAddr = 0x76;
constexpr uint8_t kBmmAuxAddr = 0x10;  // on the BMI270's auxiliary bus, not on ours

constexpr int kTaskPrio = 5;  // below the GNSS UART task
constexpr int kTaskStack = 4096;
constexpr TickType_t kRetryPeriod = pdMS_TO_TICKS(5000);  // looking for missing sensors again
constexpr TickType_t kPeriod = pdMS_TO_TICKS(20);  // FIFO every 20 ms (4 samples), mag/baro every 40 ms

// BMI270 registers
namespace bmi {
constexpr uint8_t CHIP_ID = 0x00, ERR_REG = 0x02, STATUS = 0x03, AUX_DATA_0 = 0x04;
constexpr uint8_t INTERNAL_STATUS = 0x21, FIFO_LENGTH_0 = 0x24, FIFO_DATA = 0x26;
constexpr uint8_t ACC_CONF = 0x40, ACC_RANGE = 0x41, GYR_CONF = 0x42, GYR_RANGE = 0x43, AUX_CONF = 0x44;
constexpr uint8_t FIFO_DOWNS = 0x45, FIFO_CONFIG_0 = 0x48, FIFO_CONFIG_1 = 0x49;
constexpr uint8_t AUX_DEV_ID = 0x4B, AUX_IF_CONF = 0x4C, AUX_RD_ADDR = 0x4D, AUX_WR_ADDR = 0x4E, AUX_WR_DATA = 0x4F;
constexpr uint8_t INIT_CTRL = 0x59, INIT_ADDR_0 = 0x5B, INIT_DATA = 0x5E, AUX_IF_TRIM = 0x68, IF_CONF = 0x6B;
constexpr uint8_t PWR_CONF = 0x7C, PWR_CTRL = 0x7D, CMD = 0x7E;

constexpr uint8_t kChipId = 0x24;
constexpr uint8_t kStatusAuxBusy = 0x04;
constexpr uint8_t kErrAux = 0x80;
// ACC_CONF: performance filter, normal bandwidth, 200 Hz. GYR_CONF: same plus high-performance noise.
constexpr uint8_t kAccConf = 0x80 | (0x02 << 4) | 0x09;
constexpr uint8_t kGyrConf = 0x80 | 0x40 | (0x02 << 4) | 0x09;
constexpr uint8_t kAccRange8g = 0x02, kGyrRange2000 = 0x00;
constexpr float kAccLsbPerG = 32768.0f / 8;
constexpr float kGyrLsbPerDps = 32768.0f / 2000;
// FIFO: header mode with accel and gyro (frames are header 0x8C, gyro xyz, accel xyz); aux not in FIFO.
constexpr uint8_t kFifoConfig1 = 0x80 | 0x40 | 0x10;
constexpr uint8_t kFifoDowns = 0x88;  // filtered data, no downsampling
constexpr uint8_t kPwrAux = 0x01, kPwrGyr = 0x02, kPwrAcc = 0x04, kPwrTemp = 0x08;
constexpr uint8_t kIfConfAuxEn = 0x20;
constexpr uint8_t kAuxPullup2k = 0x03;  // as M5Stack's driver for this module
// AUX_IF_CONF: manual mode, FCU write (send AUX_WR_DATA to AUX_WR_ADDR before every data mode read),
// 8 byte bursts in both modes.
constexpr uint8_t kAuxManual = 0x80, kAuxFcuWrite = 0x40, kAuxManBurst8 = 0x0C, kAuxBurst8 = 0x03;
constexpr uint8_t kAuxOdr25Hz = 0x06;
constexpr uint8_t kCmdSoftReset = 0xB6, kCmdFifoFlush = 0xB0;
constexpr size_t kFifoSize = 2048;
}  // namespace bmi

// BMM150 registers (on the aux bus)
namespace bmm {
constexpr uint8_t CHIP_ID = 0x40, DATA_X_LSB = 0x42, POWER = 0x4B, OP_MODE = 0x4C, REP_XY = 0x51, REP_Z = 0x52;
constexpr uint8_t DIG_X1 = 0x5D, DIG_Z4_LSB = 0x62, DIG_Z2_LSB = 0x68;
constexpr uint8_t kChipId = 0x32;
constexpr uint8_t kRepXyEnhanced = 0x07, kRepZEnhanced = 0x0D;  // Bosch's enhanced preset
constexpr uint8_t kForced = 0x02;
constexpr int16_t kOverflowXy = -4096, kOverflowZ = -16384;
}  // namespace bmm

// BMP280 registers
namespace bmp {
constexpr uint8_t CALIB = 0x88, ID = 0xD0, RESET = 0xE0, STATUS = 0xF3, CTRL_MEAS = 0xF4, CONFIG = 0xF5, DATA = 0xF7;
constexpr uint8_t kId = 0x58;
// CONFIG: standby 0.5 ms, IIR coefficient 4. CTRL_MEAS: temperature x2, pressure x16, normal mode.
constexpr uint8_t kConfig = (0 << 5) | (2 << 2);
constexpr uint8_t kCtrlMeas = (2 << 5) | (5 << 2) | 3;
constexpr int32_t kNoData = 0x80000;  // ADC value before the first conversion
}  // namespace bmp

struct BmmTrim {
  int8_t x1, y1, x2, y2, xy2;
  uint8_t xy1;
  uint16_t z1, xyz1;
  int16_t z2, z3, z4;
};

struct BmpCalib {
  uint16_t t1;
  int16_t t2, t3;
  uint16_t p1;
  int16_t p2, p3, p4, p5, p6, p7, p8, p9;
};

i2c_master_bus_handle_t gBus;
i2c_master_dev_handle_t gBmi, gBmp;
Status gStatus;       // written by init() and then by the sampling task only; read by others under gLock
bool gQuiet;          // detecting again in the background: no "not found" warnings every few seconds
uint32_t gBusResets;  // times the bus was freed (see recoverBus())
BmmTrim gTrim;
BmpCalib gCal;
uint8_t gFifo[bmi::kFifoSize];

// Shared with takeAverage() / latest()
portMUX_TYPE gLock = portMUX_INITIALIZER_UNLOCKED;
int64_t gSumA[3], gSumG[3];
uint32_t gSumN;
int16_t gLastA[3], gLastG[3];
bool gHaveAg;
float gMag[3];
bool gMagValid;
bool gMagCalApplied;  // gMag is calibrated
// Magnetometer calibration. gCalib / gHaveCalib are read by readMag() under gLock. The collection (gFit,
// gCollecting) and every start / finish / cancel / clear are serialised by gCalMutex instead: the fit's
// double-precision sums are software floating point on the ESP32, too slow for a critical section, and a finish
// from the display must not interleave with one from the serial console.
MagCal gCalib;
bool gHaveCalib;
SemaphoreHandle_t gCalMutex;
bool gCollecting;
uint32_t gCalSession;  // counts the starts, so the display can tell a restarted collection from the old one
MagCalFit gFit;
float gPressure, gTemp;
bool gBaroValid;

// Short waits in init: the FreeRTOS tick may be 10 ms.
void waitMs(uint32_t ms) {
  if (ms < 10) {
    esp_rom_delay_us(ms * 1000);
  } else {
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
  }
}

bool readRegs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* buf, size_t len) {
  return i2c_master_transmit_receive(dev, &reg, 1, buf, len, kTimeoutMs) == ESP_OK;
}

bool writeReg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
  const uint8_t b[2] = {reg, val};
  return i2c_master_transmit(dev, b, sizeof(b), kTimeoutMs) == ESP_OK;
}

// Registers the device once; a later call (detecting it again) only probes it.
bool addDevice(uint8_t addr, i2c_master_dev_handle_t* dev) {
  if (i2c_master_probe(gBus, addr, kTimeoutMs) != ESP_OK) return false;
  if (*dev) return true;
  i2c_device_config_t cfg = {};
  cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  cfg.device_address = addr;
  cfg.scl_speed_hz = kSclHz;
  return i2c_master_bus_add_device(gBus, &cfg, dev) == ESP_OK;
}

// ---- BMI270 ----

bool uploadConfig() {
  constexpr size_t kChunk = 256;  // must be even: INIT_ADDR counts 16-bit words
  for (size_t at = 0; at < kBmi270ConfigSize; at += kChunk) {
    const size_t words = at / 2;
    const uint8_t addr[3] = {bmi::INIT_ADDR_0, static_cast<uint8_t>(words & 0x0F), static_cast<uint8_t>(words >> 4)};
    if (i2c_master_transmit(gBmi, addr, sizeof(addr), kTimeoutMs) != ESP_OK) return false;
    const uint8_t reg = bmi::INIT_DATA;
    i2c_master_transmit_multi_buffer_info_t bufs[2] = {
        {&reg, 1}, {kBmi270Config + at, std::min(kChunk, kBmi270ConfigSize - at)}};
    if (i2c_master_multi_buffer_transmit(gBmi, bufs, 2, kTimeoutMs) != ESP_OK) return false;
  }
  return true;
}

bool initBmi270() {
  if (!addDevice(kBmiAddr, &gBmi)) {
    if (!gQuiet) ESP_LOGW(TAG, "BMI270 not found");
    return false;
  }
  uint8_t id = 0;
  if (!readRegs(gBmi, bmi::CHIP_ID, &id, 1) || id != bmi::kChipId) {
    ESP_LOGW(TAG, "BMI270: unexpected chip id 0x%02x", id);
    return false;
  }
  writeReg(gBmi, bmi::CMD, bmi::kCmdSoftReset);
  waitMs(3);
  // Upload the config file with advanced power save off, then wait for the sensor to accept it.
  uint8_t st = 0;
  if (!writeReg(gBmi, bmi::PWR_CONF, 0x00)) return false;
  waitMs(1);
  if (!writeReg(gBmi, bmi::INIT_CTRL, 0x00) || !uploadConfig() || !writeReg(gBmi, bmi::INIT_CTRL, 0x01)) {
    ESP_LOGW(TAG, "BMI270: config upload failed");
    return false;
  }
  for (int i = 0; i < 20; i++) {
    waitMs(10);
    if (readRegs(gBmi, bmi::INTERNAL_STATUS, &st, 1) && (st & 0x0F) == 0x01) break;
  }
  if ((st & 0x0F) != 0x01) {
    ESP_LOGW(TAG, "BMI270: config not accepted (internal status 0x%02x)", st);
    return false;
  }
  const uint8_t regs[][2] = {
      {bmi::ACC_CONF, bmi::kAccConf},       {bmi::ACC_RANGE, bmi::kAccRange8g},
      {bmi::GYR_CONF, bmi::kGyrConf},       {bmi::GYR_RANGE, bmi::kGyrRange2000},
      {bmi::FIFO_DOWNS, bmi::kFifoDowns},   {bmi::FIFO_CONFIG_0, 0x00},  // stream mode, no sensortime frames
      {bmi::FIFO_CONFIG_1, bmi::kFifoConfig1},
  };
  for (const auto& r : regs) {
    if (!writeReg(gBmi, r[0], r[1])) return false;
  }
  return true;
}

// Starts accel, gyro (and the aux interface if the BMM150 is set up) and empties the FIFO.
bool startBmi270(bool aux) {
  const uint8_t pwr = bmi::kPwrAcc | bmi::kPwrGyr | bmi::kPwrTemp | (aux ? bmi::kPwrAux : 0);
  if (!writeReg(gBmi, bmi::PWR_CTRL, pwr)) return false;
  waitMs(60);  // gyro start-up
  uint8_t err = 0;
  readRegs(gBmi, bmi::ERR_REG, &err, 1);
  if (err & 0x1E) ESP_LOGW(TAG, "BMI270: error register 0x%02x", err);
  return writeReg(gBmi, bmi::CMD, bmi::kCmdFifoFlush);
}

// ---- BMM150 through the BMI270's aux interface (manual mode) ----

bool auxWaitIdle() {
  for (int i = 0; i < 50; i++) {
    uint8_t st;
    if (readRegs(gBmi, bmi::STATUS, &st, 1) && !(st & bmi::kStatusAuxBusy)) return true;
    esp_rom_delay_us(500);
  }
  return false;
}

bool auxWrite(uint8_t reg, uint8_t val) {
  if (!auxWaitIdle() || !writeReg(gBmi, bmi::AUX_WR_DATA, val) || !writeReg(gBmi, bmi::AUX_WR_ADDR, reg)) return false;
  esp_rom_delay_us(1000);
  return auxWaitIdle();
}

// Reads up to 8 bytes (the manual burst length).
bool auxRead(uint8_t reg, uint8_t* buf, size_t len) {
  if (!auxWaitIdle() || !writeReg(gBmi, bmi::AUX_RD_ADDR, reg)) return false;
  esp_rom_delay_us(1000);
  return auxWaitIdle() && readRegs(gBmi, bmi::AUX_DATA_0, buf, len);
}

bool readBmmTrim() {
  uint8_t x1y1[2], xyz[4], z2z3[10];
  if (!auxRead(bmm::DIG_X1, x1y1, 2) || !auxRead(bmm::DIG_Z4_LSB, xyz, 4) || !auxRead(bmm::DIG_Z2_LSB, z2z3, 8) ||
      !auxRead(bmm::DIG_Z2_LSB + 8, z2z3 + 8, 2)) {
    return false;
  }
  auto u16 = [](uint8_t lsb, uint8_t msb) { return static_cast<uint16_t>(lsb | (msb << 8)); };
  gTrim.x1 = static_cast<int8_t>(x1y1[0]);
  gTrim.y1 = static_cast<int8_t>(x1y1[1]);
  gTrim.z4 = static_cast<int16_t>(u16(xyz[0], xyz[1]));
  gTrim.x2 = static_cast<int8_t>(xyz[2]);
  gTrim.y2 = static_cast<int8_t>(xyz[3]);
  gTrim.z2 = static_cast<int16_t>(u16(z2z3[0], z2z3[1]));
  gTrim.z1 = u16(z2z3[2], z2z3[3]);
  gTrim.xyz1 = u16(z2z3[4], z2z3[5] & 0x7F);
  gTrim.z3 = static_cast<int16_t>(u16(z2z3[6], z2z3[7]));
  gTrim.xy2 = static_cast<int8_t>(z2z3[8]);
  gTrim.xy1 = z2z3[9];
  return true;
}

bool initBmm150() {
  writeReg(gBmi, bmi::AUX_IF_TRIM, bmi::kAuxPullup2k);
  uint8_t ifConf = 0;
  if (!readRegs(gBmi, bmi::IF_CONF, &ifConf, 1) || !writeReg(gBmi, bmi::IF_CONF, ifConf | bmi::kIfConfAuxEn)) return false;
  if (!auxWaitIdle()) return false;
  const uint8_t manual[3] = {bmi::AUX_DEV_ID, kBmmAuxAddr << 1,
                             bmi::kAuxManual | bmi::kAuxFcuWrite | bmi::kAuxManBurst8 | bmi::kAuxBurst8};
  if (i2c_master_transmit(gBmi, manual, sizeof(manual), kTimeoutMs) != ESP_OK) return false;
  esp_rom_delay_us(1000);

  // Suspend -> sleep, then check the chip id.
  if (!auxWrite(bmm::POWER, 0x01)) return false;
  waitMs(3);
  uint8_t id[8] = {};
  if (!auxRead(bmm::CHIP_ID, id, 1) || id[0] != bmm::kChipId) {
    ESP_LOGW(TAG, "BMM150 not found on the aux bus (id 0x%02x)", id[0]);
    return false;
  }
  if (!readBmmTrim()) return false;
  // The last manual write is what the BMI270 sends before every data mode read (FCU write): a forced
  // measurement trigger. So the op mode write must come last.
  if (!auxWrite(bmm::REP_XY, bmm::kRepXyEnhanced) || !auxWrite(bmm::REP_Z, bmm::kRepZEnhanced) ||
      !auxWrite(bmm::OP_MODE, bmm::kForced)) {
    return false;
  }
  // Data mode: read 8 bytes from DATA_X_LSB into AUX_DATA_0..7 at 25 Hz.
  if (!auxWaitIdle()) return false;
  const uint8_t data[2] = {bmi::AUX_IF_CONF, bmi::kAuxFcuWrite | bmi::kAuxManBurst8 | bmi::kAuxBurst8};
  if (i2c_master_transmit(gBmi, data, sizeof(data), kTimeoutMs) != ESP_OK) return false;
  esp_rom_delay_us(1000);
  return writeReg(gBmi, bmi::AUX_RD_ADDR, bmm::DATA_X_LSB) && writeReg(gBmi, bmi::AUX_CONF, bmi::kAuxOdr25Hz);
}

// Bosch's floating point compensation (BMM150 SensorAPI). Returns false on overflow.
bool compensateMag(const uint8_t* d, float* out) {
  const int16_t x = static_cast<int16_t>(static_cast<int8_t>(d[1]) * 32 | (d[0] >> 3));
  const int16_t y = static_cast<int16_t>(static_cast<int8_t>(d[3]) * 32 | (d[2] >> 3));
  const int16_t z = static_cast<int16_t>(static_cast<int8_t>(d[5]) * 128 | (d[4] >> 1));
  const uint16_t rhall = static_cast<uint16_t>((d[7] << 6) | (d[6] >> 2));
  const BmmTrim& t = gTrim;
  if (x == bmm::kOverflowXy || y == bmm::kOverflowXy || z == bmm::kOverflowZ || rhall == 0 || t.xyz1 == 0 ||
      t.z1 == 0 || t.z2 == 0) {
    return false;
  }
  const float r = static_cast<float>(t.xyz1) * 16384.0f / rhall - 16384.0f;
  const float xy = t.xy2 * (r * r / 268435456.0f) + r * t.xy1 / 16384.0f + 256.0f;
  out[0] = ((x * (xy * (t.x2 + 160.0f))) / 8192.0f + t.x1 * 8.0f) / 16.0f;
  out[1] = ((y * (xy * (t.y2 + 160.0f))) / 8192.0f + t.y1 * 8.0f) / 16.0f;
  const float z5 = (static_cast<float>(z) - t.z4) * 131072.0f - t.z3 * (static_cast<float>(rhall) - t.xyz1);
  out[2] = z5 / ((t.z2 + t.z1 * static_cast<float>(rhall) / 32768.0f) * 4.0f) / 16.0f;
  return true;
}

// ---- BMP280 ----

bool initBmp280() {
  if (!addDevice(kBmpAddr, &gBmp)) {
    if (!gQuiet) ESP_LOGW(TAG, "BMP280 not found");
    return false;
  }
  uint8_t id = 0;
  if (!readRegs(gBmp, bmp::ID, &id, 1) || id != bmp::kId) {
    ESP_LOGW(TAG, "BMP280: unexpected chip id 0x%02x", id);
    return false;
  }
  writeReg(gBmp, bmp::RESET, 0xB6);
  for (int i = 0; i < 10; i++) {  // wait until the calibration data is copied
    waitMs(2);
    uint8_t st = 1;
    if (readRegs(gBmp, bmp::STATUS, &st, 1) && !(st & 0x01)) break;
  }
  uint8_t c[24];
  if (!readRegs(gBmp, bmp::CALIB, c, sizeof(c))) return false;
  auto u16 = [&](int i) { return static_cast<uint16_t>(c[i] | (c[i + 1] << 8)); };
  auto s16 = [&](int i) { return static_cast<int16_t>(u16(i)); };
  gCal = {u16(0), s16(2), s16(4), u16(6), s16(8), s16(10), s16(12), s16(14), s16(16), s16(18), s16(20), s16(22)};
  // CONFIG is only reliably written in sleep mode, so before CTRL_MEAS.
  return writeReg(gBmp, bmp::CONFIG, bmp::kConfig) && writeReg(gBmp, bmp::CTRL_MEAS, bmp::kCtrlMeas);
}

// Datasheet compensation formulas (double precision version).
void compensateBaro(int32_t adcT, int32_t adcP, float* tempC, float* hPa) {
  const BmpCalib& c = gCal;
  double v1 = (adcT / 16384.0 - c.t1 / 1024.0) * c.t2;
  double v2 = (adcT / 131072.0 - c.t1 / 8192.0) * (adcT / 131072.0 - c.t1 / 8192.0) * c.t3;
  const double tFine = v1 + v2;
  *tempC = static_cast<float>(tFine / 5120.0);
  v1 = tFine / 2.0 - 64000.0;
  v2 = v1 * v1 * c.p6 / 32768.0;
  v2 = v2 + v1 * c.p5 * 2.0;
  v2 = v2 / 4.0 + c.p4 * 65536.0;
  v1 = (c.p3 * v1 * v1 / 524288.0 + c.p2 * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * c.p1;
  if (v1 == 0) {
    *hPa = 0;
    return;
  }
  double p = 1048576.0 - adcP;
  p = (p - v2 / 4096.0) * 6250.0 / v1;
  v1 = c.p9 * p * p / 2147483648.0;
  v2 = p * c.p8 / 32768.0;
  p = p + (v1 + v2 + c.p7) / 16.0;
  *hPa = static_cast<float>(p / 100.0);
}

// ---- sampling ----

void readFifo() {
  uint8_t lb[2];
  if (!readRegs(gBmi, bmi::FIFO_LENGTH_0, lb, 2)) return;
  const size_t len = std::min<size_t>((lb[0] | (lb[1] << 8)) & 0x3FFF, sizeof(gFifo));
  if (len == 0 || !readRegs(gBmi, bmi::FIFO_DATA, gFifo, len)) return;

  int32_t sa[3] = {}, sg[3] = {};
  int16_t a[3] = {}, g[3] = {};
  uint32_t n = 0, skipped = 0;
  size_t i = 0;
  while (i < len) {
    const uint8_t h = gFifo[i] & 0xFC;  // the low bits are interrupt tags
    if (h == 0x8C) {                    // gyro + accel
      if (i + 13 > len) break;
      for (int k = 0; k < 3; k++) {
        g[k] = static_cast<int16_t>(gFifo[i + 1 + 2 * k] | (gFifo[i + 2 + 2 * k] << 8));
        a[k] = static_cast<int16_t>(gFifo[i + 7 + 2 * k] | (gFifo[i + 8 + 2 * k] << 8));
        sg[k] += g[k];
        sa[k] += a[k];
      }
      n++;
      i += 13;
    } else if (h == 0x84 || h == 0x88) {  // accel or gyro alone (only while starting up)
      i += 7;
    } else if (h == 0x40) {  // skip frame: frames lost to a FIFO overflow
      if (i + 1 < len) skipped += gFifo[i + 1];
      i += 2;
    } else if (h == 0x44) {  // sensortime
      i += 4;
    } else if (h == 0x48) {  // configuration change
      i += 5;
    } else {  // 0x80: over-read, nothing more
      break;
    }
  }
  if (skipped) ESP_LOGW(TAG, "BMI270 FIFO overflow, %lu samples lost", static_cast<unsigned long>(skipped));
  if (!n) return;
  taskENTER_CRITICAL(&gLock);
  for (int k = 0; k < 3; k++) {
    gSumA[k] += sa[k];
    gSumG[k] += sg[k];
    gLastA[k] = a[k];
    gLastG[k] = g[k];
  }
  gSumN += n;
  gHaveAg = true;
  taskEXIT_CRITICAL(&gLock);
}

void readMag() {
  uint8_t d[10];  // ERR_REG, STATUS, AUX_DATA_0..7
  float m[3];
  bool ok = readRegs(gBmi, bmi::ERR_REG, d, sizeof(d)) && !(d[0] & bmi::kErrAux) && compensateMag(d + 2, m);
  if (ok && gCalMutex) {
    xSemaphoreTake(gCalMutex, portMAX_DELAY);
    if (gCollecting) gFit.add(m);
    xSemaphoreGive(gCalMutex);
  }
  taskENTER_CRITICAL(&gLock);
  if (ok) {
    if (gHaveCalib) {
      gCalib.apply(m, gMag);
    } else {
      memcpy(gMag, m, sizeof(gMag));
    }
    gMagCalApplied = gHaveCalib;
  }
  gMagValid = ok;
  taskEXIT_CRITICAL(&gLock);
}

void readBaro() {
  uint8_t d[6];
  if (!readRegs(gBmp, bmp::DATA, d, sizeof(d))) return;
  const int32_t adcP = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
  const int32_t adcT = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
  if (adcP == bmp::kNoData || adcT == bmp::kNoData) return;
  float t, p;
  compensateBaro(adcT, adcP, &t, &p);
  taskENTER_CRITICAL(&gLock);
  gTemp = t;
  gPressure = p;
  gBaroValid = true;
  taskEXIT_CRITICAL(&gLock);
}

// A sensor that was in the middle of a transfer when the ESP32 was reset (crash, watchdog, brownout; the
// sensors stay powered) may hold SDA low: every device on the bus then looks missing until the power is cut.
// Clocking SCL frees it (i2c_master_bus_reset: up to 9 pulses and a STOP).
void recoverBus() {
  const esp_err_t err = i2c_master_bus_reset(gBus);
  taskENTER_CRITICAL(&gLock);
  gBusResets++;
  taskEXIT_CRITICAL(&gLock);
  ESP_LOGW(TAG, "I2C bus reset: %s", esp_err_to_name(err));
}

void loadCalib();  // below, with the NVS helpers

void setStatus(const Status& st) {
  taskENTER_CRITICAL(&gLock);
  gStatus = st;
  taskEXIT_CRITICAL(&gLock);
}

// Initialises the sensors that are not running. The BMM150 sits behind the BMI270, so it is only tried
// together with it (a BMI270 that runs without a BMM150 is left alone).
void detect() {
  Status st = gStatus;
  if (!st.bmi270) {
    st.bmi270 = initBmi270();
    st.bmm150 = st.bmi270 && initBmm150();
    if (st.bmi270 && !startBmi270(st.bmm150)) st.bmi270 = st.bmm150 = false;
    if (st.bmm150 && !gStatus.bmm150) loadCalib();
  }
  if (!st.bmp280) st.bmp280 = initBmp280();
  setStatus(st);
}

bool allFound() { return gStatus.bmi270 && gStatus.bmm150 && gStatus.bmp280; }

void sampleTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  TickType_t retryAt = wake + kRetryPeriod;
  for (uint32_t loop = 0;; loop++) {
    xTaskDelayUntil(&wake, kPeriod);
    if ((!gStatus.bmi270 || !gStatus.bmp280) && static_cast<int32_t>(wake - retryAt) >= 0) {
      retryAt = wake + kRetryPeriod;
      const Status before = gStatus;
      recoverBus();
      detect();
      if (gStatus.bmi270 != before.bmi270 || gStatus.bmm150 != before.bmm150 || gStatus.bmp280 != before.bmp280) {
        ESP_LOGW(TAG, "sensors found again: BMI270 %s, BMM150 %s, BMP280 %s", gStatus.bmi270 ? "ok" : "--",
                 gStatus.bmm150 ? "ok" : "--", gStatus.bmp280 ? "ok" : "--");
      }
      wake = xTaskGetTickCount();
    }
    if (gStatus.bmi270) readFifo();
    if (loop & 1) {
      if (gStatus.bmm150) readMag();
      if (gStatus.bmp280) readBaro();
    }
  }
}

// Fills everything but the accel/gyro fields. Call inside the lock.
void fillMagBaro(Sample& out) {
  out.mx = gMag[0];
  out.my = gMag[1];
  out.mz = gMag[2];
  out.magValid = gMagValid;
  out.magCal = gMagCalApplied;
  out.pressureHpa = gPressure;
  out.tempC = gTemp;
  out.baroValid = gBaroValid;
}

void fillLast(Sample& out) {
  out.ax = gLastA[0] / bmi::kAccLsbPerG;
  out.ay = gLastA[1] / bmi::kAccLsbPerG;
  out.az = gLastA[2] / bmi::kAccLsbPerG;
  out.gx = gLastG[0] / bmi::kGyrLsbPerDps;
  out.gy = gLastG[1] / bmi::kGyrLsbPerDps;
  out.gz = gLastG[2] / bmi::kGyrLsbPerDps;
}

// NVS: namespace "imu", key "magcal" = MagCal as a blob, prefixed by a version byte.
constexpr char kNvsNs[] = "imu", kNvsKey[] = "magcal";
constexpr uint8_t kCalVersion = 1;

void loadCalib() {
  nvs_handle_t h;
  if (nvs_open(kNvsNs, NVS_READONLY, &h) != ESP_OK) return;
  uint8_t blob[1 + sizeof(MagCal)];
  size_t len = sizeof(blob);
  if (nvs_get_blob(h, kNvsKey, blob, &len) == ESP_OK && len == sizeof(blob) && blob[0] == kCalVersion) {
    memcpy(&gCalib, blob + 1, sizeof(MagCal));
    gHaveCalib = true;
    ESP_LOGI(TAG, "mag calibration: offset %.1f %.1f %.1f uT, scale %.3f %.3f %.3f, field %.1f uT",
             gCalib.offset[0], gCalib.offset[1], gCalib.offset[2], gCalib.scale[0], gCalib.scale[1],
             gCalib.scale[2], gCalib.field);
  }
  nvs_close(h);
}

// Writes (cal != nullptr) or erases the stored calibration. False when NVS failed (logged).
bool storeCalib(const MagCal* cal) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(kNvsNs, NVS_READWRITE, &h);
  if (err == ESP_OK) {
    if (cal) {
      uint8_t blob[1 + sizeof(MagCal)];
      blob[0] = kCalVersion;
      memcpy(blob + 1, cal, sizeof(MagCal));
      err = nvs_set_blob(h, kNvsKey, blob, sizeof(blob));
    } else {
      err = nvs_erase_key(h, kNvsKey);
      if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    }
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
  }
  if (err != ESP_OK) ESP_LOGE(TAG, "mag calibration not saved: %s", esp_err_to_name(err));
  return err == ESP_OK;
}

// ---- Battery (IP5306 power IC of the M5Stack Gray) ----

namespace ip5306 {
constexpr uint8_t kAddr = 0x75;
constexpr uint32_t kSclHz = 100000;  // the IC is specified for 100 kHz (it runs on its own clock)
constexpr uint8_t kRegRead0 = 0x70;  // bit 3: charging
constexpr uint8_t kRegRead1 = 0x71;  // bit 3: charge full
constexpr uint8_t kRegLevel = 0x78;  // bits 7..4: level, inverted thermometer code
constexpr TickType_t kPeriod = pdMS_TO_TICKS(5000);
}  // namespace ip5306

SemaphoreHandle_t gBatMutex;
i2c_master_dev_handle_t gBatDev;
bool gBatProbed;
Battery gBattery;
TickType_t gBatAt;

uint8_t levelPercent(uint8_t reg) {
  switch (reg & 0xF0) {
    case 0x00: return 100;
    case 0x80: return 75;
    case 0xC0: return 50;
    case 0xE0: return 25;
    default: return 0;
  }
}

void readBattery() {
  if (!gBatProbed) {
    // Only "no ACK" (ESP_ERR_NOT_FOUND) means there is no I2C power IC; a timeout or a failed registration is
    // tried again at the next period.
    const esp_err_t err = i2c_master_probe(gBus, ip5306::kAddr, kTimeoutMs);
    if (err == ESP_OK) {
      i2c_device_config_t cfg = {};
      cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
      cfg.device_address = ip5306::kAddr;
      cfg.scl_speed_hz = ip5306::kSclHz;
      if (i2c_master_bus_add_device(gBus, &cfg, &gBatDev) != ESP_OK) gBatDev = nullptr;
      gBatProbed = gBatDev != nullptr;
    } else {
      gBatProbed = err == ESP_ERR_NOT_FOUND;
    }
    if (gBatProbed) ESP_LOGI(TAG, "power IC (IP5306, I2C): %s", gBatDev ? "found" : "not found, no battery level");
    else ESP_LOGW(TAG, "power IC (IP5306): probe failed (%s), trying again", esp_err_to_name(err));
  }
  if (!gBatDev) return;
  uint8_t r0, r1, lv;
  if (!readRegs(gBatDev, ip5306::kRegRead0, &r0, 1) || !readRegs(gBatDev, ip5306::kRegRead1, &r1, 1) ||
      !readRegs(gBatDev, ip5306::kRegLevel, &lv, 1)) {
    gBattery.valid = false;
    return;
  }
  gBattery.valid = true;
  gBattery.full = r1 & 0x08;
  gBattery.charging = (r0 & 0x08) && !gBattery.full;
  gBattery.level = levelPercent(lv);
}

}  // namespace

Status init() {
  i2c_master_bus_config_t bus = {};
  bus.i2c_port = kPort;
  bus.sda_io_num = kSda;
  bus.scl_io_num = kScl;
  bus.clk_source = I2C_CLK_SRC_DEFAULT;
  bus.glitch_ignore_cnt = 7;
  bus.flags.enable_internal_pullup = true;  // the board has external pull-ups; this does no harm
  if (const esp_err_t err = i2c_new_master_bus(&bus, &gBus); err != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus: %s", esp_err_to_name(err));
    return gStatus;
  }
  gCalMutex = xSemaphoreCreateMutex();
  gBatMutex = xSemaphoreCreateMutex();
  detect();
  if (!allFound()) {  // maybe a sensor holds the bus since a reset: free it and try once more
    recoverBus();
    detect();
  }
  ESP_LOGI(TAG, "BMI270 %s, BMM150 %s, BMP280 %s", gStatus.bmi270 ? "ok" : "--", gStatus.bmm150 ? "ok" : "--",
           gStatus.bmp280 ? "ok" : "--");
  gQuiet = true;
  // Always started: it also looks for missing sensors again every few seconds.
  xTaskCreate(sampleTask, "imu", kTaskStack, nullptr, kTaskPrio, nullptr);
  return gStatus;
}

void takeAverage(Sample& out) {
  int64_t sa[3], sg[3];
  taskENTER_CRITICAL(&gLock);
  const uint32_t n = gSumN;
  for (int k = 0; k < 3; k++) {
    sa[k] = gSumA[k];
    sg[k] = gSumG[k];
    gSumA[k] = gSumG[k] = 0;
  }
  gSumN = 0;
  if (!n) fillLast(out);
  fillMagBaro(out);
  taskEXIT_CRITICAL(&gLock);
  out.n = n;
  if (n) {
    out.ax = static_cast<float>(sa[0]) / n / bmi::kAccLsbPerG;
    out.ay = static_cast<float>(sa[1]) / n / bmi::kAccLsbPerG;
    out.az = static_cast<float>(sa[2]) / n / bmi::kAccLsbPerG;
    out.gx = static_cast<float>(sg[0]) / n / bmi::kGyrLsbPerDps;
    out.gy = static_cast<float>(sg[1]) / n / bmi::kGyrLsbPerDps;
    out.gz = static_cast<float>(sg[2]) / n / bmi::kGyrLsbPerDps;
  }
}

void latest(Sample& out) {
  taskENTER_CRITICAL(&gLock);
  fillLast(out);
  fillMagBaro(out);
  out.n = gHaveAg ? 1 : 0;
  taskEXIT_CRITICAL(&gLock);
}

uint32_t magCalStart() {
  if (!gCalMutex) return 0;
  xSemaphoreTake(gCalMutex, portMAX_DELAY);
  gFit.reset();
  gCollecting = true;
  const uint32_t session = ++gCalSession;
  xSemaphoreGive(gCalMutex);
  ESP_LOGI(TAG, "mag calibration: collecting");
  return session;
}

MagCalError magCalFinish() {
  if (!gCalMutex) return MagCalError::kNotCollecting;
  // Holds the mutex until the result is applied and saved, so a clear or another finish can't slip in between.
  // readMag() waits meanwhile (the solve is well under a millisecond, the NVS write a few); the BMI270 FIFO
  // keeps the accel / gyro samples.
  xSemaphoreTake(gCalMutex, portMAX_DELAY);
  if (!gCollecting) {  // cancelled, or already finished (display and console)
    xSemaphoreGive(gCalMutex);
    return MagCalError::kNotCollecting;
  }
  gCollecting = false;
  MagCal cal;
  const MagCalError err = gFit.solve(cal);
  if (err != MagCalError::kNone) {
    ESP_LOGW(TAG, "mag calibration failed (%d) after %lu samples, coverage %.2f %.2f %.2f", static_cast<int>(err),
             static_cast<unsigned long>(gFit.samples()), gFit.coverage(0), gFit.coverage(1), gFit.coverage(2));
    xSemaphoreGive(gCalMutex);
    return err;
  }
  taskENTER_CRITICAL(&gLock);
  gCalib = cal;
  gHaveCalib = true;
  taskEXIT_CRITICAL(&gLock);
  const bool saved = storeCalib(&cal);
  ESP_LOGI(TAG, "mag calibration: offset %.1f %.1f %.1f uT, scale %.3f %.3f %.3f, field %.1f uT (%lu samples)",
           cal.offset[0], cal.offset[1], cal.offset[2], cal.scale[0], cal.scale[1], cal.scale[2], cal.field,
           static_cast<unsigned long>(gFit.samples()));
  xSemaphoreGive(gCalMutex);
  return saved ? MagCalError::kNone : MagCalError::kNotSaved;
}

void magCalCancel() {
  if (!gCalMutex) return;
  xSemaphoreTake(gCalMutex, portMAX_DELAY);
  gCollecting = false;
  xSemaphoreGive(gCalMutex);
}

bool magCalClear() {
  if (!gCalMutex) return false;
  xSemaphoreTake(gCalMutex, portMAX_DELAY);
  gCollecting = false;  // a pending or automatic finish must not save the collection after the clear
  taskENTER_CRITICAL(&gLock);
  gHaveCalib = false;
  taskEXIT_CRITICAL(&gLock);
  const bool saved = storeCalib(nullptr);
  xSemaphoreGive(gCalMutex);
  ESP_LOGI(TAG, "mag calibration cleared");
  return saved;
}

MagCalInfo magCalInfo() {
  MagCalInfo info = {};
  taskENTER_CRITICAL(&gLock);
  info.calibrated = gHaveCalib;
  info.cal = gCalib;
  taskEXIT_CRITICAL(&gLock);
  if (!gCalMutex) return info;
  xSemaphoreTake(gCalMutex, portMAX_DELAY);
  info.collecting = gCollecting;
  info.session = gCalSession;
  info.samples = gFit.samples();
  for (int k = 0; k < 3; k++) info.coverage[k] = gFit.coverage(k);
  info.complete = gFit.complete();
  xSemaphoreGive(gCalMutex);
  return info;
}

Status status() {
  taskENTER_CRITICAL(&gLock);
  const Status st = gStatus;
  taskEXIT_CRITICAL(&gLock);
  return st;
}

uint32_t busResets() {
  taskENTER_CRITICAL(&gLock);
  const uint32_t n = gBusResets;
  taskEXIT_CRITICAL(&gLock);
  return n;
}

Battery battery() {
  if (!gBus || !gBatMutex) return {};
  xSemaphoreTake(gBatMutex, portMAX_DELAY);
  const TickType_t now = xTaskGetTickCount();
  if (!gBatProbed || now - gBatAt >= ip5306::kPeriod) {
    readBattery();
    gBatAt = now;
  }
  const Battery b = gBattery;
  xSemaphoreGive(gBatMutex);
  return b;
}

}  // namespace imu
