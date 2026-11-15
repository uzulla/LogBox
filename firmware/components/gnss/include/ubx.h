// u-blox UBX protocol helpers independent of ESP-IDF so they can be unit-tested
// on the host: frame reassembly, NAV-PVT decoding, frame / CFG-VALSET building
// and an NMEA sentence detector (used for baud-rate detection).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "gnss.h"

namespace ubx {

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;
constexpr size_t kMaxPayloadLen = 1024;
constexpr size_t kFrameOverhead = 8;  // sync(2) class id len(2) ... ck(2)

constexpr uint8_t kClsNav = 0x01, kIdNavPvt = 0x07;
constexpr size_t kNavPvtLen = 92;
constexpr uint8_t kClsAck = 0x05, kIdAckNak = 0x00, kIdAckAck = 0x01;
constexpr uint8_t kClsCfg = 0x06, kIdCfgValset = 0x8A, kIdCfgValget = 0x8B;
constexpr uint8_t kClsMon = 0x0A, kIdMonVer = 0x04;

// CFG-VALSET layers
constexpr uint8_t kLayerRam = 0x01, kLayerBbr = 0x02, kLayerFlash = 0x04;

// Configuration keys (u-blox M9 SPG 4.04 interface description).
constexpr uint32_t kCfgUart1Baudrate = 0x40520001;     // U4
constexpr uint32_t kCfgUart1InprotUbx = 0x10730001;    // L
constexpr uint32_t kCfgUart1InprotNmea = 0x10730002;   // L
constexpr uint32_t kCfgUart1OutprotUbx = 0x10740001;   // L
constexpr uint32_t kCfgUart1OutprotNmea = 0x10740002;  // L
constexpr uint32_t kCfgMsgoutNavPvtUart1 = 0x20910007; // U1
constexpr uint32_t kCfgRateMeas = 0x30210001;          // U2, ms
constexpr uint32_t kCfgRateNav = 0x30210002;           // U2, cycles
constexpr uint32_t kCfgNavspgDynmodel = 0x20110021;    // E1
constexpr uint8_t kDynModelAutomotive = 4;

// Value size in bytes encoded in bits 28..30 of a configuration key; 0 if invalid.
size_t keyValueSize(uint32_t key);

// Reassembles UBX frames from a byte stream (fed one byte at a time).
class Parser {
 public:
  // Returns true when a complete frame with a valid checksum has just been
  // received; it can then be read via msgClass()/msgId()/payload()/payloadLen()
  // until the next call to feed().
  bool feed(uint8_t b);
  void reset() { state_ = State::Sync1; }

  uint8_t msgClass() const { return cls_; }
  uint8_t msgId() const { return id_; }
  const uint8_t* payload() const { return payload_; }
  size_t payloadLen() const { return len_; }

  uint32_t checksumErrors() const { return checksumErrors_; }
  uint32_t lengthErrors() const { return lengthErrors_; }

 private:
  enum class State { Sync1, Sync2, Class, Id, Len1, Len2, Payload, CkA, CkB };

  void addCk(uint8_t b) {
    ckA_ += b;
    ckB_ += ckA_;
  }

  State state_ = State::Sync1;
  uint8_t cls_ = 0, id_ = 0;
  uint16_t len_ = 0, pos_ = 0;
  uint8_t ckA_ = 0, ckB_ = 0, rxCkA_ = 0;
  uint8_t payload_[kMaxPayloadLen];
  uint32_t checksumErrors_ = 0;
  uint32_t lengthErrors_ = 0;
};

// Counts NMEA sentences ("$...*hh") with a valid checksum.
class NmeaDetector {
 public:
  // Returns true when a sentence with a valid checksum has just ended.
  bool feed(uint8_t b);
  void reset() { state_ = State::Idle; }
  uint32_t sentences() const { return sentences_; }

 private:
  enum class State { Idle, Body, Hex1, Hex2 };
  State state_ = State::Idle;
  uint8_t sum_ = 0, rxSum_ = 0, len_ = 0;
  uint32_t sentences_ = 0;
};

bool decodeNavPvt(const uint8_t* payload, size_t len, gnss::Pvt& out);

// Builds a complete frame (sync, header, payload, checksum) into out. Returns
// the frame length, or 0 if out is too small.
size_t buildFrame(uint8_t cls, uint8_t id, const uint8_t* payload, size_t len, uint8_t* out,
                  size_t outSize);

struct KeyValue {
  uint32_t key;
  uint64_t value;  // little-endian truncated to the key's size
};

// Builds a UBX-CFG-VALSET (version 0, i.e. a single, atomic transaction) frame.
// Returns the frame length, or 0 if out is too small or a key has an invalid size.
size_t buildValset(uint8_t layers, const KeyValue* kv, size_t n, uint8_t* out, size_t outSize);

}  // namespace ubx
