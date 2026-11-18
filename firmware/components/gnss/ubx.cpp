#include "ubx.h"

#include <string.h>

namespace ubx {

size_t keyValueSize(uint32_t key) {
  switch ((key >> 28) & 0x07) {
    case 1: return 1;  // L (one bit, stored in one byte)
    case 2: return 1;
    case 3: return 2;
    case 4: return 4;
    case 5: return 8;
    default: return 0;
  }
}

bool Parser::feed(uint8_t b) {
  switch (state_) {
    case State::Sync1:
      if (b == kSync1) state_ = State::Sync2;
      return false;
    case State::Sync2:
      if (b == kSync2) {
        state_ = State::Class;
        ckA_ = ckB_ = 0;
      } else if (b != kSync1) {
        state_ = State::Sync1;
      }
      return false;
    case State::Class:
      cls_ = b;
      addCk(b);
      state_ = State::Id;
      return false;
    case State::Id:
      id_ = b;
      addCk(b);
      state_ = State::Len1;
      return false;
    case State::Len1:
      len_ = b;
      addCk(b);
      state_ = State::Len2;
      return false;
    case State::Len2:
      len_ |= static_cast<uint16_t>(b) << 8;
      addCk(b);
      if (len_ > kMaxPayloadLen && !(cls_ == kClsNav && id_ == kIdNavSat && len_ <= kMaxNavSatLen)) {
        lengthErrors_++;
        state_ = State::Sync1;
      } else {
        pos_ = 0;
        state_ = len_ ? State::Payload : State::CkA;
      }
      return false;
    case State::Payload:
      if (pos_ < kMaxPayloadLen) payload_[pos_] = b;
      pos_++;
      addCk(b);
      if (pos_ >= len_) state_ = State::CkA;
      return false;
    case State::CkA:
      rxCkA_ = b;
      state_ = State::CkB;
      return false;
    case State::CkB:
      state_ = State::Sync1;
      if (rxCkA_ == ckA_ && b == ckB_) return true;
      checksumErrors_++;
      return false;
  }
  return false;
}

static int hexVal(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool NmeaDetector::feed(uint8_t b) {
  if (b == '$') {  // always (re)starts a sentence
    state_ = State::Body;
    sum_ = 0;
    len_ = 0;
    return false;
  }
  switch (state_) {
    case State::Idle:
      return false;
    case State::Body:
      if (b == '*') {
        state_ = len_ >= 5 ? State::Hex1 : State::Idle;
      } else if (b < 0x20 || b > 0x7E || ++len_ > 80) {
        state_ = State::Idle;
      } else {
        sum_ ^= b;
      }
      return false;
    case State::Hex1: {
      int v = hexVal(b);
      if (v < 0) {
        state_ = State::Idle;
        return false;
      }
      rxSum_ = static_cast<uint8_t>(v << 4);
      state_ = State::Hex2;
      return false;
    }
    case State::Hex2: {
      state_ = State::Idle;
      int v = hexVal(b);
      if (v < 0 || (rxSum_ | v) != sum_) return false;
      sentences_++;
      return true;
    }
  }
  return false;
}

static uint16_t u16(const uint8_t* p) { return p[0] | (p[1] << 8); }
static uint32_t u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
static int32_t i32(const uint8_t* p) { return static_cast<int32_t>(u32(p)); }

bool decodeNavPvt(const uint8_t* p, size_t len, gnss::Pvt& d) {
  if (len < kNavPvtLen) return false;
  d.iTOW = u32(p + 0);
  d.year = u16(p + 4);
  d.month = p[6];
  d.day = p[7];
  d.hour = p[8];
  d.min = p[9];
  d.sec = p[10];
  d.valid = p[11];
  d.tAcc = u32(p + 12);
  d.nano = i32(p + 16);
  d.fixType = p[20];
  d.flags = p[21];
  d.flags2 = p[22];
  d.numSV = p[23];
  d.lon = i32(p + 24);
  d.lat = i32(p + 28);
  d.height = i32(p + 32);
  d.hMSL = i32(p + 36);
  d.hAcc = u32(p + 40);
  d.vAcc = u32(p + 44);
  d.velN = i32(p + 48);
  d.velE = i32(p + 52);
  d.velD = i32(p + 56);
  d.gSpeed = i32(p + 60);
  d.headMot = i32(p + 64);
  d.sAcc = u32(p + 68);
  d.headAcc = u32(p + 72);
  d.pDOP = u16(p + 76);
  d.flags3 = p[78];
  return true;
}

bool decodeNavSat(const uint8_t* p, size_t len, size_t frameLen, gnss::Sky& d) {
  if (len < kNavSatHeaderLen || len > frameLen || p[4] != kNavSatVersion) return false;
  const int n = p[5];
  if (frameLen != kNavSatHeaderLen + n * kNavSatBlockLen) return false;
  const int avail = static_cast<int>((len - kNavSatHeaderLen) / kNavSatBlockLen);
  const int have = n < avail ? n : avail;
  d.iTOW = u32(p);
  d.numSvs = have < gnss::kMaxSats ? have : gnss::kMaxSats;
  d.totalSvs = n;
  d.totalUsed = d.totalSignal = 0;
  memset(d.viewBy, 0, sizeof(d.viewBy));
  memset(d.usedBy, 0, sizeof(d.usedBy));
  for (int i = 0; i < have; i++) {
    const uint8_t* b = p + kNavSatHeaderLen + i * kNavSatBlockLen;
    gnss::Sat s;
    s.gnssId = b[0];
    s.svId = b[1];
    s.cno = b[2];
    s.elev = static_cast<int8_t>(b[3]);
    s.azim = static_cast<int16_t>(u16(b + 4));
    s.used = (u32(b + 8) & 0x08) != 0;  // flags bit 3: svUsed
    d.totalUsed += s.used;
    d.totalSignal += s.cno > 0;
    if (s.gnssId < 8) {
      d.viewBy[s.gnssId]++;
      d.usedBy[s.gnssId] += s.used;
    }
    if (i < gnss::kMaxSats) d.sats[i] = s;
  }
  return true;
}

bool decodeNavDop(const uint8_t* p, size_t len, gnss::Sky& d) {
  if (len != kNavDopLen) return false;
  d.gDOP = u16(p + 4);
  d.pDOP = u16(p + 6);
  d.tDOP = u16(p + 8);
  d.vDOP = u16(p + 10);
  d.hDOP = u16(p + 12);
  return true;
}

size_t buildFrame(uint8_t cls, uint8_t id, const uint8_t* payload, size_t len, uint8_t* out,
                  size_t outSize) {
  if (len > 0xFFFF || outSize < len + kFrameOverhead) return 0;
  out[0] = kSync1;
  out[1] = kSync2;
  out[2] = cls;
  out[3] = id;
  out[4] = static_cast<uint8_t>(len);
  out[5] = static_cast<uint8_t>(len >> 8);
  if (len) memmove(out + 6, payload, len);
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6 + len; i++) {
    a += out[i];
    b += a;
  }
  out[6 + len] = a;
  out[7 + len] = b;
  return len + kFrameOverhead;
}

size_t buildValset(uint8_t layers, const KeyValue* kv, size_t n, uint8_t* out, size_t outSize) {
  // Payload is built in place after the 6-byte header.
  if (outSize < kFrameOverhead + 4) return 0;
  uint8_t* p = out + 6;
  size_t len = 0;
  p[len++] = 0;  // version 0: transaction-less, applied atomically
  p[len++] = layers;
  p[len++] = 0;  // reserved
  p[len++] = 0;
  for (size_t i = 0; i < n; i++) {
    size_t vs = keyValueSize(kv[i].key);
    if (vs == 0 || kFrameOverhead + len + 4 + vs > outSize) return 0;
    for (int k = 0; k < 4; k++) p[len++] = static_cast<uint8_t>(kv[i].key >> (8 * k));
    for (size_t k = 0; k < vs; k++) p[len++] = static_cast<uint8_t>(kv[i].value >> (8 * k));
  }
  return buildFrame(kClsCfg, kIdCfgValset, p, len, out, outSize);
}

}  // namespace ubx
