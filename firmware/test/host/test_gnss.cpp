#include <string.h>
#include <ubx.h>
#include <unity.h>

#include <vector>

using namespace ubx;

static int feedAll(Parser& p, const uint8_t* data, size_t len) {
  int frames = 0;
  for (size_t i = 0; i < len; i++) {
    if (p.feed(data[i])) frames++;
  }
  return frames;
}

static void put16(uint8_t* p, uint16_t v) {
  p[0] = v;
  p[1] = v >> 8;
}
static void put32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; i++) p[i] = v >> (8 * i);
}

// NAV-PVT payload laid out per the u-blox M9 SPG 4.04 interface description.
static std::vector<uint8_t> navPvtPayload() {
  std::vector<uint8_t> p(kNavPvtLen, 0);
  put32(&p[0], 123456789);  // iTOW
  put16(&p[4], 2026);
  p[6] = 11;
  p[7] = 15;
  p[8] = 12;
  p[9] = 34;
  p[10] = 56;
  p[11] = 0x07;            // valid: date, time, fully resolved
  put32(&p[12], 25);       // tAcc
  put32(&p[16], uint32_t(-123456));  // nano
  p[20] = 3;               // 3D
  p[21] = 0x01;            // gnssFixOK
  p[22] = 0xE0;
  p[23] = 17;              // numSV
  put32(&p[24], uint32_t(1397000000));   // lon 139.7
  put32(&p[28], uint32_t(356000000));    // lat 35.6
  put32(&p[32], uint32_t(45000));
  put32(&p[36], uint32_t(5000));
  put32(&p[40], 1500);
  put32(&p[44], 2500);
  put32(&p[48], uint32_t(-1000));
  put32(&p[52], 2000);
  put32(&p[56], uint32_t(-30));
  put32(&p[60], 2236);     // gSpeed
  put32(&p[64], 11656505); // headMot 116.56505 deg
  put32(&p[68], 80);
  put32(&p[72], 150000);
  put16(&p[76], 123);      // pDOP 1.23
  p[78] = 0x00;
  return p;
}

void setUp() {}
void tearDown() {}

void test_build_frame_known_checksum() {
  uint8_t f[16];
  TEST_ASSERT_EQUAL(8, buildFrame(kClsMon, kIdMonVer, nullptr, 0, f, sizeof(f)));
  const uint8_t expect[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, f, 8);
  TEST_ASSERT_EQUAL(0, buildFrame(kClsMon, kIdMonVer, nullptr, 0, f, 7));
}

void test_valset_rate() {
  const KeyValue kv[] = {{kCfgRateMeas, 100}};
  uint8_t f[64];
  const uint8_t expect[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x00, 0x01, 0x00,
                            0x00, 0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x51, 0xB9};
  TEST_ASSERT_EQUAL(sizeof(expect), buildValset(kLayerRam, kv, 1, f, sizeof(f)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, f, sizeof(expect));
}

void test_valset_mixed_sizes() {
  const KeyValue kv[] = {{kCfgUart1Baudrate, 115200}, {kCfgUart1OutprotNmea, 0},
                         {kCfgMsgoutNavPvtUart1, 1}};
  uint8_t f[64];
  const uint8_t expect[] = {0xB5, 0x62, 0x06, 0x8A, 22,   0x00, 0x00, 0x03, 0x00, 0x00,
                            0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0x02, 0x00,
                            0x74, 0x10, 0x00, 0x07, 0x00, 0x91, 0x20, 0x01, 0x3E, 0x78};
  TEST_ASSERT_EQUAL(sizeof(expect), buildValset(kLayerRam | kLayerBbr, kv, 3, f, sizeof(f)));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, f, sizeof(expect));
  // The built frame parses back.
  Parser p;
  TEST_ASSERT_EQUAL(1, feedAll(p, f, sizeof(expect)));
  TEST_ASSERT_EQUAL_HEX8(kClsCfg, p.msgClass());
  TEST_ASSERT_EQUAL_HEX8(kIdCfgValset, p.msgId());
  TEST_ASSERT_EQUAL(22, p.payloadLen());
}

void test_valset_errors() {
  uint8_t f[64];
  const KeyValue bad[] = {{0x00000001, 0}};  // size bits 0 -> invalid
  TEST_ASSERT_EQUAL(0, buildValset(kLayerRam, bad, 1, f, sizeof(f)));
  const KeyValue kv[] = {{kCfgUart1Baudrate, 115200}};
  TEST_ASSERT_EQUAL(0, buildValset(kLayerRam, kv, 1, f, 8 + 4 + 7));
  TEST_ASSERT_EQUAL(20, buildValset(kLayerRam, kv, 1, f, 20));
  TEST_ASSERT_EQUAL(1, keyValueSize(kCfgUart1InprotUbx));
  TEST_ASSERT_EQUAL(1, keyValueSize(kCfgNavspgDynmodel));
  TEST_ASSERT_EQUAL(2, keyValueSize(kCfgRateMeas));
  TEST_ASSERT_EQUAL(4, keyValueSize(kCfgUart1Baudrate));
}

void test_nav_pvt_decode() {
  auto payload = navPvtPayload();
  uint8_t f[128];
  size_t n = buildFrame(kClsNav, kIdNavPvt, payload.data(), payload.size(), f, sizeof(f));
  TEST_ASSERT_EQUAL(100, n);
  Parser p;
  TEST_ASSERT_EQUAL(1, feedAll(p, f, n));
  TEST_ASSERT_EQUAL_HEX8(kClsNav, p.msgClass());
  TEST_ASSERT_EQUAL_HEX8(kIdNavPvt, p.msgId());

  gnss::Pvt d;
  TEST_ASSERT_TRUE(decodeNavPvt(p.payload(), p.payloadLen(), d));
  TEST_ASSERT_EQUAL_UINT32(123456789, d.iTOW);
  TEST_ASSERT_EQUAL(2026, d.year);
  TEST_ASSERT_EQUAL(11, d.month);
  TEST_ASSERT_EQUAL(15, d.day);
  TEST_ASSERT_EQUAL(12, d.hour);
  TEST_ASSERT_EQUAL(34, d.min);
  TEST_ASSERT_EQUAL(56, d.sec);
  TEST_ASSERT_EQUAL_UINT32(25, d.tAcc);
  TEST_ASSERT_EQUAL_INT32(-123456, d.nano);
  TEST_ASSERT_EQUAL(3, d.fixType);
  TEST_ASSERT_EQUAL_HEX8(0xE0, d.flags2);
  TEST_ASSERT_EQUAL(17, d.numSV);
  TEST_ASSERT_EQUAL_INT32(1397000000, d.lon);
  TEST_ASSERT_EQUAL_INT32(356000000, d.lat);
  TEST_ASSERT_EQUAL_INT32(45000, d.height);
  TEST_ASSERT_EQUAL_INT32(5000, d.hMSL);
  TEST_ASSERT_EQUAL_UINT32(1500, d.hAcc);
  TEST_ASSERT_EQUAL_UINT32(2500, d.vAcc);
  TEST_ASSERT_EQUAL_INT32(-1000, d.velN);
  TEST_ASSERT_EQUAL_INT32(2000, d.velE);
  TEST_ASSERT_EQUAL_INT32(-30, d.velD);
  TEST_ASSERT_EQUAL_INT32(2236, d.gSpeed);
  TEST_ASSERT_EQUAL_INT32(11656505, d.headMot);
  TEST_ASSERT_EQUAL_UINT32(80, d.sAcc);
  TEST_ASSERT_EQUAL_UINT32(150000, d.headAcc);
  TEST_ASSERT_EQUAL(123, d.pDOP);
  TEST_ASSERT_EQUAL(0, d.flags3);
  TEST_ASSERT_TRUE(d.dateTimeValid());
  TEST_ASSERT_TRUE(d.fixOk());
  TEST_ASSERT_TRUE(d.headingValid());  // fix + 2.2 m/s

  TEST_ASSERT_FALSE(decodeNavPvt(p.payload(), 91, d));
}

void test_pvt_predicates() {
  auto payload = navPvtPayload();
  gnss::Pvt d;
  decodeNavPvt(payload.data(), payload.size(), d);
  d.valid = 0x03;  // not fully resolved
  TEST_ASSERT_FALSE(d.dateTimeValid());
  d.fixType = 4;  // GNSS + DR
  TEST_ASSERT_TRUE(d.fixOk());
  d.fixType = 2;
  TEST_ASSERT_FALSE(d.fixOk());
  TEST_ASSERT_FALSE(d.headingValid());
  d.flags = 0x20;  // headVehValid alone
  TEST_ASSERT_TRUE(d.headingValid());
  d.fixType = 3;
  d.flags = 0x00;  // gnssFixOK missing
  TEST_ASSERT_FALSE(d.fixOk());
  d.flags = 0x01;
  d.gSpeed = gnss::kHeadingMinSpeedMmps;  // at threshold: not valid
  TEST_ASSERT_FALSE(d.headingValid());
}

void test_split_concatenated_garbage_and_bad_checksum() {
  auto payload = navPvtPayload();
  uint8_t f[128];
  const size_t n = buildFrame(kClsNav, kIdNavPvt, payload.data(), payload.size(), f, sizeof(f));
  std::vector<uint8_t> stream = {0x12, 0xB5, 0x13, 0xB5};  // garbage incl. false syncs
  stream.insert(stream.end(), f, f + n);
  std::vector<uint8_t> bad(f, f + n);
  bad[50] ^= 0xFF;
  stream.insert(stream.end(), bad.begin(), bad.end());
  const uint8_t ack[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8A, 0x98, 0xC1};
  stream.insert(stream.end(), ack, ack + sizeof(ack));
  stream.insert(stream.end(), f, f + n);

  Parser p;
  int frames = 0, acks = 0;
  for (uint8_t b : stream) {
    if (p.feed(b)) {
      frames++;
      if (p.msgClass() == kClsAck && p.msgId() == kIdAckAck) {
        acks++;
        TEST_ASSERT_EQUAL_HEX8(kClsCfg, p.payload()[0]);
        TEST_ASSERT_EQUAL_HEX8(kIdCfgValset, p.payload()[1]);
      }
    }
  }
  TEST_ASSERT_EQUAL(3, frames);  // good, ACK, good (bad checksum dropped)
  TEST_ASSERT_EQUAL(1, acks);
  TEST_ASSERT_EQUAL(1, p.checksumErrors());
}

void test_length_error() {
  const uint8_t huge[] = {0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF};
  Parser p;
  TEST_ASSERT_EQUAL(0, feedAll(p, huge, sizeof(huge)));
  TEST_ASSERT_EQUAL(1, p.lengthErrors());
}

void test_nmea_detector() {
  const char* s =
      "xx$GNRMC,,V,,,,,,,,,,N,V*37\r\n"
      "$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n"
      "$GNGGA,,,,,,0,00,99.99,,,,,,*57\r\n"  // bad checksum
      "$GN\x01GA*00\r\n";                     // non-printable
  NmeaDetector d;
  int n = 0;
  for (const char* c = s; *c; c++)
    if (d.feed(uint8_t(*c))) n++;
  TEST_ASSERT_EQUAL(2, n);
  TEST_ASSERT_EQUAL(2, d.sentences());
}

void runRecordTests();
void runMagCalTests();
void runCarframeTests();

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_build_frame_known_checksum);
  RUN_TEST(test_valset_rate);
  RUN_TEST(test_valset_mixed_sizes);
  RUN_TEST(test_valset_errors);
  RUN_TEST(test_nav_pvt_decode);
  RUN_TEST(test_pvt_predicates);
  RUN_TEST(test_split_concatenated_garbage_and_bad_checksum);
  RUN_TEST(test_length_error);
  RUN_TEST(test_nmea_detector);
  runRecordTests();
  runMagCalTests();
  runCarframeTests();
  return UNITY_END();
}
