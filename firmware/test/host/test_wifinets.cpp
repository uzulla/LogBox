// Host tests for components/wifinets: the text formats shared by BLE, the serial console and the Web page.
#include <stdlib.h>
#include <string.h>

#include <string>

#include "unity.h"
#include "wifinets.h"

using wifinets::Command;
using wifinets::Op;
using wifinets::ParseError;

void test_wifinets_parse_query() {
  Command c;
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNone),
                    static_cast<int>(wifinets::parse("ssid=Home%20AP&pass=p%26ss%3Dw+d&prio=10", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(Op::kSet), static_cast<int>(c.op));
  TEST_ASSERT_EQUAL_STRING("Home AP", c.entry.ssid);
  TEST_ASSERT_EQUAL_STRING("p&ss=w d", c.entry.pass);
  TEST_ASSERT_EQUAL(10, c.entry.prio);
  // open network, default priority
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNone), static_cast<int>(wifinets::parse("ssid=Cafe", c)));
  TEST_ASSERT_EQUAL_STRING("", c.entry.pass);
  TEST_ASSERT_EQUAL(0, c.entry.prio);
  // negative priority
  wifinets::parse("prio=-5&ssid=X", c);
  TEST_ASSERT_EQUAL(-5, c.entry.prio);
  // delete
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNone), static_cast<int>(wifinets::parse("ssid=Old&del=1", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(Op::kDelete), static_cast<int>(c.op));
  TEST_ASSERT_EQUAL_STRING("Old", c.entry.ssid);
}

void test_wifinets_parse_tab_and_errors() {
  Command c;
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNone), static_cast<int>(wifinets::parse("Home\tsecret", c)));
  TEST_ASSERT_EQUAL_STRING("Home", c.entry.ssid);
  TEST_ASSERT_EQUAL_STRING("secret", c.entry.pass);
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNoSsid), static_cast<int>(wifinets::parse("\tsecret", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNoSsid), static_cast<int>(wifinets::parse("pass=x", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNoSsid), static_cast<int>(wifinets::parse("ssid=&pass=x", c)));
  char longSsid[64] = "ssid=";
  memset(longSsid + 5, 'a', 33);
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kTooLong), static_cast<int>(wifinets::parse(longSsid, c)));
  char longPass[128] = "ssid=a&pass=";
  memset(longPass + 12, 'b', 65);
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kTooLong), static_cast<int>(wifinets::parse(longPass, c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kInvalid), static_cast<int>(wifinets::parse("ssid=a%00b", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kInvalid), static_cast<int>(wifinets::parse("ssid=a&prio=1x", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kInvalid), static_cast<int>(wifinets::parse("ssid=a&prio=", c)));
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kInvalid),
                    static_cast<int>(wifinets::parse("ssid=a&prio=99999999999999999999", c)));
  // 32-byte SSID written as %XX escapes still fits
  char enc[160] = "ssid=";
  for (int i = 0; i < 32; i++) strcat(enc, "%41");
  TEST_ASSERT_EQUAL(static_cast<int>(ParseError::kNone), static_cast<int>(wifinets::parse(enc, c)));
  TEST_ASSERT_EQUAL(32, static_cast<int>(strlen(c.entry.ssid)));
}

void test_wifinets_sort_and_json() {
  wifinets::Entry e[4] = {};
  strcpy(e[0].ssid, "low"), e[0].prio = 1;
  strcpy(e[1].ssid, "high"), e[1].prio = 20, strcpy(e[1].pass, "x");
  strcpy(e[2].ssid, "mid\"q"), e[2].prio = 5;
  strcpy(e[3].ssid, "low2"), e[3].prio = 1;
  wifinets::sortByPriority(e, 4);
  TEST_ASSERT_EQUAL_STRING("high", e[0].ssid);
  TEST_ASSERT_EQUAL_STRING("mid\"q", e[1].ssid);
  TEST_ASSERT_EQUAL_STRING("low", e[2].ssid);  // stable
  TEST_ASSERT_EQUAL_STRING("low2", e[3].ssid);
  char buf[512];
  TEST_ASSERT_GREATER_THAN(0, wifinets::listJson(e, 2, 0, 7, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING(
      "{\"networks\":[{\"ssid\":\"high\",\"prio\":20,\"secured\":true},{\"ssid\":\"mid\\\"q\",\"prio\":5,\"secured\":false}],"
      "\"total\":2,\"next\":null,\"gen\":7}",
      buf);
  TEST_ASSERT_GREATER_THAN(0, wifinets::listJson(e, 0, 0, 7, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING("{\"networks\":[],\"total\":0,\"next\":null,\"gen\":7}", buf);
  TEST_ASSERT_EQUAL(0, wifinets::listJson(e, 4, 0, 7, buf, 40));  // not even one entry fits
}

// 8 networks with 32-character SSIDs that all need escaping do not fit one 512-byte attribute: the pages
// together hold every entry, each page fits, and "next" chains them.
void test_wifinets_list_pages() {
  wifinets::Entry e[wifinets::kMax] = {};
  for (int i = 0; i < wifinets::kMax; i++) {
    memset(e[i].ssid, '"', 31);
    e[i].ssid[31] = static_cast<char>('A' + i);
    e[i].prio = -1000000 + i;
    strcpy(e[i].pass, "x");
  }
  int start = 0, seen = 0, pages = 0;
  char buf[512];
  while (true) {
    const size_t n = wifinets::listJson(e, wifinets::kMax, start, 3, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, n);
    TEST_ASSERT_LESS_THAN(512, n);
    pages++;
    for (const char* p = buf; (p = strstr(p, "\"prio\"")); p++) seen++;
    const char* next = strstr(buf, "\"next\":");
    TEST_ASSERT_NOT_NULL(next);
    if (!strncmp(next + 7, "null", 4)) break;
    uint32_t gen;
    bool hasGen;
    TEST_ASSERT_TRUE(wifinets::parseListRequest(
        (std::string("list=") + std::to_string(atoi(next + 7)) + "&gen=3").c_str(), &start, &gen, &hasGen));
    TEST_ASSERT_TRUE(hasGen);
    TEST_ASSERT_EQUAL_UINT32(3, gen);
  }
  TEST_ASSERT_EQUAL(wifinets::kMax, seen);
  TEST_ASSERT_GREATER_THAN(1, pages);
  int k;
  uint32_t g;
  bool hg;
  TEST_ASSERT_TRUE(wifinets::parseListRequest("list=2", &k, &g, &hg));  // older clients: no gen
  TEST_ASSERT_FALSE(hg);
  TEST_ASSERT_EQUAL(2, k);
  TEST_ASSERT_FALSE(wifinets::parseListRequest("list=", &k, &g, &hg));
  TEST_ASSERT_FALSE(wifinets::parseListRequest("list=9", &k, &g, &hg));
  TEST_ASSERT_FALSE(wifinets::parseListRequest("list=1x", &k, &g, &hg));
  TEST_ASSERT_FALSE(wifinets::parseListRequest("list=1&gen=", &k, &g, &hg));
  TEST_ASSERT_FALSE(wifinets::parseListRequest("list=1&gen=5x", &k, &g, &hg));
  TEST_ASSERT_FALSE(wifinets::parseListRequest("ssid=a", &k, &g, &hg));
}

void runWifinetsTests() {
  RUN_TEST(test_wifinets_parse_query);
  RUN_TEST(test_wifinets_parse_tab_and_errors);
  RUN_TEST(test_wifinets_sort_and_json);
  RUN_TEST(test_wifinets_list_pages);
}
