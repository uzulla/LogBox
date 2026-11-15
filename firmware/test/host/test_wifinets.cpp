// Host tests for components/wifinets: the text formats shared by BLE, the serial console and the Web page.
#include <string.h>

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
  TEST_ASSERT_GREATER_THAN(0, wifinets::listJson(e, 2, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING(
      "{\"networks\":[{\"ssid\":\"high\",\"prio\":20,\"secured\":true},{\"ssid\":\"mid\\\"q\",\"prio\":5,\"secured\":false}]}",
      buf);
  TEST_ASSERT_GREATER_THAN(0, wifinets::listJson(e, 0, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_STRING("{\"networks\":[]}", buf);
  TEST_ASSERT_EQUAL(0, wifinets::listJson(e, 4, buf, 20));  // does not fit
}

void runWifinetsTests() {
  RUN_TEST(test_wifinets_parse_query);
  RUN_TEST(test_wifinets_parse_tab_and_errors);
  RUN_TEST(test_wifinets_sort_and_json);
}
