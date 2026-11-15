// Wi-Fi networks the logger may join (STA): up to kMax entries with a priority, tried from the highest
// priority down until one gives an IP. Stored in NVS namespace "wifi" as ssid<i> / pass<i> / prio<i>
// (i = 0..kMax-1; the single "ssid" / "pass" of older firmware is moved into slot 0 on the first load).
//
// The text formats are shared by the BLE wifi characteristic, the serial console and the Web page:
//   "ssid=HomeAP&pass=secret&prio=10"  add, or update the entry with the same SSID (values URL-encoded,
//                                      pass may be left out for an open network, prio defaults to 0)
//   "ssid=HomeAP&del=1"                delete
//   "<ssid>\t<password>"               add / update with priority 0 (the format of older firmware)
// The list is shown as {"networks":[{"ssid":"HomeAP","prio":10,"secured":true},...]} (never the passwords).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wifinets {

constexpr int kMax = 8;

struct Entry {
  char ssid[33];
  char pass[65];
  int32_t prio;  // higher first
};

// ---- pure (host-tested) ----

enum class Op : uint8_t { kSet, kDelete };
enum class ParseError : uint8_t { kNone, kNoSsid, kTooLong };
struct Command {
  Op op;
  Entry entry;  // ssid always; pass / prio for kSet
};
// Parses one of the write formats above.
ParseError parse(const char* text, Command& out);
// Sorts by priority, highest first (stable: equal priorities keep their slot order).
void sortByPriority(Entry* entries, int n);
// The list JSON (no passwords). Returns the length, 0 if it did not fit.
size_t listJson(const Entry* entries, int n, char* buf, size_t size);

// ---- NVS (on the device) ----

enum class StoreError : uint8_t { kNone, kFull, kNotFound, kNvs };
// Entries in slot order; returns how many.
int load(Entry* out);
StoreError set(const Entry& e);      // same SSID: updated in place; otherwise the first free slot
StoreError remove(const char* ssid);
StoreError clear();

}  // namespace wifinets
