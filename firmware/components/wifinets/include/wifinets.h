// Wi-Fi networks the logger may join (STA): up to kMax entries with a priority, tried from the highest
// priority down until one gives an IP. Stored in NVS namespace "wifi" as ssid<i> / pass<i> / prio<i>
// (i = 0..kMax-1; the single "ssid" / "pass" of older firmware is moved into slot 0 on the first load).
//
// The text formats are shared by the BLE wifi characteristic, the serial console and the Web page:
//   "ssid=HomeAP&pass=secret&prio=10"  add, or update the entry with the same SSID (values URL-encoded,
//                                      pass may be left out for an open network, prio defaults to 0)
//   "ssid=HomeAP&del=1"                delete
//   "<ssid>\t<password>"               add / update with priority 0 (the format of older firmware)
// The list is shown as {"networks":[{"ssid":"HomeAP","prio":10,"secured":true},...],"total":3,"next":null,"gen":7}
// (never the passwords), in pages that fit one BLE attribute (512 bytes): "next" is the index to ask for the
// rest from (write "list=<index>&gen=<gen>" to the BLE wifi characteristic, then read again), null on the last
// page. "gen" changes whenever the list changes: a page request with an old gen is refused, and a client that sees
// a different gen on a later page throws the earlier pages away and starts over.
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
enum class ParseError : uint8_t { kNone, kNoSsid, kTooLong, kInvalid };  // kInvalid: NUL byte, bad priority
struct Command {
  Op op;
  Entry entry;  // ssid always; pass / prio for kSet
};
// Parses one of the write formats above.
ParseError parse(const char* text, Command& out);
// Sorts by priority, highest first (stable: equal priorities keep their slot order).
void sortByPriority(Entry* entries, int n);
// One page of the list JSON (no passwords): the entries from `start` on, as many as fit in `size` bytes (at
// least one). Returns the length, 0 if not even one entry fit.
size_t listJson(const Entry* entries, int n, int start, uint32_t gen, char* buf, size_t size);
// "list=<index>" or "list=<index>&gen=<gen>" (the BLE request for a later page): true, the index and the gen
// (*hasGen false without one), else false.
bool parseListRequest(const char* text, int* start, uint32_t* gen, bool* hasGen);

// ---- NVS (on the device) ----
// All of them are serialised (a mutex), so the BLE task and the serial console can't pick the same free slot.

enum class StoreError : uint8_t { kNone, kFull, kNotFound, kNvs };
// Entries in slot order; returns how many.
int load(Entry* out);
int count();                         // how many entries (no buffer needed)
StoreError set(const Entry& e);      // same SSID: updated in place; otherwise the first free slot
StoreError remove(const char* ssid);
StoreError clear();

}  // namespace wifinets
