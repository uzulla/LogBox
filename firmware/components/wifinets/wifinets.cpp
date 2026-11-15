#include "wifinets.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace wifinets {
namespace {

int hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Value of key in "a=1&b=2" (URL-decoded) into out. False if the key is absent or the value does not fit.
bool queryValue(const char* q, const char* key, char* out, size_t size, bool* present) {
  const size_t klen = strlen(key);
  *present = false;
  for (const char* p = q; *p;) {
    const char* end = strchr(p, '&');
    if (!end) end = p + strlen(p);
    if (static_cast<size_t>(end - p) > klen && !strncmp(p, key, klen) && p[klen] == '=') {
      *present = true;
      size_t o = 0;
      for (const char* s = p + klen + 1; s < end; s++) {
        char c = *s;
        if (c == '+') {
          c = ' ';
        } else if (c == '%' && s + 2 < end && hex(s[1]) >= 0 && hex(s[2]) >= 0) {
          c = static_cast<char>(hex(s[1]) * 16 + hex(s[2]));
          s += 2;
          if (!c) return false;  // %00 would cut the value short
        }
        if (o + 1 >= size) return false;
        out[o++] = c;
      }
      out[o] = '\0';
      return true;
    }
    p = *end ? end + 1 : end;
  }
  return false;
}

}  // namespace

ParseError parse(const char* text, Command& out) {
  out = {};
  out.op = Op::kSet;
  // Older firmware's "<ssid>\t<password>"
  if (const char* tab = strchr(text, '\t')) {
    const size_t sl = static_cast<size_t>(tab - text), pl = strlen(tab + 1);
    if (sl == 0) return ParseError::kNoSsid;
    if (sl > 32 || pl > 64) return ParseError::kTooLong;
    memcpy(out.entry.ssid, text, sl);
    memcpy(out.entry.pass, tab + 1, pl);
    return ParseError::kNone;
  }
  bool present;
  if (strstr(text, "%00")) return ParseError::kInvalid;
  if (!queryValue(text, "ssid", out.entry.ssid, sizeof(out.entry.ssid), &present)) {
    return present ? ParseError::kTooLong : ParseError::kNoSsid;
  }
  if (!out.entry.ssid[0]) return ParseError::kNoSsid;
  char v[16];
  if (queryValue(text, "del", v, sizeof(v), &present) && atoi(v)) {
    out.op = Op::kDelete;
    return ParseError::kNone;
  }
  if (!queryValue(text, "pass", out.entry.pass, sizeof(out.entry.pass), &present) && present) {
    return ParseError::kTooLong;
  }
  if (queryValue(text, "prio", v, sizeof(v), &present)) {
    char* end;
    const long p = strtol(v, &end, 10);
    if (!v[0] || *end || p < -1000000 || p > 1000000) return ParseError::kInvalid;
    out.entry.prio = static_cast<int32_t>(p);
  } else if (present) {
    return ParseError::kInvalid;  // too long to be a number
  }
  return ParseError::kNone;
}

void sortByPriority(Entry* e, int n) {
  for (int i = 1; i < n; i++) {  // insertion sort: stable, n <= kMax
    const Entry x = e[i];
    int j = i - 1;
    while (j >= 0 && e[j].prio < x.prio) {
      e[j + 1] = e[j];
      j--;
    }
    e[j + 1] = x;
  }
}

namespace {

// {"ssid":"...","prio":N,"secured":B} into buf; returns its length (0 if it did not fit)
size_t entryJson(const Entry& e, char* buf, size_t size) {
  char ssid[33 * 6];
  size_t s = 0;
  for (const char* p = e.ssid; *p && s + 7 < sizeof(ssid); p++) {
    const unsigned char c = static_cast<unsigned char>(*p);
    if (c == '"' || c == '\\') {
      ssid[s++] = '\\';
      ssid[s++] = static_cast<char>(c);
    } else if (c < 0x20) {
      s += snprintf(ssid + s, sizeof(ssid) - s, "\\u%04x", c);
    } else {
      ssid[s++] = static_cast<char>(c);
    }
  }
  ssid[s] = '\0';
  const int k = snprintf(buf, size, "{\"ssid\":\"%s\",\"prio\":%ld,\"secured\":%s}", ssid, static_cast<long>(e.prio),
                         e.pass[0] ? "true" : "false");
  return k > 0 && static_cast<size_t>(k) < size ? static_cast<size_t>(k) : 0;
}

}  // namespace

size_t listJson(const Entry* e, int n, int start, char* buf, size_t size) {
  if (start < 0) start = 0;
  if (start > n) start = n;
  // Room kept for the tail: ],"total":8,"next":8}
  constexpr size_t kTail = 32;
  int k = snprintf(buf, size, "{\"networks\":[");
  if (k < 0 || static_cast<size_t>(k) + kTail >= size) return 0;
  size_t o = static_cast<size_t>(k);
  int i = start;
  for (; i < n; i++) {
    char item[33 * 6 + 64];
    const size_t len = entryJson(e[i], item, sizeof(item));
    const size_t need = len + (i > start ? 1 : 0);
    if (!len || o + need + kTail >= size) break;
    if (i > start) buf[o++] = ',';
    memcpy(buf + o, item, len);
    o += len;
  }
  if (i == start && start < n) return 0;  // not even one entry fits
  char next[12] = "null";
  if (i < n) snprintf(next, sizeof(next), "%d", i);
  k = snprintf(buf + o, size - o, "],\"total\":%d,\"next\":%s}", n, next);
  return k > 0 && o + static_cast<size_t>(k) < size ? o + static_cast<size_t>(k) : 0;
}

bool parseListRequest(const char* text, int* start) {
  if (strncmp(text, "list=", 5) || !text[5]) return false;
  char* end;
  const long v = strtol(text + 5, &end, 10);
  if (*end || v < 0 || v > kMax) return false;
  *start = static_cast<int>(v);
  return true;
}

}  // namespace wifinets
