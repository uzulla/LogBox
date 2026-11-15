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
  if (queryValue(text, "prio", v, sizeof(v), &present)) out.entry.prio = atoi(v);
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

size_t listJson(const Entry* e, int n, char* buf, size_t size) {
  size_t o = 0;
  auto put = [&](const char* fmt, auto... args) {
    if (o >= size) return;
    const int k = snprintf(buf + o, size - o, fmt, args...);
    o = k < 0 ? size : o + static_cast<size_t>(k);
  };
  put("{\"networks\":[");
  for (int i = 0; i < n; i++) {
    char ssid[33 * 6];
    size_t s = 0;
    for (const char* p = e[i].ssid; *p && s + 7 < sizeof(ssid); p++) {
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
    put("%s{\"ssid\":\"%s\",\"prio\":%ld,\"secured\":%s}", i ? "," : "", ssid, static_cast<long>(e[i].prio),
        e[i].pass[0] ? "true" : "false");
  }
  put("]}");
  return o < size ? o : 0;
}

}  // namespace wifinets
