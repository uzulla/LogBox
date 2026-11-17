// Logs the records (one per GNSS epoch) to a FAT32 microSD card as CSV (record::csvHeader / csvRow:
// `#` comment lines, a header row, one row per record), on the M5Stack's TF slot.
//
// Recording starts by itself at boot if a card is present. No card, a card that cannot be mounted
// (exFAT, ...) or a full / failing card means no recording, and nothing is retried by itself. A long
// press of the button stops recording and unmounts the card (once state() is Safe it may be removed);
// a long press in any other state tries to start recording.
#pragma once

#include <stdint.h>

#include "record.h"

namespace sdlog {

enum class State : uint8_t {
  Off,     // no card (or no slot on this board)
  Rec,     // recording
  Saving,  // stop requested, flushing; do not remove the card
  Safe,    // stopped and unmounted; the card may be removed
  Failed,  // disk full or write error; recording stopped
};

// Initialises the SPI bus (shared with the LCD, so call it before logger::displayStart()), mounts the
// card and starts recording.
void init();
// Queues one record; never blocks (dropped when the queue is full).
void write(const record::Record& r);
// Stops recording and unmounts the card (returns at once; state() goes Saving, then Safe).
void requestStop();
// From Safe, Failed or Off (no card at boot): mounts the card and records to the next file
// (log_<session>_02.csv, ...). This is how a card inserted later, or after SD ERR, is used.
void requestStart();
State state();
// Name of the current / last log file ("log_0012_01.csv"), "" if none yet.
const char* fileName();

// Appends one CSV line (without the newline) to boot_log.csv on the card: the history of boots, resets and
// sensor trouble, kept apart from the log files. Columns: see kEventHeader in sdlog.cpp. The card is only
// written while recording, so lines wait in memory meanwhile (the oldest are dropped beyond 16).
void event(const char* line);

}  // namespace sdlog
