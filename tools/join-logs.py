#!/usr/bin/env python3
"""Joins the 5-minute parts of logbo SD logs into one CSV per recording.

    python3 tools/join-logs.py "/Volumes/NO NAME"            # writes log_<session>_<seq>.csv next to the parts
    python3 tools/join-logs.py "/Volumes/NO NAME" -o out/    # somewhere else

log_0123_01_001.csv, log_0123_01_002.csv, ... -> log_0123_01.csv (the name the firmware used before the split, so
log-viewer reads it as before). The first part keeps its `#` comments and header row; the later parts' are left out
(t_ms already counts from the start of the recording). Empty parts are skipped; a last line cut short by a power
loss (no newline at the end) is dropped. An existing output file is not overwritten unless --force.
"""
import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

PART = re.compile(r"^log_(\d{4})_(\d{2})_(\d{3})\.csv$")


def read_part(path):
    """Returns (comment lines, header row, data lines, dropped partial line?)."""
    text = path.read_bytes().decode("utf-8", errors="replace")
    lines = text.split("\n")
    partial = lines[-1] != ""  # no newline at the end: the last line may be cut short
    lines = lines[:-1]
    comments, header, rows = [], None, []
    for line in lines:
        if header is None:
            if line.startswith("#"):
                comments.append(line)
            elif line.startswith("t_ms,"):
                header = line
            else:
                raise ValueError(f"{path.name}: data before the header row")
        else:
            rows.append(line)
    return comments, header, rows, partial


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir", type=Path, help="folder with the log_<session>_<seq>_<part>.csv files (the SD card)")
    ap.add_argument("-o", "--out", type=Path, help="output folder (default: the same folder)")
    ap.add_argument("--force", action="store_true", help="overwrite existing output files")
    args = ap.parse_args()
    out = args.out or args.dir
    out.mkdir(parents=True, exist_ok=True)

    groups = defaultdict(list)
    for p in args.dir.iterdir():
        m = PART.match(p.name)
        if m:
            groups[(m[1], m[2])].append((int(m[3]), p))

    failed = False
    for (session, seq), parts in sorted(groups.items()):
        parts.sort()
        dest = out / f"log_{session}_{seq}.csv"
        nonempty = [(n, p) for n, p in parts if p.stat().st_size > 0]
        if not nonempty:
            print(f"{dest.name}: skipped (all {len(parts)} parts empty)")
            continue
        if dest.exists() and not args.force:
            print(f"{dest.name}: skipped (exists; --force to overwrite)")
            continue
        missing = sorted(set(range(1, parts[-1][0] + 1)) - {n for n, _ in parts})
        try:
            head, total, notes = None, 0, []
            body = []
            for n, p in nonempty:
                comments, header, rows, partial = read_part(p)
                if header is None:
                    notes.append(f"part {n} has no header row, skipped")
                    continue
                if head is None:
                    head = (comments, header)
                elif header != head[1]:
                    raise ValueError(f"{p.name}: columns differ from the first part")
                if partial:
                    notes.append(f"part {n}: cut-short last line dropped")
                body.extend(rows)
                total += len(rows)
            if head is None:
                print(f"{dest.name}: skipped (no part with a header row)")
                continue
        except (OSError, ValueError) as e:
            print(f"{dest.name}: FAILED ({e})", file=sys.stderr)
            failed = True
            continue
        comments, header = head
        comments = comments + [f"# joined: parts {nonempty[0][0]}-{nonempty[-1][0]} (tools/join-logs.py)"]
        tmp = dest.with_name(dest.name + ".tmp")
        with tmp.open("w", newline="\n") as f:
            f.write("\n".join(comments + [header] + body) + "\n")
        tmp.replace(dest)
        if missing:
            notes.append(f"missing parts {missing}")
        extra = f" ({'; '.join(notes)})" if notes else ""
        print(f"{dest.name}: {len(nonempty)} parts, {total} rows{extra}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
