#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Manage the Wi-Fi networks in the ESP32's NVS over USB serial (UART0 console).

Up to 8 networks; the logger tries them from the highest priority down. The password
is read with getpass and only sent to the device, so it never ends up in the source
tree or the firmware image.

    uv run tools/set-wifi.py [--port /dev/ttyUSB0] [--prio 10]   # add / update (prompts)
    uv run tools/set-wifi.py --list
    uv run tools/set-wifi.py --delete HomeAP
    uv run tools/set-wifi.py --clear                             # delete all
"""
import argparse
import getpass
import sys
import time

import serial


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--prio", type=int, default=0, help="priority of the network being added (higher first)")
    ap.add_argument("--list", action="store_true", help="show the networks")
    ap.add_argument("--delete", metavar="SSID", help="delete a network")
    ap.add_argument("--clear", action="store_true", help="delete all networks")
    args = ap.parse_args()

    if args.clear:
        cmd = "wifi-clear"
    elif args.list:
        cmd = "wifi-list"
    elif args.delete:
        if any(c in args.delete for c in "\r\n\t"):
            # One line is one console command: a newline would send a second command (e.g. wifi-clear)
            print("the SSID must not contain newlines or tabs", file=sys.stderr)
            return 1
        cmd = f"wifi-del\t{args.delete}"
    else:
        ssid = input("SSID: ")
        password = getpass.getpass("Password: ")
        if not ssid or "\t" in ssid + password or "\n" in ssid + password:
            print("SSID must be non-empty; tabs and newlines are not allowed", file=sys.stderr)
            return 1
        # The priority field only when given: firmware before the multi-network change would take it as part
        # of the password
        cmd = f"wifi-set\t{ssid}\t{password}" + (f"\t{args.prio}" if args.prio else "")

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.2
    # The M5Stack's CP2104 drives EN from RTS and GPIO0 from DTR (auto-reset circuit):
    # keep both released so opening the port neither resets the ESP32 nor selects the bootloader.
    ser.rts = False
    ser.dtr = False
    try:
        ser.open()
    except serial.SerialException as e:
        print(f"{e}\nRun this on the machine the ESP32 is plugged into.", file=sys.stderr)
        return 1
    # Some hosts (the Linux cp210x driver) still pulse the lines on open and reset the ESP32 anyway:
    # wait until it has booted (the console is up a second after reset) before sending the command.
    time.sleep(2.5)
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())

    deadline = time.time() + 5
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("OK") or line.startswith("ERR") or line.startswith('{"networks"'):
            print(line)
            return 1 if line.startswith("ERR") else 0
    print("no response from the device", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
