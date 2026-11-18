#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["bleak"]
# ///
"""Talk to the ESP32's BLE config service (advertised whenever the logger is on).

    uv run tools/ble-config.py status      # GNSS / SD state, Wi-Fi SSID, connected?, IP
    uv run tools/ble-config.py wifi-set    # prompts for SSID, password and priority; adds / updates a network
    uv run tools/ble-config.py wifi-list   # the networks (up to 8, tried from the highest priority)
    uv run tools/ble-config.py wifi-del HomeAP
    uv run tools/ble-config.py mag         # magnetometer calibration state and the AP / STA switches
    uv run tools/ble-config.py ctrl rec=0  # a command: rec=1|0, mag=start|done|cancel|clear, wifi=clear,
                                           #   ap=0|1, sta=0|1 (from the next boot),
                                           #   bl=1..5 (backlight), reboot=1

The same service is used by the Web Bluetooth page docs/index.html.

The link is encrypted (Just Works pairing, nothing stored): the device asks
the OS to pair as soon as it connects.
"""
import argparse
import asyncio
import getpass
import json
import sys
from urllib.parse import quote

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

SERVICE = "9b3c0001-4e5f-4a2b-8c6d-2f1e0a7b5c3d"
STATUS = "9b3c0002-4e5f-4a2b-8c6d-2f1e0a7b5c3d"
WIFI = "9b3c0003-4e5f-4a2b-8c6d-2f1e0a7b5c3d"
CTRL = "9b3c0004-4e5f-4a2b-8c6d-2f1e0a7b5c3d"


async def find(name, timeout):
    print(f"scanning for up to {timeout:.0f} s...",
          file=sys.stderr)
    found = {}

    def match(d, adv):
        if SERVICE not in [u.lower() for u in adv.service_uuids]:
            return False
        # Prefer the advertised name (from the scan response, which may not have
        # arrived yet): macOS may show a name cached from earlier firmware.
        found["name"] = adv.local_name or d.name
        return name is None or adv.local_name == name

    dev = await BleakScanner.find_device_by_filter(match, timeout=timeout)
    if dev is None:
        sys.exit("device not found")
    print(f"found {found['name']} ({dev.address})", file=sys.stderr)
    return dev


async def when_encrypted(op, timeout=30):
    """Run op, retrying while pairing is in progress (macOS asks the user to confirm it)."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while True:
        try:
            return await op()
        except BleakError as e:
            if "Encryption" not in str(e) or loop.time() > deadline:
                raise
            await asyncio.sleep(0.5)


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["status", "wifi-set", "wifi-list", "wifi-del", "mag", "ctrl"])
    ap.add_argument("value", nargs="?", help="ctrl: the command, e.g. rec=0; wifi-del: the SSID")
    ap.add_argument("--name", help="advertised name, e.g. logbo-06FA (default: first found)")
    ap.add_argument("--timeout", type=float, default=30)
    args = ap.parse_args()

    if args.command == "wifi-set":
        ssid = input("SSID: ")
        password = getpass.getpass("Password: ")
        try:
            prio = int(input("Priority (higher first) [0]: ").strip() or "0")
        except ValueError:
            prio = None
        if not ssid or prio is None:
            print("SSID must be non-empty and the priority a number", file=sys.stderr)
            return 1
        query = f"ssid={quote(ssid, safe='')}&pass={quote(password, safe='')}&prio={prio}"

    if args.command in ("ctrl", "wifi-del") and not args.value:
        print(f"{args.command} needs a value, e.g. {'rec=0' if args.command == 'ctrl' else 'HomeAP'}", file=sys.stderr)
        return 1

    dev = await find(args.name, args.timeout)
    async with BleakClient(dev) as client:
        if args.command in ("status", "mag"):
            uuid = STATUS if args.command == "status" else CTRL
            data = await when_encrypted(lambda: client.read_gatt_char(uuid))
            print(json.dumps(json.loads(data), indent=2, ensure_ascii=False))
        elif args.command == "wifi-list":
            # Pages of at most 512 bytes: "next" is where to continue ("list=<next>&gen=<gen>", then read
            # again). If the list changes meanwhile (another client, the serial console), "gen" changes or the
            # request is refused: start over.
            for _attempt in range(4):
                networks = []
                page = json.loads(await when_encrypted(lambda: client.read_gatt_char(WIFI)))
                gen = page.get("gen")
                restart = False
                while True:
                    if page.get("gen") != gen:
                        restart = True
                        break
                    networks += page["networks"]
                    if page.get("next") is None:
                        break
                    req = f"list={page['next']}" + (f"&gen={gen}" if gen is not None else "")
                    try:
                        await client.write_gatt_char(WIFI, req.encode(), response=True)
                    except BleakError:
                        restart = True
                        break
                    page = json.loads(await client.read_gatt_char(WIFI))
                if "next" in page:
                    await client.write_gatt_char(WIFI, b"list=0", response=True)
                if not restart:
                    break
            else:
                print("the list kept changing while being read", file=sys.stderr)
                return 1
            print(json.dumps({"networks": networks}, indent=2, ensure_ascii=False))
        elif args.command == "ctrl":
            try:
                await when_encrypted(lambda: client.write_gatt_char(CTRL, args.value.encode(), response=True))
            except BleakError as e:
                print(f"rejected by the device: {e}", file=sys.stderr)
                return 1
            print(f"OK {args.value}")
        elif args.command == "wifi-del":
            try:
                await when_encrypted(lambda: client.write_gatt_char(
                    WIFI, f"ssid={quote(args.value, safe='')}&del=1".encode(), response=True))
            except BleakError as e:
                print(f"rejected by the device (no such network?): {e}", file=sys.stderr)
                return 1
            print("OK wifi deleted")
        else:
            try:
                await when_encrypted(
                    lambda: client.write_gatt_char(WIFI, query.encode(), response=True))
            except BleakError as e:
                print(f"rejected by the device: {e}", file=sys.stderr)
                return 1
            print("OK wifi saved")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
