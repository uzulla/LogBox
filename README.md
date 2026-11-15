# logbo

Stand-alone GNSS / IMU logger: M5Stack (Gray / Basic) + [M5Stack GNSS module](https://docs.m5stack.com/en/module/GNSS%20Module)
(u-blox NEO-M9N, BMI270 + BMM150, BMP280). Logs to a microSD card as CSV and broadcasts JSON over Wi-Fi UDP.
Forked from esp32-racebox-bridge (RaceBox Mini over BLE); the RaceBox is no longer needed.

```
GNSS  NEO-M9N  --UART 115200, UBX NAV-PVT 25 Hz-->  record (one per epoch)  --> SD card CSV
IMU   BMI270 (+ BMM150 on its aux bus), BMP280  --I2C-->  (mean over the epoch)  \-> Wi-Fi UDP JSON
```

## Hardware

| | |
|---|---|
| M5Stack Gray (ESP32-D0WDQ6, 4 MB flash, no PSRAM) | console on UART0 (CP2104, `/dev/ttyUSB0`). The Gray's own MPU9250 (I2C 0x68) is not used |
| GNSS module DIP switches | **RX block 1 ON (NEO TX -> G16), TX block 1 ON (NEO RX <- G17)**, everything else OFF. PPS is not used |
| I2C (G21/G22) | BMI270 0x69, BMP280 0x76, BMM150 behind the BMI270 (aux I2C) |
| LCD ILI9342C 320x240 + TF slot | share VSPI (SCLK 18, MOSI 23, MISO 19); CS LCD 14, TF 4; LCD DC 27, RST 33, BL 32 |
| Buttons | A G39, B G38, C G37 |

The battery gauge (IP5306) of this unit does not answer on I2C, so `batt` / `charging` stay empty.

## Firmware (`firmware/`, ESP-IDF v6.1, target esp32)

| component | |
|---|---|
| `gnss` | finds the receiver's baud, configures it with UBX-CFG-VALSET (115200, UBX only, NAV-PVT every epoch, `LOGBO_GNSS_RATE_HZ` = 25 Hz, automotive model), recovers by itself after a receiver reset; falls back to 10 Hz if 25 Hz is rejected or not sustained |
| `imu` | BMI270 FIFO at 200 Hz (±8 g, ±2000 dps), BMM150 at 25 Hz through the BMI270 aux interface (Bosch compensation, then hard / soft iron calibration from NVS), BMP280 at ~26 Hz |
| `record` | one record = NAV-PVT + IMU mean over the epoch; CSV row and UDP JSON formatting (host-tested) |
| `sdlog` | writes the records to `log_<session>_<seq>.csv` on the card |
| `display` | LCD UI and buttons |
| `main` | wiring, Wi-Fi (STA + soft AP), UDP, BLE config service, serial console |

While the GNSS is silent (no message for 1 s) records are still made at the configured rate, IMU only (fix 0, empty UTC).

```sh
mise run fw:flash       # sync to $BUILD_HOST (wvx), build, flash $SERIAL_PORT (/dev/ttyUSB0)
mise run fw:monitor
mise run fw:test        # host unit tests (UBX parser / CFG-VALSET builder / NAV-PVT / mag calibration / CSV / JSON)
mise run fw:ap          # soft-AP SSID / password of the last build
```

### Screen and buttons

- A / C: previous / next page. Pages: **SPEED** (km/h, total horizontal G bar), **G** (G circle with a 2 s trail and peaks;
  short press B resets the peaks), **STATUS** (GNSS fix / satellites / rate, position, accel, baro, mag and its
  calibration, SD, Wi-Fi; short press B starts the magnetometer calibration, below).
- **Hold B for 1.5 s** to stop recording: the top bar shows `SAVING` then `SAFE`; only then pull the card. Holding B again
  (from `SAFE`, `SD ERR` or `NO SD` after inserting a card) records to the next file.
- Top bar: SD state (blinking dot while recording), GNSS fix and satellites, Wi-Fi bars (`BT` while the BLE config
  window is open), time (JST, from GNSS).
- The G values need a reference for gravity: they appear after the unit has been still for about a second. Lateral is
  the screen's left-right axis (BMI270 -X), longitudinal is perpendicular to it in the horizontal plane. They show the G
  the driver feels, like a ball in a bowl: braking moves the ball up (forward), a left turn to the right, and tilting
  the unit rolls it towards the side that goes down. The CSV / UDP keep the raw sensor values.

### Magnetometer calibration

The magnets in the M5Stack's base move with the sensor and add a large constant offset (~200 uT) to the BMM150's
readings; the field itself is ~45 uT. To remove it (hard iron, plus a scale per axis for soft iron):

1. On the **STATUS** page press B. The calibration page shows how far each axis has swung.
2. Rotate the unit slowly in every direction, turning it over too, away from steel and magnets (desk frames, cars'
   bodies count: do it where it will be used, or at least outside).
3. When all three bars are full it finishes by itself after about 15 s of samples, or press B. A / C cancel; it gives up
   after 2 minutes.

The result shows on the STATUS page (`CAL ON 46UT NOW 47`: field strength of the calibration and of the current
reading, which should stay close to it in any orientation). It is saved to NVS (kept across reboots and reflashing) and
applied to every reading; the CSV column `mag_cal` / JSON `mag_cal` say whether a reading is calibrated. Redo it after
changing the stack (another base, battery, module). Serial console: `mag-cal`, `mag-cal-done`, `mag-clear`, `mag`.

### SD log

A FAT32 microSD card present at boot is recorded to at once. The session number counts up per boot (NVS) and skips
numbers already on the card; flushed every second. No card, an unreadable card, a full card or a write error: no
recording, and nothing is retried by itself (hold B).

```
t_ms,utc,fix,sats,lat,lon,alt_m,speed_kmh,heading_deg,hacc_m,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,batt,charging,mag_x,mag_y,mag_z,press_hpa,temp_c,mag_cal,head_acc_deg,speed_acc_kmh
```

The columns up to `charging` are those of the RaceBox logs, so [`log-viewer/`](log-viewer/README.md) reads both. acc is
in G and gyro in deg/s, BMI270 axes (mean over the epoch); mag in uT (BMM150 axes; calibrated when `mag_cal` is 1, raw
with the base magnets' offset when 0, see above); press in hPa, temp in °C (BMP280, warmed by the stack);
head_acc / speed_acc: the receiver's accuracy estimates of heading (deg) and speed (km/h), used by log-viewer to pick the
stretches it trusts for the body heading / slip angle.

### Soft AP

The Wi-Fi runs as AP+STA. The STA joins the configured network (below); the AP `LOGBO-XXXXXX` lets a phone or PC
connect directly when there is no router (IP 192.168.4.1, DHCP). While a station is connected the UDP datagrams are also
broadcast to `192.168.4.255`. The SSID suffix and the WPA2 password are random and **change on every build**: they are
generated into `build/esp-idf/main/ap_credentials.h` (never committed) and logged at boot.

### Wi-Fi networks

Up to 8 networks (SSID, password, priority), tried from the highest priority down; a network that does not give an IP
within 30 s (or fails: wrong password, not found) hands over to the next one, and after all failed the list is tried
again 10 s later. While joined, a disconnect or a change of the list starts over from the top. Saved to NVS (they
survive reflashing; `idf.py erase-flash` removes them) as `ssid<i>` / `pass<i>` / `prio<i>` in namespace `wifi`
(`components/wifinets`; the single network of older firmware moves into slot 0), never in the source or the image.
Set them from the Web page or BLE (below), or over USB serial:

```sh
uv run tools/set-wifi.py --prio 10    # on the host the M5Stack is plugged into; prompts for SSID and password
uv run tools/set-wifi.py --list
uv run tools/set-wifi.py --delete HomeAP
uv run tools/set-wifi.py --clear      # all of them
```

This sends `wifi-set<TAB>ssid<TAB>password<TAB>priority`, `wifi-list`, `wifi-del<TAB>ssid` or `wifi-clear` to the
serial console. Opening the port resets the ESP32 on Linux, so the tool waits for it to boot first.

### AP / STA switches

The soft AP and the STA can each be turned off (Web page, `ctrl` characteristic `ap=0|1` / `sta=0|1`, or the console:
`radio`, `radio ap=0`, `radio sta=1`, `reboot`). Saved to NVS (namespace `radio`) and applied at the next boot. Both
off: Wi-Fi is not started (no UDP), BLE stays so they can be turned back on.

### BLE config service

Up to 3 clients can be connected at once (`CONFIG_BT_NIMBLE_MAX_CONNECTIONS`; advertising goes on while there is room).
In a build directory made before this was raised from 1, the old value in `sdkconfig` wins over `sdkconfig.defaults`:
set it with `idf.py menuconfig` (Component config → Bluetooth → NimBLE Options → Maximum number of concurrent
connections → 3), or delete the `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` / `CONFIG_NIMBLE_MAX_CONNECTIONS` lines from
`sdkconfig`, then rebuild. The ESP32 advertises as `logbo-XXXX` with a config service all the time (`CONFIG_LOGBO_CONFIG_WINDOW_S` > 0
limits it to that many seconds after boot; the screen's top bar shows `BT` while it advertises). The link is encrypted (Just Works pairing, no bond stored), so macOS asks to confirm ("接続") on each connection.

**Web page** (`docs/index.html`, Web Bluetooth: Chrome / Edge on Android or a PC, over HTTPS or as a local file;
not iPhone Safari): status, SD recording start / stop, Wi-Fi networks (list, add with a priority, delete), AP / STA
switches, magnetometer calibration (live coverage bars while you rotate the unit), reboot. Published with GitHub Pages from `docs/` on `main`.

```sh
uv run tools/ble-config.py status      # GNSS, SD state / file, configured SSID, Wi-Fi connected?, IP
uv run tools/ble-config.py wifi-set    # prompts for SSID / password / priority; adds or updates a network
uv run tools/ble-config.py wifi-list   # the networks
uv run tools/ble-config.py wifi-del HomeAP
uv run tools/ble-config.py mag         # magnetometer calibration state and the AP / STA switches
uv run tools/ble-config.py ctrl rec=0  # a command (see the ctrl characteristic below)
```

| Characteristic | UUID | |
|---|---|---|
| service | `9b3c0001-4e5f-4a2b-8c6d-2f1e0a7b5c3d` | |
| status | `9b3c0002-…` | read: `{"device":"logbo-CD0E","uptime_s":120,"free_heap":45000,"gnss":{"configured":true,"messages":1234,"baud":115200,"fix":3,"sats":14},"rec":"rec","log":"log_0012_01.csv","records":3000,"imu":true,"mag":true,"baro":true,"wifi_ssid":"…","wifi_connected":true,"wifi_networks":2,"ip":"192.168.11.38","rssi":-40,"ap_ssid":"LOGBO-XXXXXX","ap_clients":0}` (`rec`: off / rec / saving / safe / failed; `fix` 0 when no fix or no GNSS for 2 s; `wifi_ssid`: the network being tried or joined, `wifi_networks`: how many are saved) |
| wifi | `9b3c0003-…` | read: `{"networks":[{"ssid":"HomeAP","prio":10,"secured":true},...],"total":3,"next":null,"gen":7}` (no passwords), in pages that fit the 512-byte attribute limit: when `next` is a number, write `list=<next>&gen=<gen>` and read again. Each client has its own position. `gen` changes with every change of the list: a request with an old `gen` is refused (ATT error 0x83) and a later page with a different `gen` means start over from `list=0`. write with response: `ssid=HomeAP&pass=secret&prio=10` (URL-encoded; adds, or updates the same SSID; `pass` may be left out for an open network) or `ssid=HomeAP&del=1`; the older `<ssid>\t<password>` adds with priority 0. Refused with ATT error 0x80 (invalid: SSID 1–32 bytes, password ≤ 64), 0x81 (8 networks already), 0x82 (no such network) |
| ctrl | `9b3c0004-…` | read: `{"magcal":{…},"radio":{"ap":true,"sta":true,"active":{"ap":true,"sta":true}}}` (radio: saved switches, and those of this boot), magcal = `{"calibrated":true,"field":34.4,"collecting":false,"session":3,"samples":0,"coverage":[0.00,0.00,0.00],"complete":false,"result":1}` (`result` 0 none yet, 1 ok, 2 too short, 3 not rotated, 4 bad fit, 5 not running, 6 not saved). write with response, one command: `rec=1` / `rec=0` (as holding B), `mag=start` / `mag=done` / `mag=cancel` / `mag=clear`, `wifi=clear` (all networks), `ap=0` / `ap=1` / `sta=0` / `sta=1` (next boot), `reboot=1`; an unknown command or a failed `mag=done` gets ATT error 0x13 |

If the macOS pairing dialog shows an old device name after the firmware renamed the device:
macOS records the device in `/Library/Bluetooth/com.apple.MobileBluetooth.ledevices.paired.db`
even without a stored bond. With Bluetooth turned off, delete its row and restart `bluetoothd`:

```sh
DB=/Library/Bluetooth/com.apple.MobileBluetooth.ledevices.paired.db
U=<uuid>
sudo sqlite3 $DB "DELETE FROM PairedDevices WHERE Uuid='$U'; DELETE FROM CustomProperties WHERE Uuid='$U';"
sudo pkill bluetoothd
```

## UDP payload

UDP port 5005 (`CONFIG_LOGBO_UDP_PORT`), one JSON object per datagram, sent to the subnet broadcast address with
`MSG_DONTWAIT` (a datagram that can't go out immediately is dropped). Datagrams are never fragmented: the widest
record is checked by a unit test to fit 1000 bytes. Lost datagrams are not retransmitted.

`type: "racebox"`: one per record (25/s), in the format of
[esp32-racebox-bridge](https://github.com/uzulla/esp32-racebox-bridge) (same members, units and axes), so receivers
written for it work unchanged, e.g. lapcam's `osd-feed` (G ball, track map, speed: it matches `"type":"racebox"`).
logbo's extras follow `gyro_dps`.

```json
{"type":"racebox","seq":7,"device":"logbo-06FA","itow":118286240,
 "time":"2026-11-15T05:06:07.123456789Z","time_valid":true,"time_confirmed":true,"time_acc_ns":25,
 "fix":3,"fix_ok":true,"sats":14,"lat":35.6812360,"lon":139.7671250,"latlon_valid":true,
 "alt_wgs_m":79.761,"alt_msl_m":40.500,"h_acc_m":1.200,"v_acc_m":1.836,
 "speed_mps":27.778,"speed_acc_mps":0.208,"heading_deg":90.00000,"heading_acc_deg":0.50000,"heading_valid":true,
 "pdop":1.20,"battery":null,
 "g":{"x":-0.300,"y":0.250,"z":0.980},"gyro_dps":{"x":1.00,"y":2.00,"z":-15.50},
 "gnss":true,"vel_ned_mps":{"n":0.000,"e":27.778,"d":0.000},
 "g_sensor":{"x":0.100,"y":-0.250,"z":0.980},"gyro_sensor":{"x":1.50,"y":-2.00,"z":30.25},
 "mag_ut":{"x":20.0,"y":-5.0,"z":40.0},"mag_cal":true,"press_hpa":1013.25,"temp_c":21.50}
```

- `g` / `gyro_dps` are in the car's axes like a RaceBox: x forward, y left, z up (gravity included, so ~1 on z at
  rest; yaw counter-clockwise +). Up is the gravity learnt while standing still (GNSS < 1 km/h, or the IMU calm
  without a fix, for ~1 s), right is the screen's right edge (mount the unit with the screen facing the driver, upright
  or tilted). Both are `null` until the unit has stood still once (`components/carframe`; the screen's G circle uses the
  same). lapcam's `osd-feed` with `invert_lateral: true` (its default card config, set for the RaceBox Mini) shows the
  ball the right way round.
- `g_sensor` / `gyro_sensor`: the raw BMI270 axes (what the CSV logs).
- `gnss` is false for IMU-only records (receiver silent); `time` is `null` unless date, time and "fully resolved" are
  valid (nanoseconds, as the bridge); `time_confirmed`: the receiver also confirmed date and time.
- `heading_valid`: fix OK and moving faster than 0.5 m/s. `battery` is always `null` (not measured).
  `g_sensor` / `mag_ut` / `press_hpa` are `null` when the sensor is missing.
- `seq` increments per datagram and restarts from 0 on reboot; gaps mean lost datagrams.

`type: "status"`: once per second (GNSS link counters, sensors, SD state, Wi-Fi RSSI, free heap).

## Client (`client/`)

```sh
cd client && go build -o logbo-dump . && ./logbo-dump        # summary per line
./logbo-dump -raw                                              # raw JSON
```
