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
| `imu` | BMI270 FIFO at 200 Hz (±8 g, ±2000 dps), BMM150 at 25 Hz through the BMI270 aux interface (Bosch compensation), BMP280 at ~26 Hz |
| `record` | one record = NAV-PVT + IMU mean over the epoch; CSV row and UDP JSON formatting (host-tested) |
| `sdlog` | writes the records to `log_<session>_<seq>.csv` on the card |
| `display` | LCD UI and buttons |
| `main` | wiring, Wi-Fi (STA + soft AP), UDP, BLE config service, serial console |

While the GNSS is silent (no message for 1 s) records are still made at the configured rate, IMU only (fix 0, empty UTC).

```sh
mise run fw:flash       # sync to $BUILD_HOST (wvx), build, flash $SERIAL_PORT (/dev/ttyUSB0)
mise run fw:monitor
mise run fw:test        # host unit tests (UBX parser / CFG-VALSET builder / NAV-PVT / CSV / JSON)
mise run fw:ap          # soft-AP SSID / password of the last build
```

### Screen and buttons

- A / C: previous / next page. Pages: **SPEED** (km/h, total horizontal G bar), **G** (G circle with a 2 s trail and peaks;
  short press B resets the peaks), **STATUS** (GNSS fix / satellites / rate, position, accel, baro, mag, SD, Wi-Fi).
- **Hold B for 1.5 s** to stop recording: the top bar shows `SAVING` then `SAFE`; only then pull the card. Holding B again
  (from `SAFE`, `SD ERR` or `NO SD` after inserting a card) records to the next file.
- Top bar: SD state (blinking dot while recording), GNSS fix and satellites, Wi-Fi bars (`BT` while the BLE config
  window is open), time (JST, from GNSS).
- The G values need a reference for gravity: they appear after the unit has been still for about a second. Lateral is
  the screen's left-right axis (BMI270 -X), longitudinal is perpendicular to it in the horizontal plane. They show the G
  the driver feels, like a ball in a bowl: braking moves the ball up (forward), a left turn to the right, and tilting
  the unit rolls it towards the side that goes down. The CSV / UDP keep the raw sensor values.

### SD log

A FAT32 microSD card present at boot is recorded to at once. The session number counts up per boot (NVS) and skips
numbers already on the card; flushed every second. No card, an unreadable card, a full card or a write error: no
recording, and nothing is retried by itself (hold B).

```
t_ms,utc,fix,sats,lat,lon,alt_m,speed_kmh,heading_deg,hacc_m,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,batt,charging,mag_x,mag_y,mag_z,press_hpa,temp_c
```

The columns up to `charging` are those of the RaceBox logs, so [`log-viewer/`](log-viewer/README.md) reads both. acc is
in G and gyro in deg/s, BMI270 axes (mean over the epoch); mag in uT (raw BMM150 axes, not calibrated: the M5Stack's base
magnets add a large offset); press in hPa, temp in °C (BMP280, warmed by the stack).

### Soft AP

The Wi-Fi runs as AP+STA. The STA joins the configured network (below); the AP `LOGBO-XXXXXX` lets a phone or PC
connect directly when there is no router (IP 192.168.4.1, DHCP). While a station is connected the UDP datagrams are also
broadcast to `192.168.4.255`. The SSID suffix and the WPA2 password are random and **change on every build**: they are
generated into `build/esp-idf/main/ap_credentials.h` (never committed) and logged at boot.

### Wi-Fi credentials

Saved to NVS (they survive reflashing; `idf.py erase-flash` removes them), never in the source or the image:

```sh
uv run tools/set-wifi.py              # on the host the M5Stack is plugged into; prompts for SSID and password
uv run tools/set-wifi.py --clear
```

This sends `wifi-set<TAB>ssid<TAB>password` (or `wifi-clear`) to the serial console. Opening the port resets the
ESP32 on Linux, so the tool waits for it to boot first.

### BLE config service

For `CONFIG_LOGBO_CONFIG_WINDOW_S` (60) seconds after boot the ESP32 advertises as `logbo-XXXX` with a config
service. The link is encrypted (Just Works pairing, no bond stored), so macOS asks to confirm ("接続") on each connection.

```sh
uv run tools/ble-config.py status      # GNSS, SD state / file, configured SSID, Wi-Fi connected?, IP
uv run tools/ble-config.py wifi-set    # prompts for SSID/password, saved to NVS and applied
```

| Characteristic | UUID | |
|---|---|---|
| service | `9b3c0001-4e5f-4a2b-8c6d-2f1e0a7b5c3d` | |
| status | `9b3c0002-…` | read: `{"gnss":{"configured":true,"messages":1234},"rec":"rec","log":"log_0012_01.csv","wifi_ssid":"…","wifi_connected":true,"ip":"192.168.11.38"}` |
| wifi | `9b3c0003-…` | write with response: `<ssid>\t<password>` (SSID 1–32 bytes, password ≤ 64); rejected with ATT error 0x13 if invalid |

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

`type: "sample"`: one per record (25/s).

```json
{"type":"sample","seq":7,"gnss":true,"itow":118286240,"time":"2026-11-15T05:06:06.999000Z","time_acc_ns":25,
 "fix":3,"fix_ok":true,"sats":14,"lat":35.6812360,"lon":139.7671250,
 "alt_wgs_m":79.761,"alt_msl_m":40.500,"h_acc_m":1.200,"v_acc_m":1.836,
 "speed_mps":27.778,"speed_acc_mps":0.208,"vel_ned_mps":{"n":0.000,"e":27.778,"d":0.000},
 "heading_deg":90.00000,"heading_acc_deg":0.50000,"heading_valid":true,"pdop":1.20,
 "g":{"x":0.100,"y":-0.250,"z":0.980},"gyro_dps":{"x":1.50,"y":-2.00,"z":30.25},
 "mag_ut":{"x":20.0,"y":-5.0,"z":40.0},"press_hpa":1013.25,"temp_c":21.50}
```

- `gnss` is false for IMU-only records (receiver silent); `time` is `null` unless date, time and "fully resolved" are valid.
- `heading_valid`: fix OK and moving faster than 0.5 m/s. `g` / `gyro_dps` / `mag_ut` / `press_hpa` are `null` when the sensor is missing.
- `seq` increments per datagram and restarts from 0 on reboot; gaps mean lost datagrams.

`type: "status"`: once per second (GNSS link counters, sensors, SD state, Wi-Fi RSSI, free heap).

## Client (`client/`)

```sh
cd client && go build -o logbo-dump . && ./logbo-dump        # summary per line
./logbo-dump -raw                                              # raw JSON
```
