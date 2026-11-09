# cc1101-esp32

Arduino sketch for an ESP32-C3 with a CC1101 sub-GHz radio module. The board is
configured over its USB serial port with AT-style commands. The config (WiFi,
NTP server, radio settings) is stored in EEPROM and survives reboots.

> **Status: radio path enabled, not yet tested on hardware.** A sender board
> sends each non-AT line it receives over USB serial as one packet. A receiver
> board prints each packet it receives on USB serial. Hardware testing is the
> next step.

## Layout

| Path | What it is |
| --- | --- |
| `cc1101-esp32/cc1101-esp32.ino` | The sketch: AT command handler, config in EEPROM, CC1101 setup, SPI register access |
| `deploy.py` | Build, flash, configure and monitor the boards |
| `boards.toml` | The boards (name, chip, role, MAC) and the radio settings |
| `AGENTS.md` | Guidance for contributors and coding agents |

## Requirements

- [`arduino-cli`](https://arduino.github.io/arduino-cli/) on `PATH`
- Python 3.11+ and `pyserial` (`pip install pyserial`)
- ESP32 Arduino core `esp32:esp32` (3.x, tested with 3.3.12); `./deploy.py setup` installs it
- Libraries, also installed by `setup`:
  - `SerialCommands`
  - `SmartRC-CC1101-Driver-Lib` (provides `ELECHOUSE_cc1101`)

## Boards

`boards.toml` has one `[[board]]` entry per physical board:

```toml
[[board]]
name = "sender"
chip = "esp32c3"
role = "sender"        # sender | receiver: sets is_sender in AT+CC1101
mac  = "aa:bb:cc:dd:ee:ff"
```

Boards are found by MAC, not by port name. Each board's USB serial number is
its MAC, so a board keeps its name when it moves to another `/dev/ttyACM*`.

To add a board:

1. Plug it in and run `./deploy.py list`. Each port shows its `serial=` value.
2. Copy that value into the board's `mac` field in `boards.toml`.

`deploy.py list` also tells you which boards in `boards.toml` have no `mac` yet.

The `[radio]` table holds the 26 `AT+CC1101` settings that every board gets.
`is_sender` comes from the board's role.

## Deploy

```sh
./deploy.py setup                       # once: core and libraries
./deploy.py build                       # compile for every chip in boards.toml
./deploy.py flash all                   # flash connected boards
WIFI_SSID='my-net' WIFI_PASS='secret' ./deploy.py configure all
./deploy.py monitor sender              # serial output of one board
./deploy.py deploy --configure all      # build, flash, then configure
```

Selectors are board names (`sender`), roles (`receiver`) or `all`. A board
name must be connected or the command fails. A role or `all` skips boards that
are not plugged in.

`build` writes one image per chip to `dist/<chip>/`. Flashing uses those files
and writes only the bootloader, partition table and app. Other flash data
survives.

| Option | Effect |
| --- | --- |
| `build --release` | No `DEBUG`, `AT_DEBUG` or `VERBOSE` |
| `build --ntp-server HOST` | Default NTP server compiled into the sketch |

Environment:

| Variable | Use |
| --- | --- |
| `WIFI_SSID`, `WIFI_PASS` | Needed by `configure` and `deploy --configure`. Not stored in the repo. |
| `ESPTOOL` | Path to `esptool` if the one from the esp32 core is not found |

`configure` sends the WiFi settings and the radio config, then prints the
board's replies. It stops with an error if the board answers `ERROR`.

`deploy.py flash` checks the MAC reported by `esptool read-mac` against
`boards.toml` before writing. It refuses to flash a board whose MAC differs.

## Radio data

- **Sender:** any line that does not start with `AT` is queued and sent as one
  packet. It must be 64 bytes or less (the CC1101 FIFO size). The sender
  replies `ERROR tx busy` if a previous line is still queued, and
  `ERROR not a sender` on a receiver. Sending waits up to about 1 s for GDO0.
- **Receiver:** each packet is printed on USB serial as raw bytes followed by
  `\r\n`. A packet with a bad CRC is dropped when CRC and status are enabled
  in the radio config (the defaults).
- Packets longer than 64 bytes are not supported by this driver.
- GDO0 is read as an input on `CC1101_GDO0_PIN` (GPIO0 in the sketch). Check
  the wiring before you test.

## AT commands

Commands are terminated by `\r\n`. Each successful command replies `OK`. An
invalid value replies with a message and `ERROR`. A command must match
exactly, except commands that take a value (`AT+X=...`): `AT+DEBUG=10` is
an error, not `AT+DEBUG=1`.

| Command | Reply | Notes |
| --- | --- | --- |
| `AT` | `OK` | Liveness check |
| `AT+WIFI_SSID=<ssid>` | `OK` | Max 31 chars. Saves to EEPROM and reconnects. |
| `AT+WIFI_SSID?` | SSID | |
| `AT+WIFI_PASS=<pass>` | `OK` | Max 63 chars. Saves to EEPROM and reconnects. There is no read-back. |
| `AT+WIFI_STATUS?` | status text | `connected`, `disconnected`, ... |
| `AT+NTP_HOST=<host>` | `OK` | Max 63 chars. Saves to EEPROM and re-syncs. |
| `AT+NTP_HOST?` | host | |
| `AT+NTP_STATUS?` | `ntp synced` / `not ntp synced` | Set when the SNTP client completes a sync |
| `AT+VERBOSE=0\|1`, `AT+VERBOSE?` | `OK` / value | Only in `VERBOSE` builds |
| `AT+DEBUG=0\|1`, `AT+DEBUG?` | `OK` / value | Only in `DEBUG` builds |
| `AT+CC1101=<27 fields>` | `OK` | Radio config, see below. Saves to EEPROM only if it changed. |
| `AT+SCAN=<start_mhz>,<stop_mhz>,<step_khz>` | `SCAN <mhz> <rssi>` per step, then `OK` | Receive-only RSSI scan (dBm). 300–928 MHz, step 10–1000 kHz, at most 2000 steps. Needs the CC1101 to be found. The saved radio config is re-applied afterwards. Example: `AT+SCAN=863,870,100` |
| `AT+RESET` | `OK`, then reboot | Restarts the board |

Any other `AT+...` command returns `ERROR unkown AT command`. Lines that do not
start with `AT` are radio data (see above).

### `AT+CC1101` fields

27 comma-separated values, in this order:

`is_sender, CCMode, Modulation, MHz, Deviation, Channel, Chsp, RxBW, DRate, PA, SyncMode, SyncWord1, SyncWord2, AdrChk, Addr, WhiteData, PktFormat, LengthConfig, PacketLength, Crc, CRC_AF, DcFilterOff, Manchester, FEC, PRE, PQT, AppendStatus`

Each value is range-checked before anything is stored. Examples:

- `is_sender` 0 or 1; `MHz` 300–928
- `Modulation` 0–4; `SyncMode` 0–7; `PRE` and `PQT` 0–7
- `PA` must be one of `-30 -20 -15 -10 -6 0 5 7 10 11 12`
- integer fields must be whole numbers

## Notes

- `DEBUG`, `AT_DEBUG` and `VERBOSE` are set by `deploy.py build` (on by default,
  off with `--release`), not forced in the sketch.
- The WiFi password is stored unencrypted in EEPROM, and it is not echoed back by any command.
- After a config change, the new radio settings are applied to the CC1101 on the
  next `loop()` pass.
- If you change the layout of `cfg_t` or `cc1101_cfg_t`, bump `CFGVERSION` in
  the sketch. This resets stored config on the next boot.
- Configs written by the buggy parser in earlier builds may be wrong. Run
  `./deploy.py configure all` after flashing.

## License

See [LICENSE](LICENSE).
