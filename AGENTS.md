# AGENTS.md

Guidance for coding agents and contributors working on this repo. See
[README.md](README.md) for what the project does and how to use it.

## What this repo is

- One Arduino sketch: `cc1101-esp32/cc1101-esp32.ino`, for an ESP32-C3 with a
  CC1101 radio. The sketch's folder name must match the `.ino` file name.
- `deploy.py` builds, flashes, configures and monitors with `arduino-cli`,
  `esptool` and `pyserial`. Boards and radio settings live in `boards.toml`.
- Boards are identified by the MAC in their USB serial number (see `boards.toml`).
  Do not key them by `/dev/ttyACM*`.
- The radio TX/RX path is enabled, but it has not been tested on hardware yet.
  Changes to it need a board to verify.

## Build and check

Run these before you call a change done:

1. Install dependencies once: `./deploy.py setup`
   (needs `arduino-cli` and network access).
2. Compile: `./deploy.py build`
   Must finish with `Sketch uses ...` and no `error`.
3. For changes to the AT parser or the CC1101 SPI code, also compile with
   warnings on:
   `arduino-cli -b esp32:esp32:esp32c3 compile --output-dir /tmp/<you>/build --build-property build.partitions=min_spiffs --build-property compiler.cpp.extra_flags="-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -DDEBUG -DVERBOSE -DAT_DEBUG -Wall -Wextra" cc1101-esp32`
   Should report no warnings from the sketch.
4. Python: `python3 -m py_compile deploy.py`, and `./deploy.py list` should run
   without errors even with no boards plugged in.

There is no automated test suite and no hardware in CI. Say so in your summary
when you could not check something on a real board.

## Things that will bite you

- **Do not pass `build.extra_flags` to `arduino-cli`.** It makes the link fail
  with `undefined reference to HWCDCSerial`. `deploy.py` passes defines through
  `compiler.cpp.extra_flags` and `compiler.c.extra_flags` only.
- **`sscanf` into `uint8_t` fields is wrong.** `%d` writes an `int`. The AT
  `AT+CC1101=` handler parses into `double v[27]`, range-checks, then copies.
  Keep that pattern when adding fields.
- **Range checks must use `&&`, and parsed values must be range-checked before
  they are stored.** Use `cc1101_int_in()` and `cc1101_pa_ok()`.
- **Do not `memcmp()` config structs.** Use `cc1101_cfg_equal()`. Padding is not
  set consistently.
- **Keep the EEPROM layout in mind.** `cfg_t` and `cc1101_cfg_t` are stored
  with `EEPROM.put`. If you change a struct's layout, bump `CFGVERSION`. That
  resets the stored config on boot, which the user should know about.
- **`cc1101_cfg_t.initialized` and `.version` are not AT config.** Keep them when
  building a new config from the parser (see `tmp_cc1101_cf` in the handler).
- **Do not reintroduce a password read-back.** `AT+WIFI_PASS?` was removed on
  purpose. `DEBUG` builds are not a reason to echo secrets.
- **GDO0 must be an input.** Use `setGDO0()`. `setGDO()` makes it an output,
  and then `digitalRead` returns the latched output value, not the CC1101's
  signal.
- **`setMHZ` before `setPA`.** The PA table depends on the band.
- **The sender idles after config.** Do not strobe TX there. `SendData()`
  starts a packet itself.
- **SNTP status resets after one read.** Latch `sntp_get_sync_status()` in
  `ntp_is_synced`. Use `ntp_start()` to restart a sync.
- **Keep `DEBUG` and `VERBOSE` build-controlled.** Set them through
  `deploy.py build` (`--release` turns them off), not with `#define` in the sketch.
- **Reboots:** use `ESP.restart()`, not a function pointer to address 0.
- **Serial:** the AT protocol shares `Serial` with any debug output. Do not add
  unconditional prints to `Serial`. Gate them with `DOLOG`/`DOLOGLN`.

## Hardware-facing code

- SPI and CC1101 register code (`cc1101_fetch_cfg`, `cc1101_spi_addr`,
  `cc1101_wait_miso`) talks to real hardware. Changes here can only be fully
  verified on a board. Say what you did and did not check.
- The CC1101 library's `SendData()` and `CheckReceiveFlag()` poll GDO0 with
  bounded waits. Keep the loop short around them.
- Keep MISO waits bounded (`cc1101_wait_miso` times out). Always release `SS`
  and call `SPI.endTransaction()` on every path.
- Do not call `SPI.end()` from the periodic register read. Setup owns `SPI.begin()`.

## Deploy script

- `deploy.py` and `boards.toml` are the deployment path. Do not add a second one.
- `boards.toml` holds the user's hardware: names, roles, MACs and the radio
  settings. Do not change a MAC or a radio value without asking. A wrong MAC
  makes `flash` refuse the board, and a wrong radio value changes the link.
- The WiFi password comes from the `WIFI_PASS` environment variable and is sent
  by pyserial as bytes, so shell quoting and backslashes do not affect it. Keep
  it that way (no `echo`, no shell `printf` pipeline). It must never be written
  to a file in the repo.
- `flash` must check the MAC with `esptool read-mac` before writing. Keep that check.

## Git

- Commit messages follow the existing history: `<path>: <short summary>`, for
  example `cc1101-esp32/cc1101-esp32.ino: ...`. Look at `git log --oneline`
  first.
- Commit or push only when the user asks. Do not open a pull request unless
  the user asks.
- `dist/` is build output and is git-ignored. Do not commit it.
- `.gitignore` may have uncommitted changes from the user. Do not revert them.

## Documentation

- When you change AT commands, the `AT+CC1101` field list, or build options,
  update the tables in `README.md` in the same change.
- Keep `README.md` accurate about the radio data path and its limits.
