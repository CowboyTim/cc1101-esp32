#!/usr/bin/env python3
"""Build, flash, configure and monitor the cc1101-esp32 boards.

Boards are listed in boards.toml. A board is found by the MAC in its USB serial
number, so it keeps its name whichever /dev/ttyACM* it lands on.

Run `./deploy.py -h` for the commands. Typical flow:

    ./deploy.py list                   # see which boards are plugged in
    ./deploy.py setup                  # once: install the core and libraries
    ./deploy.py build                  # compile for every chip in boards.toml
    ./deploy.py flash all              # flash connected boards
    WIFI_SSID=.. WIFI_PASS=.. ./deploy.py configure all
    ./deploy.py monitor sender

Requires: arduino-cli, pyserial, Python 3.11+ (tomllib).
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import time
import tomllib
from dataclasses import dataclass
from pathlib import Path

import serial
import serial.tools.list_ports

ROOT = Path(__file__).resolve().parent
SKETCH = "cc1101-esp32"
SKETCH_DIR = ROOT / SKETCH
BOARDS_FILE = ROOT / "boards.toml"
DIST = ROOT / "dist"

CORE = "esp32:esp32"
CORE_URLS = ["https://dl.espressif.com/dl/package_esp32_index.json"]
LIBRARIES = ["SerialCommands", "SmartRC-CC1101-Driver-Lib"]
FQBN = {
    "esp32c3": "esp32:esp32:esp32c3",
    "esp32s3": "esp32:esp32:esp32s3",
}
# Offsets for the three images arduino-cli writes; the rest of flash is left alone.
FLASH_IMAGES = [
    ("0x0", f"{SKETCH}.ino.bootloader.bin"),
    ("0x8000", f"{SKETCH}.ino.partitions.bin"),
    ("0x10000", f"{SKETCH}.ino.bin"),
]
BAUD_MONITOR = 115200  # the sketch uses Serial.begin(115200); USB CDC ignores it anyway
ROLES = ("sender", "receiver")
# AT+CC1101 fields after is_sender, in the order the sketch parses them.
RADIO_FIELDS = [
    "CCMode", "Modulation", "MHz", "Deviation", "Channel", "Chsp", "RxBW",
    "DRate", "PA", "SyncMode", "SyncWord1", "SyncWord2", "AdrChk", "Addr",
    "WhiteData", "PktFormat", "LengthConfig", "PacketLength", "Crc", "CRC_AF",
    "DcFilterOff", "Manchester", "FEC", "PRE", "PQT", "AppendStatus",
]


@dataclass
class Board:
    name: str
    chip: str
    role: str
    mac: str  # normalised: 12 lowercase hex digits, or "" if not filled in yet


def die(msg):
    sys.exit(f"deploy.py: {msg}")


def norm_mac(text):
    """'AA:BB:CC:DD:EE:FF' or 'aabbccddeeff' -> 'aabbccddeeff'."""
    return re.sub(r"[^0-9a-fA-F]", "", text or "").lower()


def load_boards():
    """Return (boards, radio_values) from boards.toml, validated."""
    if not BOARDS_FILE.exists():
        die(f"{BOARDS_FILE} not found")
    data = tomllib.loads(BOARDS_FILE.read_text())
    boards = []
    for entry in data.get("board", []):
        b = Board(
            name=entry["name"],
            chip=entry["chip"],
            role=entry["role"],
            mac=norm_mac(entry.get("mac", "")),
        )
        if b.chip not in FQBN:
            die(f"board {b.name}: unknown chip {b.chip!r} (known: {', '.join(FQBN)})")
        if b.role not in ROLES:
            die(f"board {b.name}: role must be one of {', '.join(ROLES)}")
        if entry.get("mac") and len(b.mac) != 12:
            die(f"board {b.name}: mac {entry['mac']!r} is not 6 bytes of hex")
        boards.append(b)

    names = [b.name for b in boards]
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        die(f"duplicate board names in boards.toml: {', '.join(sorted(dupes))}")
    if not boards:
        die("boards.toml has no [[board]] entries")

    radio = data.get("radio", {})
    missing = [f for f in RADIO_FIELDS if f not in radio]
    if missing:
        die(f"boards.toml [radio] is missing: {', '.join(missing)}")
    return boards, radio


def select(boards, selectors):
    """Turn command-line selectors into [(board, required)].

    A board name is required: it must be plugged in. A role or 'all' is optional:
    boards of that role that are not plugged in are skipped.
    """
    if not selectors:
        selectors = ["all"]
    out, seen = [], set()

    def add(b, required):
        if b.name not in seen:
            seen.add(b.name)
            out.append((b, required))

    for sel in selectors:
        if sel == "all":
            for b in boards:
                add(b, False)
            continue
        named = [b for b in boards if b.name == sel]
        by_role = [b for b in boards if b.role == sel]
        if named:
            for b in named:
                add(b, True)
        elif by_role:
            for b in by_role:
                add(b, False)
        else:
            die(f"unknown board or role {sel!r}; boards: "
                f"{', '.join(b.name for b in boards)}; roles: {', '.join(ROLES)}")
    return out


def port_by_mac(mac):
    """Port whose USB serial number matches mac, or None."""
    for p in serial.tools.list_ports.comports():
        if p.serial_number and norm_mac(p.serial_number) == mac:
            return p.device
    return None


def wait_for_port(mac, timeout=15.0):
    """After a flash the board re-enumerates; wait for its port to come back."""
    end = time.time() + timeout
    while time.time() < end:
        port = port_by_mac(mac)
        if port:
            return port
        time.sleep(0.5)
    return None


def find_esptool():
    env = os.environ.get("ESPTOOL")
    if env:
        return [env]
    # the esptool that ships with the esp32 core
    candidates = sorted(glob.glob(os.path.expanduser(
        "~/.arduino15/packages/esp32/tools/esptool_py/*/esptool")))
    if candidates:
        return [candidates[-1]]
    for name in ("esptool", "esptool.py"):
        if shutil.which(name):
            return [shutil.which(name)]
    die("esptool not found: install the esp32 core (./deploy.py setup) or set ESPTOOL")


def arduino(*args, check=True, capture=False):
    cmd = ["arduino-cli", *args]
    if not shutil.which("arduino-cli"):
        die("arduino-cli not found in PATH")
    return subprocess.run(cmd, cwd=ROOT, check=check, text=True,
                          capture_output=capture)


# ---- commands ---------------------------------------------------------------

def cmd_list(args):
    boards, _ = load_boards()
    by_mac = {b.mac: b.name for b in boards if b.mac}
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("no serial ports found")
        return
    for p in ports:
        sn = p.serial_number or ""
        key = norm_mac(sn)
        match = by_mac.get(key, "")
        label = f"-> {match}" if match else ("(not in boards.toml)" if sn else "")
        print(f"{p.device:14} serial={sn or '-':18} {label}")
    unknown = [b.name for b in boards if not b.mac]
    if unknown:
        print(f"\nboards without a mac in boards.toml: {', '.join(unknown)}")
        print("copy the serial= value of the board you want into its mac field")


def cmd_setup(args):
    urls = []
    for u in CORE_URLS:
        urls += ["--additional-urls", u]
    arduino(*urls, "core", "install", CORE)
    arduino(*urls, "lib", "install", *LIBRARIES)
    arduino(*urls, "lib", "list")


def compile_flags(args):
    flags = ["-DARDUINO_USB_MODE=1", "-DARDUINO_USB_CDC_ON_BOOT=1"]
    if not args.release:
        flags += ["-DDEBUG", "-DAT_DEBUG", "-DVERBOSE"]
    if args.ntp_server:
        flags.append(f'-DDEFAULT_NTP_SERVER="{args.ntp_server}"')
    return " ".join(flags)


def build_chip(chip, args):
    flags = compile_flags(args)
    out = DIST / chip
    print(f"building {SKETCH} for {chip} -> {out.relative_to(ROOT)}")
    arduino(
        "compile",
        "-b", FQBN[chip],
        "--output-dir", str(out),
        "--build-property", "build.partitions=min_spiffs",
        "--build-property", f"compiler.cpp.extra_flags={flags}",
        "--build-property", f"compiler.c.extra_flags={flags}",
        str(SKETCH_DIR),
    )


def cmd_build(args):
    boards, _ = load_boards()
    chips = sorted({b.chip for b, _ in select(boards, args.boards)})
    for chip in chips:
        build_chip(chip, args)


def read_mac(esptool, port, chip):
    res = subprocess.run([*esptool, "--chip", chip, "--port", port, "read-mac"],
                         text=True, capture_output=True)
    m = re.search(r"MAC:\s*((?:[0-9a-fA-F]{2}[:-]?){6})", res.stdout + res.stderr)
    if res.returncode != 0 or not m:
        die(f"could not read MAC on {port}:\n{res.stdout}{res.stderr}")
    return norm_mac(m.group(1))


def flash_board(board, port, esptool, build_dir):
    seen = read_mac(esptool, port, board.chip)
    if seen != board.mac:
        die(f"{board.name}: {port} reports MAC {seen}, boards.toml says {board.mac}; "
            "refusing to flash")
    files = []
    for offset, name in FLASH_IMAGES:
        path = build_dir / name
        if not path.exists():
            die(f"missing {path}; run ./deploy.py build first")
        files += [offset, str(path)]
    print(f"flashing {board.name} on {port}")
    subprocess.run([*esptool, "--chip", board.chip, "--port", port,
                    "write-flash", *files], check=True)


def cmd_flash(args, configure_after=False):
    boards, radio = load_boards()
    sel = select(boards, args.boards)
    esptool = find_esptool()
    did = []
    for board, required in sel:
        if not board.mac:
            if required:
                die(f"{board.name} has no mac in boards.toml (see ./deploy.py list)")
            print(f"skipping {board.name}: no mac in boards.toml")
            continue
        port = port_by_mac(board.mac)
        if not port:
            if required:
                die(f"{board.name} ({board.mac}) is not connected")
            print(f"skipping {board.name}: not connected")
            continue
        build_dir = DIST / board.chip
        flash_board(board, port, esptool, build_dir)
        did.append(board)
    if configure_after and did:
        time.sleep(1)
        cmd_configure_boards(did, radio, args)


def at_line_radio(board, radio):
    is_sender = 1 if board.role == "sender" else 0
    values = [is_sender] + [radio[f] for f in RADIO_FIELDS]
    return "AT+CC1101=" + ",".join(str(v) for v in values)


def cmd_configure_boards(sel_boards, radio, args):
    ssid = os.environ.get("WIFI_SSID")
    pw = os.environ.get("WIFI_PASS")
    if not ssid or not pw:
        die("set WIFI_SSID and WIFI_PASS in the environment")
    for v in (ssid, pw):
        if "\r" in v or "\n" in v:
            die("WIFI_SSID / WIFI_PASS must not contain CR or LF")

    for board in sel_boards:
        port = wait_for_port(board.mac)
        if not port:
            die(f"{board.name} ({board.mac}) did not come back after flashing")
        lines = [
            "AT",
            f"AT+WIFI_PASS={pw}",
            f"AT+WIFI_SSID={ssid}",
            "AT+VERBOSE=1",
            "AT+DEBUG=0",
            at_line_radio(board, radio),
        ]
        print(f"configuring {board.name} ({board.role}) on {port}")
        with serial.Serial(port, BAUD_MONITOR, timeout=0.3) as s:
            s.reset_input_buffer()
            s.write(b"\n")
            for line in lines:
                s.write(line.encode() + b"\r\n")
                time.sleep(0.2)
            # echo the replies; the sketch answers OK or ERROR per command
            end = time.time() + 1.5
            reply = b""
            while time.time() < end:
                reply += s.read(256)
            text = reply.decode(errors="replace")
            for line in text.splitlines():
                if line.strip():
                    print(f"  {line.rstrip()}")
            if "ERROR" in text:
                die(f"{board.name} reported an error; see output above")


def cmd_configure(args):
    boards, radio = load_boards()
    sel = select(boards, args.boards)
    sel_boards = []
    for board, required in sel:
        if not board.mac:
            if required:
                die(f"{board.name} has no mac in boards.toml (see ./deploy.py list)")
            print(f"skipping {board.name}: no mac in boards.toml")
            continue
        if not port_by_mac(board.mac):
            if required:
                die(f"{board.name} ({board.mac}) is not connected")
            print(f"skipping {board.name}: not connected")
            continue
        sel_boards.append(board)
    if sel_boards:
        cmd_configure_boards(sel_boards, radio, args)


def cmd_deploy(args):
    cmd_build(args)
    cmd_flash(args, configure_after=args.configure)


def cmd_monitor(args):
    boards, _ = load_boards()
    sel = select(boards, [args.board])
    if len(sel) != 1:
        die("monitor needs exactly one board name")
    board, _ = sel[0]
    if not board.mac:
        die(f"{board.name} has no mac in boards.toml (see ./deploy.py list)")
    port = port_by_mac(board.mac)
    if not port:
        die(f"{board.name} ({board.mac}) is not connected")
    print(f"monitoring {board.name} on {port} (Ctrl-C to quit)")
    s = serial.Serial()
    s.port = port
    s.baudrate = BAUD_MONITOR
    s.timeout = 0.2
    # clear RTS and DTR before opening: asserting them resets USB-Serial/JTAG boards
    s.rts = False
    s.dtr = False
    s.open()
    try:
        while True:
            data = s.readline()
            if data:
                sys.stdout.write(data.decode(errors="replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        s.close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def selectors(sp):
        sp.add_argument("boards", nargs="*",
                        help="board names, roles (sender/receiver) or 'all' (default)")

    sub.add_parser("list", help="show serial ports and which board they match")\
        .set_defaults(fn=cmd_list)
    sub.add_parser("setup", help="install the esp32 core and libraries")\
        .set_defaults(fn=cmd_setup)

    def build_opts(sp):
        selectors(sp)
        sp.add_argument("--release", action="store_true",
                        help="build without DEBUG, AT_DEBUG and VERBOSE")
        sp.add_argument("--ntp-server", help="default NTP server compiled into the sketch")

    b = sub.add_parser("build", help="compile for each chip in the selection")
    build_opts(b)
    b.set_defaults(fn=cmd_build)

    f = sub.add_parser("flash", help="flash connected boards (builds must exist)")
    selectors(f)
    f.set_defaults(fn=cmd_flash, configure_after=False)

    c = sub.add_parser("configure", help="send WiFi and radio settings (WIFI_SSID, WIFI_PASS env)")
    selectors(c)
    c.set_defaults(fn=cmd_configure)

    d = sub.add_parser("deploy", help="build, then flash")
    build_opts(d)
    d.add_argument("--configure", action="store_true",
                   help="also configure the flashed boards (needs WIFI_SSID and WIFI_PASS)")
    d.set_defaults(fn=cmd_deploy)

    m = sub.add_parser("monitor", help="show serial output of one board")
    m.add_argument("board", help="board name")
    m.set_defaults(fn=cmd_monitor)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
