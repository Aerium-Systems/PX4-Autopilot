#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aerium Neo CAN GNSS - flash over ST-Link and verify on a PX4 Pixhawk.

What it does
  1. Checks the firmware images: the CAN bootloader and the application, which
     must carry a valid CAN boot descriptor (CRC), or the bootloader will not
     start it.
  2. Connects to the ST-Link through STM32CubeProgrammer and checks that the
     target is an STM32F412.
  3. Programs and verifies the flash, then resets the CAN node.
  4. Connects to the Pixhawk (PX4) on its USB COM port over MAVLink, opens the
     MAVLink shell, and waits for the node to come online on the CAN bus.
  5. Checks that the GNSS and the compass of the node are identified on the
     Pixhawk (UAVCAN device IDs) and that their data is live and plausible.
  6. Prints a PASS/FAIL summary of every check.

Exit code: 0 = pass (warnings allowed), 1 = a check failed, 2 = setup error,
130 = interrupted.

Requirements (Windows 10/11)
  - Python 3.8 or newer:  py -m pip install pyserial pymavlink
  - STM32CubeProgrammer (provides STM32_Programmer_CLI.exe). The default
    install folder is found automatically, otherwise pass --cubeprog.
  - ST-Link on the SWD pads of the node (SWDIO, SWCLK, GND, plus NRST for
    --connect-mode UR).
  - Pixhawk on USB, with UAVCAN_ENABLE >= 2 (dynamic node ID allocation).
    The node is on the Pixhawk CAN bus, which also powers it.
  - Close QGroundControl first: it would hold the Pixhawk COM port.

Firmware files (searched next to this script, in ./firmware, in the current
folder and in the PX4 build folder when run from the source tree)
  - aerium_neo-can-gnss_merged.hex    (bootloader + app, from make_merged_hex.sh)
  - or aerium_neo-can-gnss_canbootloader.bin + deploy/1.bin (app with boot descriptor)
  The plain aerium_neo-can-gnss_default.bin has no CRC filled in and is rejected.

Examples
  py flash_and_verify.py                          auto-find firmware and COM port
  py flash_and_verify.py --hex aerium_neo-can-gnss_merged.hex --port COM7
  py flash_and_verify.py --bootloader aerium_neo-can-gnss_canbootloader.bin --app 1.bin
  py flash_and_verify.py --verify-only            only check the node on the Pixhawk
  py flash_and_verify.py --require-fix            fail unless the GNSS gets a 3D fix
  py flash_and_verify.py --loop --reboot-fc --log production: one unit after another
  py flash_and_verify.py --list-ports             show serial ports and exit
"""

from __future__ import annotations

import argparse
import datetime as _dt
import glob
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SCRIPT_VERSION = "1.0.0"

# --------------------------------------------------------------------------
# Board constants. They mirror boards/aerium/neo-can-gnss; keep them in sync.
# --------------------------------------------------------------------------
BOARD = "aerium_neo-can-gnss"
UAVCAN_NAME = "org.aerium.neo-can-gnss"      # uavcan_board_identity (in bootloader and app)
FLASH_BASE = 0x08000000                 # canbootloader load address
PARAMS_START = 0x08008000               # src/init.c: flash params in sectors 2-3
APP_ADDR = 0x08010000                   # src/boot_config.h: 64 KB bootloader area
FLASH_END = 0x08100000                  # STM32F412CG: 1 MB
SRAM_START, SRAM_END = 0x20000000, 0x20040000
EXPECTED_DEVICE_ID = 0x441              # DBGMCU_IDCODE of the STM32F412
UID_ADDR = 0x1FFF7A10                   # 96-bit unique device ID
APP_DESC_SIG = bytes((0x40, 0xA2, 0xE4, 0xF1, 0x64, 0x68, 0x91, 0x06))
F412_SECTORS = [(0, 0x08000000, 0x4000), (1, 0x08004000, 0x4000),
                (2, 0x08008000, 0x4000), (3, 0x0800C000, 0x4000),
                (4, 0x08010000, 0x10000)] + \
               [(5 + i, 0x08020000 + i * 0x20000, 0x20000) for i in range(7)]

# PX4 device IDs (src/lib/drivers/device/Device.hpp, src/drivers/drv_sensor.h)
BUS_TYPE_UAVCAN = 3
BUS_NAMES = {0: "UNKNOWN", 1: "I2C", 2: "SPI", 3: "UAVCAN", 4: "SIM", 5: "SERIAL", 6: "MAVLINK"}
DEVTYPE_GPS_UAVCAN = 0x85
DEVTYPE_MAG_UAVCAN = 0x88

# USB vendor IDs of PX4 flight controllers (Aerium boards use 0x1B8C)
FC_USB_VIDS = {0x1B8C: "Aerium", 0x26AC: "PX4/3DR", 0x3162: "Holybro", 0x2DAE: "CubePilot"}
FC_TEXT_RE = re.compile(r"PX4|FMU|Pixhawk|Aerium|Apex|Radian|Cube", re.I)

FIX_NAMES = {0: "none", 1: "none", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed", 8: "extrapolated"}
JAMMING_NAMES = {0: "unknown", 1: "ok", 2: "warning", 3: "critical"}
MODE_NAMES = {"OPERAT": "OPERATIONAL", "INIT": "INITIALIZATION", "MAINT": "MAINTENANCE",
              "SW_UPD": "SOFTWARE_UPDATE", "OFFLN": "OFFLINE"}

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"

# --------------------------------------------------------------------------
# Console output
# --------------------------------------------------------------------------
ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
RESET, BOLD, DIM = "0", "1", "2"
RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE, GREY = "31", "32", "33", "34", "35", "36", "37", "90"
BLACK_FG, BG_RED, BG_GREEN, BG_YELLOW = "30", "41", "42", "43"
STATUS_STYLE = {PASS: (GREEN, BOLD), WARN: (YELLOW, BOLD), FAIL: (RED, BOLD), SKIP: (GREY,)}


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def _stdout_isatty() -> bool:
    try:
        return sys.stdout.isatty()
    except Exception:
        return False


def enable_ansi() -> bool:
    """Turn on ANSI escape processing (needed for colors in the Windows console)."""
    if not _stdout_isatty() or os.environ.get("NO_COLOR"):
        return False
    if os.name != "nt":
        return True
    try:
        import ctypes
        from ctypes import wintypes
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = wintypes.DWORD()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            return False
        if mode.value & 0x0004:  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
            return True
        return bool(kernel32.SetConsoleMode(handle, mode.value | 0x0004))
    except Exception:
        return False


class UI:
    """Colored console output, a transient status line, and an optional plain log file."""

    def __init__(self, color: bool, verbose: bool):
        self.color = color
        self.verbose = verbose
        self.tty = _stdout_isatty()
        self.width = max(80, min(shutil.get_terminal_size((110, 24)).columns, 160))
        self._log = None
        self.log_path: Optional[Path] = None
        self._status_len = 0
        self._lock = threading.RLock()
        self.step_no = 0
        self.step_total = 0

    # -- basics --------------------------------------------------------------
    def c(self, text: str, *codes: str) -> str:
        if not self.color or not codes:
            return text
        return "\x1b[" + ";".join(codes) + "m" + text + "\x1b[0m"

    def open_log(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self._log = open(path, "w", encoding="utf-8")
        self.log_path = path

    def close_log(self):
        if self._log:
            self._log.close()
            self._log = None

    def log_only(self, text: str):
        if self._log:
            try:
                self._log.write(strip_ansi(text) + "\n")
                self._log.flush()
            except Exception:
                pass

    def out(self, text: str = "", log: bool = True):
        with self._lock:
            self._clear_status()
            try:
                print(text, flush=True)
            except UnicodeEncodeError:
                print(text.encode("ascii", "replace").decode("ascii"), flush=True)
            if log:
                self.log_only(text)

    # -- transient status line (only on a terminal) ---------------------------
    def status(self, text: str):
        if not self.tty:
            return
        with self._lock:
            text = text[: self.width - 2]
            pad = max(0, self._status_len - len(text))
            sys.stdout.write("\r" + self.c(text, GREY) + " " * pad)
            sys.stdout.flush()
            self._status_len = len(text)

    def _clear_status(self):
        if self._status_len:
            sys.stdout.write("\r" + " " * self._status_len + "\r")
            sys.stdout.flush()
            self._status_len = 0

    def clear_status(self):
        with self._lock:
            self._clear_status()

    # -- structured output ---------------------------------------------------
    def rule(self, ch: str = "-", style: Tuple[str, ...] = (GREY,)):
        self.out(self.c(ch * (self.width - 1), *style))

    def banner(self, title: str, lines: List[str]):
        self.out()
        self.rule("=", (CYAN,))
        self.out(self.c("  " + title, BOLD, CYAN))
        for line in lines:
            self.out(self.c("  " + line, GREY))
        self.rule("=", (CYAN,))

    def step(self, title: str):
        self.step_no += 1
        self.out()
        self.out(self.c(f"[{self.step_no}/{self.step_total}] {title}", BOLD, BLUE))
        self.out(self.c("-" * (len(title) + 8), BLUE))

    def info(self, msg: str):
        self.out("  " + msg)

    def kv(self, key: str, value: str):
        self.out("      " + self.c(f"{key:<20}", GREY) + " " + value)

    def warn(self, msg: str):
        self.out("  " + self.c("! " + msg, YELLOW))

    def error(self, msg: str):
        self.out("  " + self.c("x " + msg, RED))

    def tag(self, status: str) -> str:
        return self.c(f"[{status}]", *STATUS_STYLE.get(status, ()))

    def check(self, chk: "Check"):
        line = f"  {self.tag(chk.status)} {chk.name:<26} {chk.detail}"
        self.out(line)
        if chk.hint and chk.status in (FAIL, WARN):
            self.out("         " + self.c("hint: ", YELLOW, BOLD) + self.c(chk.hint, YELLOW))

    def raw(self, title: str, text: str):
        """Tool output: always written to the log, shown on screen with --verbose."""
        self.log_only(f"----- {title} -----")
        self.log_only(text.rstrip())
        self.log_only("----- end -----")
        if self.verbose:
            for line in text.rstrip().splitlines():
                self.out(self.c("    | " + line, GREY), log=False)


class Spinner:
    """Shows an animated status line with elapsed time while a blocking call runs."""

    def __init__(self, ui: UI, label: str):
        self.ui = ui
        self.label = label
        self.t0 = 0.0
        self.elapsed = 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def __enter__(self):
        self.t0 = time.monotonic()
        if self.ui.tty:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        else:
            self.ui.out(f"  ... {self.label}")
        return self

    def _run(self):
        frames = "|/-\\"
        i = 0
        while not self._stop.wait(0.12):
            self.ui.status(f"  {frames[i % 4]} {self.label} ({time.monotonic() - self.t0:.0f} s)")
            i += 1

    def __exit__(self, *exc):
        self.elapsed = time.monotonic() - self.t0
        self._stop.set()
        if self._thread:
            self._thread.join()
        self.ui.clear_status()
        return False


# --------------------------------------------------------------------------
# Check results
# --------------------------------------------------------------------------
@dataclass
class Check:
    group: str
    name: str
    status: str
    detail: str = ""
    hint: str = ""


@dataclass
class UnitInfo:
    uid: str = ""
    mcu: str = ""
    stlink_sn: str = ""
    firmware: str = ""
    node_id: Optional[int] = None
    pixhawk: str = ""

    def label(self) -> str:
        parts = []
        if self.uid:
            parts.append(f"UID {self.uid}")
        if self.firmware:
            parts.append(f"FW {self.firmware}")
        if self.node_id is not None:
            parts.append(f"CAN node {self.node_id}")
        return " | ".join(parts)


class Results:
    def __init__(self, ui: UI):
        self.ui = ui
        self.items: List[Check] = []
        self.unit = UnitInfo()

    def add(self, group: str, name: str, status: str, detail: str = "", hint: str = "") -> bool:
        chk = Check(group, name, status, detail, hint)
        self.items.append(chk)
        self.ui.check(chk)
        return status != FAIL

    def skip(self, group: str, names: List[str], reason: str):
        for name in names:
            self.add(group, name, SKIP, reason)

    def count(self, status: str) -> int:
        return sum(1 for c in self.items if c.status == status)

    @property
    def passed(self) -> bool:
        return self.count(FAIL) == 0 and any(c.status in (PASS, WARN) for c in self.items)


# --------------------------------------------------------------------------
# Firmware images
# --------------------------------------------------------------------------
class FirmwareError(Exception):
    pass


Segments = List[Tuple[int, bytes]]
ROLE_LABEL = {"merged": "bootloader+app", "bootloader": "bootloader", "app": "application"}


@dataclass
class FlashImage:
    role: str                 # merged | bootloader | app
    path: Path
    addr: Optional[int]       # load address for .bin files, None for .hex
    segments: Segments = field(default_factory=list)

    @property
    def is_hex(self) -> bool:
        return self.path.suffix.lower() in (".hex", ".ihex")

    @property
    def size(self) -> int:
        return sum(len(d) for _, d in self.segments)


def _merge_segments(segs: List[Tuple[int, bytearray]]) -> Segments:
    segs.sort(key=lambda s: s[0])
    merged: List[Tuple[int, bytearray]] = []
    for addr, data in segs:
        if merged and merged[-1][0] + len(merged[-1][1]) == addr:
            merged[-1][1].extend(data)
        else:
            merged.append((addr, bytearray(data)))
    return [(a, bytes(d)) for a, d in merged]


def load_ihex(path: Path) -> Segments:
    segs: List[Tuple[int, bytearray]] = []
    base = 0
    with open(path, "r", encoding="ascii", errors="replace") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise FirmwareError(f"{path.name} line {lineno}: not an Intel HEX record")
            try:
                rec = bytes.fromhex(line[1:])
            except ValueError:
                raise FirmwareError(f"{path.name} line {lineno}: invalid hex digits")
            if len(rec) < 5 or len(rec) != rec[0] + 5:
                raise FirmwareError(f"{path.name} line {lineno}: bad record length")
            if sum(rec) & 0xFF:
                raise FirmwareError(f"{path.name} line {lineno}: checksum error")
            offset, rtype, data = (rec[1] << 8) | rec[2], rec[3], rec[4:4 + rec[0]]
            if rtype == 0x00:
                addr = base + offset
                if segs and segs[-1][0] + len(segs[-1][1]) == addr:
                    segs[-1][1].extend(data)
                else:
                    segs.append((addr, bytearray(data)))
            elif rtype == 0x01:
                break
            elif rtype == 0x02:
                base = ((data[0] << 8) | data[1]) << 4
            elif rtype == 0x04:
                base = ((data[0] << 8) | data[1]) << 16
    if not segs:
        raise FirmwareError(f"{path.name}: no data records")
    return _merge_segments(segs)


def load_image(role: str, path: Path, default_addr: int) -> FlashImage:
    if not path.is_file():
        raise FirmwareError(f"file not found: {path}")
    img = FlashImage(role, path.resolve(), None)
    if img.is_hex:
        img.segments = load_ihex(path)
    else:
        img.addr = default_addr
        img.segments = [(default_addr, path.read_bytes())]
        if not img.segments[0][1]:
            raise FirmwareError(f"{path.name} is empty")
    return img


def read_region(segs: Segments, start: int, length: int) -> Optional[bytes]:
    for addr, data in segs:
        if addr <= start and start + length <= addr + len(data):
            return data[start - addr:start - addr + length]
    return None


def crc32_px4(data: bytes) -> int:
    """CRC-32 of make_can_boot_descriptor.py: zlib polynomial, init 0, no final xor."""
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


def sectors_for(start: int, end: int) -> str:
    nums = [n for n, a, s in F412_SECTORS if a < end and a + s > start]
    if not nums:
        return "-"
    return str(nums[0]) if len(nums) == 1 else f"{nums[0]}-{nums[-1]}"


def analyze_bootloader(segs: Segments) -> Tuple[str, str, str]:
    bl = [(a, d) for a, d in segs if FLASH_BASE <= a < APP_ADDR]
    if not bl:
        return FAIL, "no data at 0x08000000", "the image does not contain the CAN bootloader"
    end = max(a + len(d) for a, d in bl)
    if bl[0][0] != FLASH_BASE:
        return FAIL, f"bootloader starts at 0x{bl[0][0]:08X}, expected 0x08000000", ""
    vec = read_region(segs, FLASH_BASE, 8)
    if vec is None:
        return FAIL, "vector table missing at 0x08000000", ""
    sp, reset = struct.unpack("<II", vec)
    if not (SRAM_START < sp <= SRAM_END and FLASH_BASE <= (reset & ~1) < PARAMS_START):
        return FAIL, f"invalid vector table (SP 0x{sp:08X}, reset 0x{reset:08X})", "not a bootloader image"
    blob = b"".join(d for _, d in bl)
    size_kb = (end - FLASH_BASE) / 1024
    detail = f"{size_kb:.1f} KB @ 0x08000000, reset 0x{reset:08X}, '{UAVCAN_NAME}'"
    if UAVCAN_NAME.encode() not in blob:
        return FAIL, f"{size_kb:.1f} KB, board name '{UAVCAN_NAME}' not found", \
            "this is not the neo-can-gnss canbootloader (wrong board build?)"
    if end > PARAMS_START:
        return WARN, detail + ", overlaps param sectors", \
            "flash parameters at 0x08008000-0x0800FFFF will be erased"
    return PASS, detail, ""


def analyze_app(segs: Segments) -> Tuple[str, str, str, str]:
    """Returns (status, detail, hint, short version string)."""
    app = [(a, d) for a, d in segs if a >= APP_ADDR]
    if not app:
        return FAIL, "no data at 0x08010000", "the image does not contain the application", ""
    if app[0][0] != APP_ADDR:
        return FAIL, f"application starts at 0x{app[0][0]:08X}, expected 0x08010000", "", ""
    end = max(a + len(d) for a, d in app)
    if end > FLASH_END:
        return FAIL, f"image ends at 0x{end:08X}, beyond the 1 MB flash", "", ""
    data = app[0][1]
    sp, reset = struct.unpack_from("<II", data, 0)
    if not (SRAM_START < sp <= SRAM_END and APP_ADDR <= (reset & ~1) < FLASH_END):
        return FAIL, f"invalid vector table (SP 0x{sp:08X}, reset 0x{reset:08X})", "", ""
    off = data.find(APP_DESC_SIG, 0, 0x2000)
    if off < 0:
        return FAIL, "no CAN boot descriptor found", "not a PX4 cannode application image", ""
    _, crc1, crc2, image_size, git, major, minor, board_id, _ = struct.unpack_from("<8sIIIIBBH8s", data, off)
    if crc1 == 0 and crc2 == 0 and image_size == 0:
        return FAIL, "boot descriptor not filled in (CRC = 0)", \
            "this is the raw .bin; the CAN bootloader would not start it. " \
            "Use build/aerium_neo-can-gnss_default/deploy/1.bin or the merged .hex", ""
    if image_size > len(data) or image_size < off + 36:
        return FAIL, f"descriptor image_size {image_size} does not match the {len(data)} bytes in the file", "", ""
    calc1 = crc32_px4(data[:off + 8])
    calc2 = crc32_px4(data[off + 24:image_size])
    version = f"v{major}.{minor} git {git:08x}"
    if (calc1, calc2) != (crc1, crc2):
        return FAIL, f"{version}: CRC mismatch (descriptor {crc1:08x}/{crc2:08x}, " \
                     f"computed {calc1:08x}/{calc2:08x})", "image file is corrupt or was modified", version
    if UAVCAN_NAME.encode() not in data[:image_size]:
        return FAIL, f"{version}: board name '{UAVCAN_NAME}' not found", \
            "this is not the neo-can-gnss application (wrong board build?)", version
    detail = f"{version}, HW {board_id >> 8}.{board_id & 0xFF}, {image_size / 1024:.1f} KB, CRC OK"
    return PASS, detail, "", version


def _search_dirs() -> List[Path]:
    here = Path(__file__).resolve().parent
    dirs = [Path.cwd(), Path.cwd() / "firmware", here, here / "firmware"]
    try:
        root = here.parents[3]  # boards/aerium/neo-can-gnss/scripts -> PX4 source root
        build = root / "build"
        if (root / "boards" / "aerium" / "neo-can-gnss").is_dir() and build.is_dir():
            dirs += [build / f"{BOARD}_default", build / f"{BOARD}_default" / "deploy",
                     build / f"{BOARD}_canbootloader"]
    except IndexError:
        pass
    seen, out = set(), []
    for d in dirs:
        key = str(d.resolve()) if d.exists() else str(d)
        if key not in seen and d.is_dir():
            seen.add(key)
            out.append(d)
    return out


def _first_file(dirs: List[Path], names: List[str]) -> Optional[Path]:
    for d in dirs:
        for name in names:
            p = d / name
            if p.is_file():
                return p
    return None


def _find_app(dirs: List[Path]) -> Optional[Path]:
    p = _first_file(dirs, [str(Path("deploy") / "1.bin"), "1.bin"])
    if p:
        return p
    found = []
    for d in dirs:
        found += [Path(x) for x in glob.glob(str(d / "*.uavcan.bin"))]
    return max(found, key=lambda x: x.stat().st_mtime) if found else None


def select_images(args) -> Tuple[List[FlashImage], str]:
    if args.hex:
        return [load_image("merged", Path(args.hex), FLASH_BASE)], "merged image given with --hex"
    if args.bootloader or args.app:
        imgs = []
        if args.bootloader:
            imgs.append(load_image("bootloader", Path(args.bootloader), FLASH_BASE))
        if args.app:
            imgs.append(load_image("app", Path(args.app), APP_ADDR))
        return imgs, "files given on the command line"
    dirs = _search_dirs()
    merged = _first_file(dirs, [f"{BOARD}_merged.hex"])
    bl = _first_file(dirs, [f"{BOARD}_canbootloader.bin"])
    app = _find_app(dirs)
    if bl and app and (not merged or merged.stat().st_mtime + 1 < max(bl.stat().st_mtime, app.stat().st_mtime)):
        why = "auto: separate bootloader + application" + (" (newer than the merged .hex)" if merged else "")
        return [load_image("bootloader", bl, FLASH_BASE), load_image("app", app, APP_ADDR)], why
    if merged:
        return [load_image("merged", merged, FLASH_BASE)], "auto: merged image"
    raise FirmwareError("no firmware found (searched: " + ", ".join(str(d) for d in dirs) + ")")


# --------------------------------------------------------------------------
# STM32CubeProgrammer
# --------------------------------------------------------------------------
def find_cubeprog(explicit: Optional[str]) -> Optional[Path]:
    exe_names = ["STM32_Programmer_CLI.exe", "STM32_Programmer_CLI"]
    if explicit:
        p = Path(explicit)
        if p.is_dir():
            for n in exe_names:
                if (p / n).is_file():
                    return p / n
                if (p / "bin" / n).is_file():
                    return p / "bin" / n
            return None
        return p if p.is_file() else None
    env_dir = os.environ.get("STM32_PRG_PATH")
    if env_dir:
        for n in exe_names:
            if (Path(env_dir) / n).is_file():
                return Path(env_dir) / n
    for n in exe_names:
        w = shutil.which(n)
        if w:
            return Path(w)
    rel = os.path.join("STMicroelectronics", "STM32Cube", "STM32CubeProgrammer", "bin")
    patterns = []
    for env in ("ProgramFiles", "ProgramFiles(x86)", "ProgramW6432", "LOCALAPPDATA"):
        base = os.environ.get(env)
        if base:
            patterns.append(os.path.join(base, rel, "STM32_Programmer_CLI.exe"))
            patterns.append(os.path.join(base, "Programs", rel, "STM32_Programmer_CLI.exe"))
    patterns += [
        r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe",
        os.path.expanduser("~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
        "/opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/STM32_Programmer_CLI",
    ]
    for pat in patterns:
        hits = sorted(glob.glob(pat))
        if hits:
            return Path(hits[-1])
    return None


def clean_tool_output(text: str) -> str:
    lines = []
    for line in text.replace("\r\n", "\n").split("\n"):
        line = line.split("\r")[-1]
        printable = "".join(ch for ch in line if 32 <= ord(ch) < 127 or ch == "\t").rstrip()
        if not printable.strip():
            if line.strip():
                continue  # progress bar made of block characters
            if lines and not lines[-1]:
                continue
            lines.append("")
            continue
        lines.append(printable)
    return "\n".join(lines).strip("\n")


def error_lines(text: str) -> List[str]:
    return [ln.strip() for ln in text.splitlines() if re.search(r"\berror\b", ln, re.I)]


class CubeProgrammer:
    def __init__(self, ui: UI, exe: Path, args):
        self.ui = ui
        self.exe = exe
        self.sn: Optional[str] = None
        self.mode = args.connect_mode
        self.freq = args.swd_freq

    def connect_args(self, mode: Optional[str] = None) -> List[str]:
        a = ["-c", "port=SWD", f"mode={mode or self.mode}"]
        if self.freq:
            a.append(f"freq={self.freq}")
        if self.sn:
            a.append(f"sn={self.sn}")
        return a

    def run(self, args: List[str], label: str, timeout: float = 120) -> Tuple[Optional[int], str, float]:
        cmd = [str(self.exe)] + args
        self.ui.log_only("$ " + subprocess.list2cmdline(cmd))
        rc: Optional[int] = None
        with Spinner(self.ui, label) as sp:
            try:
                proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      stdin=subprocess.DEVNULL, timeout=timeout)
                rc, raw = proc.returncode, proc.stdout
            except subprocess.TimeoutExpired as e:
                raw = (e.output or b"") + f"\nError: timed out after {timeout:.0f} s".encode()
            except OSError as e:
                raw = f"Error: cannot run {self.exe}: {e}".encode()
        text = clean_tool_output(raw.decode("utf-8", errors="replace"))
        self.ui.raw(f"{label} (exit code {rc}, {sp.elapsed:.1f} s)", text)
        return rc, text, sp.elapsed


def parse_probes(text: str) -> List[Dict[str, str]]:
    # Only the "STLink Interface" section: the UART section repeats the SN of
    # the ST-Link virtual COM port.
    m = re.search(r"=+\s*ST-?Link Interface\s*=+(.*?)(?:\n\s*=+\s*\w[^\n]*=+|\Z)", text, re.I | re.S)
    if m:
        text = m.group(1)
    probes: List[Dict[str, str]] = []
    for line in text.splitlines():
        m = re.search(r"ST-?LINK\s+SN\s*:\s*(\S+)", line, re.I)
        if m:
            if all(p["sn"] != m.group(1) for p in probes):
                probes.append({"sn": m.group(1), "fw": "?"})
            continue
        m = re.search(r"ST-?LINK\s+FW\s*:\s*(\S+)", line, re.I)
        if m and probes:
            probes[-1]["fw"] = m.group(1)
    return probes


def parse_target(text: str) -> Dict[str, str]:
    info: Dict[str, str] = {}
    pats = {
        "voltage": r"Voltage\s*:\s*([\d.]+)\s*V",
        "device_id": r"Device ID\s*:\s*(0x[0-9A-Fa-f]+)",
        "revision": r"Revision ID\s*:\s*([^\n]+)",
        "name": r"Device name\s*:\s*([^\n]+)",
        "flash": r"Flash size\s*:\s*([^\n]+)",
        "cpu": r"Device CPU\s*:\s*([^\n]+)",
        "swd_freq": r"SWD freq\s*:\s*([^\n]+)",
        "connect_mode": r"Connect mode\s*:\s*([^\n]+)",
    }
    for key, pat in pats.items():
        m = re.search(pat, text, re.I)
        if m:
            info[key] = m.group(1).strip()
    m = re.search(r"0x%08X\s*:\s*([0-9A-Fa-f]{8})\s+([0-9A-Fa-f]{8})\s+([0-9A-Fa-f]{8})" % UID_ADDR, text, re.I)
    if m:
        info["uid"] = "-".join(g.upper() for g in m.groups())
    return info


# --------------------------------------------------------------------------
# MAVLink / PX4 shell
# --------------------------------------------------------------------------
mavutil = None  # imported lazily so that --flash-only works without pymavlink


def import_mavlink_deps() -> Optional[str]:
    global mavutil
    try:
        import serial  # noqa: F401
        import serial.tools.list_ports  # noqa: F401
    except ImportError:
        return "pyserial"
    try:
        from pymavlink import mavutil as _mavutil
    except ImportError:
        return "pymavlink"
    mavutil = _mavutil
    return None


def list_serial_ports():
    import serial.tools.list_ports
    return sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)


def port_text(p) -> str:
    parts = [p.description or ""]
    if p.vid is not None:
        parts.append(f"[{p.vid:04X}:{p.pid:04X}]")
    if p.serial_number:
        parts.append(f"SN {p.serial_number}")
    return " ".join(x for x in parts if x)


def find_fc_ports():
    cands = []
    for p in list_serial_ports():
        text = " ".join(x for x in (p.description, p.manufacturer, p.product, p.hwid) if x)
        if p.vid == 0x0403 or re.search(r"st-?link", text, re.I):
            continue  # FTDI console cable / ST-Link virtual COM port
        score = 0
        if p.vid in FC_USB_VIDS:
            score += 2
        if p.vid == 0x0483 and p.pid == 0x5740:
            score += 1  # generic STM32 virtual COM port used by some PX4 boards
        if FC_TEXT_RE.search(text):
            score += 1
        if score:
            cands.append((score, p))
    cands.sort(key=lambda t: (-t[0], t[1].device))
    return [p for _, p in cands]


class Mav:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.conn = None
        self.sysid = 1
        self.compid = 1
        self._next_hb = 0.0

    def open(self):
        self.conn = mavutil.mavlink_connection(self.port, baud=self.baud, source_system=254,
                                               source_component=190, autoreconnect=False)

    def heartbeat_tick(self):
        now = time.monotonic()
        if now >= self._next_hb:
            self._next_hb = now + 1.0
            try:
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass

    def recv(self, msg_type: str, timeout: float):
        self.heartbeat_tick()
        return self.conn.recv_match(type=msg_type, blocking=True, timeout=timeout)

    def wait_heartbeat(self, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            m = self.recv("HEARTBEAT", 0.3)
            if m is None:
                continue
            if m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID or m.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            self.sysid, self.compid = m.get_srcSystem(), m.get_srcComponent()
            return m
        return None

    def close(self):
        try:
            if self.conn:
                self.conn.close()
        except Exception:
            pass
        self.conn = None


class MavShell:
    """PX4 NSH over MAVLink SERIAL_CONTROL (like Tools/mavlink_shell.py)."""
    DEVICE_SHELL = 10
    PROMPT = "nsh> "

    def __init__(self, mav: Mav, ui: UI):
        self.mav = mav
        self.ui = ui
        self.buf = ""

    def _send(self, text: str):
        data = text.encode("ascii", "replace")
        flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
        while data:
            chunk, data = data[:70], data[70:]
            payload = list(chunk) + [0] * (70 - len(chunk))
            self.mav.conn.mav.serial_control_send(self.DEVICE_SHELL, flags, 0, 0, len(chunk), payload)

    def _pump(self, wait: float = 0.05) -> bool:
        m = self.mav.recv("SERIAL_CONTROL", wait)
        if m is not None and m.count:
            self.buf += bytes(m.data[:m.count]).decode("utf-8", "replace")
            return True
        return False

    def _drain(self, quiet: float = 0.15):
        t_end = time.monotonic() + quiet
        while time.monotonic() < t_end:
            if self._pump(0.03):
                t_end = time.monotonic() + quiet
        self.buf = ""

    def sync(self, timeout: float = 6.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.buf = ""
            self._send("\n")
            t_end = min(deadline, time.monotonic() + 1.0)
            while time.monotonic() < t_end:
                self._pump()
                if self.PROMPT in self.buf:
                    self._drain()
                    return True
        return False

    def run(self, cmd: str, timeout: float = 6.0) -> Tuple[bool, str]:
        t_start = time.monotonic()
        self._drain(0.05)
        self._send(cmd + "\n")
        deadline = time.monotonic() + timeout
        done = False
        while time.monotonic() < deadline:
            self._pump()
            text = self.buf.replace("\r", "")
            nl = text.find("\n")
            if nl >= 0 and text.rstrip(" ").endswith(self.PROMPT.strip()) and len(text) > nl + 1:
                done = True
                break
        text = strip_ansi(self.buf.replace("\r", ""))
        self.buf = ""
        lines = text.split("\n")
        if lines and cmd.strip() and cmd.strip() in lines[0]:
            lines = lines[1:]  # command echo
        out = "\n".join(lines)
        if out.rstrip(" ").endswith(self.PROMPT.strip()):
            out = out.rstrip(" ")[: -len(self.PROMPT.strip())]
        out = out.rstrip()
        self.ui.raw(f"nsh> {cmd}  [{time.monotonic() - t_start:.2f} s{'' if done else ', TIMEOUT: no prompt'}]", out)
        if not done:
            # Stop a command that is still running (e.g. listener waiting for
            # data), otherwise it would swallow the start of the next command.
            self._send("\x03")
            self.sync(3.0)
        return done, out

    def params(self, prefix: str, names: List[str]) -> Dict[str, Optional[float]]:
        """Read several parameters with one 'param show PREFIX*' call."""
        _, out = self.run(f"param show {prefix}*", 6)
        vals: Dict[str, Optional[float]] = {}
        for name in names:
            m = re.search(r"\b%s\s*\[[^\]]*\]\s*:\s*(-?[0-9.]+)" % re.escape(name), out)
            vals[name] = float(m.group(1)) if m else None
        return vals

    def close(self):
        try:
            self.mav.conn.mav.serial_control_send(self.DEVICE_SHELL, 0, 0, 0, 0, [0] * 70)
        except Exception:
            pass


def parse_kv_lines(text: str) -> Dict[str, str]:
    out = {}
    for line in text.splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            out.setdefault(k.strip(), v.strip())
    return out


def _bridge_key(name: str) -> str:
    # The section header prints the short name ("Sensor 'gnss':") and the next
    # line the full one ("name: uavcan_gnss"); both map to "gnss".
    name = name.strip()
    return name[len("uavcan_"):] if name.startswith("uavcan_") else name


def parse_uavcan_status(text: str) -> Dict:
    """Bridges are keyed by short name: "gnss", "mag", "gnss_relative", ..."""
    st: Dict = {"running": "Online nodes" in text, "nodes": {}, "bridges": {}, "ifaces": {}}
    section, bridge, iface = None, None, None
    for line in text.splitlines():
        m = re.match(r"^Sensor '([^']+)':", line)
        if m:
            section, bridge = "sensor", _bridge_key(m.group(1))
            st["bridges"].setdefault(bridge, {})
            continue
        m = re.match(r"^CAN(\d+) status:", line)
        if m:
            section, iface = "iface", int(m.group(1))
            st["ifaces"][iface] = {}
            continue
        if line.startswith("Online nodes"):
            section = "nodes"
            continue
        if section == "sensor":
            m = re.search(r"channel \d+: node id (\d+) --> instance (-?\d+)", line)
            if m:
                st["bridges"][bridge][int(m.group(1))] = int(m.group(2))
        elif section == "iface":
            m = re.match(r"^\s*(HW errors|IO errors|RX frames|TX frames):\s*(\d+)", line)
            if m:
                st["ifaces"][iface][m.group(1)] = int(m.group(2))
        elif section == "nodes":
            m = re.match(r"^\s*(\d+)\s+(\S+)\s+(\S+)", line)
            if m:
                st["nodes"][int(m.group(1))] = (m.group(2), m.group(3))
            elif not line.strip():
                section = None
    return st


def parse_listener(text: str) -> Optional[Dict[str, str]]:
    if "never published" in text or "TOPIC" not in text:
        return None
    fields: Dict[str, str] = {}
    for line in text.splitlines():
        m = re.match(r"^\s+(\w+):\s*(.*)$", line)
        if m:
            fields.setdefault(m.group(1), m.group(2).strip())
    return fields or None


def fnum(fields: Dict[str, str], key: str) -> Optional[float]:
    raw = fields.get(key)
    if raw is None:
        return None
    tok = raw.split()[0] if raw.split() else ""
    try:
        return float(tok)
    except ValueError:
        return None


def fint(fields: Dict[str, str], key: str) -> Optional[int]:
    v = fnum(fields, key)
    return int(v) if v is not None and math.isfinite(v) else None


def age_s(fields: Dict[str, str]) -> Optional[float]:
    m = re.search(r"\(([\d.]+) seconds ago\)", fields.get("timestamp", ""))
    return float(m.group(1)) if m else None


def decode_devid(devid: int) -> Dict[str, int]:
    return {"bus_type": devid & 0x7, "bus": (devid >> 3) & 0x1F,
            "address": (devid >> 8) & 0xFF, "devtype": (devid >> 16) & 0xFF}


def devid_text(devid: int) -> str:
    d = decode_devid(devid)
    return (f"{devid} (type 0x{d['devtype']:02X}, {BUS_NAMES.get(d['bus_type'], '?')}:{d['bus']}, "
            f"addr 0x{d['address']:02X} = {d['address']})")


def fmt(v: Optional[float], spec: str = ".2f", unit: str = "", limit: float = 1e9) -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)) or abs(v) >= limit:
        return "n/a"
    return f"{v:{spec}}{unit}"


def plural(n: int, word: str) -> str:
    return f"{n} {word}" + ("" if n == 1 else "s")


# --------------------------------------------------------------------------
# Stages
# --------------------------------------------------------------------------
ST_CHECKS = ["STM32CubeProgrammer CLI", "ST-Link probe", "Target connection", "Target MCU"]
PX_CHECKS = ["Pixhawk COM port", "MAVLink heartbeat", "MAVLink shell", "UAVCAN configuration"]
NODE_CHECKS = ["CAN node online"]
GNSS_CHECKS = ["GNSS identified", "GNSS data streaming", "GNSS fix"]
MAG_CHECKS = ["Compass identified", "Compass data streaming", "Compass field strength", "Compass data changing"]


def flash_check_names(images: List[FlashImage], args) -> List[str]:
    names = ["Mass erase"] if args.erase_all else []
    for img in images:
        names += [f"Program {ROLE_LABEL[img.role]}", f"Verify {ROLE_LABEL[img.role]}"]
    return names + ["Reset target"]


def stage_firmware(args, ui: UI, res: Results) -> Optional[List[FlashImage]]:
    G = "Firmware"
    ui.step("Firmware images")
    try:
        images, why = select_images(args)
    except FirmwareError as e:
        res.add(G, "Firmware files", FAIL, str(e),
                "pass --hex <merged.hex>, or --bootloader <bl.bin> --app <deploy/1.bin>; "
                "build with boards/aerium/neo-can-gnss/scripts/make_merged_hex.sh")
        return None
    except OSError as e:
        res.add(G, "Firmware files", FAIL, f"cannot read firmware: {e}")
        return None

    ui.info(ui.c(why, GREY))
    for img in images:
        mtime = _dt.datetime.fromtimestamp(img.path.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
        ui.kv(ROLE_LABEL[img.role], f"{img.path}  ({img.path.stat().st_size / 1024:.1f} KB, built {mtime})")

    segs: Segments = []
    for img in images:
        segs += img.segments
    segs = _merge_segments([(a, bytearray(d)) for a, d in segs])

    ok = True
    roles = {img.role for img in images}
    if roles & {"merged", "bootloader"}:
        st, detail, hint = analyze_bootloader(segs)
        ok &= res.add(G, "Bootloader image", st, detail, hint)
    else:
        res.add(G, "Bootloader image", SKIP, "not selected: the bootloader on the unit is kept")
    if roles & {"merged", "app"}:
        st, detail, hint, version = analyze_app(segs)
        ok &= res.add(G, "Application image", st, detail, hint)
        res.unit.firmware = version
    else:
        res.add(G, "Application image", SKIP, "not selected: the application on the unit is kept")

    if ok:
        ui.info("Flash plan:")
        for img in images:
            for a, d in img.segments:
                ui.kv(f"  {ROLE_LABEL[img.role]}",
                      f"0x{a:08X}-0x{a + len(d) - 1:08X}  {len(d) / 1024:7.1f} KB  sectors {sectors_for(a, a + len(d))}")
        if not args.erase_all:
            ui.info(ui.c("Parameter sectors 2-3 (0x08008000-0x0800FFFF) are kept.", GREY))
    return images if ok else None


def stage_stlink(args, ui: UI, res: Results) -> Optional[CubeProgrammer]:
    G = "ST-Link"
    ui.step("ST-Link and target MCU")
    exe = find_cubeprog(args.cubeprog)
    if not exe:
        res.add(G, ST_CHECKS[0], FAIL, "STM32_Programmer_CLI not found",
                "install STM32CubeProgrammer, or pass --cubeprog <path to STM32_Programmer_CLI.exe>")
        res.skip(G, ST_CHECKS[1:], "no programmer tool")
        return None
    prog = CubeProgrammer(ui, exe, args)

    rc, out, _ = prog.run(["-l"], "Scanning for ST-Link probes", timeout=60)
    m = re.search(r"STM32CubeProgrammer\s+(?:version\s*:?\s*)?v?(\d+\.\d+(?:\.\d+)?)", out, re.I)
    res.add(G, ST_CHECKS[0], PASS, f"v{m.group(1) if m else '?'}  {exe}")

    probes = parse_probes(out)
    if not probes:
        errs = error_lines(out)
        res.add(G, ST_CHECKS[1], FAIL, "no ST-Link probe detected" + (f" ({errs[0]})" if errs else ""),
                "check the ST-Link USB cable and driver (STSW-LINK009); close STM32CubeIDE/OpenOCD "
                "and anything else using the probe")
        res.skip(G, ST_CHECKS[2:], "no ST-Link")
        return None
    status, note = PASS, ""
    if args.stlink_sn:
        chosen = [p for p in probes if p["sn"].upper() == args.stlink_sn.upper()]
        if not chosen:
            res.add(G, ST_CHECKS[1], FAIL, f"SN {args.stlink_sn} not found; connected: "
                    + ", ".join(p["sn"] for p in probes))
            res.skip(G, ST_CHECKS[2:], "no ST-Link")
            return None
        probe = chosen[0]
    else:
        probe = probes[0]
        if len(probes) > 1:
            status, note = WARN, f" (first of {len(probes)} probes)"
    prog.sn = probe["sn"]
    res.unit.stlink_sn = probe["sn"]
    res.add(G, ST_CHECKS[1], status, f"SN {probe['sn']}, FW {probe['fw']}{note}",
            "pass --stlink-sn to choose the probe" if status == WARN else "")

    read_uid = ["-r32", f"0x{UID_ADDR:08X}", "0x0C"]
    rc, out, _ = prog.run(prog.connect_args() + read_uid, "Connecting to target")
    info = parse_target(out)
    if "device_id" not in info and args.connect_mode.lower() == "normal":
        ui.warn("no answer in normal mode, retrying with connect-under-reset (needs NRST)")
        rc, out, _ = prog.run(prog.connect_args("UR") + read_uid, "Connecting to target (under reset)")
        info = parse_target(out)
        if "device_id" in info:
            prog.mode = "UR"
    if "device_id" not in info:
        errs = error_lines(out)
        res.add(G, ST_CHECKS[2], FAIL, errs[0] if errs else "target did not answer",
                "check the SWD wiring (SWDIO, SWCLK, GND, NRST), that the node is powered "
                "(CAN cable or 3V3), and that the ST-Link sits on the right pads")
        res.skip(G, ST_CHECKS[3:], "no target")
        return None

    volt = info.get("voltage")
    res.add(G, ST_CHECKS[2], PASS, f"VDD {volt or '?'} V, SWD {info.get('swd_freq', '?')}, "
                                   f"mode {info.get('connect_mode', prog.mode)}")
    for key, label in (("name", "device"), ("revision", "revision"), ("flash", "flash size"),
                       ("cpu", "cpu"), ("uid", "unique ID")):
        if key in info:
            ui.kv(label, info[key])

    dev_id = int(info["device_id"], 16)
    res.unit.uid = info.get("uid", "")
    res.unit.mcu = info.get("name", "")
    detail = f"{info.get('name', '?')} (ID 0x{dev_id:03X}, {info.get('revision', '?')}), flash {info.get('flash', '?')}"
    if dev_id != EXPECTED_DEVICE_ID:
        res.add(G, ST_CHECKS[3], FAIL, detail + f"; expected 0x{EXPECTED_DEVICE_ID:03X} STM32F412",
                "this is not a Neo CAN GNSS board (STM32F412CGU)")
        return None
    flash = info.get("flash", "")
    if flash and not re.match(r"1\s*M", flash, re.I):
        res.add(G, ST_CHECKS[3], WARN, detail, "expected 1 MByte flash (STM32F412CG)")
    else:
        res.add(G, ST_CHECKS[3], PASS, detail)
    return prog


def stage_flash(args, ui: UI, res: Results, prog: CubeProgrammer, images: List[FlashImage]) -> bool:
    G = "Flash"
    ui.step("Flash and reset the CAN node")
    names = flash_check_names(images, args)
    conn = prog.connect_args()

    if args.erase_all:
        rc, out, dt = prog.run(conn + ["-e", "all"], "Mass erase", timeout=180)
        errs = error_lines(out)
        if rc != 0 or errs:
            res.add(G, "Mass erase", FAIL, errs[0] if errs else f"exit code {rc}")
            res.skip(G, names[1:], "erase failed")
            return False
        res.add(G, "Mass erase", PASS, f"whole flash erased in {dt:.1f} s (parameters reset)")

    for img in images:
        label = ROLE_LABEL[img.role]
        cmd = conn + ["-d", str(img.path)]
        if not img.is_hex:
            cmd.append(f"0x{img.addr:08X}")
        cmd.append("-v")
        rc, out, dt = prog.run(cmd, f"Programming {label} ({img.size / 1024:.0f} KB)", timeout=300)
        errs = error_lines(out)
        downloaded = re.search(r"download complete", out, re.I) is not None
        verified = re.search(r"verified successfully", out, re.I) is not None
        m = re.search(r"Time elapsed during download operation:\s*(\S+)", out)
        where = ", ".join(f"0x{a:08X}" for a, _ in img.segments)
        if not downloaded:
            res.add(G, f"Program {label}", FAIL, errs[0] if errs else f"download failed (exit code {rc})",
                    "re-run with --verbose to see the STM32CubeProgrammer output")
            res.skip(G, names[names.index(f"Program {label}") + 1:], "programming failed")
            return False
        res.add(G, f"Program {label}", PASS,
                f"{img.path.name}: {img.size / 1024:.1f} KB @ {where} in {m.group(1) if m else f'{dt:.1f} s'}")
        if not verified:
            res.add(G, f"Verify {label}", FAIL, errs[0] if errs else "read-back verification failed",
                    "flash read-back differs: retry; if it repeats, check the SWD wiring and supply")
            res.skip(G, names[names.index(f"Verify {label}") + 1:], "verify failed")
            return False
        res.add(G, f"Verify {label}", PASS, "flash read-back matches the file")

    rc, out, dt = prog.run(conn + ["-rst"], "Resetting the CAN node")
    errs = error_lines(out)
    if rc not in (0, None) or errs:
        res.add(G, "Reset target", FAIL, errs[0] if errs else f"exit code {rc}",
                "power-cycle the node by hand, then use --verify-only")
        return False
    res.add(G, "Reset target", PASS, "system reset sent; bootloader hands over to the app in ~3 s")
    return True


def connect_pixhawk(port: str, args, ui: UI, timeout: float) -> Tuple[Optional[Mav], Optional[object], str]:
    """Open the port and wait for a PX4 heartbeat. Returns (mav, heartbeat, error)."""
    mav = Mav(port, args.baud)
    deadline = time.monotonic() + timeout
    last_err = ""
    while True:
        try:
            mav.open()
            hb = mav.wait_heartbeat(max(1.0, min(4.0, deadline - time.monotonic())))
            if hb is not None:
                return mav, hb, ""
            last_err = f"no MAVLink heartbeat on {port}"
            mav.close()
        except Exception as e:  # serial.SerialException, OSError
            last_err = str(e)
            mav.close()
            if re.search(r"access is denied|not functioning", last_err, re.I):
                return None, None, last_err  # port busy or USB device broken: waiting does not help
        if time.monotonic() >= deadline:
            return None, None, last_err
        ui.status(f"  waiting for the Pixhawk on {port} ({deadline - time.monotonic():.0f} s): {last_err[:60]}")
        time.sleep(1.0)


def pick_fc_port(args, ui: UI) -> Tuple[Optional[str], str, int]:
    if args.port:
        return args.port, "given with --port", 1
    deadline = time.monotonic() + args.port_timeout
    while True:
        cands = find_fc_ports()
        if cands or time.monotonic() >= deadline:
            break
        ui.status(f"  waiting for a Pixhawk USB port ({deadline - time.monotonic():.0f} s)")
        time.sleep(0.5)
    ui.clear_status()
    if not cands:
        return None, "", 0
    return cands[0].device, port_text(cands[0]), len(cands)


def stage_pixhawk(args, ui: UI, res: Results) -> Optional[Tuple[Mav, MavShell]]:
    G = "Pixhawk"
    ui.step("Pixhawk (PX4) connection")
    port, desc, n_cands = pick_fc_port(args, ui)
    if not port:
        seen = [f"{p.device} {port_text(p)}" for p in list_serial_ports()]
        res.add(G, PX_CHECKS[0], FAIL, "no Pixhawk COM port found; ports: " + ("; ".join(seen) or "none"),
                "connect the Pixhawk USB, or pass --port COMx (see --list-ports)")
        res.skip(G, PX_CHECKS[1:], "no COM port")
        return None
    res.add(G, PX_CHECKS[0], WARN if n_cands > 1 else PASS,
            f"{port}  {desc}" + (f" (first of {n_cands} candidates)" if n_cands > 1 else ""),
            "pass --port to choose" if n_cands > 1 else "")

    mav, hb, err = connect_pixhawk(port, args, ui, args.hb_timeout)
    ui.clear_status()
    if mav is None:
        if re.search(r"access is denied|busy|errno 16", err, re.I):
            hint = "the port is in use: close QGroundControl / Mission Planner / serial terminals"
        elif re.search(r"not functioning|cannot configure|errno 5\b|I/O error", err, re.I):
            hint = "the Pixhawk USB serial is not responding: unplug and replug the Pixhawk USB, then retry"
        elif re.search(r"could not open|FileNotFound|no such file", err, re.I):
            hint = "the port does not exist (yet): check --port, see --list-ports"
        else:
            hint = "wrong port, or MAVLink is not running on USB (SYS_USB_AUTO=2)"
        res.add(G, PX_CHECKS[1], FAIL, err, hint)
        res.skip(G, PX_CHECKS[2:], "no MAVLink")
        return None
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        res.add(G, PX_CHECKS[1], FAIL, f"autopilot type {hb.autopilot} is not PX4", "this script needs PX4 firmware")
        res.skip(G, PX_CHECKS[2:], "not PX4")
        mav.close()
        return None
    res.add(G, PX_CHECKS[1], PASS, f"PX4 autopilot, system id {mav.sysid}, component {mav.compid}")

    shell = MavShell(mav, ui)
    if not shell.sync():
        res.add(G, PX_CHECKS[2], FAIL, "no answer from the MAVLink shell (nsh)", "reconnect the USB and retry")
        res.skip(G, PX_CHECKS[3:], "no shell")
        mav.close()
        return None
    _, ver_out = shell.run("ver all", 6)
    ver = parse_kv_lines(ver_out)
    for key in ("HW arch", "PX4 version", "PX4 git-hash", "Build datetime", "MCU"):
        if key in ver:
            ui.kv(key, ver[key])
    res.unit.pixhawk = ver.get("HW arch", "")
    res.add(G, PX_CHECKS[2], PASS, f"nsh ok, {ver.get('HW arch', '?')}, PX4 {ver.get('PX4 version', '?')}")

    if args.reboot_fc:
        ui.info("Rebooting the Pixhawk to clear stale UAVCAN nodes and sensor channels ...")
        shell.close()
        mav.conn.mav.command_long_send(mav.sysid, mav.compid,
                                       mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1.0)
        mav.close()
        time.sleep(3.0)
        t0 = time.monotonic()
        if not args.port:
            port, desc, _ = pick_fc_port(args, ui)
        mav, hb, err = (connect_pixhawk(port, args, ui, 40) if port else (None, None, "port did not come back"))
        ui.clear_status()
        shell = MavShell(mav, ui) if mav else None
        if mav is None or not shell.sync():
            res.add(G, "Pixhawk reboot", FAIL, err or "shell did not come back after the reboot")
            res.skip(G, PX_CHECKS[3:], "Pixhawk lost")
            if mav:
                mav.close()
            return None
        res.add(G, "Pixhawk reboot", PASS, f"back on {port} after {time.monotonic() - t0 + 4:.0f} s")

    p = shell.params("UAVCAN_", ["UAVCAN_ENABLE", "UAVCAN_SUB_GPS", "UAVCAN_SUB_MAG", "UAVCAN_BITRATE"])
    shown = ", ".join(f"{k}={int(v) if v is not None else '?'}" for k, v in p.items())
    en, sub_gps, sub_mag = p["UAVCAN_ENABLE"], p["UAVCAN_SUB_GPS"], p["UAVCAN_SUB_MAG"]
    if en is None:
        res.add(G, PX_CHECKS[3], FAIL, "UAVCAN_ENABLE not found", "this PX4 build has no UAVCAN driver")
        return mav, shell
    if en < 1:
        res.add(G, PX_CHECKS[3], FAIL, shown, "run 'param set UAVCAN_ENABLE 2' on the Pixhawk and reboot it")
    elif sub_gps == 0 or sub_mag == 0:
        res.add(G, PX_CHECKS[3], FAIL, shown, "set UAVCAN_SUB_GPS=1 and UAVCAN_SUB_MAG=1, then reboot the Pixhawk")
    elif en < 2:
        res.add(G, PX_CHECKS[3], WARN, shown,
                "UAVCAN_ENABLE=1 has no dynamic node ID server; the node needs a static CANNODE_NODE_ID")
    else:
        res.add(G, PX_CHECKS[3], PASS, shown)
    return mav, shell


def stage_node(args, ui: UI, res: Results, shell: MavShell) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    """Wait until the node is online and publishing. Returns (node_id, gnss_instance, mag_instance)."""
    G = "CAN node"
    ui.step("CAN node on the Pixhawk bus")
    t0 = time.monotonic()
    deadline = t0 + args.node_timeout
    seen_modes = set()
    st: Dict = {}
    pick: Optional[int] = None
    timed_out = False
    while True:
        _, out = shell.run("uavcan status", 8)
        st = parse_uavcan_status(out)
        if not st["running"]:
            res.add(G, NODE_CHECKS[0], FAIL, "UAVCAN driver is not running on the Pixhawk",
                    "set UAVCAN_ENABLE=2 and reboot the Pixhawk")
            return None, None, None
        online = {n: hm for n, hm in st["nodes"].items() if hm[1] != "OFFLN"}
        gnss = st["bridges"].get("gnss", {})
        mag = st["bridges"].get("mag", {})
        for n, (_, mode) in online.items():
            seen_modes.add(mode)
        if args.node_id is not None:
            cands = [args.node_id] if args.node_id in online else []
        else:
            cands = sorted(n for n in online if n in gnss or n in mag)
        ready = [n for n in cands if online[n][1] == "OPERAT" and gnss.get(n, -1) >= 0 and mag.get(n, -1) >= 0]
        elapsed = time.monotonic() - t0
        if ready:
            pick = ready[0]
            break
        if time.monotonic() >= deadline:
            pick = cands[0] if cands else None
            timed_out = True
            break
        nodes_txt = ", ".join(f"{n} {h}/{m}" + ("" if n in gnss else " no-gnss") + ("" if n in mag else " no-mag")
                              for n, (h, m) in sorted(online.items())) or "none"
        ui.status(f"  waiting for the node ({elapsed:.0f}/{args.node_timeout:.0f} s)  online: {nodes_txt}")
        t_next = time.monotonic() + 0.5
        while time.monotonic() < t_next:
            shell._pump(0.1)
    ui.clear_status()

    ui.info("Online nodes (Node ID, Health, Mode):")
    for n, (h, m) in sorted(st["nodes"].items()):
        tags = []
        if n in st["bridges"].get("gnss", {}):
            tags.append(f"gnss -> sensor_gps[{st['bridges']['gnss'][n]}]")
        if n in st["bridges"].get("mag", {}):
            tags.append(f"mag -> sensor_mag[{st['bridges']['mag'][n]}]")
        ui.kv(f"  node {n:3d}", f"{h:<5} {MODE_NAMES.get(m, m):<16} {', '.join(tags)}")
    if not st["nodes"]:
        ui.kv("  (none)", "")
    if st["ifaces"]:
        ui.info("Pixhawk CAN interfaces:")
        for i, c in sorted(st["ifaces"].items()):
            ui.kv(f"  CAN{i}", ", ".join(f"{k} {v}" for k, v in c.items()))

    if pick is None:
        hint = ("no CAN node is talking to the Pixhawk: check CANH/CANL wiring and 120 R termination, node power, "
                "that the bitrates match (UAVCAN_BITRATE / CANNODE_BITRATE), and the node console (USART6, 57600)")
        if st["nodes"]:
            hint = ("nodes are online but none sends GNSS/compass data: GPS/mag not detected on the node, or the "
                    "Pixhawk sensor channels are full (max 4 per type since boot): reboot it or use --reboot-fc")
        if args.node_id is not None:
            hint = f"node {args.node_id} is not online. " + hint
        res.add(G, NODE_CHECKS[0], FAIL, f"no matching node after {args.node_timeout:.0f} s", hint)
        return None, None, None

    res.unit.node_id = pick
    health, mode = st["nodes"][pick]
    gnss_i = st["bridges"].get("gnss", {}).get(pick)
    mag_i = st["bridges"].get("mag", {}).get(pick)
    detail = f"node {pick}, health {health}, mode {MODE_NAMES.get(mode, mode)}, after {time.monotonic() - t0:.1f} s"
    if timed_out:
        missing = [what for what, br in (("GNSS", "gnss"), ("compass", "mag"))
                   if st["bridges"].get(br, {}).get(pick, -1) < 0]
        if missing:
            detail += f" (timed out waiting for {' and '.join(missing)} data)"
    multi = [n for n in st["nodes"] if n != pick and (n in st["bridges"].get("gnss", {}))]
    if mode in ("MAINT", "SW_UPD", "INIT"):
        hint = "node is stuck in the CAN bootloader: the application did not start (image/CRC)"
        if "SW_UPD" in seen_modes:
            hint = ("the Pixhawk is updating the node over CAN (a different image is in the SD card ufw folder); "
                    "remove it or wait, then use --verify-only")
        res.add(G, NODE_CHECKS[0], FAIL, detail, hint)
    elif health != "OK":
        res.add(G, NODE_CHECKS[0], WARN, detail, "node reports a degraded health; see its console log")
    elif multi and args.node_id is None:
        res.add(G, NODE_CHECKS[0], WARN, detail + f"; other GNSS nodes: {multi}",
                "several CAN GNSS nodes are on the bus; pass --node-id to pick the unit under test")
    else:
        res.add(G, NODE_CHECKS[0], PASS, detail)
    return pick, gnss_i, mag_i


def _samples(shell: MavShell, topic: str, inst: int, n: int) -> List[Dict[str, str]]:
    """Read n consecutive messages with one listener call (it waits for each new one)."""
    _, text = shell.run(f"listener {topic} -i {inst} -n {n}", 3.0 + 1.5 * n)
    out = []
    for block in re.split(r"^TOPIC: ", text, flags=re.M)[1:]:
        f = parse_listener("TOPIC: " + block)
        if f:
            out.append(f)
    return out


def _check_identity(res: Results, group: str, name: str, fields: Dict[str, str], devtype: int,
                    node: int, topic: str, inst: int, what: str) -> bool:
    devid = fint(fields, "device_id")
    if devid is None:
        return res.add(group, name, FAIL, f"{topic}[{inst}] has no device_id")
    d = decode_devid(devid)
    detail = f"{topic}[{inst}] device_id {devid_text(devid)}"
    problems = []
    if d["bus_type"] != BUS_TYPE_UAVCAN:
        problems.append("bus is not UAVCAN")
    if d["devtype"] != devtype:
        problems.append(f"type 0x{d['devtype']:02X} != 0x{devtype:02X} ({what})")
    if d["address"] != node:
        problems.append(f"address {d['address']} != node {node}")
    if problems:
        return res.add(group, name, FAIL, detail + ": " + "; ".join(problems))
    return res.add(group, name, PASS, detail)


def _check_streaming(res: Results, group: str, name: str, samples: List[Dict[str, str]], max_age: float) -> bool:
    ages = [age_s(s) for s in samples]
    stamps = [fint(s, "timestamp") for s in samples]
    last = ages[-1]
    advancing = len(samples) < 2 or len({t for t in stamps if t}) > 1
    if last is None:
        return res.add(group, name, FAIL, "no timestamp in the data")
    detail = f"last sample {last * 1000:.0f} ms old, {len(samples)} reads"
    if last > max_age or not advancing:
        return res.add(group, name, FAIL, detail + ("" if advancing else ", timestamp not advancing"),
                       "the node stopped sending data; check the node console for driver errors")
    return res.add(group, name, PASS, detail + ", updating")


def stage_gnss(args, ui: UI, res: Results, shell: MavShell, node: int, inst: Optional[int]):
    G = "GNSS"
    ui.step("GNSS (u-blox over UAVCAN Fix2)")
    if inst is None or inst < 0:
        res.add(G, GNSS_CHECKS[0], FAIL, f"no uavcan_gnss channel for node {node}",
                "the node sends no GNSS messages: is the u-blox detected on the node (USART1)? "
                "Is UAVCAN_SUB_GPS=1? Are the 4 Pixhawk GNSS channels used up (reboot it)?")
        res.skip(G, GNSS_CHECKS[1:], "GNSS not identified")
        return
    samples = _samples(shell, "sensor_gps", inst, 3)
    if not samples:
        res.add(G, GNSS_CHECKS[0], FAIL, f"sensor_gps[{inst}] returned no data")
        res.skip(G, GNSS_CHECKS[1:], "no data")
        return
    _check_identity(res, G, GNSS_CHECKS[0], samples[-1], DEVTYPE_GPS_UAVCAN, node, "sensor_gps", inst, "UAVCAN GPS")
    streaming = _check_streaming(res, G, GNSS_CHECKS[1], samples, 2.0)

    def show(s: Dict[str, str]):
        fix = fint(s, "fix_type") or 0
        ui.kv("fix type", f"{fix} ({FIX_NAMES.get(fix, '?')})")
        ui.kv("satellites used", str(fint(s, "satellites_used")))
        ui.kv("position", f"lat {fmt(fnum(s, 'latitude_deg'), '.7f')}  lon {fmt(fnum(s, 'longitude_deg'), '.7f')}  "
                          f"alt {fmt(fnum(s, 'altitude_msl_m'), '.1f', ' m')}")
        ui.kv("accuracy", f"eph {fmt(fnum(s, 'eph'), '.2f', ' m', 1e5)}  epv {fmt(fnum(s, 'epv'), '.2f', ' m', 1e5)}  "
                          f"hdop {fmt(fnum(s, 'hdop'), limit=99)}  vdop {fmt(fnum(s, 'vdop'), limit=99)}")
        ui.kv("velocity", f"{fmt(fnum(s, 'vel_m_s'), '.2f', ' m/s')}")
        utc = fint(s, "time_utc_usec") or 0
        ui.kv("UTC time", _dt.datetime.fromtimestamp(utc / 1e6, _dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
              if utc else "not yet")
        jam = fint(s, "jamming_state")
        if jam is not None:
            ui.kv("jamming", f"{JAMMING_NAMES.get(jam, jam)} (indicator {fint(s, 'jamming_indicator')})")

    s = samples[-1]
    if not streaming:
        show(s)
        res.add(G, GNSS_CHECKS[2], SKIP, "no live data")
        return

    def has_fix(f: Dict[str, str]) -> bool:
        return (fint(f, "fix_type") or 0) >= 3 and (fint(f, "satellites_used") or 0) >= args.min_sats

    if args.require_fix and not has_fix(s):
        deadline = time.monotonic() + args.fix_timeout
        while time.monotonic() < deadline:
            fix = fint(s, "fix_type") or 0
            ui.status(f"  waiting for a 3D fix ({deadline - time.monotonic():.0f} s left): "
                      f"fix {FIX_NAMES.get(fix, fix)}, {fint(s, 'satellites_used') or 0} sats")
            got = _samples(shell, "sensor_gps", inst, 1)
            if got:
                s = got[0]
                if has_fix(s):
                    break
            t_end = time.monotonic() + 2.0
            while time.monotonic() < t_end:
                shell._pump(0.1)
        ui.clear_status()
    show(s)

    fix = fint(s, "fix_type") or 0
    sats = fint(s, "satellites_used") or 0
    lat, lon = fnum(s, "latitude_deg") or 0.0, fnum(s, "longitude_deg") or 0.0
    detail = f"fix {FIX_NAMES.get(fix, fix)}, {sats} sats"
    if fix >= 2:
        detail += f", eph {fmt(fnum(s, 'eph'), '.1f', ' m', 1e5)}"
    if has_fix(s):
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            res.add(G, GNSS_CHECKS[2], FAIL, detail + ", but position is 0/0")
        else:
            res.add(G, GNSS_CHECKS[2], PASS, detail + f", {lat:.5f}/{lon:.5f}")
    elif args.require_fix:
        res.add(G, GNSS_CHECKS[2], FAIL, detail + f" after {args.fix_timeout:.0f} s",
                f"needs a 3D fix with >= {args.min_sats} satellites: check the antenna and the sky view")
    else:
        res.add(G, GNSS_CHECKS[2], WARN, detail + " (receiver is talking, no 3D fix yet)",
                "normal indoors; use --require-fix outdoors to make this a hard check")


def stage_compass(args, ui: UI, res: Results, shell: MavShell, node: int, inst: Optional[int]):
    G = "Compass"
    ui.step("Compass (LIS3MDL over UAVCAN MagneticFieldStrength2)")
    if inst is None or inst < 0:
        res.add(G, MAG_CHECKS[0], FAIL, f"no uavcan_mag channel for node {node}",
                "the node sends no compass messages: is the LIS3MDL detected on the node (I2C1, 0x1E)? "
                "Is UAVCAN_SUB_MAG=1? Are the 4 Pixhawk mag channels used up (reboot it)?")
        res.skip(G, MAG_CHECKS[1:], "compass not identified")
        return
    samples = _samples(shell, "sensor_mag", inst, args.mag_samples)
    if not samples:
        res.add(G, MAG_CHECKS[0], FAIL, f"sensor_mag[{inst}] returned no data")
        res.skip(G, MAG_CHECKS[1:], "no data")
        return
    _check_identity(res, G, MAG_CHECKS[0], samples[-1], DEVTYPE_MAG_UAVCAN, node, "sensor_mag", inst, "UAVCAN mag")
    streaming = _check_streaming(res, G, MAG_CHECKS[1], samples, 1.0)

    vecs = []
    for s in samples:
        x, y, z = fnum(s, "x"), fnum(s, "y"), fnum(s, "z")
        if None not in (x, y, z) and all(math.isfinite(v) for v in (x, y, z)):
            vecs.append((x, y, z))
    if not vecs:
        res.add(G, MAG_CHECKS[2], FAIL, "no finite x/y/z values")
        res.add(G, MAG_CHECKS[3], SKIP, "no values")
        return
    for i, (x, y, z) in enumerate(vecs, 1):
        ui.kv(f"sample {i}", f"x {x:+.4f}  y {y:+.4f}  z {z:+.4f} G   |B| {math.sqrt(x * x + y * y + z * z):.4f} G")
    norms = [math.sqrt(x * x + y * y + z * z) for x, y, z in vecs]
    mean = sum(norms) / len(norms)
    x, y, z = (sum(v[i] for v in vecs) / len(vecs) for i in range(3))
    detail = f"|B| {mean:.3f} G (x {x:+.3f} y {y:+.3f} z {z:+.3f}), limits {args.mag_min:.2f}-{args.mag_max:.2f} G"
    if mean < 1e-4:
        res.add(G, MAG_CHECKS[2], FAIL, detail, "all axes read zero: sensor not measuring")
    elif not (args.mag_min <= mean <= args.mag_max):
        res.add(G, MAG_CHECKS[2], FAIL, detail,
                "outside the Earth-field range: magnets, steel or current-carrying wires near the unit, "
                "or a faulty sensor")
    else:
        res.add(G, MAG_CHECKS[2], PASS, detail)

    if len(vecs) < 2:
        res.add(G, MAG_CHECKS[3], SKIP, "need at least 2 samples")
        return
    spread = max(max(v[i] for v in vecs) - min(v[i] for v in vecs) for i in range(3))
    pp = (max(norms) - min(norms)) * 1000
    if spread == 0.0:
        res.add(G, MAG_CHECKS[3], FAIL if streaming else WARN, f"{len(vecs)} samples are identical",
                "values are frozen: the node repeats a stale reading")
    else:
        res.add(G, MAG_CHECKS[3], PASS, f"{len(vecs)} samples, noise {pp:.1f} mG peak-peak on |B|")


# --------------------------------------------------------------------------
# Summary and main flow
# --------------------------------------------------------------------------
def print_summary(ui: UI, res: Results, elapsed: float, unit_no: Optional[int]):
    w = ui.width
    ui.out()
    ui.rule("=", (CYAN,))
    title = "SUMMARY" + (f"  unit #{unit_no}" if unit_no else "")
    ui.out(ui.c("  " + title, BOLD, CYAN) + ("   " + res.unit.label() if res.unit.label() else ""))
    ui.rule("=", (CYAN,))
    ui.out(ui.c(f"  {'#':>2}  {'Group':<9} {'Check':<26} {'Result':<7} Details", BOLD))
    detail_w = max(20, w - 52)
    for i, chk in enumerate(res.items, 1):
        detail = chk.detail if len(chk.detail) <= detail_w else chk.detail[:detail_w - 3] + "..."
        ui.out(f"  {i:>2}  {chk.group:<9} {chk.name:<26} {ui.tag(chk.status)}  {detail}", log=False)
        ui.log_only(f"  {i:>2}  {chk.group:<9} {chk.name:<26} [{chk.status}]  {chk.detail}")
    ui.rule("-")
    n_pass, n_warn, n_fail, n_skip = (res.count(s) for s in (PASS, WARN, FAIL, SKIP))
    counts = f"{n_pass} passed, {plural(n_warn, 'warning')}, {n_fail} failed, {n_skip} skipped  ({elapsed:.1f} s)"
    if res.passed:
        verdict = ui.c("  PASS  ", BOLD, BLACK_FG, BG_GREEN) if ui.color else "PASS"
    else:
        verdict = ui.c("  FAIL  ", BOLD, WHITE, BG_RED) if ui.color else "FAIL"
    ui.out(f"  RESULT: {verdict}   {counts}")
    problems = [c for c in res.items if c.status in (FAIL, WARN)]
    if problems:
        ui.out()
        ui.out(ui.c("  To look at:", BOLD))
        for c in problems:
            ui.out(f"   {ui.tag(c.status)} {c.group} / {c.name}: {c.detail}")
            if c.hint:
                ui.out("          " + ui.c(c.hint, YELLOW))
    ui.rule("=", (CYAN,))


def run_once(args, ui: UI, unit_no: Optional[int] = None) -> Results:
    res = Results(ui)
    t0 = time.monotonic()
    stamp = _dt.datetime.now()
    if args.log:
        path = Path(args.log) if args.log != "auto" else Path("logs") / f"neo-can-gnss_{stamp:%Y%m%d-%H%M%S}.log"
        if args.log != "auto" and unit_no and unit_no > 1:
            path = path.with_name(f"{path.stem}_unit{unit_no}{path.suffix}")
        ui.open_log(path)

    do_flash, do_verify = not args.verify_only, not args.flash_only
    ui.step_no, ui.step_total = 0, (3 if do_flash else 0) + (5 if do_verify else 0)
    ui.banner("Aerium Neo CAN GNSS  -  flash & verify" + (f"   [unit #{unit_no}]" if unit_no else ""),
              [f"{stamp:%Y-%m-%d %H:%M:%S}   script v{SCRIPT_VERSION}   python {sys.version.split()[0]}   "
               f"mode: {'flash + verify' if do_flash and do_verify else 'flash only' if do_flash else 'verify only'}"])

    mav: Optional[Mav] = None
    shell: Optional[MavShell] = None
    try:
        flashed = True
        if do_flash:
            flashed = False
            images = stage_firmware(args, ui, res)
            if images is None:
                res.skip("ST-Link", ST_CHECKS, "firmware check failed")
                res.skip("Flash", ["Program and verify", "Reset target"], "firmware check failed")
            else:
                prog = stage_stlink(args, ui, res)
                if prog is None:
                    res.skip("Flash", flash_check_names(images, args), "no ST-Link / target")
                else:
                    flashed = stage_flash(args, ui, res, prog, images)

        if do_verify:
            if not flashed:
                res.skip("Pixhawk", PX_CHECKS, "flashing did not complete")
                res.skip("CAN node", NODE_CHECKS, "flashing did not complete")
                res.skip("GNSS", GNSS_CHECKS, "flashing did not complete")
                res.skip("Compass", MAG_CHECKS, "flashing did not complete")
            else:
                conn = stage_pixhawk(args, ui, res)
                if conn is None:
                    res.skip("CAN node", NODE_CHECKS, "no Pixhawk")
                    res.skip("GNSS", GNSS_CHECKS, "no Pixhawk")
                    res.skip("Compass", MAG_CHECKS, "no Pixhawk")
                else:
                    mav, shell = conn
                    node, gnss_i, mag_i = stage_node(args, ui, res, shell)
                    if node is None:
                        res.skip("GNSS", GNSS_CHECKS, "node not online")
                        res.skip("Compass", MAG_CHECKS, "node not online")
                    else:
                        stage_gnss(args, ui, res, shell, node, gnss_i)
                        stage_compass(args, ui, res, shell, node, mag_i)
    except KeyboardInterrupt:
        ui.clear_status()
        ui.out()
        ui.error("interrupted by the user")
        res.items.append(Check("Run", "Interrupted", FAIL, "Ctrl-C"))
        raise
    finally:
        if shell:
            shell.close()
        if mav:
            mav.close()
        print_summary(ui, res, time.monotonic() - t0, unit_no)
        log_path = ui.log_path
        ui.close_log()
        if log_path and args.log == "auto":
            final = log_path.with_name(f"{log_path.stem}_{res.unit.uid or 'noUID'}_"
                                       f"{'PASS' if res.passed else 'FAIL'}.log")
            try:
                log_path.rename(final)
                log_path = final
            except OSError:
                pass
        if log_path:
            ui.out(ui.c(f"  Log: {log_path.resolve()}", GREY), log=False)
        ui.log_path = None
    return res


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Flash the Aerium Neo CAN GNSS over ST-Link and verify its GNSS and compass on a PX4 Pixhawk.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Example: py flash_and_verify.py --hex aerium_neo-can-gnss_merged.hex --port COM7 --log")
    fw = p.add_argument_group("firmware")
    fw.add_argument("--hex", metavar="FILE", help="merged bootloader+app Intel HEX (make_merged_hex.sh)")
    fw.add_argument("--bootloader", metavar="FILE", help="canbootloader .bin (flashed at 0x08000000)")
    fw.add_argument("--app", metavar="FILE", help="application .bin with boot descriptor, deploy/1.bin (0x08010000)")

    st = p.add_argument_group("ST-Link / STM32CubeProgrammer")
    st.add_argument("--cubeprog", metavar="PATH", help="path to STM32_Programmer_CLI(.exe) or its folder")
    st.add_argument("--stlink-sn", metavar="SN", help="serial number of the ST-Link to use")
    st.add_argument("--connect-mode", default="normal", choices=["normal", "UR", "HOTPLUG"],
                    help="SWD connect mode (default normal, falls back to UR); UR needs NRST wired")
    st.add_argument("--swd-freq", metavar="KHZ", type=int, help="SWD clock in kHz (default: tool default)")
    st.add_argument("--erase-all", action="store_true", help="mass-erase first (also resets node parameters)")

    fc = p.add_argument_group("Pixhawk")
    fc.add_argument("--port", metavar="COMx", help="Pixhawk serial port (default: auto-detect)")
    fc.add_argument("--baud", type=int, default=115200, help="baud rate (ignored on USB, default 115200)")
    fc.add_argument("--port-timeout", type=float, default=10, metavar="S", help="wait for the USB port (10 s)")
    fc.add_argument("--hb-timeout", type=float, default=15, metavar="S", help="wait for a heartbeat (15 s)")
    fc.add_argument("--reboot-fc", action="store_true",
                    help="reboot the Pixhawk before the checks (clears stale nodes; use when testing many units)")
    fc.add_argument("--node-id", type=int, metavar="N", help="UAVCAN node ID of the unit (default: auto)")
    fc.add_argument("--node-timeout", type=float, default=45, metavar="S",
                    help="wait for the node with GNSS and compass data (45 s)")

    ck = p.add_argument_group("sensor limits")
    ck.add_argument("--require-fix", action="store_true", help="fail unless the GNSS reaches a 3D fix")
    ck.add_argument("--fix-timeout", type=float, default=120, metavar="S", help="wait for the fix (120 s)")
    ck.add_argument("--min-sats", type=int, default=6, metavar="N", help="satellites for a good fix (6)")
    ck.add_argument("--mag-min", type=float, default=0.15, metavar="G", help="lowest field strength (0.15 G)")
    ck.add_argument("--mag-max", type=float, default=0.90, metavar="G", help="highest field strength (0.90 G)")
    ck.add_argument("--mag-samples", type=int, default=5, metavar="N", help="compass samples to read (5)")

    run = p.add_argument_group("run")
    mode = run.add_mutually_exclusive_group()
    mode.add_argument("--flash-only", action="store_true", help="flash and reset, skip the Pixhawk checks")
    mode.add_argument("--verify-only", action="store_true", help="skip flashing, only check the node")
    run.add_argument("--loop", action="store_true", help="production mode: repeat for one unit after another")
    run.add_argument("--log", nargs="?", const="auto", metavar="FILE",
                     help="write a plain-text log (default name: logs/neo-can-gnss_<time>_<UID>_<result>.log)")
    run.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    run.add_argument("--no-color", action="store_true", help="disable colors")
    run.add_argument("-v", "--verbose", action="store_true", help="show raw tool and shell output")
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.hex and (args.bootloader or args.app):
        print("error: use either --hex or --bootloader/--app, not both")
        return 2
    color = not args.no_color and enable_ansi()
    ui = UI(color, args.verbose)

    if not args.flash_only or args.list_ports:
        missing = import_mavlink_deps()
        if missing:
            ui.error(f"Python package '{missing}' is missing. Install the dependencies with:")
            ui.out("      py -m pip install pyserial pymavlink")
            return 2
    if args.list_ports:
        cands = {p.device for p in find_fc_ports()}
        ports = list_serial_ports()
        for p in ports:
            mark = ui.c("  <- Pixhawk candidate", GREEN) if p.device in cands else ""
            ui.out(f"  {p.device:<14} {port_text(p)}{mark}")
        if not ports:
            ui.out("  no serial ports found")
        return 0

    if not args.loop:
        try:
            res = run_once(args, ui)
        except KeyboardInterrupt:
            return 130
        return 0 if res.passed else 1

    tally: List[Tuple[int, str, bool]] = []
    unit = 0
    try:
        while True:
            unit += 1
            res = run_once(args, ui, unit)
            tally.append((unit, res.unit.label() or "?", res.passed))
            ok = sum(1 for t in tally if t[2])
            ui.out(ui.c(f"  Units so far: {len(tally)}   pass {ok}   fail {len(tally) - ok}", BOLD))
            try:
                ans = input(ui.c("\n  Connect the next unit and press Enter (q + Enter to quit): ", BOLD, CYAN))
            except EOFError:
                break
            if ans.strip().lower().startswith("q"):
                break
    except KeyboardInterrupt:
        pass
    ui.out()
    ui.out(ui.c("  Session results:", BOLD))
    for n, label, ok in tally:
        ui.out(f"   unit #{n:<3} {ui.tag(PASS if ok else FAIL)}  {label}")
    return 0 if tally and all(t[2] for t in tally) else 1


if __name__ == "__main__":
    sys.exit(main())
