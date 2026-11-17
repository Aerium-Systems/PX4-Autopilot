#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aerium Apex H7 rev A - flash over USB DFU and verify the sensors (Windows).

What it does
  1. Checks the two images: the PX4 bootloader (0x08000000) and the PX4 app
     (0x08020000). A .px4 app file also gives the board ID and git hash.
  2. Waits for the Apex in STM32 ROM DFU mode (hold BOOT while plugging USB),
     checks that the target is an STM32H743 (2 MB), programs and verifies both
     images, then starts the board.
  3. Waits for PX4 on the Apex USB COM port and opens the MAVLink shell.
  4. Checks firmware identity, then the presence, identity (device ID) and
     sanity of every onboard sensor:
       ICM-42688-P (SPI2), BMI270 or ICM-45686 (SPI1), DPS368 baro (SPI1),
       LIS3MDL mag (I2C2, internal), plus SD card, battery ADC and boot log.
  5. Prints a PASS/WARN/FAIL summary and writes a log file.

  The test takes about 10 s. Keep the board still on the bench during the test.
  With --flat (board flat, top side up) the absolute IMU orientation is
  checked as well.

Exit code: 0 = pass (warnings allowed), 1 = a check failed, 2 = setup error,
130 = interrupted.

Requirements (Windows 10/11)
  - Python 3.8 or newer:  py -m pip install pyserial pymavlink
  - STM32CubeProgrammer (STM32_Programmer_CLI.exe, found automatically in its
    default install folder) or dfu-util on PATH (needs the WinUSB driver on the
    "STM32 BOOTLOADER" device, e.g. via Zadig).
  - Close QGroundControl first: it would hold the Apex COM port.

Firmware files (searched next to this script, in ./firmware, in the current
folder and in the PX4 source tree when run from it)
  - aerium_apex_h7_rev_a_bootloader.bin   (boards/aerium/apex_h7_rev_a/extras)
  - aerium_apex_h7_rev_a_default.px4      (or the .bin, from the build folder)

Examples
  py flash_and_verify.py                       flash both images, then test
  py flash_and_verify.py --test-only           test a running board (no DFU)
  py flash_and_verify.py --flat                also check absolute orientation
  py flash_and_verify.py --mass-erase          also wipe parameters (sector 15)
  py flash_and_verify.py --port COM43 --test-only
"""

import argparse
import base64
import datetime
import glob
import json
import math
import os
import queue
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
import zlib
from pathlib import Path
from typing import Dict, List, Optional, Tuple

BOARD = "aerium_apex_h7_rev_a"
BOARD_NAME = "AERIUM_APEX_H7_REV_A"   # 'ver hw' -> "HW arch: ..."
BOARD_ID = 1803
USB_VID = 0x1B8C                      # Aerium (bootloader and app)
DFU_VID_PID = "0483:df11"             # STM32 ROM DFU
STM32H7_DEV_ID = 0x450                # STM32H742/743/753/750

BL_ADDR = 0x08000000
BL_MAX = 128 * 1024                   # sector 0
APP_ADDR = 0x08020000
APP_MAX = 0x081E0000 - APP_ADDR       # sectors 1..14; sector 15 = params
RAM_RANGES = ((0x20000000, 0x20020000), (0x24000000, 0x24080000))

G = 9.80665

# PX4 device IDs: devtype << 16 | address << 8 | bus << 3 | bus_type
BUS_I2C, BUS_SPI = 1, 2


def dev_id(devtype: int, bus_type: int, bus: int, addr: int = 0) -> int:
    return (devtype << 16) | (addr << 8) | (bus << 3) | bus_type


ID_ICM42688P = dev_id(0x26, BUS_SPI, 2)        # 0x260012
ID_BMI270 = dev_id(0x37, BUS_SPI, 1)           # 0x37000A
ID_ICM45686 = dev_id(0x34, BUS_SPI, 1)         # 0x34000A
ID_DPS310 = dev_id(0x68, BUS_SPI, 1)           # 0x68000A
ID_LIS3MDL = dev_id(0x05, BUS_I2C, 2, 0x1E)    # 0x051E11

SPI1_IMUS = {ID_BMI270: "BMI270", ID_ICM45686: "ICM-45686"}

# Sanity limits (board at rest on the bench)
ACCEL_NORM = (8.8, 10.8)          # m/s^2
ACCEL_AGREE_DEG = 8.0             # max angle between the two IMUs' gravity
GYRO_MAX = 0.10                   # rad/s, uncalibrated at rest
FLAT_MAX_DEG = 15.0               # --flat: max tilt from (0, 0, -g)
TEMP_RANGE = (-20.0, 90.0)        # degC (heater regulates the IMUs to ~55)
PRESS_RANGE = (70000.0, 110000.0)  # Pa
MAG_NORM_FAIL = (0.05, 4.0)       # Gauss: outside = broken
MAG_NORM_WARN = (0.20, 0.70)      # Gauss: outside = strong local field / uncalibrated
MAX_AGE_S = 0.5                   # data must be fresher than this
MIN_RATE = {"sensor_accel": 100, "sensor_gyro": 100, "sensor_baro": 5, "sensor_mag": 5}

# Known boot-log errors and what they mean on this board
BOOTLOG_HINTS = [
    (r"param import failed|importing failed",
     "no valid parameter set in flash (sector 15): normal on the first boot after an erase; "
     "the SD backup was loaded instead. Repeats on every boot = params are not saved to flash."),
    (r"BAT4_|param 65535 invalid",
     "UAVCAN_SUB_BAT != 0: the DroneCAN battery bridge creates battery #4, but PX4 only has "
     "BAT1..3 params (upstream PX4 issue, harmless). Set UAVCAN_SUB_BAT 0 if no CAN battery monitor."),
]

ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


# --------------------------------------------------------------------------- UI

class UI:
    def __init__(self, color: bool, verbose: bool):
        self.color = color
        self.verbose = verbose
        self.log = None
        self.results: List[Tuple[str, str, str]] = []
        self.tty = sys.stdout.isatty()
        self.lock = threading.RLock()
        self._status_len = 0
        self.step_no = 0
        self.total_steps = 0
        self.t_start = time.monotonic()

    def c(self, text: str, code: str) -> str:
        return f"\x1b[{code}m{text}\x1b[0m" if self.color else text

    def _log(self, text: str):
        if self.log:
            self.log.write(ANSI_RE.sub("", text) + "\n")
            self.log.flush()

    def status(self, text: str):
        """One live line, redrawn in place (console only)."""
        if not self.tty:
            return
        with self.lock:
            cols = shutil.get_terminal_size((100, 20)).columns
            text = text[: (cols if cols >= 40 else 100) - 1]
            sys.stdout.write("\r" + text + " " * max(0, self._status_len - len(text)))
            sys.stdout.flush()
            self._status_len = len(text)

    def clear_status(self):
        with self.lock:
            if self._status_len:
                sys.stdout.write("\r" + " " * self._status_len + "\r")
                sys.stdout.flush()
                self._status_len = 0

    def out(self, text: str = ""):
        with self.lock:
            self.clear_status()
            print(text, flush=True)
        self._log(text)

    def step(self, title: str, numbered: bool = True):
        if numbered:
            self.step_no += 1
        n = f"[{self.step_no}/{self.total_steps}] " if numbered and self.total_steps else ""
        self.out()
        self.out(self.c(f"== {n}{title}", "1;36") + self.c(f"   +{time.monotonic() - self.t_start:.0f} s", "90"))

    def info(self, text: str):
        self.out("   " + text)

    def raw(self, title: str, text: str):
        self._log(f"--- {title}\n{text}\n---")
        if self.verbose:
            with self.lock:
                self.clear_status()
                print(self.c(f"   --- {title}", "90"))
                for line in text.splitlines():
                    print(self.c("   | " + line, "90"))

    def check(self, name: str, status: str, detail: str = "") -> bool:
        colors = {"PASS": "32", "WARN": "33", "FAIL": "1;31", "INFO": "36"}
        self.results.append((status, name, detail))
        self.out(f"   [{self.c(status, colors[status])}] {name}" + (f": {detail}" if detail else ""))
        return status != "FAIL"

    def summary(self) -> int:
        n = {s: sum(1 for r in self.results if r[0] == s) for s in ("PASS", "WARN", "FAIL")}
        self.step("Summary", numbered=False)
        for status, name, detail in self.results:
            if status in ("FAIL", "WARN"):
                self.info(f"{status}: {name}" + (f" ({detail})" if detail else ""))
        verdict = self.c(" PASS ", "1;42") if n["FAIL"] == 0 else self.c(" FAIL ", "1;41")
        self.out(f"\n   {verdict}  {n['PASS']} passed, {n['WARN']} warnings, {n['FAIL']} failed")
        return 0 if n["FAIL"] == 0 else 1


class SetupError(Exception):
    pass


class Activity:
    """Live status line for a running step: spinner, label, optional progress
    bar, detail and elapsed time. Without a console only label changes print."""
    SPIN = "|/-\\"
    BAR = 24

    def __init__(self, ui: UI, label: str, timeout: Optional[float] = None):
        self.ui, self.label, self.timeout = ui, label, timeout
        self.percent: Optional[int] = None
        self.detail = ""
        self.t0 = self.elapsed = 0.0
        self._stop = threading.Event()
        self._thread = None

    def set(self, label: Optional[str] = None, percent: Optional[int] = -1, detail: Optional[str] = None):
        if label is not None and label != self.label:
            self.label = label
            if not self.ui.tty:
                self.ui.out(f"   ... {label}")
        if percent != -1:
            self.percent = percent
        if detail is not None:
            self.detail = detail

    def render(self) -> str:
        el = time.monotonic() - self.t0
        s = f"   {self.SPIN[int(el * 8) % 4]} {self.label}"
        if self.percent is not None:
            fill = int(max(0, min(100, self.percent)) * self.BAR / 100)
            s += f"  [{'#' * fill}{'.' * (self.BAR - fill)}] {self.percent:3d}%"
        if self.detail:
            s += f"  {self.detail}"
        return s + f"  {el:.0f} s" + (f"/{self.timeout:.0f} s" if self.timeout else "")

    def _run(self):
        while not self._stop.wait(0.1):
            self.ui.status(self.render())

    def __enter__(self):
        self.t0 = time.monotonic()
        if self.ui.tty:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        else:
            self.ui.out(f"   ... {self.label}")
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._thread:
            self._thread.join()
        self.ui.clear_status()
        self.elapsed = time.monotonic() - self.t0
        return False


def enable_ansi() -> bool:
    if not sys.stdout.isatty():
        return False
    if os.name == "nt":
        os.system("")  # enables VT100 escape processing in the Windows console
    return True


# --------------------------------------------------------------------- images

class Image:
    def __init__(self, role: str, path: Path, data: bytes, addr: int, meta: Optional[dict] = None):
        self.role, self.path, self.data, self.addr, self.meta = role, path, data, addr, meta or {}


def load_image(role: str, path: Path, addr: int) -> Image:
    if not path.is_file():
        raise SetupError(f"{role}: file not found: {path}")
    if path.suffix.lower() == ".px4":
        try:
            meta = json.loads(path.read_text(encoding="utf-8"))
            data = zlib.decompress(base64.b64decode(meta["image"]))
        except Exception as e:
            raise SetupError(f"{role}: cannot decode {path.name}: {e}")
        if int(meta.get("board_id", BOARD_ID)) != BOARD_ID:
            raise SetupError(f"{role}: {path.name} is for board_id {meta.get('board_id')}, not {BOARD_ID}")
        meta.pop("image", None)
        meta.pop("parameter_xml", None)
        meta.pop("airframe_xml", None)
        return Image(role, path, data, addr, meta)
    if path.suffix.lower() != ".bin":
        raise SetupError(f"{role}: use a .bin or .px4 file, not {path.name}")
    return Image(role, path, path.read_bytes(), addr)


def check_vector_table(img: Image, max_size: int):
    """The first two words are the initial stack pointer and the reset vector."""
    if len(img.data) < 1024 or len(img.data) > max_size:
        raise SetupError(f"{img.role}: size {len(img.data)} B outside 1 KB..{max_size // 1024} KB")
    sp, reset = struct.unpack_from("<II", img.data, 0)
    if not any(lo <= sp <= hi for lo, hi in RAM_RANGES):
        raise SetupError(f"{img.role}: initial SP 0x{sp:08X} is not in RAM - not a firmware image?")
    if not (reset & 1) or not (img.addr <= reset < img.addr + len(img.data)):
        raise SetupError(f"{img.role}: reset vector 0x{reset:08X} is outside 0x{img.addr:08X}+"
                         f"{len(img.data)} - wrong file or wrong link address")


def search_dirs() -> List[Path]:
    here = Path(__file__).resolve().parent
    dirs = [here, here / "firmware", Path.cwd(), Path.cwd() / "firmware"]
    board_dir = here.parent
    if (board_dir / "default.px4board").is_file():
        dirs.append(board_dir / "extras")
        repo = board_dir.parent.parent.parent
        dirs.append(repo / "build" / f"{BOARD}_default")
    out = []
    for d in dirs:
        if d.is_dir() and d not in out:
            out.append(d)
    return out


def find_file(names: List[str]) -> Optional[Path]:
    for d in search_dirs():
        for n in names:
            if (d / n).is_file():
                return d / n
    return None


def select_images(args, ui: UI) -> Tuple[Image, Image]:
    bl_path = Path(args.bootloader) if args.bootloader else find_file([f"{BOARD}_bootloader.bin"])
    app_path = Path(args.app) if args.app else find_file([f"{BOARD}_default.px4", f"{BOARD}_default.bin"])
    if not bl_path:
        raise SetupError(f"bootloader image {BOARD}_bootloader.bin not found (use --bootloader)")
    if not app_path:
        raise SetupError(f"app image {BOARD}_default.px4/.bin not found (use --app)")
    bl = load_image("bootloader", bl_path, BL_ADDR)
    app = load_image("app", app_path, APP_ADDR)
    check_vector_table(bl, BL_MAX)
    check_vector_table(app, APP_MAX)
    for img in (bl, app):
        age = datetime.datetime.fromtimestamp(img.path.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
        ui.info(f"{img.role:<10} {img.path}  ({len(img.data) / 1024:.0f} KB @ 0x{img.addr:08X}, {age})")
    if app.meta:
        ui.info(f"{'':<10} git {app.meta.get('git_identity', '?')}  hash {app.meta.get('git_hash', '?')[:12]}")
    return bl, app


# ---------------------------------------------------------------------- DFU

def run_tool(ui: UI, cmd: List[str], label: str, timeout: float = 180, on_line=None) -> Tuple[Optional[int], str]:
    """Run a tool and stream its output; on_line() gets every line and every
    progress-bar redraw (\\r) as it arrives."""
    ui._log("$ " + subprocess.list2cmdline(cmd))
    t0 = time.monotonic()
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    except OSError as e:
        text = f"Error: cannot run {cmd[0]}: {e}"
        ui.raw(label, text)
        return None, text
    q: "queue.Queue[Optional[bytes]]" = queue.Queue()

    def reader():
        read = getattr(proc.stdout, "read1", None) or (lambda n: proc.stdout.read(1))
        while True:
            b = read(512)
            if not b:
                break
            q.put(b)
        q.put(None)

    threading.Thread(target=reader, daemon=True).start()
    chunks: List[bytes] = []
    pending = ""
    rc: Optional[int] = None
    while True:
        try:
            b = q.get(timeout=0.1)
        except queue.Empty:
            if time.monotonic() - t0 > timeout:
                proc.kill()
                chunks.append(f"\nError: timed out after {timeout:.0f} s".encode())
                break
            continue
        if b is None:
            break
        chunks.append(b)
        if on_line:
            pending += b.decode("utf-8", "replace")
            parts = re.split(r"[\r\n]", pending)
            pending = parts.pop()
            for part in parts:
                if part.strip():
                    on_line(part)
            if "%" in pending:
                on_line(pending)  # progress bar not yet terminated by \r
    try:
        rc = proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    text = b"".join(chunks).decode("utf-8", "replace").replace("\r\n", "\n")
    # drop progress-bar redraws
    text = "\n".join(ln.split("\r")[-1] for ln in text.split("\n"))
    text = "\n".join(ln for ln in text.split("\n") if not ln.strip() or any(32 <= ord(c) < 127 for c in ln))
    ui.raw(f"{label} (exit {rc}, {time.monotonic() - t0:.1f} s)", text.strip())
    return rc, text


PERCENT_RE = re.compile(r"(\d{1,3})\s*%")


def progress_parser(act: Activity, role: str):
    """Map STM32CubeProgrammer / dfu-util output to phase + percent."""
    def on_line(line: str):
        low = line.lower()
        if "erasing" in low or re.match(r"erase\s*\[", low):
            act.set(f"{role}: erase", None)
        elif "download in progress" in low or re.match(r"download\s*\[", low):  # CubeProgrammer / dfu-util
            if not act.label.endswith("write"):
                act.set(f"{role}: write", 0)
        elif "read progress" in low or "verifying" in low:
            if not act.label.endswith("verify"):
                act.set(f"{role}: verify", 0)
        elif "running program" in low or "start operation" in low:
            act.set(f"{role}: start", None)
        m = PERCENT_RE.search(line)
        if m:
            act.set(percent=min(100, int(m.group(1))))
    return on_line


def error_lines(text: str) -> List[str]:
    return [ln.strip() for ln in text.splitlines() if re.search(r"\berror\b", ln, re.I)]


def find_cubeprog(explicit: Optional[str]) -> Optional[Path]:
    names = ["STM32_Programmer_CLI.exe", "STM32_Programmer_CLI"]
    if explicit:
        p = Path(explicit)
        for cand in ([p] if p.is_file() else [p / n for n in names] + [p / "bin" / n for n in names]):
            if cand.is_file():
                return cand
        return None
    for n in names:
        if shutil.which(n):
            return Path(shutil.which(n))
    rel = os.path.join("STMicroelectronics", "STM32Cube", "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI.exe")
    pats = []
    for env in ("ProgramFiles", "ProgramW6432", "ProgramFiles(x86)", "LOCALAPPDATA"):
        if os.environ.get(env):
            pats += [os.path.join(os.environ[env], rel), os.path.join(os.environ[env], "Programs", rel)]
    pats.append(r"C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe")
    for pat in pats:
        hits = sorted(glob.glob(pat))
        if hits:
            return Path(hits[-1])
    return None


class CubeProgDfu:
    name = "STM32CubeProgrammer"

    def __init__(self, ui: UI, exe: Path):
        self.ui, self.exe, self.port = ui, exe, "USB1"

    def present(self) -> bool:
        rc, out = run_tool(self.ui, [str(self.exe), "-l", "usb"], "list DFU devices", 30)
        m = re.search(r"Device Index\s*:\s*(USB\d+)", out)
        if m:
            self.port = m.group(1)
        return bool(m)

    def identify(self) -> str:
        rc, out = run_tool(self.ui, [str(self.exe), "-c", f"port={self.port}"], "connect", 60)
        m = re.search(r"Device ID\s*:\s*0x([0-9A-Fa-f]+)", out)
        if rc != 0 or not m:
            raise SetupError("cannot connect to the DFU device (see the log; driver installed?)")
        did = int(m.group(1), 16)
        name = re.search(r"Device name\s*:\s*(.+)", out)
        if did != STM32H7_DEV_ID:
            raise SetupError(f"DFU device ID 0x{did:03X} is not an STM32H743 (0x{STM32H7_DEV_ID:03X})")
        return f"device ID 0x{did:03X} ({name.group(1).strip() if name else 'STM32H7'})"

    def _run(self, args: List[str], act: Activity, role: str) -> Tuple[Optional[int], str]:
        return run_tool(self.ui, [str(self.exe), "-c", f"port={self.port}"] + args, role, 300,
                        on_line=progress_parser(act, role))

    def flash(self, bl: Image, app: Image, mass_erase: bool) -> bool:
        if mass_erase:
            with Activity(self.ui, "mass erase: connect") as act:
                rc, out = self._run(["-e", "all"], act, "mass erase")
            if not report(self.ui, "Mass erase", rc == 0 and not error_lines(out), act, out, "whole flash"):
                return False
        # one call per image so each gets its own progress line; the app call also starts the board
        for img, extra in ((bl, []), (app, ["-s", f"0x{BL_ADDR:08X}"])):
            with Activity(self.ui, f"{img.role}: connect") as act:
                rc, out = self._run(["-d", str(tmp_bin(img)), f"0x{img.addr:08X}", "-v"] + extra, act, img.role)
            ok = re.search(r"Download verified successfully", out, re.I) is not None
            if not report(self.ui, img.role.capitalize(), ok, act, out, "written + verified", img):
                return False
        return True


class DfuUtil:
    name = "dfu-util"

    def __init__(self, ui: UI, exe: Path):
        self.ui, self.exe = ui, exe
        self.listing = ""

    def present(self) -> bool:
        rc, out = run_tool(self.ui, [str(self.exe), "-l"], "list DFU devices", 30)
        self.listing = out
        return f"[{DFU_VID_PID}]" in out.lower()

    def identify(self) -> str:
        m = re.search(r'name="@Internal Flash\s*/0x08000000/([^"]+)"', self.listing)
        layout = m.group(1).strip() if m else "?"
        if "16*128K" not in layout.replace(" ", ""):
            raise SetupError(f"DFU flash layout '{layout}' is not a 2 MB STM32H743")
        return f"internal flash {layout}"

    def flash(self, bl: Image, app: Image, mass_erase: bool) -> bool:
        base = [str(self.exe), "-d", DFU_VID_PID, "-a", "0"]
        if mass_erase:
            with Activity(self.ui, "mass erase") as act:
                rc, out = run_tool(self.ui, base + ["-s", f"0x{BL_ADDR:08X}:mass-erase:force"], "mass erase", 120,
                                   on_line=progress_parser(act, "mass erase"))
            if not report(self.ui, "Mass erase", rc == 0, act, out, "whole flash"):
                return False
        for img, leave in ((bl, False), (app, True)):
            addr = f"0x{img.addr:08X}" + (":leave" if leave else "")
            with Activity(self.ui, f"{img.role}: connect") as act:
                rc, out = run_tool(self.ui, base + ["-s", addr, "-D", str(tmp_bin(img))], img.role, 300,
                                   on_line=progress_parser(act, img.role))
            # dfu-util often reports an error status after ':leave' although the download is complete
            ok = re.search(r"Download done|File downloaded successfully", out) is not None
            if not report(self.ui, img.role.capitalize(), ok, act, out, "written (dfu-util does not verify)", img):
                return False
        return True


def report(ui: UI, name: str, ok: bool, act: Activity, out: str, what: str, img: Optional[Image] = None) -> bool:
    if img is not None:
        name += f" ({len(img.data) / 1024:.0f} KB @ 0x{img.addr:08X})"
    ui.check(name, "PASS" if ok else "FAIL", f"{what}, {act.elapsed:.1f} s" if ok else "see the log")
    if not ok:
        for ln in error_lines(out)[:5]:
            ui.info("  " + ln)
    return ok


_tmp_files: List[Path] = []


def tmp_bin(img: Image) -> Path:
    if img.path.suffix.lower() == ".bin":
        return img.path
    import tempfile
    fd, name = tempfile.mkstemp(prefix=f"{BOARD}_{img.role}_", suffix=".bin")
    with os.fdopen(fd, "wb") as f:
        f.write(img.data)
    _tmp_files.append(Path(name))
    return Path(name)


def select_tool(ui: UI, args):
    if args.tool in ("auto", "cubeprog"):
        exe = find_cubeprog(args.cubeprog)
        if exe:
            return CubeProgDfu(ui, exe)
        if args.tool == "cubeprog":
            raise SetupError("STM32_Programmer_CLI not found (install STM32CubeProgrammer or use --cubeprog)")
    exe = args.dfu_util or shutil.which("dfu-util") or shutil.which("dfu-util.exe")
    if exe and Path(exe).is_file():
        return DfuUtil(ui, Path(exe))
    raise SetupError("no DFU tool found: install STM32CubeProgrammer, or put dfu-util on PATH")


def do_flash(ui: UI, args, bl: Image, app: Image):
    ui.step("Flash over USB DFU")
    tool = select_tool(ui, args)
    ui.info(f"tool: {tool.name} ({tool.exe})")
    with Activity(ui, "looking for the STM32 DFU device") as act:
        found = tool.present()
    if not found:
        ui.info(ui.c("Put the Apex in DFU mode: hold BOOT and plug in USB", "33"))
        with Activity(ui, "waiting for the STM32 DFU device", timeout=args.dfu_timeout) as act:
            while not tool.present():
                if time.monotonic() - act.t0 > args.dfu_timeout:
                    raise SetupError("no STM32 DFU device found")
                time.sleep(0.5)
    with Activity(ui, "connecting to the DFU device"):
        ident = tool.identify()
    ui.check("DFU target", "PASS", ident)
    t0 = time.monotonic()
    if not tool.flash(bl, app, args.mass_erase):
        raise SetupError("flashing failed")
    ui.info(f"flashed in {time.monotonic() - t0:.0f} s, board started")


# ------------------------------------------------------------------- MAVLink

mavutil = None


def import_mavlink():
    global mavutil
    os.environ.setdefault("MAVLINK20", "1")
    try:
        import serial.tools.list_ports  # noqa: F401
        from pymavlink import mavutil as m
    except ImportError as e:
        raise SetupError(f"missing Python package ({e.name}): py -m pip install pyserial pymavlink")
    mavutil = m


def apex_ports(explicit: Optional[str]) -> List[str]:
    if explicit:
        return [explicit]
    import serial.tools.list_ports
    ports = []
    for p in serial.tools.list_ports.comports():
        text = " ".join(x for x in (p.description, p.manufacturer, p.product) if x)
        if p.vid == USB_VID or re.search(r"apex|aerium", text, re.I):
            ports.append(p.device)
    return sorted(ports)


class Mav:
    def __init__(self, port: str):
        self.port = port
        self.conn = mavutil.mavlink_connection(port, baud=115200, source_system=254,
                                               source_component=190, autoreconnect=False)
        self.next_hb = 0.0

    def recv(self, msg_type: str, timeout: float):
        now = time.monotonic()
        if now >= self.next_hb:  # only after PX4 was seen: never write to the bootloader
            self.next_hb = now + 1.0
            try:
                self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                             0, 0, 0)
            except Exception:
                pass
        return self.conn.recv_match(type=msg_type, blocking=True, timeout=timeout)

    def close(self):
        try:
            self.conn.close()
        except Exception:
            pass


def probe_heartbeat(port: str) -> Tuple[bool, str]:
    """Listen (never write) on a port for up to 2.5 s for a PX4 heartbeat."""
    try:
        conn = mavutil.mavlink_connection(port, baud=115200, source_system=254,
                                          source_component=190, autoreconnect=False)
    except Exception as e:
        return False, f"{port} busy ({e})"
    found, err = False, ""
    try:
        t_end = time.monotonic() + 2.5
        while time.monotonic() < t_end and not found:
            m = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
            found = bool(m and m.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4)
    except Exception as e:  # port vanished (bootloader -> app re-enumeration)
        err = f"{port}: {e}"
    try:
        conn.close()
    except Exception:
        pass
    return found, err


def connect_px4(ui: UI, args) -> Mav:
    """Wait for a PX4 heartbeat. The PX4 bootloader uses the same USB ID, so
    ports without a heartbeat are only listened to, never written to."""
    last_err, found, port = "", False, ""
    with Activity(ui, "waiting for PX4 (bootloader 5 s, then boot)", timeout=args.boot_timeout) as act:
        while time.monotonic() - act.t0 < args.boot_timeout:
            ports = apex_ports(args.port)
            if not ports:
                act.set(detail="no Apex COM port yet")
                time.sleep(0.5)
                continue
            for port in ports:
                act.set(detail=f"{port}: no heartbeat yet")
                found, err = probe_heartbeat(port)
                last_err = err or last_err
                if found:
                    break
            if found:
                break
            time.sleep(0.3)
    if found:
        ui.check("PX4 on USB", "PASS", f"{port}, heartbeat after {act.elapsed:.0f} s")
        return Mav(port)
    hint = f" (last error: {last_err})" if last_err else ""
    raise SetupError(f"no PX4 heartbeat on {args.port or 'any Apex COM port'} within "
                     f"{args.boot_timeout:.0f} s{hint}. QGroundControl open? Replug USB without BOOT.")


class Shell:
    """PX4 NSH over MAVLink SERIAL_CONTROL (as QGC's MAVLink console)."""
    DEVICE = 10
    PROMPT = "nsh>"

    def __init__(self, mav: Mav, ui: UI):
        self.mav, self.ui, self.buf = mav, ui, ""

    def _send(self, text: str):
        data = text.encode("ascii", "replace")
        flags = (mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND)
        while data:
            chunk, data = data[:70], data[70:]
            self.mav.conn.mav.serial_control_send(self.DEVICE, flags, 0, 0, len(chunk),
                                                  list(chunk) + [0] * (70 - len(chunk)))

    def _pump(self, wait: float = 0.05):
        m = self.mav.recv("SERIAL_CONTROL", wait)
        if m is not None and m.count:
            self.buf += bytes(m.data[:m.count]).decode("utf-8", "replace")

    def sync(self, timeout: float = 6.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.buf = ""
            self._send("\n")
            t_end = min(deadline, time.monotonic() + 1.0)
            while time.monotonic() < t_end:
                self._pump()
                if self.PROMPT in self.buf:
                    t_quiet = time.monotonic() + 0.15
                    while time.monotonic() < t_quiet:
                        self._pump(0.03)
                    self.buf = ""
                    return True
        return False

    def run(self, cmd: str, timeout: float = 5.0) -> str:
        t0 = time.monotonic()
        self.ui.status(f"   > {cmd} ...")
        self.buf = ""
        self._send(cmd + "\n")
        done = False
        while time.monotonic() < t0 + timeout:
            self._pump()
            text = self.buf.replace("\r", "")
            if "\n" in text and text.rstrip().endswith(self.PROMPT):
                done = True
                break
        text = ANSI_RE.sub("", self.buf.replace("\r", ""))
        self.buf = ""
        lines = text.split("\n")
        if lines and cmd in lines[0]:
            lines = lines[1:]  # echo
        out = "\n".join(lines).rstrip()
        if out.endswith(self.PROMPT):
            out = out[: -len(self.PROMPT)].rstrip()
        self.ui.clear_status()
        self.ui.raw(f"nsh> {cmd}  [{time.monotonic() - t0:.2f} s{'' if done else ', no prompt'}]", out)
        if not done:
            self._send("\x03")
            self.sync(3.0)
        return out

    def close(self):
        try:
            self.mav.conn.mav.serial_control_send(self.DEVICE, 0, 0, 0, 0, [0] * 70)
        except Exception:
            pass


# ----------------------------------------------------------------- parsing

def parse_listener(text: str) -> List[Dict[str, str]]:
    """Split 'listener <topic>' output into one dict per instance."""
    msgs: List[Dict[str, str]] = []
    for line in text.splitlines():
        m = re.match(r"^\s*(\w+):\s*(.*)$", line)
        if not m:
            continue
        key, val = m.group(1), m.group(2).strip()
        if key == "timestamp":
            msgs.append({})
        if msgs:
            msgs[-1].setdefault(key, val)
    return msgs


def num(msg: Dict[str, str], key: str) -> Optional[float]:
    m = re.match(r"^\s*(-?[0-9.]+(?:e[-+]?\d+)?|nan|-?inf)", msg.get(key, ""), re.I)
    return float(m.group(1)) if m else None


def age_s(msg: Dict[str, str]) -> Optional[float]:
    m = re.search(r"\(([0-9.]+) seconds ago\)", msg.get("timestamp", ""))
    return float(m.group(1)) if m else None


def vec(msg: Dict[str, str]) -> Optional[Tuple[float, float, float]]:
    v = tuple(num(msg, k) for k in ("x", "y", "z"))
    return v if all(x is not None and math.isfinite(x) for x in v) else None


def norm(v) -> float:
    return math.sqrt(sum(x * x for x in v))


def angle_deg(a, b) -> float:
    d = sum(x * y for x, y in zip(a, b)) / max(norm(a) * norm(b), 1e-9)
    return math.degrees(math.acos(max(-1.0, min(1.0, d))))


def fmt_v(v) -> str:
    return "(" + ", ".join(f"{x:+.2f}" for x in v) + ")"


def by_id(msgs: List[Dict[str, str]]) -> Dict[int, Dict[str, str]]:
    out = {}
    for m in msgs:
        d = num(m, "device_id")
        if d is not None:
            out[int(d)] = m
    return out


def parse_rates(text: str) -> Dict[Tuple[str, int], int]:
    rates = {}
    for line in text.splitlines():
        m = re.match(r"^\s*(\w+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$", line)
        if m:
            rates[(m.group(1), int(m.group(2)))] = int(m.group(4))
    return rates


# -------------------------------------------------------------------- tests

def test_board(ui: UI, sh: Shell, args, app: Optional[Image]):
    ui.step("Firmware")
    ver = sh.run("ver all")
    hw = re.search(r"HW arch:\s*(\S+)", ver)
    ui.check("Board", "PASS" if hw and hw.group(1) == BOARD_NAME else "FAIL",
             hw.group(1) if hw else "no 'HW arch' in 'ver all'")
    fw_hash = re.search(r"PX4 git-hash:\s*([0-9a-f]+)", ver)
    fw_ver = re.search(r"PX4 version:\s*(.+?)\s*\(", ver)
    uid = re.search(r"PX4GUID:\s*(\S+)", ver)
    detail = f"{fw_ver.group(1) if fw_ver else '?'} {fw_hash.group(1)[:12] if fw_hash else '?'}"
    if app is not None and app.meta.get("git_hash") and fw_hash:
        same = fw_hash.group(1).startswith(app.meta["git_hash"][:12])
        ui.check("Running firmware = flashed image", "PASS" if same else "FAIL",
                 detail if same else f"running {detail}, image {app.meta['git_hash'][:12]}")
    else:
        ui.check("Firmware version", "INFO", detail)
    if uid:
        ui.check("PX4GUID", "INFO", uid.group(1))

    # Let freshly started drivers settle (heater, first publications).
    if args.settle > 0:
        time.sleep(args.settle)

    ui.step("Sensors")
    accel = by_id(parse_listener(sh.run("listener sensor_accel")))
    gyro = by_id(parse_listener(sh.run("listener sensor_gyro")))
    baro = by_id(parse_listener(sh.run("listener sensor_baro")))
    mag = by_id(parse_listener(sh.run("listener sensor_mag")))
    rates = parse_rates(sh.run("uorb top -1 sensor_accel sensor_gyro sensor_baro sensor_mag", 6))
    # sensor_mag has no internal/external flag; the calibration status line has it
    status = sh.run("sensors status", 6)

    # which part sits on the SPI1 IMU footprint (U19)
    spi1 = [i for i in SPI1_IMUS if i in accel or i in gyro]
    imus = [(ID_ICM42688P, "ICM-42688-P (SPI2)")]
    if spi1:
        imus.append((spi1[0], f"{SPI1_IMUS[spi1[0]]} (SPI1)"))
    else:
        ui.check("SPI1 IMU (BMI270 / ICM-45686)", "FAIL", "neither found on SPI1")

    def common(name: str, msg: Dict[str, str]) -> bool:
        ok = True
        a = age_s(msg)
        if a is None or a > MAX_AGE_S:
            ok = ui.check(f"{name} live", "FAIL", f"last sample {a if a is not None else '?'} s ago")
        ec = num(msg, "error_count") or 0
        if ec > 0:
            ui.check(f"{name} errors", "WARN", f"error_count {int(ec)}")
        t = num(msg, "temperature")
        if t is not None and not (TEMP_RANGE[0] <= t <= TEMP_RANGE[1]):
            ok = ui.check(f"{name} temperature", "FAIL", f"{t:.1f} degC")
        return ok

    accel_vecs = []
    for did, name in imus:
        a, g = accel.get(did), gyro.get(did)
        if not a or not g:
            ui.check(name, "FAIL", f"device 0x{did:06X} missing ({'accel' if not a else ''}"
                                   f"{' gyro' if not g else ''} not published)")
            continue
        ok = common(f"{name} accel", a) & common(f"{name} gyro", g)
        va, vg = vec(a), vec(g)
        if va is None or vg is None:
            ui.check(name, "FAIL", "no valid x/y/z")
            continue
        n = norm(va)
        if not (ACCEL_NORM[0] <= n <= ACCEL_NORM[1]):
            ok = ui.check(f"{name} accel |g|", "FAIL", f"{n:.2f} m/s^2 {fmt_v(va)}")
        if norm(vg) > GYRO_MAX:
            ok = ui.check(f"{name} gyro at rest", "FAIL", f"{norm(vg):.3f} rad/s {fmt_v(vg)} - board moving?")
        if args.flat:
            tilt = angle_deg(va, (0.0, 0.0, -1.0))
            if tilt > FLAT_MAX_DEG:
                ok = ui.check(f"{name} orientation (flat)", "FAIL",
                              f"gravity {fmt_v(va)} is {tilt:.0f} deg from (0, 0, -g): wrong -R rotation?")
        accel_vecs.append((name, va))
        if ok:
            t = num(a, "temperature")
            ui.check(name, "PASS", f"0x{did:06X}, accel {fmt_v(va)} |{n:.2f}|, gyro {norm(vg):.3f} rad/s"
                                   + (f", {t:.1f} degC" if t is not None else ""))

    if len(accel_vecs) == 2:
        (n1, v1), (n2, v2) = accel_vecs
        ang = angle_deg(v1, v2)
        ui.check("IMU orientation agreement", "PASS" if ang <= ACCEL_AGREE_DEG else "FAIL",
                 f"{ang:.1f} deg between the gravity vectors"
                 + ("" if ang <= ACCEL_AGREE_DEG else " - one IMU has the wrong -R rotation in rc.board_sensors"))

    b = baro.get(ID_DPS310)
    if not b:
        ui.check("DPS368 baro (SPI1)", "FAIL", f"device 0x{ID_DPS310:06X} not published")
    elif common("DPS368 baro", b):
        p = num(b, "pressure")
        ok = p is not None and PRESS_RANGE[0] <= p <= PRESS_RANGE[1]
        ui.check("DPS368 baro (SPI1)", "PASS" if ok else "FAIL",
                 f"0x{ID_DPS310:06X}, {p / 100 if p else 0:.1f} hPa, {num(b, 'temperature') or 0:.1f} degC")

    m = mag.get(ID_LIS3MDL)
    if not m:
        ui.check("LIS3MDL mag (I2C2)", "FAIL", f"device 0x{ID_LIS3MDL:06X} not published")
    elif common("LIS3MDL mag", m):
        v = vec(m)
        n = norm(v) if v else 0.0
        cal = re.search(r"\b%d\b[^\n]*?(Internal|Ext ROT)" % ID_LIS3MDL, status)
        if cal and cal.group(1) != "Internal":
            ui.check("LIS3MDL mag (I2C2)", "FAIL", "treated as external (i2c.cpp override not active)")
        elif not (MAG_NORM_FAIL[0] <= n <= MAG_NORM_FAIL[1]):
            ui.check("LIS3MDL mag (I2C2)", "FAIL", f"|B| {n:.2f} G {fmt_v(v or (0, 0, 0))}")
        else:
            ok = MAG_NORM_WARN[0] <= n <= MAG_NORM_WARN[1]
            ui.check("LIS3MDL mag (I2C2)", "PASS" if ok else "WARN",
                     f"0x{ID_LIS3MDL:06X} internal, |B| {n:.2f} G"
                     + ("" if ok else " (outside 0.2..0.7 G: local field / not calibrated)"))
    extra = [f"0x{d:06X}" for d in mag if d != ID_LIS3MDL]
    if extra:
        ui.check("External mags", "INFO", ", ".join(extra))

    slow = []
    for (topic, inst), rate in sorted(rates.items()):
        if topic in MIN_RATE and rate < MIN_RATE[topic]:
            slow.append(f"{topic}[{inst}] {rate} Hz")
    if rates:
        ui.check("Publication rates", "WARN" if slow else "PASS",
                 ", ".join(slow) if slow else ", ".join(f"{t.replace('sensor_', '')}[{i}] {r}"
                                                         for (t, i), r in sorted(rates.items())))

    ui.step("Board")
    mounts = sh.run("mount")
    sd = "/fs/microsd" in mounts
    ui.check("microSD (SPI4)", "PASS" if sd else ("FAIL" if args.require_sd else "WARN"),
             "mounted" if sd else "not mounted (no card?)")

    bat = parse_listener(sh.run("listener battery_status"))
    if bat:
        v = num(bat[0], "voltage_v")
        ui.check("Battery ADC", "INFO", f"{v:.2f} V" if v is not None else "no voltage")
    else:
        ui.check("Battery ADC", "WARN", "battery_status not published")

    log = sh.run("dmesg", 8)
    counts: Dict[str, int] = {}
    for ln in log.splitlines():
        if re.search(r"hardfault|ERROR|panic|assert", ln, re.I):
            counts[ln.strip()] = counts.get(ln.strip(), 0) + 1
    fatal = any(re.search(r"hardfault|panic|assert", ln, re.I) for ln in counts)
    ui.check("Boot log", "FAIL" if fatal else ("WARN" if counts else "PASS"),
             f"{sum(counts.values())} error lines" if counts else "no errors")
    for ln, n in list(counts.items())[:10]:
        ui.info(f"  {n}x {ln}" if n > 1 else f"     {ln}")
    for pattern, hint in BOOTLOG_HINTS:
        if any(re.search(pattern, ln) for ln in counts):
            ui.info(ui.c(f"  -> {hint}", "90"))


# --------------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description="Flash the Aerium Apex H7 rev A over USB DFU and verify its sensors.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test-only", action="store_true", help="skip flashing, test a running board")
    ap.add_argument("--flash-only", action="store_true", help="flash, then stop (no test)")
    ap.add_argument("--bootloader", help=f"bootloader .bin (default: {BOARD}_bootloader.bin)")
    ap.add_argument("--app", help=f"app .px4 or .bin (default: {BOARD}_default.px4)")
    ap.add_argument("--mass-erase", action="store_true", help="erase the whole flash first (also the parameters)")
    ap.add_argument("--tool", choices=("auto", "cubeprog", "dfu-util"), default="auto")
    ap.add_argument("--cubeprog", help="path to STM32_Programmer_CLI.exe or its folder")
    ap.add_argument("--dfu-util", help="path to dfu-util.exe")
    ap.add_argument("--port", help="Apex COM port (default: auto, USB VID 1B8C)")
    ap.add_argument("--flat", action="store_true", help="board lies flat, top up: also check absolute IMU orientation")
    ap.add_argument("--require-sd", action="store_true", help="fail when no microSD card is mounted")
    ap.add_argument("--dfu-timeout", type=float, default=60, help="seconds to wait for DFU (default 60)")
    ap.add_argument("--boot-timeout", type=float, default=45, help="seconds to wait for PX4 (default 45)")
    ap.add_argument("--settle", type=float, default=1.0, help="seconds to wait before reading sensors (default 1)")
    ap.add_argument("--log-dir", help="log folder (default: ./logs next to this script)")
    ap.add_argument("-v", "--verbose", action="store_true", help="print tool and shell output")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()

    ui = UI(color=not args.no_color and enable_ansi(), verbose=args.verbose)
    log_dir = Path(args.log_dir) if args.log_dir else Path(__file__).resolve().parent / "logs"
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"apex_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"
        ui.log = open(log_path, "w", encoding="utf-8")
    except OSError:
        log_path = None

    ui.total_steps = (0 if args.test_only else 2) + (0 if args.flash_only else 4)
    t_start = ui.t_start = time.monotonic()
    ui.out(ui.c("Aerium Apex H7 rev A - flash and verify", "1"))
    mav = sh = None
    app = None
    try:
        if not args.test_only:
            ui.step("Firmware images")
            bl, app = select_images(args, ui)
            do_flash(ui, args, bl, app)
        elif args.app:
            app = load_image("app", Path(args.app), APP_ADDR)
        if args.flash_only:
            rc = ui.summary()
        else:
            import_mavlink()
            ui.step("Connect")
            mav = connect_px4(ui, args)
            sh = Shell(mav, ui)
            if not sh.sync():
                raise SetupError("no MAVLink shell prompt (another GCS holding the shell?)")
            test_board(ui, sh, args, app)
            rc = ui.summary()
    except SetupError as e:
        ui.out()
        ui.out(ui.c(f"SETUP ERROR: {e}", "1;31"))
        rc = 2
    except KeyboardInterrupt:
        ui.out(ui.c("\ninterrupted", "33"))
        rc = 130
    finally:
        if sh:
            sh.close()
        if mav:
            mav.close()
        for f in _tmp_files:
            try:
                f.unlink()
            except OSError:
                pass
    ui.out(f"\n   {time.monotonic() - t_start:.0f} s total" + (f", log: {log_path}" if log_path else ""))
    if ui.log:
        ui.log.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
