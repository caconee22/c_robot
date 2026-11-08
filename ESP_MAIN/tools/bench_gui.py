"""C_ROBOT bench GUI for robot_bench_v2_manual (UART0, 57600).

Two tabs:
  시험    - live status, log, command buttons, hold-to-drive pad, Sunday checklist
  색 교정 - guided floor color calibration (48 steps) run inside the bench firmware

Run with a Python that has tkinter and pyserial (PlatformIO's own python has
no tkinter):
    python tools/bench_gui.py
    python -m pip install pyserial        # once, if "No module named serial"
Pick "DEMO" in the port list to try the window without a board.

Command-line mode (no window), e.g. for an AI assistant driving the bench:
    python tools/bench_gui.py --cmd COM5 "STATUS" "MODE DATA" --listen 3
DRIVE is refused in --cmd mode unless --allow-drive is given; then each
"DRIVE l r" is repeated for --drive-seconds (max 3) and followed by STOP.

Session logs: ESP_MAIN/logs/bench_<date>_<time>.log (git-ignored). Every line
starts with a local timestamp and a marker, so a person or an AI (Codex,
Claude) can replay what happened:
    <  line received from the board          >  line sent to the board
    #  GUI action or note                     !  warning, error or Python traceback
"""
import argparse
import datetime
import math
import os
import queue
import sys
import threading
import time
import traceback

REPEAT_S = 0.1          # well under the firmware's 250 ms manual hold
DRIVE_LIMIT = 400       # firmware bench output limit (permille)
LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
CORNERS = {1: "앞 왼쪽", 2: "앞 오른쪽", 3: "뒤 왼쪽", 4: "뒤 오른쪽"}
# Firmware order (color_calibration.cpp STEPS): per color level all 4, lifted all 4,
# pressed front pair, pressed rear pair. Yellow is not calibrated (11/04).
CAL_ORDER = [(color, pose, sensors) for color in ("BLACK", "RED", "BLUE")
             for pose, sensors in (("LEVEL", (1, 2, 3, 4)), ("LIFTED", (1, 2, 3, 4)),
                                   ("PRESSED", (1, 2)), ("PRESSED", (3, 4)))]
CAL_STEPS = len(CAL_ORDER)
# AS7341 ADC full scale = (ATIME+1)*(ASTEP+1) from sensor_config.hpp; 5 ms gives 1798.
# shortcut: fixed to the 5 ms setting, update it if AS7341_EXPOSURE_US changes.
AS7341_FULL_SCALE = 1798
NEAR_SATURATION = int(AS7341_FULL_SCALE * 0.9)  # firmware drops samples at full scale
# Live spectrum bars in the calibration tab: channel -> (bar color, label).
SPECTRUM = {"F2": ("#6A4FC9", "보라 445"), "F3": ("#2F6FD6", "파랑 480"), "F5": ("#2E9E4F", "초록 555"),
            "F6": ("#E0A21B", "주황 590"), "F7": ("#D23B2E", "빨강 630"), "C": ("#8A8A8A", "전체"),
            "R": ("#D23B2E", "R"), "G": ("#2E9E4F", "G"), "B": ("#2F6FD6", "B")}
SENSOR_NAMES = {1: "1 앞 왼쪽", 2: "2 앞 오른쪽", 3: "3 뒤 왼쪽", 4: "4 뒤 오른쪽"}
CAL_STATES = {0: "Idle", 1: "WaitingForReady", 2: "Countdown", 3: "Measuring",
              4: "WaitingForAccept", 5: "Completed", 6: "Error"}
COLOR_KO = {"RED": ("빨강", "가운데 빨간 홈"), "YELLOW": ("노랑", "노란 직사각형 홈"),
            "BLUE": ("파랑", "모서리 파란 홈"), "BLACK": ("검정", "검은 바닥")}
# pose: (short name, gap mm, what it stands for) - gaps from sensor_config.hpp
POSE_KO = {"LEVEL": ("평평", 10, "로봇을 평평하게 두고 센서-바닥 간격 10mm"),
           "LIFTED": ("들림", 20, "로봇 전체를 10mm 받침에 수평으로 올려 간격 20mm (단차에 걸친 상태)"),
           "PRESSED": ("눌림", 3, "로봇을 기울여 이쪽 센서를 바닥에 붙여 간격 3mm")}


# Pin panel rows: (key, label, GPIO). GPIO numbers copied from include/pins_v2.hpp.
# Digital rows come from the PIN row, analog rows (mV at the GPIO) from the ADC row.
PIN_ROWS = [
    [("sw1", "SW1", 15), ("sw2", "SW2", 16), ("sda", "SDA", 11), ("scl", "SCL", 12),
     ("en1", "EN 왼", 40), ("en2", "EN 오", 35)],
    [("lpwm1", "왼 LPWM", 42), ("rpwm1", "왼 RPWM", 41), ("lpwm2", "오 LPWM", 47),
     ("rpwm2", "오 RPWM", 21), ("bat", "BAT 핀", 14)],
    [("is_l1", "왼 L_IS", 1), ("is_r1", "왼 R_IS", 2), ("is_l2", "오 L_IS", 20), ("is_r2", "오 R_IS", 19)],
    [("ir1", "IR1", 4), ("ir2", "IR2", 5), ("ir3", "IR3", 6), ("ir4", "IR4", 7), ("ir5", "IR5", 8),
     ("ir6", "IR6", 13)],
]
ADC_KEYS = ["is_l1", "is_r1", "is_l2", "is_r2", "ir1", "ir2", "ir3", "ir4", "ir5", "ir6", "bat"]

IMU_STALE = {1: "준비 안 됨", 2: "샘플 끊김", 3: "값 고정"}
I2C_NAMES = {"39": "AS7341", "29": "TCS34725", "68": "IMU", "69": "IMU", "70": "먹스"}


def i2c_summary(scan):
    """One line for the I2C SCAN result: mux present?, what sits on each floor-sensor channel."""
    if not scan:
        return "— (I2C 스캔 버튼)"
    if not scan.get("finished"):
        return "스캔 중…"
    def names(addrs):
        return ", ".join(f"{a}({I2C_NAMES[a]})" if a in I2C_NAMES else a for a in addrs) or "없음"
    text = f"본선 {names(scan['main'])}"
    if scan.get("mux"):
        chans = [f"ch{c} {', '.join(scan['ch'].get(c, [])) or '—'}"
                 for c in range(8) if c < 4 or scan["ch"].get(c)]
        text = f"먹스 0x{scan['mux']} 있음 · " + " · ".join(chans) + " · " + text
    else:
        text = "먹스 없음 · " + text
    if scan.get("err"):
        text += f" · 버스 오류(0x{scan['err']}) → 배선·풀업·전원 확인"
    return text


def imu_freshness(m):
    """'신선', or why not, plus a warning when the chip reset itself and fell asleep."""
    text = "신선" if m["fresh"] else "멈춤" + (f"({IMU_STALE[m['stale']]})" if m.get("stale") in IMU_STALE else "")
    if m.get("pwr") and m["pwr"] != "00" and int(m["pwr"], 16) & 0x40:
        text += " · 칩 리셋됨(잠자기)"
    return text


# ---------- parsing (pure, tested by test_bench_gui.py) ----------

def requires_drive_permission(line):
    """Software SW2 may start MOTOR mode, so it needs the same guard as DRIVE."""
    command = line.strip().upper()
    return command == "SW2" or command.startswith("DRIVE")

def _int(text, default=0):
    try:
        return int(text)
    except ValueError:
        return default


def _float(text, default=0.0):
    try:
        return float(text)
    except ValueError:
        return default


def _keyvals(text):
    out = {}
    for part in text.split():
        if "=" in part:
            k, v = part.split("=", 1)
            out[k] = v
    return out


def parse_line(line):
    """Return (kind, fields) for a firmware line, or (None, None)."""
    line = line.strip()
    if not line:
        return None, None
    if line == "ESTOP":
        return "estop", {}
    if line.startswith("BATCHECK,"):
        return "batcheck", {"on": line.split(",")[1].strip() == "1"}
    if line.startswith("BAT "):
        parts = line.split()
        if len(parts) >= 4:
            return "bat", {"result": parts[1], "mv": _int(parts[2].rstrip("mV")),
                           "cells": _int(parts[3].rstrip("S"))}
        return None, None
    if line == "OK I2C SCAN":
        return "i2c", {"part": "start"}
    if line.startswith("I2C "):
        parts = line.split()
        if len(parts) >= 2 and parts[1] == "MAIN":
            return "i2c", {"part": "main", "addrs": parts[2:]}
        if len(parts) >= 3 and parts[1] == "MUX":
            return "i2c", {"part": "mux", "mux": None if parts[2] == "NONE" else parts[2]}
        if len(parts) >= 3 and parts[1] == "CH":
            return "i2c", {"part": "ch", "ch": _int(parts[2]), "addrs": parts[3:]}
        if len(parts) >= 2 and parts[1] in ("DONE", "ERR"):
            return "i2c", {"part": "end", "err": parts[-1] if parts[1] == "ERR" else None}
        return None, None
    if line.startswith("[CAL]"):
        return _parse_cal_text(line)
    f = line.split(",")
    tag = f[0]
    if tag == "D" and len(f) >= 12:
        extra = len(f) >= 16  # y, width, flags, seq (bench firmware from 11/04)
        return "d", {"ms": _int(f[1]), "mode": f[2], "allowed": f[3] == "1",
                     "left": _int(f[4]), "right": _int(f[5]), "link": f[6] == "1",
                     "fresh": f[7] == "1", "target": f[8] == "1", "x": _int(f[9]),
                     "height": _int(f[10]), "hazard": _int(f[11]),
                     "y": _int(f[12]) if extra else None, "width": _int(f[13]) if extra else None,
                     "flags": _int(f[14]) if extra else None, "seq": _int(f[15]) if extra else None}
    if tag == "LINK" and len(f) >= 8:
        # LINK,ms,lastSeq,lastPacketMs,lastFrameMs,valid,invalid,duplicate
        return "link", {"ms": _int(f[1]), "valid": _int(f[5]), "invalid": _int(f[6]),
                        "duplicate": _int(f[7])}
    if tag == "S" and len(f) >= 14:
        return "s", {"ms": _int(f[1]), "index": _int(f[2]), "valid": f[4] == "1",
                     "color": f[5], "reliable": f[6] == "1", "brightness": _int(f[7]),
                     "error": _int(f[13])}
    if tag == "I" and len(f) >= 13:
        return "i", {"ms": _int(f[1]), "state": f[2], "who": f[3], "fresh": f[4] == "1",
                     "samples": _int(f[5]), "bus_errors": _int(f[6]),
                     "accel": [_int(v) for v in f[7:10]], "gyro": [_int(v) for v in f[10:13]],
                     # newer firmware: why not fresh (imu_monitor.hpp ImuStale) and PWR_MGMT_1 read back
                     "stale": _int(f[13]) if len(f) >= 15 else None,
                     "pwr": f[14].strip() if len(f) >= 15 else None}
    if tag == "SYSTEM" and len(f) >= 10:
        system_state, fault_flags = _int(f[2]), _int(f[9])
        # system_manager.hpp: EmergencyStop=7, SystemFaultEmergencyStop=1<<4.
        # f[3] is the motor latch, also set during normal startup before SW2.
        return "system", {"state": system_state, "motor_latched": f[3] == "1",
                          "fault_flags": fault_flags,
                          "estop": system_state == 7 or bool(fault_flags & (1 << 4)),
                          "run": f[4] == "1", "allowed": f[5] == "1", "rearm": f[6] == "1"}
    if tag == "PIN" and len(f) >= 10:
        # PIN,ms,sw1,sw2,en1,en2,lpwm1,rpwm1,lpwm2,rpwm2 (levels read back from the board)
        return "pin", {"ms": _int(f[1]), "sw1": f[2] == "1", "sw2": f[3] == "1",
                       "en": (f[4] == "1", f[5] == "1"),
                       "pwm": (_int(f[6]), _int(f[7]), _int(f[8]), _int(f[9])),
                       "sda": f[10] == "1" if len(f) >= 12 else None,
                       "scl": f[11].strip() == "1" if len(f) >= 12 else None}
    if tag == "ADC" and len(f) >= 2 + len(ADC_KEYS):
        # ADC,ms,IS_L1,IS_R1,IS_L2,IS_R2,IR1..IR6,BAT pin (mV at the GPIO)
        return "adc", {"ms": _int(f[1]), **{k: _int(v) for k, v in zip(ADC_KEYS, f[2:])}}
    if tag == "CAL" and len(f) >= 9:
        mask = _int(f[4])
        return "cal", {"ms": _int(f[1]), "state": CAL_STATES.get(_int(f[2]), "Idle"),
                       "step": _int(f[3]), "sensors": [n + 1 for n in range(4) if mask >> n & 1], "color": f[5],
                       "pose": f[6], "samples": _int(f[7]), "stored": f[8].strip() == "1"}
    return None, None


def _parse_cal_text(line):
    if line.startswith("[CAL][RESULT]"):
        rest = line[13:].split()
        kv = _keyvals(line[13:])
        sensor = _int(rest[0][1:]) if rest and rest[0].startswith("S") else 0
        return "cal_result", {"sensor": sensor, "samples": _int(kv.get("samples", "0")),
                              "brightness": _float(kv.get("brightness", "0")),
                              "min": _int(kv.get("min", "0")), "max": _int(kv.get("max", "0")),
                              "noise": _float(kv.get("noise", "0"))}
    if line.startswith("[CAL][STATUS]"):
        kv = _keyvals(line[13:])
        return "cal_status", {"active": kv.get("active") == "1", "stored": kv.get("stored") == "1"}
    if line.startswith("[CAL][PREVIEW]"):
        rest = line[14:].split()
        sensor = _int(rest[0][1:]) if rest and rest[0].startswith("S") else 0
        if len(rest) >= 2 and rest[1] == "NOT_READY":
            return "cal_preview", {"sensor": sensor, "ready": False, "clear": 0, "channels": {}}
        kv = _keyvals(" ".join(rest[1:]))
        # AS7341: F2 F3 F5 F6 F7 C, TCS34725: R G B C (firmware order kept)
        return "cal_preview", {"sensor": sensor, "ready": True, "clear": _int(kv.get("C", "0")),
                               "channels": {k: _int(v) for k, v in kv.items()}}
    if "Starting in" in line:
        return "cal_count", {"n": _int(line.rsplit(" ", 1)[-1].rstrip("."))}
    for key, kind in (("Measuring for", "cal_measuring"), ("saved to NVS", "cal_saved"),
                      ("aborted", "cal_aborted"), ("Rejected", "cal_rejected"),
                      ("Too few samples", "cal_too_few"), ("NVS save failed", "cal_save_failed"),
                      ("All measurements completed", "cal_done"), ("erased", "cal_erased"),
                      ("Calibration started", "cal_started")):
        if key in line:
            return kind, {"text": line}
    return "cal_text", {"text": line}


def cal_quality(result):
    """Plain-language checks on one [CAL][RESULT]. Thresholds are rough guides."""
    notes = []
    if result["max"] >= NEAR_SATURATION:
        notes.append(("bad", f"포화 직전(최대 {result['max']} / {AS7341_FULL_SCALE}): 너무 밝거나 가깝습니다. 학현에게 알리기 (게인 조정 필요)"))
    if result["brightness"] < 30:
        notes.append(("warn", "매우 어둡습니다. 센서 LED와 위치를 확인하세요"))
    if result["noise"] > max(3.0, 0.05 * result["brightness"]):
        notes.append(("warn", "값이 흔들렸습니다. 로봇을 움직이지 말고 다시 재 보세요"))
    if result["samples"] < 10:
        notes.append(("warn", "샘플이 적습니다. 다시 재는 것을 권합니다"))
    return notes or [("ok", "좋아 보입니다")]


def cal_step_info(step):
    color, pose, sensors = CAL_ORDER[step]
    return {"sensors": list(sensors), "color": color, "pose": pose}


BAT_FAILED = ("battery_missing", "battery_low_or_wrong", "battery_too_high")


def led_coach(estop, system, bat, allowed):
    """What the board LED should show right now (status_led.cpp), to check by eye."""
    if estop:
        return "빨강 계속 (비상정지, 재부팅해야 풀림)"
    if (bat and bat["result"] in BAT_FAILED) or (system and system["state"] in (4, 5, 6)):
        return "빨강·흰색 번갈아 (고장)"
    if allowed:
        return "초록 계속 (구동 허용)"
    if bat is None:  # BAT line comes once, when the 3 s check ends
        return "흰색 천천히 깜빡 (준비 중, SW2 안 먹음) → 약 3초 뒤 초록 깜빡"
    return "초록 천천히 깜빡 (출발 대기, SW2를 누르면 시작)"


class BenchState:
    """Latest values seen in the log, for the status panels."""

    def __init__(self):
        self.bat = None
        self.bat_check = None      # saved power-on battery check switch (BATCHECK row)
        self.d = None
        self.sensors = {}
        self.imu = None
        self.system = None
        self.pins = None
        self.pins_at = 0.0
        self.adc = None
        self.i2c = None            # last I2C SCAN result, filled line by line
        self.adc_at = 0.0
        self.estop = False
        self.link = None           # last LINK row: Pi packet counters
        self.cal = None            # last CAL status line (only while calibrating)
        self.cal_results = {}      # sensor -> last measurement result, until the next step
        self.cal_previews = {}     # sensor -> live preview
        self.cal_count = None
        self.cal_measuring = False
        self.cal_finished = None   # "saved", "aborted" or None
        self.cal_message = None    # last notable [CAL] text
        self.cal_stored = None     # board has a saved calibration (from CAL lines / CAL STATUS)

    def feed(self, line):
        kind, v = parse_line(line)
        if kind == "estop":
            self.estop = True
        elif kind == "bat":
            self.bat = v
        elif kind == "batcheck":
            self.bat_check = v["on"]
        elif kind == "d":
            self.d = v
        elif kind == "link":
            self.link = v
        elif kind == "s" and 1 <= v["index"] <= 4:
            self.sensors[v["index"]] = v
        elif kind == "i":
            self.imu = v
        elif kind == "pin":
            self.pins, self.pins_at = v, time.monotonic()
        elif kind == "i2c":
            part = v["part"]
            if part == "start" or self.i2c is None:
                self.i2c = {"main": [], "mux": None, "ch": {}, "finished": False, "err": None}
            if part == "main":
                self.i2c["main"] = v["addrs"]
            elif part == "mux":
                self.i2c["mux"] = v["mux"]
            elif part == "ch":
                self.i2c["ch"][v["ch"]] = v["addrs"]
            elif part == "end":
                self.i2c["finished"], self.i2c["err"] = True, v["err"]
        elif kind == "adc":
            self.adc, self.adc_at = v, time.monotonic()
        elif kind == "system":
            self.system = v
            self.estop = self.estop or v["estop"]
        elif kind == "cal":
            if self.cal and v["step"] != self.cal["step"]:
                self.cal_results, self.cal_previews, self.cal_message = {}, {}, None
            if v["state"] not in ("Countdown", "Measuring"):
                self.cal_count, self.cal_measuring = None, False
            if v["state"] == "WaitingForReady" and self.cal and self.cal["state"] == "WaitingForAccept":
                self.cal_results = {}   # RETRY / BACK
            self.cal, self.cal_finished, self.cal_stored = v, None, v["stored"]
        elif kind == "cal_started":
            self.cal_finished, self.cal_results, self.cal_message = None, {}, None
            # Show step 1 at once; the firmware's CAL line follows within 250 ms.
            self.cal = {"ms": 0, "state": "WaitingForReady", "step": 0, **cal_step_info(0),
                        "samples": 0, "stored": self.cal_stored}
        elif kind == "cal_status":
            self.cal_stored = v["stored"]
        elif kind == "cal_result":
            self.cal_results[v["sensor"]] = v
            self.cal_measuring, self.cal_count, self.cal_message = False, None, None
        elif kind == "cal_preview":
            self.cal_previews[v["sensor"]] = v
        elif kind == "cal_count":
            # A new attempt: an old "Too few samples" must not stay on screen (11/04 bug).
            self.cal_count, self.cal_message = v["n"], None
        elif kind == "cal_measuring":
            self.cal_measuring, self.cal_count, self.cal_message = True, None, None
        elif kind in ("cal_saved", "cal_aborted"):
            self.cal_finished = "saved" if kind == "cal_saved" else "aborted"
            if kind == "cal_saved":
                self.cal_stored = True
            if self.cal:
                self.cal = dict(self.cal, state="Idle")
            self.cal_message = v["text"]
        elif kind == "cal_erased":
            self.cal_stored, self.cal_message = False, v["text"]
        elif kind in ("cal_rejected", "cal_too_few", "cal_save_failed", "cal_done"):
            self.cal_message = v["text"]
        return kind


def drive_command(direction, power):
    power = max(0, min(DRIVE_LIMIT, int(power)))
    left, right = {"fwd": (power, power), "back": (-power, -power),
                   "left": (-power, power), "right": (power, -power)}[direction]
    return f"DRIVE {left} {right}"


# ---------- session log ----------

class SessionLog:
    """Timestamped text log a person or an AI can replay (see module docstring)."""

    def __init__(self, path, header):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self.file = open(path, "w", encoding="utf-8")
        self.lock = threading.Lock()
        self.write("#", f"C_ROBOT bench session · {header}")

    def write(self, marker, text):
        stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with self.lock:
            if self.file:
                self.file.write(f"{stamp} {marker} {text}\n")
                self.file.flush()

    def close(self):
        with self.lock:
            if self.file:
                self.file.close()
                self.file = None


def new_log_path():
    return os.path.join(LOG_DIR, datetime.datetime.now().strftime("bench_%Y%m%d_%H%M%S.log"))


# ---------- serial link ----------

class Link:
    """Owns the port: a reader thread, and a DRIVE repeater that a STOP always beats."""

    def __init__(self, port, baud, log):
        import serial
        self.port = serial.Serial(timeout=0.2)
        self.port.port, self.port.baudrate = port, baud
        self.port.dtr = self.port.rts = False  # leave EN/BOOT released: opening must not reset the board
        self.port.open()
        self.log = log
        self.lines = queue.Queue()
        self.lock = threading.Lock()
        self.drive = None
        self.running = True
        for loop in (self._read_loop, self._repeat_loop):
            threading.Thread(target=loop, daemon=True).start()

    def send(self, line):
        line = line.strip() or "STOP"
        # Same lock as the repeater, so no stale DRIVE can follow a STOP.
        with self.lock:
            self.drive = line if line.upper().startswith("DRIVE ") else None
            self.port.write((line + "\n").encode("ascii", "replace"))
        self.log.write(">", line)

    def release(self):
        """Stop repeating DRIVE. The firmware stops within 250 ms and SW2 permission stays."""
        with self.lock:
            self.drive = None

    def _repeat_loop(self):
        while self.running:
            with self.lock:
                if self.drive:
                    self.port.write((self.drive + "\n").encode("ascii"))
            time.sleep(REPEAT_S)

    def _read_loop(self):
        while self.running:
            try:
                data = self.port.readline()
            except Exception as error:  # unplugged cable
                self.log.write("!", f"serial read failed: {error}")
                self.lines.put(f"[연결 끊김: {error}]")
                self.running = False
                break
            if data:
                text = data.decode("ascii", "replace").rstrip("\r\n")
                self.log.write("<", text)
                self.lines.put(text)

    def close(self):
        if self.port.is_open:
            try:
                self.send("STOP")
            except Exception:
                pass
        self.running = False
        time.sleep(0.3)
        self.port.close()


class DemoLink:
    """Plays plausible firmware lines so the window can be tried without a board."""

    def __init__(self, log):
        import random
        self.random = random
        self.log = log
        self.lines = queue.Queue()
        self.drive = None
        self.running = True
        self.mode, self.allowed, self.out = "STOP", False, (0, 0)
        self.estop = False
        self.cal = None  # [state, step, until_ms]
        threading.Thread(target=self._loop, daemon=True).start()

    def send(self, line):
        self.log.write(">", line)
        line = line.strip().upper() or "STOP"
        if self.estop:
            return
        if line == "SW1":
            self.estop, self.allowed, self.drive, self.out = True, False, None, (0, 0)
            self.cal = None
            self.lines.put("ESTOP")
        elif line == "SW2":
            if self.cal is None and self.mode not in ("STOP", "DATA"):
                self.allowed = True
                self.lines.put("OK SW2 press/release queued")
            else:
                self.lines.put("ERR SW2 blocked")
        elif line == "I2C SCAN":
            self.allowed, self.drive, self.out = False, None, (0, 0)
            for reply in ("OK I2C SCAN", "I2C MAIN 68 70", "I2C MUX 70", "I2C CH 0 39", "I2C CH 1 39",
                          "I2C CH 2", "I2C CH 3 39", "I2C CH 4", "I2C CH 5", "I2C CH 6", "I2C CH 7", "I2C DONE 160"):
                self.lines.put(reply)
        elif line == "CAL STATUS":
            self.lines.put(f"[CAL][STATUS] active={int(self.cal is not None)} stored=0 state=0 step=1/{CAL_STEPS}")
        elif self.cal is not None:
            self._cal_command(line)
        elif line.startswith("CAL START"):
            self.cal = [1, 0, 0]
            self.lines.put("[CAL] Calibration started. Motors are braked until it ends.")
        elif line.startswith("DRIVE ") and self.allowed:
            parts = line.split()
            self.drive, self.out = line, (int(parts[1]), int(parts[2]))
        elif line.startswith("MODE "):
            self.mode, self.allowed, self.drive, self.out = line[5:], False, None, (0, 0)
            self.lines.put(f"OK MODE {self.mode}; SW2 required")
        elif line == "STOP":
            self.allowed, self.drive, self.out = False, None, (0, 0)
            self.lines.put("OK STOP")

    def release(self):
        self.drive, self.out = None, (0, 0)

    def _cal_command(self, line):
        state, step = self.cal[0], self.cal[1]
        if line == "READY" and state == 1:
            self.cal = [2, step, time.time() + 1]
        elif line == "ACCEPT" and state == 4:
            self.cal = [5, step, 0] if step + 1 >= CAL_STEPS else [1, step + 1, 0]
            if step + 1 >= CAL_STEPS:
                self.lines.put("[CAL] All measurements completed.")
        elif line == "RETRY" and state == 4:
            self.cal = [1, step, 0]
        elif line == "BACK" and state in (1, 4):
            self.cal = [1, max(0, step - 1), 0]
        elif line == "CAL SAVE" and state == 5:
            self.cal = None
            self.lines.put("[CAL] Calibration saved to NVS.")
        elif line == "ABORT":
            self.cal = None
            self.lines.put("[CAL] Calibration aborted: USER")
        else:
            self.lines.put("[CAL] Command is not valid in the current state.")

    def _loop(self):
        r, t = self.random, 0
        time.sleep(0.5)
        self.lines.put("BAT battery_ok 15180mV 4S")
        colors = ["BLACK", "BLACK", "RED", "BLUE"]
        while self.running:
            t += 100
            if self.estop:
                if t % 3000 == 0:
                    self.lines.put("ESTOP")
            elif self.cal is not None:
                self._cal_tick(t)
            else:
                seen = t % 4000 < 3000  # tower visible 3 s, then 1 s without
                x = 960 + int(500 * math.sin(t / 2000))
                vis = f"1,{x},330,0,700,280,51" if seen else "0,0,0,0,0,0,48"
                self.lines.put(f"D,{t},{self.mode},{int(self.allowed)},{self.out[0]},{self.out[1]},1,1,{vis},{t // 20 % 256}")
                if t % 1600 == 0:
                    self.lines.put(f"LINK,{t},{t // 20 % 256},{t},{t},{t // 20},0,0")
                for i, color in enumerate(colors, 1):
                    self.lines.put(f"S,{t},{i},{t // 10},1,{color},1,{r.randint(300, 900)},1,2,3,4,5,0")
                if t % 300 == 0:
                    duty = [abs(v) * 255 // 1000 for v in self.out]
                    on = int(self.allowed)
                    self.lines.put(f"PIN,{t},0,0,{on},{on},{duty[0] if self.out[0] < 0 else 0},"
                                   f"{duty[0] if self.out[0] > 0 else 0},{duty[1] if self.out[1] < 0 else 0},"
                                   f"{duty[1] if self.out[1] > 0 else 0},1,1")
                    is_mv = [abs(v) // 4 for v in self.out]
                    self.lines.put(f"ADC,{t},{is_mv[0]},0,{is_mv[1]},0,{r.randint(0, 40)},{r.randint(0, 40)},"
                                   f"{r.randint(0, 40)},{r.randint(0, 40)},{r.randint(0, 40)},{r.randint(0, 40)},1380")
                if t % 200 == 0:
                    self.lines.put(f"I,{t},READY,68,1,{t // 10},0,{r.randint(-80, 80)},{r.randint(-80, 80)},"
                                   f"{4096 + r.randint(-60, 60)},{r.randint(-20, 20)},{r.randint(-20, 20)},{r.randint(-20, 20)},0,01")
            time.sleep(0.1)

    def _cal_tick(self, t):
        state, step, until = self.cal
        info = cal_step_info(step)
        if state == 2 and time.time() >= until:
            self.cal = [3, step, time.time() + 1]
            self.lines.put("[CAL] Measuring for 1000 ms. Do not move.")
        elif state == 2:
            self.lines.put(f"[CAL] Starting in {max(1, int(until - time.time()) + 1)}...")
        elif state == 3 and time.time() >= until:
            self.cal = [4, step, 0]
            b = {"BLACK": 120, "RED": 600, "BLUE": 450}[info["color"]]
            for n in info["sensors"]:
                self.lines.put(f"[CAL][RESULT] S{n} samples=28 brightness={b + self.random.random() * 20:.1f} "
                               f"min={b - 9} max={b + 11} noise={self.random.random() * 4:.2f}")
        elif state == 1 and t % 200 == 0:
            for n in range(1, 5):
                c = self.random.randint(400, 700)
                self.lines.put(f"[CAL][PREVIEW] S{n} F2={c // 6} F3={c // 5} F5={c // 4} F6={c // 3} "
                               f"F7={c // 2} C={c}")
        if t % 300 == 0 and self.cal is not None:
            mask = sum(1 << (n - 1) for n in info["sensors"])
            self.lines.put(f"CAL,{t},{self.cal[0]},{step},{mask},{info['color']},{info['pose']},"
                           f"{28 if self.cal[0] >= 3 else 0},0")

    def close(self):
        self.running = False


def list_ports():
    from serial.tools import list_ports as lp
    # On the Pi, ttyAMA*/ttyS* are the camera packet line to the ESP32 and the
    # console, never the bench port (that is the ESP32 USB, /dev/ttyACM0).
    return [(p.device, p.description) for p in lp.comports()
            if not p.device.startswith(("/dev/ttyAMA", "/dev/ttyS"))]


# ---------- command-line mode ----------

def run_cmd(args):
    drives = [c for c in args.commands if requires_drive_permission(c)]
    if drives and not args.allow_drive:
        print("DRIVE/SW2는 --allow-drive 없이 보낼 수 없습니다 (바퀴를 띄웠는지 사람이 확인한 뒤에).", file=sys.stderr)
        return 2
    log = SessionLog(new_log_path(), f"cmd mode port={args.port} baud={args.baud}")
    try:
        link = Link(args.port, args.baud, log)
    except Exception as error:
        log.write("!", f"open failed: {error}")
        log.close()
        print(f"포트를 열 수 없습니다: {error}", file=sys.stderr)
        return 1
    printer = threading.Thread(target=lambda: _print_lines(link), daemon=True)
    printer.start()
    try:
        for command in args.commands:
            link.send(command)
            if command.strip().upper().startswith("DRIVE"):
                time.sleep(min(3.0, max(0.1, args.drive_seconds)))
                link.send("STOP")
            time.sleep(0.2)
        time.sleep(max(0.0, args.listen))
    finally:
        link.close()
        log.close()
        print(f"[log: {log.path}]")
    return 0


def _print_lines(link):
    while link.running or not link.lines.empty():
        try:
            print(link.lines.get(timeout=0.2), flush=True)
        except queue.Empty:
            pass


# ---------- GUI ----------

# Warm neutral palette; clay accent for actions, semantic colors for state.
THEME = {
    "bg": "#F4F2EC", "surface": "#FFFFFF", "sunk": "#EDEAE2", "line": "#E2DED4",
    "ink": "#1F1E1C", "muted": "#6E6B64", "faint": "#A19D94",
    "accent": "#C96442", "accent_hi": "#B4553A", "accent_soft": "#F6E3DA",
    "ok": "#3B8A57", "ok_soft": "#E2F1E6", "warn": "#B7811A", "warn_soft": "#F8EDD3",
    "bad": "#BF3B2C", "bad_soft": "#F7E0DC",
    "log_bg": "#1E1D1B", "log_ink": "#D9D5CB", "log_dim": "#8C877C",
}
FLOOR = {"BLACK": "#232220", "RED": "#D0402F", "YELLOW": "#E5B52B", "BLUE": "#2F67C9"}
FLOOR_TOWER = "#2FA84F"  # green color tower in the camera box
UI_FONT = "Malgun Gothic"
MONO_FONT = "Consolas"

# Sunday bring-up checklist: (key, title, how to check)
CHECKS = [
    ("bat", "배터리 전압", "보드 RST → BAT 줄이 멀티미터 값과 같은지"),
    ("boot", "부팅 때 오른쪽 바퀴", "바퀴 띄우고 전원 켤 때 움찔하지 않는지 (GPIO35)"),
    ("motor", "모터 방향", "주행 준비 → ▲ 앞: 두 바퀴가 앞으로"),
    ("sensors", "센서 1–4 순서", "DATA → 하나씩 가려서 그 번호 칸이 바뀌는지"),
    ("imu", "IMU", "READY · 신선이면 자동 체크"),
    ("sw1", "SW1 비상정지", "마지막에: 누르면 ESTOP (재부팅 필요)"),
]

CAL_PREP = [
    "보드에 연결합니다 (위 포트 → 연결).",
    "모터 전원은 빼 두는 것을 권합니다. 교정 중에는 펌웨어가 모터를 브레이크로 묶어 둡니다.",
    "경기장 실사 현수막 위에서, 대회장과 비슷한 조명으로 합니다.",
    "센서-바닥 간격을 맞출 자(또는 3·10·20mm 블록)를 준비합니다.",
    "로봇 전체를 수평으로 10mm 올릴 받침과 로봇보다 큰 검정·빨강·파랑 면을 준비합니다.",
    f"{CAL_STEPS}단계 (색마다 평평 → 들림 → 앞 눌림 → 뒤 눌림) ≈ 5–10분. 중간에 멈추면 처음부터 다시 해야 합니다.",
]


def run_gui():
    import tkinter as tk
    from tkinter import messagebox, ttk

    T = THEME
    root = tk.Tk()
    root.title("C_ROBOT 보드 시험")
    root.geometry("1280x840")
    root.minsize(1100, 720)
    root.configure(bg=T["bg"])

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TCombobox", fieldbackground=T["surface"], background=T["surface"],
                    bordercolor=T["line"], arrowcolor=T["muted"], padding=5)
    style.configure("Vertical.TScrollbar", background=T["sunk"], troughcolor=T["log_bg"],
                    bordercolor=T["log_bg"], arrowcolor=T["log_dim"])
    style.configure("TNotebook", background=T["bg"], borderwidth=0, tabmargins=(0, 0, 0, 0))
    style.configure("TNotebook.Tab", background=T["sunk"], foreground=T["muted"], padding=(18, 8),
                    font=(UI_FONT, 11, "bold"), borderwidth=0)
    style.map("TNotebook.Tab", background=[("selected", T["surface"])], foreground=[("selected", T["ink"])])
    style.configure("Cal.Horizontal.TProgressbar", troughcolor=T["sunk"], background=T["accent"],
                    bordercolor=T["sunk"], lightcolor=T["accent"], darkcolor=T["accent"])

    def font(size=10, weight="normal", mono=False):
        return (MONO_FONT if mono else UI_FONT, size, weight)

    state = BenchState()
    link = {"obj": None}
    session = {"log": None}
    ui = {"driving": None, "seen_data": False}
    checks_done = {k: tk.BooleanVar(value=False) for k, _, _ in CHECKS}
    filters = {k: tk.BooleanVar(value=True) for k in ("D", "S", "I", "기타")}
    paused = tk.BooleanVar(value=False)
    wheels_up = tk.BooleanVar(value=False)
    power = tk.DoubleVar(value=DRIVE_LIMIT)  # tk Scale writes floats
    history, hist_pos = [], [0]

    def note(marker, text):
        if session["log"]:
            session["log"].write(marker, text)

    # Python errors go to the session log (or a fallback file) instead of vanishing.
    def report_error(exc, value, tb):
        detail = "".join(traceback.format_exception(exc, value, tb)).rstrip()
        if session["log"]:
            session["log"].write("!", "GUI error\n" + detail)
        else:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(os.path.join(LOG_DIR, "bench_gui_errors.log"), "a", encoding="utf-8") as f:
                f.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} ! GUI error\n{detail}\n")
        set_hint(f"프로그램 오류가 로그에 기록됐습니다: {value}")

    root.report_callback_exception = report_error

    # ----- small widget helpers -----
    def card(parent, title=None, **pack):
        outer = tk.Frame(parent, bg=T["surface"], highlightbackground=T["line"], highlightthickness=1)
        outer.pack(**({"fill": "x", "pady": (0, 10)} | pack))
        if title:
            tk.Label(outer, text=title, bg=T["surface"], fg=T["muted"], font=font(9, "bold"),
                     anchor="w").pack(fill="x", padx=12, pady=(10, 2))
        inner = tk.Frame(outer, bg=T["surface"])
        inner.pack(fill="both", expand=True, padx=12, pady=(2, 12))
        return inner

    def button(parent, text, command=None, kind="plain", width=None):
        colors = {"plain": (T["sunk"], T["ink"], T["line"]),
                  "accent": (T["accent"], "#FFFFFF", T["accent_hi"]),
                  "stop": (T["bad"], "#FFFFFF", "#A53224")}[kind]
        return tk.Button(parent, text=text, command=command, bg=colors[0], fg=colors[1],
                         activebackground=colors[2], activeforeground=colors[1], relief="flat",
                         bd=0, padx=10, pady=6, font=font(10, "bold" if kind != "plain" else "normal"),
                         cursor="hand2", width=width, highlightthickness=0, disabledforeground=T["faint"])

    def chip(parent, text, var):
        def paint():
            on = var.get()
            lbl.config(bg=T["accent_soft"] if on else T["sunk"], fg=T["accent_hi"] if on else T["faint"])
        lbl = tk.Label(parent, text=text, font=font(9, "bold"), padx=9, pady=2, cursor="hand2")
        lbl.bind("<Button-1>", lambda e: (var.set(not var.get()), paint()))
        paint()
        return lbl

    # ----- top bar -----
    top = tk.Frame(root, bg=T["bg"])
    top.pack(fill="x", padx=16, pady=(14, 8))
    tk.Label(top, text="C_ROBOT", bg=T["bg"], fg=T["ink"], font=font(16, "bold")).pack(side="left")
    tk.Label(top, text="보드 시험", bg=T["bg"], fg=T["muted"], font=font(12)).pack(side="left", padx=(6, 18))
    port_var, baud_var = tk.StringVar(), tk.StringVar(value="57600")
    port_box = ttk.Combobox(top, textvariable=port_var, width=34, state="readonly", font=font(10))
    port_box.pack(side="left")

    def refresh_ports():
        ports = list_ports()
        port_box["values"] = [f"{d}  ·  {desc}" for d, desc in ports] + ["DEMO  ·  보드 없이 화면 미리보기"]
        if ports and not port_var.get():
            port_box.current(0)
        if not ports:
            set_hint("포트가 없습니다. 케이블(COM 쪽 USB-C)을 확인하고 ↻ · 미리보기는 DEMO")

    button(top, "↻", refresh_ports).pack(side="left", padx=4)
    ttk.Combobox(top, textvariable=baud_var, values=["57600", "115200"], width=7,
                 state="readonly", font=font(10)).pack(side="left", padx=(4, 8))
    conn_btn = button(top, "연결", kind="accent", width=8)
    conn_btn.pack(side="left")
    log_label = tk.Label(top, text="", bg=T["bg"], fg=T["faint"], font=font(9), cursor="hand2")
    log_label.pack(side="right")
    log_label.bind("<Button-1>", lambda e: os.startfile(LOG_DIR) if os.path.isdir(LOG_DIR) else None)

    # ----- state banner: the one thing to know right now -----
    banner = tk.Frame(root, bg=T["sunk"])
    banner.pack(fill="x", padx=16, pady=(0, 10))
    banner_dot = tk.Label(banner, text="●", font=font(18), bg=T["sunk"], fg=T["faint"])
    banner_dot.pack(side="left", padx=(14, 8), pady=10)
    banner_text = tk.Frame(banner, bg=T["sunk"])
    banner_text.pack(side="left", fill="x", expand=True)
    banner_title = tk.Label(banner_text, text="", font=font(14, "bold"), bg=T["sunk"], fg=T["ink"], anchor="w")
    banner_title.pack(fill="x")
    banner_sub = tk.Label(banner_text, text="", font=font(10), bg=T["sunk"], fg=T["muted"], anchor="w")
    banner_sub.pack(fill="x")
    banner_led = tk.Label(banner_text, text="", font=font(10, "bold"), bg=T["sunk"], fg=T["ink"], anchor="w")
    banner_led.pack(fill="x")
    stop_big = button(banner, "■  STOP   (Esc)", lambda: send("STOP"), kind="stop")
    stop_big.config(font=font(12, "bold"), padx=18, pady=10)
    stop_big.pack(side="right", padx=12, pady=8)

    def set_banner(level, title, sub):
        bg, fg = {"idle": (T["sunk"], T["faint"]), "ok": (T["ok_soft"], T["ok"]),
                  "warn": (T["warn_soft"], T["warn"]), "bad": (T["bad_soft"], T["bad"]),
                  "drive": (T["accent_soft"], T["accent"])}[level]
        for w in (banner, banner_dot, banner_text, banner_title, banner_sub, banner_led):
            w.config(bg=bg)
        banner_dot.config(fg=fg)
        banner_title.config(text=title)
        banner_sub.config(text=sub)

    # status line (packed before the notebook so it stays visible at the bottom)
    hint = tk.Label(root, text="", bg=T["bg"], fg=T["muted"], font=font(9), anchor="w")
    hint.pack(side="bottom", fill="x", padx=18, pady=(2, 10))

    def set_hint(message):
        hint.config(text=message)

    tabs = ttk.Notebook(root)
    tabs.pack(fill="both", expand=True, padx=16)
    test_tab = tk.Frame(tabs, bg=T["bg"])
    cal_tab = tk.Frame(tabs, bg=T["bg"])
    cam_tab = tk.Frame(tabs, bg=T["bg"])
    tabs.add(test_tab, text="  시험  ")
    tabs.add(cal_tab, text="  색 교정  ")
    tabs.add(cam_tab, text="  라즈 카메라  ")

    # =================== 시험 tab ===================
    cols = tk.Frame(test_tab, bg=T["bg"])
    cols.pack(fill="both", expand=True, pady=(10, 0))
    left = tk.Frame(cols, bg=T["bg"], width=330)
    left.pack(side="left", fill="y")
    left.pack_propagate(False)
    right = tk.Frame(cols, bg=T["bg"], width=340)
    right.pack(side="right", fill="y")
    right.pack_propagate(False)
    mid = tk.Frame(cols, bg=T["bg"])
    mid.pack(side="left", fill="both", expand=True, padx=12)

    def robot_canvas(parent, width=300, height=250):
        """Top view: four floor-sensor tiles and two motor bars. Returns (canvas, tiles, bars)."""
        c = tk.Canvas(parent, width=width, height=height, bg=T["surface"], highlightthickness=0)
        c.create_text(width // 2, 10, text="▲ 앞", fill=T["muted"], font=font(9, "bold"))
        x0, x1 = width // 2 - 90, width // 2 + 90
        c.create_rectangle(x0, 24, x1, height - 10, outline=T["line"], width=2, fill=T["bg"])
        th = (height - 46) // 2
        tiles = {}
        for i, (x, y) in {1: (x0 + 6, 30), 2: (x0 + 90, 30), 3: (x0 + 6, 30 + th + 4), 4: (x0 + 90, 30 + th + 4)}.items():
            rect = c.create_rectangle(x, y, x + 84, y + th, fill=T["sunk"], outline="", width=3)
            num = c.create_text(x + 10, y + 12, text=str(i), fill=T["faint"], font=font(10, "bold"))
            name = c.create_text(x + 42, y + th * 0.45, text="—", fill=T["muted"], font=font(10, "bold"))
            info = c.create_text(x + 42, y + th * 0.72, text=CORNERS[i], fill=T["faint"], font=font(8))
            tiles[i] = (rect, num, name, info)
        return c, tiles

    robot_box = card(left, "로봇 위에서 본 모습")
    robot, tiles = robot_canvas(robot_box)
    robot.pack()
    bars = {}
    for side, x in (("left", 18), ("right", 258)):
        robot.create_rectangle(x, 40, x + 24, 224, outline=T["line"], fill=T["surface"])
        robot.create_line(x, 132, x + 24, 132, fill=T["line"])
        bars[side] = (robot.create_rectangle(x + 3, 132, x + 21, 132, fill=T["accent"], outline=""),
                      robot.create_text(x + 12, 236, text="0", fill=T["muted"], font=font(8, "bold", mono=True)))

    def paint_robot():
        for i, (rect, num, name, info) in tiles.items():
            s = state.sensors.get(i)
            if not s:
                robot.itemconfig(rect, fill=T["sunk"])
                robot.itemconfig(name, text="—", fill=T["faint"])
                robot.itemconfig(info, text=CORNERS[i], fill=T["faint"])
                continue
            bad = not s["valid"] or s["error"]
            color = FLOOR.get(s["color"], T["sunk"])
            dark = s["color"] in FLOOR and s["color"] != "YELLOW"
            robot.itemconfig(rect, fill=T["bad_soft"] if bad else color)
            ink = T["bad"] if bad else ("#FFFFFF" if dark else T["ink"])
            label = f"오류 {s['error']}" if bad else s["color"] + ("" if s["reliable"] else " ?")
            robot.itemconfig(num, fill=ink)
            robot.itemconfig(name, text=label, fill=ink)
            robot.itemconfig(info, text=f"밝기 {s['brightness']}", fill=ink)
        d = state.d
        for side in ("left", "right"):
            value = (d[side] if d else 0)
            bar, label = bars[side]
            x0 = 18 if side == "left" else 258
            h = int(abs(value) / 1000 * 88)
            y0, y1 = (132 - h, 132) if value >= 0 else (132, 132 + h)
            robot.coords(bar, x0 + 3, y0, x0 + 21, y1)
            robot.itemconfig(label, text=str(value))

    stat = card(left, "전원 · IMU")
    stat_rows = {}
    for key, label in (("bat", "배터리"), ("estop", "비상정지"), ("imu", "IMU"), ("acc", "가속도 g")):
        r = tk.Frame(stat, bg=T["surface"])
        r.pack(fill="x", pady=2)
        tk.Label(r, text=label, width=8, anchor="w", bg=T["surface"], fg=T["muted"], font=font(10)).pack(side="left")
        v = tk.Label(r, text="—", anchor="w", bg=T["surface"], fg=T["ink"], font=font(10, "bold"))
        v.pack(side="left", fill="x")
        stat_rows[key] = v
    # 11/05: off for matches (keep running on a sagging pack), on for bench tests.
    bat_check_btn = button(stat, "배터리 판정: —",
                           lambda: send("BAT CHECK OFF" if state.bat_check else "BAT CHECK ON"))
    bat_check_btn.config(font=font(9))
    bat_check_btn.pack(fill="x", pady=(4, 0))
    tk.Label(stat, text="저장됨 · 끄기는 바로, 켜기는 다음 전원부터 · 경기용 펌웨어도 같은 값", bg=T["surface"],
             fg=T["faint"], font=font(8)).pack(anchor="w")

    # middle: log + manual command
    # pin panel: levels the board read back from its own pins (PIN row)
    pin_box = card(mid, "핀 상태 · 보드가 읽은 실제 값 (0.8초마다) · 아날로그는 GPIO에서 잰 mV")
    pin_grid = tk.Frame(pin_box, bg=T["surface"])
    pin_grid.pack(fill="x")
    pin_tiles = {}
    for c in range(6):
        pin_grid.columnconfigure(c, weight=1, uniform="pin")
    for r, row in enumerate(PIN_ROWS):
        for c, (key, name, gpio) in enumerate(row):
            tile = tk.Frame(pin_grid, bg=T["sunk"], padx=6, pady=3)
            tile.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)
            top_row = tk.Frame(tile, bg=T["sunk"])
            top_row.pack(fill="x")
            n = tk.Label(top_row, text=name, bg=T["sunk"], fg=T["ink"], font=font(9, "bold"), anchor="w")
            n.pack(side="left")
            g = tk.Label(top_row, text=str(gpio), bg=T["sunk"], fg=T["faint"], font=font(8, mono=True))
            g.pack(side="right")
            v = tk.Label(tile, text="—", bg=T["sunk"], fg=T["faint"], font=font(11, "bold", mono=True), anchor="w")
            v.pack(fill="x")
            pin_tiles[key] = (tile, top_row, n, g, v)
    scan_row = tk.Frame(pin_box, bg=T["surface"])
    scan_row.pack(fill="x", pady=(6, 0))
    button(scan_row, "I2C 스캔", lambda: send("I2C SCAN")).pack(side="left")
    scan_label = tk.Label(scan_row, text="", bg=T["surface"], fg=T["ink"], font=font(9, "bold"), anchor="w",
                          justify="left", wraplength=440)
    scan_label.pack(side="left", fill="x", expand=True, padx=(8, 0))
    pin_note = tk.Label(pin_box, text="", bg=T["surface"], fg=T["faint"], font=font(9), anchor="w",
                        justify="left", wraplength=520)
    pin_note.pack(fill="x", pady=(4, 0))

    # camera box: where the Pi says the tower is (1920x1080 frame, from the D row)
    cam_box = card(cam_tab, "타워 위치 · ESP32가 라즈에게 받은 좌표 (1920×1080 화면) · 로그가 켜져 있어야 함 (MODE DATA 등)")
    cam_box.master.pack_configure(pady=(10, 0))
    cam_row = tk.Frame(cam_box, bg=T["surface"])
    cam_row.pack(fill="x")
    CAM_W, CAM_H = 640, 360
    cam = tk.Canvas(cam_row, width=CAM_W, height=CAM_H, bg=T["sunk"], highlightthickness=1,
                    highlightbackground=T["line"])
    cam.pack(side="left")
    cam.create_line(CAM_W // 2, 0, CAM_W // 2, CAM_H, fill=T["line"], dash=(3, 3))
    cam_target = cam.create_rectangle(0, 0, 0, 0, outline=FLOOR_TOWER, width=3, state="hidden")
    cam_dot = cam.create_oval(0, 0, 0, 0, fill=FLOOR_TOWER, outline="", state="hidden")
    cam_text = cam.create_text(CAM_W // 2, CAM_H // 2, text="연결 안 됨", fill=T["faint"], font=font(16, "bold"))
    cam_info = tk.Label(cam_row, text="", bg=T["surface"], fg=T["ink"], font=font(12), anchor="nw",
                        justify="left")
    cam_info.pack(side="left", fill="both", expand=True, padx=(16, 0))
    cam_btns = tk.Frame(cam_box, bg=T["surface"])
    cam_btns.pack(fill="x", pady=(10, 0))
    button(cam_btns, "로그 켜기 (MODE DATA)", lambda: (send("LOG ON"), send("MODE DATA"))).pack(side="left")
    tk.Label(cam_btns, text="라즈는 부팅하면 자동 송신 · 화면·색 설정은 라즈 바탕화면 'C_ROBOT 타워 설정'   ·   "
             "배선: 라즈 8번(TX)→ESP32 RX, 10번(RX)←TX, GND 공통",
             bg=T["surface"], fg=T["muted"], font=font(9), wraplength=900, justify="left").pack(side="left", padx=10)

    def paint_camera():
        d = state.d
        has_box = bool(d and d["link"] and d["target"] and d.get("y") is not None)
        if has_box:
            sx, sy = CAM_W / 1920, CAM_H / 1080
            cx, cy, w, h = d["x"] * sx, d["y"] * sy, d["width"] * sx, d["height"] * sy
            cam.coords(cam_target, cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
            cam.coords(cam_dot, cx - 6, cy - 6, cx + 6, cy + 6)
        state_flag = "normal" if has_box else "hidden"
        cam.itemconfig(cam_target, state=state_flag, outline=FLOOR_TOWER if d and d["fresh"] else T["faint"])
        cam.itemconfig(cam_dot, state=state_flag)
        if not d:
            cam.itemconfig(cam_text, text="보드 로그 없음", state="normal")
            cam_info.config(text="")
            return
        if not d["link"]:
            cam.itemconfig(cam_text, text="라즈 연결 안 됨", state="normal")
        elif not d["target"]:
            cam.itemconfig(cam_text, text="타워 없음", state="normal")
        else:
            cam.itemconfig(cam_text, state="hidden")
        lines = [("라즈 링크  " + ("연결" if d["link"] else "끊김 (120ms 넘게 정상 패킷 없음)")),
                 ("영상  " + ("새 프레임" if d["fresh"] else "멈춤 (sequence가 안 바뀜)") if d["link"] else "영상  —")]
        if d["link"] and d["target"]:
            side = "가운데" if abs(d["x"] - 960) < 120 else ("왼쪽" if d["x"] < 960 else "오른쪽")
            lines.append(f"타워  x {d['x']}  y {d.get('y', '—')}  ({side})")
            lines.append(f"박스  {d.get('width', '—')} × {d['height']}")
        flags = d.get("flags")
        if d["link"] and flags is not None:
            names = [n for bit, n in ((2, "안정"), (4, "여러 개"), (8, "가장자리"))
                     if flags & bit]
            lines.append(f"seq {d['seq']}  flags 0x{flags:02X} " + " ".join(names))
            if not (flags & 0x10 and flags & 0x20):
                lines.append("⚠ 라즈 카메라/처리 이상 보고")
        lk = state.link
        if lk:
            lines.append(f"패킷 정상 {lk['valid']}  오류 {lk['invalid']}  중복 {lk['duplicate']}")
        cam_info.config(text="\n".join(lines), fg=T["ink"] if d["link"] else T["muted"])

    log_card = card(mid, None, fill="both", expand=True)
    head = tk.Frame(log_card, bg=T["surface"])
    head.pack(fill="x", pady=(0, 8))
    tk.Label(head, text="로그", bg=T["surface"], fg=T["muted"], font=font(9, "bold")).pack(side="left", padx=(0, 10))
    for k, var in filters.items():
        chip(head, k, var).pack(side="left", padx=2)
    chip(head, "일시정지", paused).pack(side="right")
    logf = tk.Frame(log_card, bg=T["log_bg"])
    logf.pack(fill="both", expand=True)
    text = tk.Text(logf, bg=T["log_bg"], fg=T["log_ink"], insertbackground=T["log_ink"], relief="flat",
                   font=font(9, mono=True), wrap="none", padx=10, pady=8, highlightthickness=0)
    scroll = ttk.Scrollbar(logf, command=text.yview)
    text.configure(yscrollcommand=scroll.set)
    scroll.pack(side="right", fill="y")
    text.pack(fill="both", expand=True)
    for tag, color in (("sent", "#F0A585"), ("bad", "#F08A7B"), ("dim", T["log_dim"]),
                       ("imu", "#9FC6A8"), ("cal", "#E8C77A")):
        text.tag_configure(tag, foreground=color)

    entry_row = tk.Frame(log_card, bg=T["surface"])
    entry_row.pack(fill="x", pady=(8, 0))
    tk.Label(entry_row, text="›", bg=T["surface"], fg=T["accent"], font=font(14, "bold")).pack(side="left")
    entry = tk.Entry(entry_row, relief="flat", bg=T["sunk"], fg=T["ink"], font=font(11, mono=True),
                     insertbackground=T["ink"], highlightthickness=1, highlightbackground=T["line"],
                     highlightcolor=T["accent"])
    entry.pack(side="left", fill="x", expand=True, ipady=6, padx=(6, 6))
    send_btn = button(entry_row, "보내기", kind="accent")
    send_btn.pack(side="left")

    # right: drive pad
    drive_card = card(right, "주행 · 누르는 동안만")
    tk.Checkbutton(drive_card, text="바퀴를 바닥에서 띄웠음", variable=wheels_up, bg=T["surface"],
                   activebackground=T["surface"], fg=T["ink"], font=font(10, "bold"),
                   highlightthickness=0, cursor="hand2",
                   command=lambda: note("#", f"wheels lifted = {wheels_up.get()}")).pack(anchor="w")
    ready_btn = button(drive_card, "▶  주행 준비  (MANUAL + SW2)", lambda: arm(), kind="accent")
    ready_btn.config(font=font(12, "bold"), pady=10)
    ready_btn.pack(fill="x", pady=(8, 4))
    switch_row = tk.Frame(drive_card, bg=T["surface"])
    switch_row.pack(fill="x", pady=(0, 4))
    button(switch_row, "SW2만 · 시작", lambda: send("SW2")).pack(side="left")
    button(switch_row, "SW1 · 비상정지", lambda: send("SW1"), kind="stop").pack(side="right")
    tk.Label(drive_card, text="주행 준비 한 번이면 방향키에서 손을 떼도 허가가 유지됩니다 · "
             "■ STOP을 누르면 다시 주행 준비 · SW1은 재부팅해야 풀림",
             bg=T["surface"], fg=T["muted"], font=font(9), wraplength=300,
             justify="left").pack(anchor="w")
    prow = tk.Frame(drive_card, bg=T["surface"])
    prow.pack(fill="x", pady=(4, 6))
    tk.Label(prow, text="출력", bg=T["surface"], fg=T["muted"], font=font(9)).pack(side="left")
    power_label = tk.Label(prow, text=str(DRIVE_LIMIT), width=4, bg=T["surface"], fg=T["ink"], font=font(10, "bold", mono=True))
    power_label.pack(side="right")
    tk.Scale(prow, from_=20, to=DRIVE_LIMIT, orient="horizontal", variable=power, showvalue=False,
             bg=T["surface"], troughcolor=T["sunk"], highlightthickness=0, bd=0, sliderrelief="flat",
             activebackground=T["accent"], command=lambda v: power_label.config(text=str(int(float(v))))
             ).pack(side="left", fill="x", expand=True, padx=6)
    pad = tk.Frame(drive_card, bg=T["surface"])
    pad.pack()
    pad_keys = {}

    def pad_key(r, c, label, direction):
        k = tk.Label(pad, text=label, width=6, height=2, font=font(11, "bold"), bg=T["sunk"],
                     fg=T["ink"], cursor="hand2")
        k.grid(row=r, column=c, padx=3, pady=3)
        k.bind("<ButtonPress-1>", lambda e: drive_start(direction))
        k.bind("<ButtonRelease-1>", lambda e: drive_stop())
        pad_keys[direction] = k

    pad_key(0, 1, "▲\n앞", "fwd")
    pad_key(1, 0, "◀ 좌", "left")
    pad_key(1, 2, "우 ▶", "right")
    pad_key(2, 1, "▼\n뒤", "back")
    stop_small = tk.Label(pad, text="■", width=6, height=2, font=font(11, "bold"), bg=T["bad"],
                          fg="#FFFFFF", cursor="hand2")
    stop_small.grid(row=1, column=1, padx=3, pady=3)
    stop_small.bind("<Button-1>", lambda e: send("STOP"))
    tk.Label(drive_card, text="방향키로도 됩니다 · Esc / Space = STOP", bg=T["surface"], fg=T["faint"],
             font=font(8)).pack(pady=(4, 0))

    # right: checklist
    check_card = card(right, "일요일 시험 순서")
    check_rows = {}
    for key, title, how in CHECKS:
        r = tk.Frame(check_card, bg=T["surface"])
        r.pack(fill="x", pady=3)
        tk.Checkbutton(r, variable=checks_done[key], bg=T["surface"], activebackground=T["surface"],
                       highlightthickness=0, bd=0, cursor="hand2",
                       command=lambda k=key: note("#", f"check {k} = {checks_done[k].get()}")).pack(side="left", anchor="n")
        txt = tk.Frame(r, bg=T["surface"])
        txt.pack(side="left", fill="x", expand=True)
        t = tk.Label(txt, text=title, bg=T["surface"], fg=T["ink"], font=font(10, "bold"), anchor="w")
        t.pack(fill="x")
        auto = tk.Label(txt, text=how, bg=T["surface"], fg=T["faint"], font=font(8), anchor="w",
                        justify="left", wraplength=250)
        auto.pack(fill="x")
        check_rows[key] = (t, auto, how)

    def copy_summary():
        lines = ["[C_ROBOT 보드 시험 결과]"]
        for key, title, _ in CHECKS:
            lines.append(f"{'✓' if checks_done[key].get() else '·'} {title}: {check_rows[key][1].cget('text')}")
        if session["log"]:
            lines.append(f"로그 파일: {session['log'].path}")
        summary = "\n".join(lines)
        root.clipboard_clear()
        root.clipboard_append(summary)
        note("#", "copied summary:\n" + summary)
        set_hint("결과를 복사했습니다. 단톡이나 Claude·Codex에 붙여 넣으세요")

    # left: commands
    cmd_card = card(left, "명령")
    grid = tk.Frame(cmd_card, bg=T["surface"])
    grid.pack(fill="x")
    commands = [("STATUS", "STATUS"), ("HELP", "HELP"), ("LOG ON", "LOG ON"), ("LOG OFF", "LOG OFF"),
                ("DATA", "MODE DATA"), ("MANUAL", "MODE MANUAL"), ("MOTOR", "MODE MOTOR"),
                ("IMU RESET", "IMU RESET")]
    for n, (label, cmd) in enumerate(commands):
        b = button(grid, label, lambda c=cmd: send(c))
        b.config(font=font(9))
        b.grid(row=n // 4, column=n % 4, padx=2, pady=2, sticky="ew")
    for c in range(4):
        grid.columnconfigure(c, weight=1)
    button(cmd_card, "시험 결과 복사 (단톡·Claude에 붙여 넣기)", copy_summary, kind="accent").pack(fill="x", pady=(8, 0))

    # =================== 색 교정 tab ===================
    cal_top = tk.Frame(cal_tab, bg=T["bg"])
    cal_top.pack(fill="x", pady=(12, 8))
    cal_step_label = tk.Label(cal_top, text="교정 전", bg=T["bg"], fg=T["ink"], font=font(13, "bold"))
    cal_step_label.pack(side="left")
    cal_progress = ttk.Progressbar(cal_top, style="Cal.Horizontal.TProgressbar", maximum=CAL_STEPS, length=420)
    cal_progress.pack(side="left", padx=16)
    cal_stored_label = tk.Label(cal_top, text="", bg=T["bg"], fg=T["muted"], font=font(10))
    cal_stored_label.pack(side="right")

    cal_cols = tk.Frame(cal_tab, bg=T["bg"])
    cal_cols.pack(fill="both", expand=True)
    cal_left = tk.Frame(cal_cols, bg=T["bg"], width=420)
    cal_left.pack(side="left", fill="y")
    cal_left.pack_propagate(False)
    cal_right = tk.Frame(cal_cols, bg=T["bg"])
    cal_right.pack(side="left", fill="both", expand=True, padx=(12, 0))

    # left: what to do now
    todo = card(cal_left, "지금 할 일", fill="both", expand=True)
    cal_title = tk.Label(todo, text="", bg=T["surface"], fg=T["ink"], font=font(15, "bold"), anchor="w",
                         justify="left", wraplength=380)
    cal_title.pack(fill="x")
    cal_canvas, cal_tiles = robot_canvas(todo, width=380, height=230)
    cal_canvas.pack(pady=(8, 8))
    cal_steps_text = tk.Label(todo, text="", bg=T["surface"], fg=T["ink"], font=font(11), anchor="w",
                              justify="left", wraplength=380)
    cal_steps_text.pack(fill="x")

    # right: measurement state, result, buttons, overview
    meas = card(cal_right, "측정")
    cal_state_label = tk.Label(meas, text="", bg=T["surface"], fg=T["ink"], font=font(22, "bold"), anchor="w")
    cal_state_label.pack(fill="x")
    cal_state_sub = tk.Label(meas, text="", bg=T["surface"], fg=T["muted"], font=font(10), anchor="w",
                             justify="left", wraplength=640)
    cal_state_sub.pack(fill="x")
    # Four sensors laid out like the robot seen from above (front on top).
    spectrum = tk.Canvas(meas, height=280, bg=T["surface"], highlightthickness=0)
    spectrum.pack(fill="x", pady=(8, 0))

    def draw_spectrum(previews, in_step, live):
        spectrum.delete("all")
        w, h = max(spectrum.winfo_width(), 400), int(spectrum["height"])
        pw, ph = w / 2, h / 2
        for n in range(1, 5):
            x0, y0 = (n - 1) % 2 * pw, (n - 1) // 2 * ph
            on, p = n in in_step, previews.get(n)
            spectrum.create_rectangle(x0 + 3, y0 + 3, x0 + pw - 3, y0 + ph - 3, outline=T["accent"] if on else T["line"],
                                      width=2 if on else 1)
            spectrum.create_text(x0 + 12, y0 + 14, anchor="w", fill=T["ink"] if on else T["muted"], font=font(9, "bold"),
                                 text=SENSOR_NAMES[n] + ("  · 이번 단계" if on else ""))
            msg = ("준비 단계에서만 실시간 표시" if not live else "값 없음" if p is None else
                   "무효: 포화(너무 가깝거나 밝음) 또는 무수광" if not p["ready"] else None)
            if msg:
                spectrum.create_text(x0 + pw / 2, y0 + ph / 2 + 6, text=msg, font=font(9),
                                     fill=T["bad"] if live and p is not None else T["faint"])
                continue
            chans = [(k, v) for k, v in p["channels"].items() if k in SPECTRUM]
            top, bottom = y0 + 40, y0 + ph - 22

            def level(v):  # sqrt scale so black (low counts) still shows
                return bottom - (bottom - top) * min(1.0, max(v, 0) / AS7341_FULL_SCALE) ** 0.5

            sat_y = level(NEAR_SATURATION)  # dashed line = near saturation
            spectrum.create_line(x0 + 10, sat_y, x0 + pw - 10, sat_y, fill=T["bad"], dash=(3, 3))
            slot = (pw - 20) / max(len(chans), 1)
            for i, (k, v) in enumerate(chans):
                color, label = SPECTRUM[k]
                bx, y, hot = x0 + 10 + i * slot, level(v), v >= NEAR_SATURATION
                spectrum.create_rectangle(bx + slot * 0.2, y, bx + slot * 0.8, bottom, fill=color,
                                          outline=T["bad"] if hot else "", width=2)
                spectrum.create_text(bx + slot / 2, y - 7, text=str(v), font=font(8, mono=True),
                                     fill=T["bad"] if hot else T["ink"])
                spectrum.create_text(bx + slot / 2, bottom + 10, text=label, font=font(8), fill=T["muted"])

    result_card = card(cal_right, "이번 측정 결과")
    result_text = tk.Label(result_card, text="아직 없음", bg=T["surface"], fg=T["faint"], font=font(10, mono=True),
                           anchor="w", justify="left")
    result_text.pack(fill="x")
    result_notes = tk.Frame(result_card, bg=T["surface"])
    result_notes.pack(fill="x", pady=(6, 0))

    action_row = tk.Frame(cal_right, bg=T["bg"])
    action_row.pack(fill="x", pady=(0, 10))
    primary_btn = button(action_row, "", kind="accent")
    primary_btn.config(font=font(13, "bold"), padx=22, pady=10)
    primary_btn.pack(side="left")
    second_btn = button(action_row, "")
    second_btn.pack(side="left", padx=8)
    back_btn = button(action_row, "이전 단계", lambda: cal_send("BACK"))
    back_btn.pack(side="left")
    abort_btn = button(action_row, "중단", kind="stop")
    abort_btn.pack(side="right")

    overview = card(cal_right, "전체 순서 (색마다 평평 · 들림 · 앞 눌림 · 뒤 눌림)")
    ov = tk.Canvas(overview, height=110, bg=T["surface"], highlightthickness=0)
    ov.pack(fill="x")
    ov_cells = {}
    for p, label in enumerate(("평평 4개", "들림 4개", "눌림 앞 1·2", "눌림 뒤 3·4")):
        ov.create_text(130 + p * 90, 10, text=label, fill=T["muted"], font=font(9, "bold"))
    for ci, color in enumerate(("BLACK", "RED", "BLUE")):
        ov.create_text(40, 36 + ci * 30, text=COLOR_KO[color][0], fill=T["muted"], font=font(9, "bold"))
        for p in range(4):
            x = 90 + p * 90
            ov_cells[ci * 4 + p] = ov.create_rectangle(x, 26 + ci * 30, x + 80, 46 + ci * 30,
                                                       fill=T["sunk"], outline="")

    def cal_send(line):
        note("#", f"calibration button -> {line}")
        send(line)

    def confirm_abort():
        if messagebox.askyesno("교정 중단", "지금까지 잰 값이 모두 버려집니다. 중단할까요?"):
            cal_send("ABORT")

    def confirm_erase():
        if messagebox.askyesno("저장된 교정 지우기", "보드에 저장된 교정값을 지웁니다. 계속할까요?"):
            cal_send("CAL ERASE")

    def check_colors():
        note("#", "go check colors after calibration")
        tabs.select(test_tab)
        send("LOG ON")
        send("MODE DATA")
        set_hint("시험 탭: 센서를 각 색 위에 올려 로봇 그림의 색이 맞는지 보세요")

    abort_btn.config(command=confirm_abort)

    def paint_cal_robot(sensors, color):
        for i, (rect, num, name, info) in cal_tiles.items():
            on = i in sensors
            fill = FLOOR.get(color, T["sunk"]) if on else T["sunk"]
            ink = ("#FFFFFF" if color != "YELLOW" else T["ink"]) if on else T["faint"]
            cal_canvas.itemconfig(rect, fill=fill, outline=T["accent"] if on else "")
            cal_canvas.itemconfig(num, fill=ink)
            cal_canvas.itemconfig(name, text=COLOR_KO[color][0] if on else "", fill=ink)
            cal_canvas.itemconfig(info, text=CORNERS[i], fill=ink)

    def set_buttons(primary=None, second=None, back=False, abort=True):
        for btn, spec in ((primary_btn, primary), (second_btn, second)):
            if spec:
                btn.config(text=spec[0], command=spec[1], state="normal")
                btn.pack_configure()
                btn.lift()
            else:
                btn.config(text="", state="disabled")
        back_btn.config(state="normal" if back else "disabled")
        abort_btn.config(state="normal" if abort else "disabled")

    def render_cal():
        c = state.cal
        connected = link["obj"] is not None
        active = c is not None and c["state"] not in ("Idle",) and state.cal_finished is None
        step = c["step"] if c else 0
        done_steps = step if active else (CAL_STEPS if state.cal_finished == "saved" else 0)
        cal_progress["value"] = done_steps
        for k, cell in ov_cells.items():
            ov.itemconfig(cell, fill=T["ok"] if k < done_steps else (T["accent"] if active and k == step else T["sunk"]))
        stored = state.cal_stored
        cal_stored_label.config(text="저장된 교정: " + ("있음" if stored else "없음" if stored is False else "—"))

        rs = state.cal_results
        if rs and active and c["state"] == "WaitingForAccept":
            result_text.config(text="\n".join(
                f"센서 {n}: 샘플 {r['samples']}  밝기 {r['brightness']:.1f}  최소 {r['min']}  최대 {r['max']}  흔들림 {r['noise']:.2f}"
                for n, r in sorted(rs.items())), fg=T["ink"])
            notes = [(level, f"센서 {n}: {msg}") for n, r in sorted(rs.items())
                     for level, msg in cal_quality(r) if level != "ok"] or [("ok", "좋아 보입니다")]
        else:
            result_text.config(text="아직 없음", fg=T["faint"])
            notes = []
        for w in result_notes.winfo_children():
            w.destroy()
        for level, msg in notes:
            tk.Label(result_notes, text=("✓ " if level == "ok" else "⚠ ") + msg, bg=T["surface"],
                     fg={"ok": T["ok"], "warn": T["warn"], "bad": T["bad"]}[level], font=font(10, "bold"),
                     anchor="w").pack(fill="x")

        # Firmware reports NOT_READY for saturated (too close / too bright) samples too.
        draw_spectrum(state.cal_previews, c.get("sensors", []) if c else [],
                      bool(active and c and c["state"] == "WaitingForReady"))

        if not connected:
            cal_step_label.config(text="교정 전")
            cal_title.config(text="먼저 보드에 연결하세요")
            cal_steps_text.config(text="\n".join(f"{n}. {t}" for n, t in enumerate(CAL_PREP, 1)))
            cal_state_label.config(text="연결 안 됨", fg=T["faint"])
            cal_state_sub.config(text="위의 포트를 고르고 연결을 누르세요. 펌웨어는 robot_bench_v2_manual이어야 합니다.")
            paint_cal_robot((), "BLACK")
            set_buttons(abort=False)
            return
        if not active:
            cal_step_label.config(text="교정 전" if state.cal_finished != "saved" else "교정 완료")
            if state.cal_finished == "saved":
                cal_title.config(text="저장했습니다. 이제 색이 맞게 나오는지 확인하세요")
                cal_steps_text.config(text="1. 아래 '색 확인하러 가기'를 누릅니다.\n"
                                           "2. 각 센서를 빨강·파랑·검정 위에 올려 봅니다 (노랑은 교정하지 않음).\n"
                                           "3. 로봇 그림의 칸 색이 바닥과 같으면 끝입니다.\n"
                                           "4. 경기 전에는 yd_esp32_s3_v2를 다시 올리세요 (교정값은 그대로 남습니다).")
                cal_state_label.config(text="저장 완료", fg=T["ok"])
                cal_state_sub.config(text="교정값은 보드의 NVS에 남아서 경기용 펌웨어도 그대로 씁니다.")
                set_buttons(("색 확인하러 가기  ▶", check_colors), ("다시 교정", lambda: cal_send("CAL START")), abort=False)
            else:
                cal_title.config(text="준비")
                cal_steps_text.config(text="\n".join(f"{n}. {t}" for n, t in enumerate(CAL_PREP, 1)))
                msg = state.cal_message or ""
                if "Rejected" in msg:
                    cal_state_label.config(text="시작할 수 없음", fg=T["bad"])
                    cal_state_sub.config(text="센서 4개가 모두 준비되지 않았습니다. 시험 탭의 센서 1–4 상태와 배선을 확인하세요.")
                elif state.cal_finished == "aborted":
                    cal_state_label.config(text="중단됨", fg=T["warn"])
                    cal_state_sub.config(text="잰 값은 버려졌습니다. 다시 시작하면 1단계부터입니다.")
                else:
                    cal_state_label.config(text="시작 전", fg=T["ink"])
                    cal_state_sub.config(text="교정 중에는 모터가 브레이크로 묶이고 시험 로그가 멈춥니다.")
                set_buttons(("교정 시작  ▶", lambda: cal_send("CAL START")), ("저장된 교정 지우기", confirm_erase), abort=False)
            paint_cal_robot((), "BLACK")
            return

        info = {"sensors": c["sensors"], "color": c["color"], "pose": c["pose"]}
        cname, cplace = COLOR_KO.get(info["color"], (info["color"], info["color"]))
        pname, gap, pdesc = POSE_KO.get(info["pose"], (info["pose"], 0, ""))
        cal_step_label.config(text=f"단계 {step + 1} / {CAL_STEPS}")
        which = ("센서 4개" if len(info["sensors"]) == 4 else
                 "앞 센서 1·2" if info["sensors"] == [1, 2] else "뒤 센서 3·4")
        tilt = ("" if info["pose"] != "PRESSED" else
                " (앞으로 기울임)" if info["sensors"] == [1, 2] else " (뒤로 기울임)")
        cal_title.config(text=f"{which} · {cname} · {pname} {gap}mm{tilt}")
        paint_cal_robot(info["sensors"], info["color"])
        cal_steps_text.config(text=f"1. {which}가 모두 {cplace} 위에 있게 로봇을 둡니다.\n"
                                   f"2. {pdesc}.\n"
                                   f"3. 손으로 잡아 움직이지 않게 한 뒤 '측정 시작'을 누릅니다.\n"
                                   f"   1초 카운트다운 → 1초 측정. 그동안 움직이지 마세요.")
        st = c["state"]
        if st == "WaitingForReady":
            if state.cal_message and "Too few" in state.cal_message:
                cal_state_label.config(text="다시 재야 합니다", fg=T["warn"])
                cal_state_sub.config(text="샘플이 너무 적었습니다. 센서 연결과 위치를 확인하고 다시 측정하세요.")
            else:
                cal_state_label.config(text="위치를 맞추세요", fg=T["ink"])
                cal_state_sub.config(text="실시간 밝기가 가득 차서 빨개지면 너무 가깝거나 밝은 것입니다.")
            set_buttons(("측정 시작  ▶   (Enter)", lambda: cal_send("READY")), back=step > 0)
        elif st == "Countdown":
            n = state.cal_count
            cal_state_label.config(text=f"{n}…" if n else "곧 시작", fg=T["accent"])
            cal_state_sub.config(text="움직이지 마세요")
            set_buttons()
        elif st == "Measuring":
            cal_state_label.config(text=f"측정 중 · 샘플 {c['samples']}", fg=T["accent"])
            cal_state_sub.config(text="1초 동안 움직이지 마세요")
            set_buttons()
        elif st == "WaitingForAccept":
            bad = any(level != "ok" for r in rs.values() for level, _ in cal_quality(r))
            cal_state_label.config(text="결과 확인", fg=T["warn"] if bad else T["ok"])
            cal_state_sub.config(text="좋으면 다음 단계로, 이상하면 다시 재세요.")
            set_buttons(("좋음 · 다음 단계  ▶   (Enter)", lambda: cal_send("ACCEPT")),
                        ("다시 측정", lambda: cal_send("RETRY")), back=True)
        elif st == "Completed":
            cal_step_label.config(text=f"단계 {CAL_STEPS} / {CAL_STEPS}")
            cal_title.config(text=f"{CAL_STEPS}단계를 모두 쟀습니다")
            cal_steps_text.config(text="'보드에 저장'을 눌러야 교정값이 남습니다.\n저장하지 않고 중단하면 모두 버려집니다.")
            cal_state_label.config(text="저장 대기", fg=T["accent"])
            cal_state_sub.config(text="")
            set_buttons(("보드에 저장  ▶   (Enter)", lambda: cal_send("CAL SAVE")))
        elif st == "Error":
            cal_state_label.config(text="저장 실패", fg=T["bad"])
            cal_state_sub.config(text="NVS 저장에 실패했습니다. 다시 저장을 눌러 보고, 계속 실패하면 로그를 보내 주세요.")
            set_buttons(("다시 저장", lambda: cal_send("CAL SAVE")))

    def on_tab(_=None):
        # Ask once per connection whether the board already holds a calibration.
        if tabs.select() == str(cal_tab) and link["obj"] and not calibrating() and not ui.get("asked_cal"):
            ui["asked_cal"] = True
            send("CAL STATUS")

    tabs.bind("<<NotebookTabChanged>>", on_tab)

    def cal_enter(_=None):
        if tabs.select() == str(cal_tab) and str(primary_btn.cget("state")) == "normal" and primary_btn.cget("text"):
            primary_btn.invoke()

    # ----- actions -----
    def send(line):
        if not link["obj"]:
            set_hint("먼저 보드에 연결하세요")
            return
        if requires_drive_permission(line) and not wheels_up.get():
            set_hint("SW2/DRIVE는 바퀴를 띄우고 확인란을 체크한 뒤 보낼 수 있습니다")
            return
        if line.strip().upper() == "SW2" and calibrating():
            set_hint("색 교정 중에는 SW2 시작을 사용할 수 없습니다")
            return
        link["obj"].send(line)
        add_log("› " + line, "sent")
        up = line.strip().upper()
        if up.startswith("DRIVE"):
            return
        ui["driving"] = None
        if up in ("MODE MANUAL", "MODE MOTOR"):
            set_hint("이제 SW2를 눌렀다 떼면 움직일 수 있습니다")
        elif up == "SW1":
            set_hint("SW1 비상정지를 요청했습니다. 재부팅 전까지 구동할 수 없습니다")
        elif up == "SW2":
            set_hint("SW2 눌림·해제를 요청했습니다. 보드의 시작 허용 상태를 확인하세요")
        elif up == "STOP" or up == "":
            set_hint("정지했습니다. 다시 움직이려면 주행 준비(또는 SW2)를 누르세요")

    def calibrating():
        c = state.cal
        return c is not None and c["state"] != "Idle" and state.cal_finished is None

    def arm():
        if not wheels_up.get():
            set_hint("바퀴를 띄우고 '바퀴를 바닥에서 띄웠음'에 체크하세요")
            return
        send("MODE MANUAL")
        # A mode change cancels a pending start, so the SW2 press goes after it.
        root.after(400, lambda: send("SW2"))

    def drive_start(direction):
        if not link["obj"]:
            set_hint("먼저 보드에 연결하세요")
            return
        if calibrating():
            set_hint("색 교정 중에는 주행할 수 없습니다")
            return
        if not wheels_up.get():
            set_hint("바퀴를 띄우고 '바퀴를 바닥에서 띄웠음'에 체크하세요")
            return
        send(drive_command(direction, power.get()))
        ui["driving"] = direction
        pad_keys[direction].config(bg=T["accent"], fg="#FFFFFF")

    def drive_stop():
        for k in pad_keys.values():
            k.config(bg=T["sunk"], fg=T["ink"])
        if link["obj"] and link["obj"].drive:
            link["obj"].release()  # board stops within 250 ms; no STOP, so no new SW2 needed
            add_log("› (방향키 뗌 · 반복 중지)", "sent")
        ui["driving"] = None

    def send_entry(_=None):
        line = entry.get().strip()
        if not line:
            return
        if line.upper().startswith("DRIVE") and not wheels_up.get():
            set_hint("DRIVE는 '바퀴를 바닥에서 띄웠음' 체크 후에 보낼 수 있습니다")
            return
        history.append(line)
        hist_pos[0] = len(history)
        entry.delete(0, "end")
        note("#", "typed command")
        send(line)

    def walk(step):
        if history:
            hist_pos[0] = max(0, min(len(history) - 1, hist_pos[0] + step))
            entry.delete(0, "end")
            entry.insert(0, history[hist_pos[0]])

    entry.bind("<Return>", send_entry)
    entry.bind("<Up>", lambda e: walk(-1))
    entry.bind("<Down>", lambda e: walk(1))
    send_btn.config(command=send_entry)

    keymap = {"Up": "fwd", "Down": "back", "Left": "left", "Right": "right"}
    held = set()

    def key_down(e):
        if e.keysym == "Escape":
            send("STOP")
            return
        if e.widget is entry:
            return
        if e.keysym == "Return":
            cal_enter()
        elif e.keysym == "space":
            send("STOP")
        elif e.keysym in keymap and e.keysym not in held and tabs.select() == str(test_tab):
            held.add(e.keysym)
            drive_start(keymap[e.keysym])

    def key_up(e):
        if e.keysym in held:
            held.discard(e.keysym)
            drive_stop()

    root.bind_all("<KeyPress>", key_down)
    root.bind_all("<KeyRelease>", key_up)

    # ----- rendering -----
    def add_log(line, tag=None):
        if paused.get():
            return
        at_end = text.yview()[1] > 0.98
        text.insert("end", line + "\n", tag or ())
        if int(text.index("end-1c").split(".")[0]) > 5000:
            text.delete("1.0", "1000.0")
        if at_end:
            text.see("end")

    def visible(line):
        key = line.split(",", 1)[0]
        return filters[key if key in ("D", "S", "I") else "기타"].get()

    def render_banner():
        d, b = state.d, state.bat
        if not link["obj"]:
            set_banner("idle", "연결 안 됨", "포트를 고르고 연결하세요 · 펌웨어는 robot_bench_v2_manual")
        elif state.estop:
            set_banner("bad", "비상정지 (SW1)", "모터 출력이 막혔습니다 · 보드를 재부팅해야 다시 움직입니다")
        elif calibrating():
            c = state.cal
            set_banner("warn", f"색 교정 중 · 단계 {c['step'] + 1}/{CAL_STEPS}",
                       "모터는 브레이크 상태입니다 · 색 교정 탭의 안내를 따라가세요")
        elif b and b["result"] in BAT_FAILED:
            set_banner("bad", f"배터리 판정 실패 · {b['result']}",
                       f"{b['mv'] / 1000:.2f} V · 3S/4S 범위가 아니면 모터가 움직이지 않습니다")
        elif not ui["seen_data"]:
            set_banner("idle", "데이터 기다리는 중", "보드의 RST를 한 번 누르세요 · BAT 줄과 상태 줄이 나옵니다")
        elif ui["driving"]:
            arrow = {"fwd": "▲ 앞", "back": "▼ 뒤", "left": "◀ 좌회전", "right": "우회전 ▶"}[ui["driving"]]
            set_banner("drive", f"구동 중 · {arrow} · 출력 {int(power.get())}", "손을 떼면 바로 멈춥니다")
        elif d and d["allowed"]:
            set_banner("ok", f"움직일 수 있음 · MODE {d['mode']}", "주행 버튼이나 방향키를 누르는 동안 움직입니다")
        elif d and d["mode"] in ("MANUAL", "MOTOR", "ALIGN", "FOLLOW", "AVOID"):
            set_banner("warn", f"SW2 대기 · MODE {d['mode']}", "주행 준비 버튼(또는 SW2)을 누르면 움직일 수 있습니다")
        elif d:
            set_banner("idle", f"MODE {d['mode']}", "센서 값을 보려면 DATA, 주행하려면 MANUAL")
        else:
            set_banner("idle", "연결됨", "로그가 멈춰 있으면 LOG ON")
        coach = led_coach(state.estop, state.system, b, bool(d and d["allowed"]))
        banner_led.config(text=f"보드 LED는 지금: {coach} · 다르면 로그를 학현에게"
                          if link["obj"] and ui["seen_data"] else "")

    def render_status():
        bat_check_btn.config(text="배터리 판정: " + ("— (보드 연결 후 표시)" if state.bat_check is None else
                                                   "켜짐 → 누르면 끔 (경기용)" if state.bat_check else
                                                   "꺼짐 → 누르면 켬 (시험용 보호)"))
        b = state.bat
        stat_rows["bat"].config(text=f"{b['mv'] / 1000:.2f} V · {b['cells']}S · {b['result']}" if b else "— (부팅 직후 한 번)",
                                fg=(T["ok"] if b["result"].endswith("ok") else T["warn"] if b["result"].endswith("check_off")
                                    else T["bad"]) if b else T["faint"])
        stat_rows["estop"].config(text="ESTOP · 재부팅 필요" if state.estop else "정상",
                                  fg=T["bad"] if state.estop else T["ok"] if link["obj"] else T["faint"])
        m = state.imu
        if m:
            stat_rows["imu"].config(text=f"{m['state']} · {m['who']} · {imu_freshness(m)} · {m['samples']}",
                                    fg=T["ok"] if m["fresh"] else T["bad"])
            stat_rows["acc"].config(text="  ".join(f"{v / 4096:+.2f}" for v in m["accel"]), fg=T["ink"])
        else:
            stat_rows["imu"].config(text="—", fg=T["faint"])
            stat_rows["acc"].config(text="—", fg=T["faint"])

    def paint_tile(key, text, level):
        bg, fg = {"off": (T["sunk"], T["ink"]), "on": (T["ok_soft"], T["ok"]),
                  "drive": (T["accent_soft"], T["accent"]), "bad": (T["bad_soft"], T["bad"]),
                  "stale": (T["sunk"], T["faint"])}[level]
        tile, top_row, n, g, v = pin_tiles[key]
        for w in (tile, top_row, n, g, v):
            w.config(bg=bg)
        v.config(text=text, fg=fg)

    def render_pins():
        scan = state.i2c
        scan_label.config(text="I2C: " + i2c_summary(scan),
                          fg=T["faint"] if not scan else T["bad"] if scan.get("err") or
                          (scan.get("finished") and not scan.get("mux")) else T["ok"] if scan.get("finished") else T["warn"])
        now = time.monotonic()
        p = state.pins if state.pins and now - state.pins_at <= 2.0 else None
        a = state.adc if state.adc and now - state.adc_at <= 2.0 else None
        if p:
            paint_tile("sw1", "눌림" if p["sw1"] else "안 눌림", "bad" if p["sw1"] else "off")
            paint_tile("sw2", "눌림" if p["sw2"] else "안 눌림", "on" if p["sw2"] else "off")
            for key in ("sda", "scl"):  # idle I2C lines sit HIGH; LOW = bus held down
                on = p.get(key)
                paint_tile(key, "—" if on is None else "HIGH" if on else "LOW",
                           "stale" if on is None else "off" if on else "bad")
            for key, on in zip(("en1", "en2"), p["en"]):
                paint_tile(key, "HIGH" if on else "LOW", "on" if on else "off")
            for key, duty in zip(("lpwm1", "rpwm1", "lpwm2", "rpwm2"), p["pwm"]):
                paint_tile(key, f"{duty * 100 // 255}% ({duty})" if duty else "0", "drive" if duty else "off")
        else:
            for key in ("sw1", "sw2", "sda", "scl", "en1", "en2", "lpwm1", "rpwm1", "lpwm2", "rpwm2"):
                paint_tile(key, "—", "stale")
        for key in ADC_KEYS:
            if not a:
                paint_tile(key, "—", "stale")
            elif key == "bat":
                paint_tile(key, f"{a[key]}mV", "off")
            else:
                paint_tile(key, f"{a[key]}mV", "drive" if a[key] >= 100 else "off")
        if not p and not a:
            pin_note.config(text="PIN/ADC 줄 없음 · 최신 robot_bench_v2_manual인지, LOG ON인지 확인")
        else:
            pin_note.config(text="EN HIGH + PWM 한쪽만 % = 그 방향으로 구동 · 브레이크는 EN HIGH + PWM 0 · "
                                 "IS는 1k/470Ω 분압 뒤 GPIO 전압 · BAT 핀 ×11 = 배터리 · 레지스터·ADC 값이며 파형 측정 아님")

    def render_checks():
        auto = {}
        if state.bat:
            auto["bat"] = f"BAT {state.bat['mv'] / 1000:.2f} V {state.bat['cells']}S · 멀티미터와 같으면 체크"
        seen = [i for i in range(1, 5) if i in state.sensors and state.sensors[i]["valid"]]
        if seen:
            auto["sensors"] = f"응답: {', '.join(map(str, seen))}번 · 가려서 번호가 맞으면 체크"
        if state.imu:
            auto["imu"] = f"{state.imu['state']} · {imu_freshness(state.imu)}"
            if state.imu["state"] == "READY" and state.imu["fresh"] and not checks_done["imu"].get():
                checks_done["imu"].set(True)
                note("#", "check imu = True (auto)")
        if state.estop:
            auto["sw1"] = "ESTOP 확인됨 · 재부팅하세요"
            if not checks_done["sw1"].get():
                checks_done["sw1"].set(True)
                note("#", "check sw1 = True (auto)")
        for key, (title, sub, how) in check_rows.items():
            done = checks_done[key].get()
            title.config(fg=T["ok"] if done else T["ink"])
            sub.config(text=auto.get(key, how), fg=T["ok"] if done else (T["muted"] if key in auto else T["faint"]))

    def render():
        render_banner()
        render_status()
        render_pins()
        render_checks()
        paint_robot()
        paint_camera()
        render_cal()
        enabled = wheels_up.get() and link["obj"] is not None and not calibrating()
        for d, k in pad_keys.items():
            if ui["driving"] != d:
                k.config(bg=T["sunk"] if enabled else T["bg"], fg=T["ink"] if enabled else T["faint"])

    def pump():
        l = link["obj"]
        if l:
            for _ in range(300):
                try:
                    line = l.lines.get_nowait()
                except queue.Empty:
                    break
                kind = state.feed(line)
                if kind in ("d", "s", "bat", "cal"):
                    ui["seen_data"] = True
                if kind == "cal_result" and state.cal:
                    c, r = state.cal, parse_line(line)[1]
                    verdict = "; ".join(m for _, m in cal_quality(r))
                    note("#", f"CAL step {c['step'] + 1} sensor {r['sensor']} {c['color']} {c['pose']} -> {verdict}")
                if kind in ("cal_rejected", "cal_save_failed", "cal_too_few"):
                    note("!", f"calibration problem: {line}")
                if visible(line):
                    tag = ("bad" if line.startswith(("ERR", "ESTOP", "[연결")) else
                           "cal" if line.startswith(("[CAL]", "CAL,")) else
                           "imu" if kind == "i" else "dim" if kind in ("d", "s") else None)
                    add_log(line, tag)
            if not l.running:
                disconnect()
                set_hint("연결이 끊겼습니다. 케이블을 확인하고 다시 연결하세요")
        render()
        root.after(50, pump)

    def connect():
        if link["obj"]:
            disconnect()
            set_hint("연결을 끊었습니다 (STOP을 보냄)")
            return
        if not port_var.get():
            set_hint("포트를 고르세요 · 목록이 비었으면 ↻")
            return
        port = port_var.get().split()[0]
        log = SessionLog(new_log_path(), f"port={port} baud={baud_var.get()} python={sys.version.split()[0]}")
        try:
            link["obj"] = DemoLink(log) if port == "DEMO" else Link(port, int(baud_var.get()), log)
        except Exception as error:
            log.write("!", f"open failed: {error}")
            log.close()
            set_hint(f"연결 실패: {error} · 시리얼 모니터가 열려 있으면 닫으세요")
            return
        session["log"] = log
        state.__init__()
        ui["seen_data"] = False
        ui["asked_cal"] = False
        conn_btn.config(text="끊기", bg=T["sunk"], fg=T["ink"], activebackground=T["line"])
        log_label.config(text=f"로그: logs/{os.path.basename(log.path)}  (클릭하면 폴더 열기)")
        set_hint(f"{port} {baud_var.get()} 연결됨 · 보드 RST를 누르면 BAT 줄이 나옵니다")

    def disconnect():
        l, link["obj"] = link["obj"], None
        if l:
            if calibrating():
                try:
                    l.send("ABORT")  # leave the board in normal bench use
                except Exception:
                    pass
            l.close()
        if session["log"]:
            session["log"].write("#", "disconnected")
            session["log"].close()
            session["log"] = None
        ui["driving"] = None
        conn_btn.config(text="연결", bg=T["accent"], fg="#FFFFFF", activebackground=T["accent_hi"])

    conn_btn.config(command=connect)

    def on_close():
        disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    refresh_ports()
    pump()
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cmd", metavar="PORT", dest="port", help="no window: open PORT, send commands, print replies")
    parser.add_argument("commands", nargs="*", help="commands for --cmd, e.g. \"MODE DATA\"")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--listen", type=float, default=2.0, help="seconds to keep printing after the last command")
    parser.add_argument("--allow-drive", action="store_true", help="permit DRIVE in --cmd mode (wheels lifted!)")
    parser.add_argument("--drive-seconds", type=float, default=1.0, help="how long each DRIVE repeats (max 3)")
    args = parser.parse_args()
    if args.port:
        sys.exit(run_cmd(args))
    run_gui()


if __name__ == "__main__":
    main()
