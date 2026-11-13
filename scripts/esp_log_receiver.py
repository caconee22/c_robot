"""Pi: write the ESP32 run log (UART1, ESP TX -> Pi RX) into one file per run.

The ESP32 sends 14-byte frames in the vision packet layout with version 0x81
(ESP_MAIN/include/communication/run_log.hpp) only on state changes and
decisions, plus a 5 Hz Alive frame while driving:

    AA 55 | 81 | 0A | sequence | kind | a | b | c | d   (uint16 little endian)

Start (SW2) opens logs/runs/run_YYYYmmdd_HHMMSS.csv, Stop closes it. SW1 / SW2
emergency stop sends nothing more (the ESP enters its permanent ESTOP loop),
so the file is also closed when no frame arrives for SILENCE_S.

Boot (11/13): every ESP power-on sends a frame (repeated every 2 s until the
first start, recorded once thanks to a random boot id) with esp_reset_reason and,
when it survived in RTC RAM, the state just before the restart. It is appended
to logs/runs/boots.csv (and into the run file when one is still open), so a
brownout or watchdog restart in a match is on record.

Reads the same /dev/ttyAMA0 the vision sender writes to; it only reads, so the
sender is not touched. Runs as the c-robot-esplog service.

  python3 scripts/esp_log_receiver.py                  # /dev/ttyAMA0
  python3 scripts/esp_log_receiver.py --port /dev/ttyAMA0 --dir logs/runs
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

VERSION, LENGTH, FRAME = 0x81, 10, 14
SILENCE_S = 1.0
DEFAULT_DIR = Path(__file__).resolve().parents[1] / "logs" / "runs"

KINDS = {1: "시작", 2: "상태", 3: "회피", 4: "원돌기", 5: "생존", 6: "정지", 7: "위치", 8: "부팅"}
STATES = ["Boot", "대기", "탐색", "TargetConfirm", "정렬", "돌진", "ContactCheck", "DiveReady",
          "포크브레이크", "포크삽입", "UndercutDrive", "포크밀기", "ScoreHold", "Backoff", "LostTarget",
          "WallEscape", "회피", "StuckRecovery", "비상정지", "정지", "Wander", "시작루틴", "원돌기"]
SIDES = ["없음", "앞왼", "앞오", "앞둘다", "뒤왼", "뒤오", "뒤둘다", "왼쪽면", "오른쪽면", "불분명"]
COLORS = {0: "빨강", 1: "노랑", 2: "파랑", 3: "검정", 255: "모름"}
INTENTS = ["정지", "Coast", "탐색좌", "탐색우", "정렬좌", "정렬우", "전진", "AttackCharge", "SpokeInsert",
           "Push", "DiveBrake", "후진", "EscapeLeft", "EscapeRight"]
SAFETY = ["리셋잠금", "시작대기", "구동", "비상정지"]
# esp_reset_reason_t (11/13). Abnormal = not a power-on or reset button.
RESETS = {0: "모름", 1: "전원켜짐", 2: "리셋핀", 3: "소프트웨어재시작", 4: "패닉(크래시)",
          5: "인터럽트워치독", 6: "태스크워치독", 7: "워치독", 8: "딥슬립", 9: "브라운아웃(전압강하)", 10: "SDIO"}
ABNORMAL = {3, 4, 5, 6, 7, 9}


def name(table, i):
    if isinstance(table, dict):
        return table.get(i, str(i))
    return table[i] if 0 <= i < len(table) else str(i)


def describe(kind: int, a: int, b: int, c: int, d: int) -> str:
    if kind == 1:
        return f"시작 배율={a} 보정L={b} 보정R={c} 배터리={d}S" if d else f"시작 배율={a} 보정L={b} 보정R={c} 배터리=모름"
    if kind == 2:
        return f"{name(STATES, b)} -> {name(STATES, a)} 타워x={c} h={d}"
    if kind == 3:
        mask = "".join(n for bit, n in ((1, "FL"), (2, "FR"), (4, "RL"), (8, "RR")) if b & bit)
        # d: bit0 = charging/fork attack, bit1 = straight back-out (11/11)
        return (f"먼저={name(SIDES, a)} 모서리={mask or b} 색={name(COLORS, c)} 돌진중={d & 1}"
                + (" 일자후진" if d & 2 else ""))
    if kind == 4:
        return f"방향={'오른쪽' if a else '왼쪽'} 타워h={b} x={c}"
    if kind == 5:
        tower = f"x={b}" if b else "안보임"
        return f"{name(STATES, a)} 타워 {tower} 동작={name(INTENTS, c)} {d}"
    if kind == 6:
        return f"이유={name(SAFETY, a)}"
    if kind == 8:   # a: reset reason | boot id << 8; state before it (b 0xFFFF = not kept)
        reason = a & 0xFF
        head = f"리셋원인={name(RESETS, reason)}({reason})" + (" ⚠비정상" if reason in ABNORMAL else "")
        if b == 0xFFFF:
            return head + " 직전상태=없음(전원이 끊겼거나 첫 부팅)"
        return (head + f" 직전상태={name(STATES, b & 0xFF)} 구동중={1 if b & 0x100 else 0}"
                f" 부팅후={c / 10:.1f}s 직전동작={name(INTENTS, d >> 12)} {d & 0x0FFF}")
    if kind == 7:   # rough pose: heading 0.1 deg, x/y mm, signed
        s16 = lambda v: v - 65536 if v >= 32768 else v
        return f"방향={s16(a) / 10:.1f}도 x={s16(b)}mm y={s16(c)}mm IMU={'정상' if d else '없음'}"
    return ""


def parse(buffer: bytearray):
    """Pull complete log frames out of buffer; yields (seq, kind, a, b, c, d, raw)."""
    while True:
        start = buffer.find(b"\xAA\x55")
        if start < 0:
            del buffer[:-1]
            return
        del buffer[:start]
        if len(buffer) < FRAME:
            return
        frame = bytes(buffer[:FRAME])
        if frame[2] != VERSION or frame[3] != LENGTH or frame[5] not in KINDS:
            del buffer[:2]   # not a log frame (or noise): look for the next header
            continue
        del buffer[:FRAME]
        a, b, c, d = struct.unpack_from("<4H", frame, 6)
        yield frame[4], frame[5], a, b, c, d, frame


class RunFile:
    def __init__(self, folder: Path) -> None:
        self.folder = folder
        self.file = None
        self.started = 0.0
        self.last = 0.0

    def open(self, note: str = "") -> None:
        self.close("새 시작")
        self.folder.mkdir(parents=True, exist_ok=True)
        path = self.folder / f"run_{datetime.now():%Y%m%d_%H%M%S}{note}.csv"
        self.file = path.open("w", encoding="utf-8")
        self.file.write("시간s,순번,종류,내용,원본\n")
        self.started = time.monotonic()
        print(f"log: {path}", flush=True)

    def write(self, seq, kind, a, b, c, d, raw) -> None:
        now = time.monotonic()
        self.last = now
        self.file.write(f"{now - self.started:.3f},{seq},{KINDS[kind]},{describe(kind, a, b, c, d)},"
                        f"{raw.hex(' ').upper()}\n")
        self.file.flush()   # power cut: keep what was written

    def close(self, why: str) -> None:
        if self.file:
            self.file.write(f"{time.monotonic() - self.started:.3f},,끝,{why},\n")
            self.file.close()
            self.file = None
            print(f"log closed: {why}", flush=True)


def last_boot(folder: Path):
    """Payload (a, b, c, d) of the last boot in boots.csv, so a receiver restart
    does not record the boot the ESP is still repeating a second time."""
    try:
        lines = (folder / "boots.csv").read_text(encoding="utf-8").strip().splitlines()
        raw = bytes.fromhex(lines[-1].rsplit(",", 1)[1].replace(" ", ""))
        return struct.unpack_from("<4H", raw, 6) if len(raw) == FRAME else None
    except (OSError, IndexError, ValueError, struct.error):
        return None


def note_boot(folder: Path, a, b, c, d, raw) -> bool:
    """Append one ESP boot to boots.csv (kept across runs and Pi reboots).
    The ESP repeats the frame every 2 s until the first start; the random boot
    id in a makes each boot's payload unique, so repeats are skipped. True when
    it was a new boot."""
    if last_boot(folder) == (a, b, c, d):
        return False
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / "boots.csv"
    new = not path.exists()
    with path.open("a", encoding="utf-8") as f:
        if new:
            f.write("시각,리셋원인번호,비정상,내용,원본\n")
        reason = a & 0xFF
        f.write(f"{datetime.now():%Y-%m-%d %H:%M:%S},{reason},{1 if reason in ABNORMAL else 0},"
                f"{describe(8, a, b, c, d)},{raw.hex(' ').upper()}\n")
    print(f"부팅 {describe(8, a, b, c, d)}", flush=True)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="ESP32 run log -> files")
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dir", type=Path, default=DEFAULT_DIR)
    args = parser.parse_args()
    import serial  # pyserial
    port = serial.Serial(args.port, args.baud, timeout=0.1)
    run = RunFile(args.dir)
    buffer = bytearray()
    try:
        while True:
            buffer += port.read(256)
            for seq, kind, a, b, c, d, raw in parse(buffer):
                if kind == 8:
                    if note_boot(args.dir, a, b, c, d, raw) and run.file:
                        run.write(seq, kind, a, b, c, d, raw)
                    continue
                if kind == 1:
                    run.open()
                elif run.file is None:
                    if kind != 5:   # boot / stopped: state frames outside a run, console only
                        print(f"{KINDS[kind]} {describe(kind, a, b, c, d)}", flush=True)
                        continue
                    run.open("_중간부터")   # driving, but the receiver started after the run did
                run.write(seq, kind, a, b, c, d, raw)
                if kind == 6:
                    run.close("정지")
            if run.file and time.monotonic() - run.last > SILENCE_S:
                run.close("신호 끊김 (비상정지 또는 리셋)")
    except KeyboardInterrupt:
        pass
    finally:
        run.close("수신 종료")
        port.close()


if __name__ == "__main__":
    sys.exit(main())
