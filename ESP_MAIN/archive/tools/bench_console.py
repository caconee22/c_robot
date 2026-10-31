"""PC serial console for robot_bench / color_calibration (UART0).

Why: the firmware drops a line if keys arrive more than 250 ms apart, and
DRIVE must be resent every 250 ms. A normal serial monitor sends each key as
you type, so commands fail. This tool sends whole lines and repeats DRIVE.

Run in the PlatformIO terminal (its python already has pyserial):
    python tools/bench_console.py COM5            # robot_bench_v2, 57600
    python tools/bench_console.py COM5 --baud 57600
Commands are typed exactly as the firmware expects (MODE DATA, MODE MANUAL,
DRIVE 100 100, STOP, STATUS, CAL START ...).
    DRIVE l r   -> sent every 100 ms until the next command
    empty line  -> STOP (stops the DRIVE repeat too)
    Ctrl+C      -> STOP and quit
Everything received is also saved to bench_<date>_<time>.log in this folder.
"""
import argparse
import datetime
import sys
import threading
import time

import serial

REPEAT_S = 0.1  # well under the firmware's 250 ms manual hold


class Console:
    def __init__(self, port, log):
        self.port = port
        self.log = log
        self.lock = threading.Lock()
        self.drive = None  # line being repeated, or None
        self.running = True

    def command(self, line):
        line = line.strip()
        if not line:
            line = "STOP"
        # Same lock as the repeater, so no stale DRIVE can follow a STOP.
        with self.lock:
            self.drive = line if line.upper().startswith("DRIVE ") else None
            self.port.write((line + "\n").encode("ascii"))

    def repeat_loop(self):
        while self.running:
            with self.lock:
                if self.drive:
                    self.port.write((self.drive + "\n").encode("ascii"))
            time.sleep(REPEAT_S)

    def read_loop(self):
        while self.running:
            data = self.port.readline()
            if not data:
                continue
            text = data.decode("ascii", "replace").rstrip("\r\n")
            print(text, flush=True)
            self.log.write(text + "\n")
            self.log.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="COM port of the board (see: pio device list)")
    parser.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()

    name = datetime.datetime.now().strftime("bench_%Y%m%d_%H%M%S.log")
    port = serial.Serial(timeout=0.2)
    port.port, port.baudrate = args.port, args.baud
    port.dtr = port.rts = False  # leave EN/BOOT released: opening must not hold the board in reset
    port.open()
    with port, open(name, "w", encoding="utf-8") as log:
        console = Console(port, log)
        for loop in (console.read_loop, console.repeat_loop):
            threading.Thread(target=loop, daemon=True).start()
        print(f"[{args.port} {args.baud}] log: {name}  (empty line = STOP, Ctrl+C = quit)", flush=True)
        try:
            for line in sys.stdin:
                console.command(line)
        except KeyboardInterrupt:
            pass
        finally:
            console.command("STOP")
            console.running = False
            time.sleep(0.3)


if __name__ == "__main__":
    main()
