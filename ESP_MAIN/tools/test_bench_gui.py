"""Self-check for bench_gui.py parsing: python tools/test_bench_gui.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_gui import BenchState, cal_quality, cal_step_info, drive_command, parse_line  # noqa: E402

# Lines in the exact formats bench_main.cpp / bench_v2_app.cpp print.
state = BenchState()
assert state.feed("BAT battery_ok 15200mV 4S") == "bat"
assert state.bat == {"result": "battery_ok", "mv": 15200, "cells": 4}
assert state.feed("D,1234,MANUAL,1,100,-100,0,0,0,0,0,0") == "d"
assert state.d["mode"] == "MANUAL" and state.d["allowed"] and state.d["right"] == -100
assert state.feed("S,1240,3,77,1,RED,1,812,10,20,30,40,50,0") == "s"
assert state.sensors[3]["color"] == "RED" and state.sensors[3]["error"] == 0
assert state.feed("S,1240,9,77,1,RED,1,812,10,20,30,40,50,0") == "s" and 9 not in state.sensors
assert state.feed("I,1300,READY,68,1,120,0,4096,-12,30,1,2,3") == "i"
assert state.imu["fresh"] and state.imu["accel"] == [4096, -12, 30]
assert state.feed("SYSTEM,1400,2,0,1,1,0,2,1,0") == "system" and not state.estop

# The motor latch starts set before SW2 and also applies to ordinary faults.
# Neither is evidence that the physical SW1 emergency switch was pressed.
for line in (
    "SYSTEM,20846,1,1,0,0,0,0,1,0",  # actual field log: waiting, no faults
    "SYSTEM,20847,1,1,0,0,1,0,1,1",  # reset rearm only
    "SYSTEM,20848,5,1,0,0,0,0,1,4",  # motor fault
    "SYSTEM,20849,6,1,0,0,0,0,1,8",  # application/battery fault
):
    waiting = BenchState()
    assert waiting.feed(line) == "system" and not waiting.estop
    assert waiting.system["motor_latched"] and not waiting.system["estop"]

# A real emergency state or emergency fault bit is sufficient evidence.
for line in (
    "SYSTEM,21000,7,1,0,0,0,0,1,16",
    "SYSTEM,21000,7,1,0,0,0,0,1,0",
    "SYSTEM,21000,1,1,0,0,0,0,1,16",
):
    emergency = BenchState()
    assert emergency.feed(line) == "system" and emergency.estop
    emergency.feed("SYSTEM,21001,1,1,0,0,0,0,1,0")
    assert emergency.estop  # only a new GUI connection clears observed ESTOP
assert state.feed("MOTOR,1500,100,100,1,0,0,1750,0") is None  # shown in log only
assert state.feed("ERR command rejected; stopped") is None
assert state.feed("D,broken") is None and state.feed("") is None
assert state.feed("ESTOP") == "estop" and state.estop
state.feed("SYSTEM,22000,1,1,0,0,0,0,1,0")
assert state.estop
assert parse_line("BAT battery_missing 0mV 0S")[1]["result"] == "battery_missing"

assert drive_command("fwd", 100) == "DRIVE 100 100"
assert drive_command("back", 100) == "DRIVE -100 -100"
assert drive_command("left", 50) == "DRIVE -50 50"
assert drive_command("right", 999) == "DRIVE 200 -200"  # clamped to the bench limit

# Calibration lines (formats from color_calibration.cpp and bench_v2_app.cpp).
cal = BenchState()
assert cal.feed("[CAL][STATUS] active=0 stored=1 state=0 step=1/48 sensor=1 FRONT_LEFT color=RED pose=LEVEL samples=0 margin=15/20%") == "cal_status"
assert cal.cal_stored is True
assert cal.feed("[CAL] Calibration started. Motors are braked until it ends.") == "cal_started"
assert cal.cal["state"] == "WaitingForReady" and cal.cal["step"] == 0
assert cal.feed("CAL,1234,1,6,0,BLUE,LEVEL,0,1") == "cal"
assert cal.cal == {"ms": 1234, "state": "WaitingForReady", "step": 6, "sensor": 1, "color": "BLUE",
                   "pose": "LEVEL", "samples": 0, "stored": True}
assert cal.feed("[CAL][PREVIEW] S1 F2=1 F3=2 F5=3 F6=4 F7=5 C=512") == "cal_preview" and cal.cal_preview["clear"] == 512
assert cal.feed("[CAL][PREVIEW] S1 NOT_READY") == "cal_preview" and not cal.cal_preview["ready"]
assert cal.feed("[CAL] Starting in 2...") == "cal_count" and cal.cal_count == 2
assert cal.feed("[CAL] Measuring for 3 seconds. Do not move.") == "cal_measuring" and cal.cal_measuring
assert cal.feed("[CAL][RESULT] samples=28 brightness=612.3 min=600 max=625 noise=2.10") == "cal_result"
assert cal.cal_result == {"samples": 28, "brightness": 612.3, "min": 600, "max": 625, "noise": 2.1}
assert cal_quality(cal.cal_result) == [("ok", "좋아 보입니다")]
assert cal.feed("CAL,1500,1,7,0,BLUE,LIFTED,0,1") == "cal" and cal.cal_result is None  # next step clears it
assert cal.feed("[CAL] Calibration saved to NVS.") == "cal_saved" and cal.cal_finished == "saved"
assert cal.feed("[CAL] Stored calibration erased.") == "cal_erased" and cal.cal_stored is False
assert cal.feed("[CAL] Rejected: sensor ready mask is 0x07.") == "cal_rejected"
levels = [lvl for lvl, _ in cal_quality({"samples": 4, "brightness": 20.0, "min": 1, "max": 1700, "noise": 9.0})]
assert levels == ["bad", "warn", "warn", "warn"]
assert cal_quality({"samples": 28, "brightness": 1500.0, "min": 1490, "max": 1610, "noise": 2.0})[0][0] == "ok"
assert cal_step_info(0) == {"sensor": 1, "color": "RED", "pose": "LEVEL"}
assert cal_step_info(4) == {"sensor": 1, "color": "YELLOW", "pose": "LIFTED"}
assert cal_step_info(47) == {"sensor": 4, "color": "BLACK", "pose": "PRESSED"}
print("bench_gui parse checks passed")
