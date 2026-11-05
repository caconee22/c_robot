"""Self-check for bench_gui.py parsing: python tools/test_bench_gui.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_gui import BenchState, DemoLink, cal_quality, cal_step_info, drive_command, i2c_summary, imu_freshness, parse_line, requires_drive_permission  # noqa: E402

# Lines in the exact formats bench_main.cpp / bench_v2_app.cpp print.
state = BenchState()
assert state.feed("BAT battery_ok 15200mV 4S") == "bat"
assert state.bat == {"result": "battery_ok", "mv": 15200, "cells": 4}
assert state.feed("D,1234,MANUAL,1,100,-100,0,0,0,0,0,0") == "d"
assert state.d["mode"] == "MANUAL" and state.d["allowed"] and state.d["right"] == -100
assert state.d["y"] is None  # older bench firmware: no camera box fields
assert state.feed("D,1300,DATA,0,0,0,1,1,1,700,300,0,540,100,49,9") == "d"
assert (state.d["x"], state.d["y"], state.d["width"], state.d["height"]) == (700, 540, 100, 300)
assert state.d["flags"] == 0x31 and state.d["seq"] == 9 and state.d["link"] and state.d["target"]
assert state.feed("LINK,1400,9,1390,1390,120,2,1") == "link"
assert state.link == {"ms": 1400, "valid": 120, "invalid": 2, "duplicate": 1}
assert state.feed("S,1240,3,77,1,RED,1,812,10,20,30,40,50,0") == "s"
assert state.sensors[3]["color"] == "RED" and state.sensors[3]["error"] == 0
assert state.feed("S,1240,9,77,1,RED,1,812,10,20,30,40,50,0") == "s" and 9 not in state.sensors
assert state.feed("I,1300,READY,68,1,120,0,4096,-12,30,1,2,3") == "i"
assert imu_freshness(state.imu) == "신선"  # older firmware without the diagnostic fields
assert imu_freshness(parse_line("I,1400,READY,72,0,120,0,1,2,3,4,5,6,2,40")[1]) == "멈춤(샘플 끊김) · 칩 리셋됨(잠자기)"
assert imu_freshness(parse_line("I,1400,READY,72,0,120,0,1,2,3,4,5,6,3,01")[1]) == "멈춤(값 고정)"
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
kind, pin = parse_line("PIN,1200,0,1,1,0,0,51,0,0")
assert kind == "pin" and pin["sw2"] and not pin["sw1"] and pin["en"] == (True, False) and pin["pwm"] == (0, 51, 0, 0)
assert pin["sda"] is None  # older PIN row without SDA/SCL
assert parse_line("PIN,1200,0,0,1,1,0,0,0,0,1,0")[1]["scl"] is False
kind, adc = parse_line("ADC,1300,10,20,3000,40,1,2,3,4,5,6,1380")
assert kind == "adc" and adc["is_l2"] == 3000 and adc["ir6"] == 6 and adc["bat"] == 1380

assert drive_command("fwd", 100) == "DRIVE 100 100"
assert drive_command("back", 100) == "DRIVE -100 -100"
assert drive_command("left", 50) == "DRIVE -50 50"
assert drive_command("right", 999) == "DRIVE 400 -400"  # clamped to the bench limit
assert requires_drive_permission("SW2") and requires_drive_permission(" sw2 ")
assert requires_drive_permission("DRIVE 100 100")
assert not requires_drive_permission("SW1") and not requires_drive_permission("STOP")

class DemoLog:
    def write(self, *args):
        pass

demo = DemoLink(DemoLog())
try:
    demo.send("MODE MOTOR")
    assert not demo.allowed
    demo.send("SW2")
    assert demo.allowed
    demo.send("STOP")
    assert not demo.allowed
    demo.send("SW1")
    demo.send("SW2")
    assert demo.estop and not demo.allowed and demo.out == (0, 0)
finally:
    demo.close()

# Calibration lines (formats from color_calibration.cpp and bench_v2_app.cpp).
cal = BenchState()
assert cal.feed("[CAL][STATUS] active=0 stored=1 state=0 step=1/12 sensors=0xF color=BLACK pose=LEVEL samples=0 margin=15/20%") == "cal_status"
assert cal.cal_stored is True
assert cal.feed("[CAL] Calibration started. Motors are braked until it ends.") == "cal_started"
assert cal.cal["state"] == "WaitingForReady" and cal.cal["step"] == 0 and cal.cal["sensors"] == [1, 2, 3, 4]
assert cal.feed("CAL,1234,1,6,3,RED,PRESSED,0,1") == "cal"
assert cal.cal == {"ms": 1234, "state": "WaitingForReady", "step": 6, "sensors": [1, 2], "color": "RED",
                   "pose": "PRESSED", "samples": 0, "stored": True}
assert cal.feed("[CAL][PREVIEW] S1 F2=1 F3=2 F5=3 F6=4 F7=5 C=512") == "cal_preview" and cal.cal_previews[1]["clear"] == 512
assert cal.feed("[CAL][PREVIEW] S2 NOT_READY") == "cal_preview" and not cal.cal_previews[2]["ready"]
assert cal.feed("[CAL] Starting in 1...") == "cal_count" and cal.cal_count == 1
assert cal.feed("[CAL] Measuring for 1000 ms. Do not move.") == "cal_measuring" and cal.cal_measuring
assert cal.feed("[CAL][RESULT] S1 samples=28 brightness=612.3 min=600 max=625 noise=2.10") == "cal_result"
assert cal.feed("[CAL][RESULT] S2 samples=30 brightness=500.0 min=490 max=510 noise=1.00") == "cal_result"
assert cal.cal_results[1] == {"sensor": 1, "samples": 28, "brightness": 612.3, "min": 600, "max": 625, "noise": 2.1}
assert sorted(cal.cal_results) == [1, 2]
assert cal_quality(cal.cal_results[1]) == [("ok", "좋아 보입니다")]
assert cal.feed("CAL,1500,1,7,12,RED,PRESSED,0,1") == "cal" and cal.cal_results == {}  # next step clears it
assert cal.cal["sensors"] == [3, 4]
assert cal.feed("[CAL] Calibration saved to NVS.") == "cal_saved" and cal.cal_finished == "saved"
assert cal.feed("[CAL] Stored calibration erased.") == "cal_erased" and cal.cal_stored is False
assert cal.feed("[CAL] Rejected: sensor ready mask is 0x07.") == "cal_rejected"
levels = [lvl for lvl, _ in cal_quality({"samples": 4, "brightness": 20.0, "min": 1, "max": 1700, "noise": 9.0})]
assert levels == ["bad", "warn", "warn", "warn"]
assert cal_quality({"samples": 28, "brightness": 1500.0, "min": 1490, "max": 1610, "noise": 2.0})[0][0] == "ok"
# Same order as color_calibration.cpp STEPS: no yellow, 12 steps.
assert cal_step_info(0) == {"sensors": [1, 2, 3, 4], "color": "BLACK", "pose": "LEVEL"}
assert cal_step_info(1) == {"sensors": [1, 2, 3, 4], "color": "BLACK", "pose": "LIFTED"}
assert cal_step_info(2) == {"sensors": [1, 2], "color": "BLACK", "pose": "PRESSED"}
assert cal_step_info(3) == {"sensors": [3, 4], "color": "BLACK", "pose": "PRESSED"}
assert cal_step_info(4)["color"] == "RED" and cal_step_info(11) == {"sensors": [3, 4], "color": "BLUE", "pose": "PRESSED"}
assert all(cal_step_info(i)["color"] != "YELLOW" for i in range(12))
print("bench_gui parse checks passed")

# I2C SCAN result, fed line by line as the firmware prints it
scan_state = BenchState()
for scan_line in ("OK I2C SCAN", "I2C MAIN 68 70", "I2C MUX 70", "I2C CH 0 39", "I2C CH 1 39", "I2C CH 2",
                  "I2C CH 3 39", "I2C CH 4", "I2C CH 5", "I2C CH 6", "I2C CH 7", "I2C DONE 160"):
    scan_state.feed(scan_line)
assert i2c_summary(scan_state.i2c).startswith("먹스 0x70 있음 · ch0 39 · ch1 39 · ch2 — · ch3 39 · 본선 68(IMU), 70(먹스)")
scan_state = BenchState()
for scan_line in ("OK I2C SCAN", "I2C MAIN 68", "I2C MUX NONE", "I2C DONE 20"):
    scan_state.feed(scan_line)
assert i2c_summary(scan_state.i2c) == "먹스 없음 · 본선 68(IMU)"
scan_state.feed("OK I2C SCAN")
assert i2c_summary(scan_state.i2c) == "스캔 중…"
print("i2c scan parse checks passed")

# 11/04 bug: "Too few samples" stayed on screen after the retry and the next step.
retry = BenchState()
retry.feed("[CAL] Calibration started. Motors are braked until it ends.")
retry.feed("[CAL] Too few samples: 3. Type READY to retry.")
assert "Too few" in retry.cal_message
retry.feed("[CAL] Starting in 1...")
assert retry.cal_message is None
retry.feed("[CAL] Too few samples: 3. Type READY to retry.")
retry.feed("[CAL][RESULT] S1 samples=28 brightness=100.0 min=90 max=110 noise=1.00")
assert retry.cal_message is None
print("cal retry message checks passed")

batcheck = BenchState()
assert batcheck.feed("BATCHECK,0") == "batcheck" and batcheck.bat_check is False
assert batcheck.feed("BATCHECK,1") == "batcheck" and batcheck.bat_check is True
assert batcheck.feed("BAT battery_check_off 0mV 0S") == "bat"
print("battery check switch parse checks passed")
