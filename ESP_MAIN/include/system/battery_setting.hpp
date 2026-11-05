#pragma once

// Contract
// Role: on/off switch for the power-on battery check, kept in NVS (namespace
//   "battery", key "check") so the match firmware obeys what was set on the
//   bench. 2026-11-05 학현: bench tests need the pack protected; in a match,
//   keeping the robot running matters more than the pack.
// Called from: BatteryMonitor::begin (read once per boot), bench console
//   "BAT CHECK ON|OFF" (write). ON applies from the next power-on; OFF also
//   applies at once on the bench (BatteryMonitor::stopCheck).
// Failure: NVS unreadable -> the check stays on (the earlier behavior).
// Safety code: no.
namespace BatterySetting {
bool checkEnabled();
bool setCheckEnabled(bool enabled);  // false when the NVS write failed
}
