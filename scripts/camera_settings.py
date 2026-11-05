"""Raspberry Pi camera settings shared by the boot sender and the settings app.

Stored in config/camera.json. The boot sender (c-robot-vision) applies them at
start, so what the settings app shows is what the ESP32 gets during a match.

Defaults (11/05, for a robot that spins fast under 60 Hz mains lighting):
  - Shutter fixed at 1 ms, gain automatic. Motion blur: the Camera Module 3
    Wide gives ~12.5 px/deg at 720p, so a 360-720 deg/s spin smears 4-9 px at
    1 ms but 45-90 px at the ~10 ms auto exposure may pick. PhotonVision's
    advice is the same: exposure as low as tracking allows, gain up instead.
  - 60 fps (a divisor of the 120 Hz light flicker on 60 Hz mains): any rolling
    shutter banding stays in the same place every frame instead of crawling
    through the image. Full flicker removal needs a shutter of n x 8.33 ms,
    which blurs too much here; "auto" mode below does that with AeFlickerPeriod.
  - White balance automatic until locked at the venue: with AWB on, the
    tower's hue moves with the light.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

CAMERA_FILE = Path(__file__).resolve().parents[1] / "config" / "camera.json"

# mode -> (main size, search size, fps). fps divide 120 so 60 Hz flicker bands
# stand still. Changing mode needs a camera restart.
MODES = {
    "720p": ((1280, 720), (640, 360), 60.0),
    "1080p": ((1920, 1080), (960, 540), 40.0),    # 2304x1296 sensor mode tops out near 56 fps
    "fast": ((1536, 864), (768, 432), 120.0),     # binned 1536x864 sensor mode
}
EXPOSURE_MODES = ("short", "auto", "manual")       # short = fixed shutter + auto gain

DEFAULTS = {
    "mode": "720p",
    "exposure_mode": "short", "exposure_us": 1000, "gain": 4.0,
    "mains_hz": 60,                                           # Korea: 60 Hz -> 8333 us flicker period
    "awb": True, "red_gain": 2.0, "blue_gain": 1.8,          # manual when awb is off
    "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "sharpness": 1.0,
    "af_continuous": True, "lens": 1.0,                       # lens in dioptres, 0 = infinity
}

# key -> (min, max) for numbers; anything outside falls back to the default.
LIMITS = {
    "exposure_us": (100, 33000), "gain": (1.0, 16.0), "mains_hz": (50, 60),
    "red_gain": (0.5, 8.0), "blue_gain": (0.5, 8.0),
    "brightness": (-1.0, 1.0), "contrast": (0.0, 4.0), "saturation": (0.0, 4.0),
    "sharpness": (0.0, 8.0), "lens": (0.0, 15.0),
}
CHOICES = {"mode": tuple(MODES), "exposure_mode": EXPOSURE_MODES}


def clean(raw: dict) -> dict:
    """Defaults merged with valid saved values; bad values are dropped."""
    settings = dict(DEFAULTS)
    if "exposure_mode" not in raw and isinstance(raw.get("ae"), bool):   # file saved before 11/05
        raw = dict(raw, exposure_mode="auto" if raw["ae"] else "manual")
    for key, default in DEFAULTS.items():
        if key not in raw:
            continue
        value = raw[key]
        if key in CHOICES:
            if value in CHOICES[key]:
                settings[key] = value
        elif isinstance(default, bool):
            if isinstance(value, bool):
                settings[key] = value
        else:
            try:
                number = type(default)(float(value))
            except (TypeError, ValueError):
                continue
            low, high = LIMITS[key]
            if low <= number <= high:
                settings[key] = number
    return settings


def load(path: Path = CAMERA_FILE) -> dict:
    try:
        return clean(json.loads(path.read_text(encoding="utf-8")))
    except (OSError, ValueError, TypeError, AttributeError):
        return dict(DEFAULTS)


def save(settings: dict, path: Path = CAMERA_FILE) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(clean(settings), indent=1), encoding="utf-8")
    os.replace(tmp, path)  # a power cut leaves the old file or the new one


def controls(settings: dict) -> dict:
    """Picamera2 controls (mode is applied at configure time). Enum values from
    libcamera control_ids: *Mode Auto=0 Manual=1, AeFlickerMode Off=0 Manual=1,
    AfMode Manual=0 Continuous=2."""
    s = clean(settings)
    out = {"AwbEnable": s["awb"], "Brightness": s["brightness"], "Contrast": s["contrast"],
           "Saturation": s["saturation"], "Sharpness": s["sharpness"],
           "AfMode": 2 if s["af_continuous"] else 0}
    if s["exposure_mode"] == "auto":
        out.update(AeEnable=True, ExposureTimeMode=0, AnalogueGainMode=0,
                   AeFlickerMode=1, AeFlickerPeriod=round(1e6 / (2 * s["mains_hz"])))
    else:
        out.update(AeEnable=True, ExposureTimeMode=1, ExposureTime=int(s["exposure_us"]),
                   AeFlickerMode=0)
        if s["exposure_mode"] == "manual":
            out.update(AnalogueGainMode=1, AnalogueGain=float(s["gain"]))
        else:
            out.update(AnalogueGainMode=0)
    if not s["awb"]:
        out["ColourGains"] = (float(s["red_gain"]), float(s["blue_gain"]))
    if not s["af_continuous"]:
        out["LensPosition"] = float(s["lens"])
    return out


def apply(camera, settings: dict) -> dict:
    """Sends only the controls this camera has. Without ExposureTimeMode (older
    libcamera) a fixed shutter falls back to AeEnable off with the manual gain.
    Returns what was sent."""
    wanted = controls(settings)
    known = getattr(camera, "camera_controls", None) or {}
    if known and "ExposureTimeMode" not in known and wanted.get("ExposureTimeMode") == 1:
        wanted["AeEnable"] = False
        wanted.setdefault("AnalogueGain", float(clean(settings)["gain"]))
    if "ExposureTimeMode" in known:
        wanted.pop("AeEnable")   # AeEnable would reset both modes to auto on libcamera that has both
    if known:
        wanted = {k: v for k, v in wanted.items() if k in known}
    camera.set_controls(wanted)
    return wanted
