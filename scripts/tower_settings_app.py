"""Tower settings app (desktop icon "C_ROBOT 타워 설정"), opened by
pi_vision_sender.py --preview. Detection and packet sending keep running in
their own thread; this window only shows frames and changes settings.

Left: live view. Green box = chosen tower, grey boxes = rejected blobs with the
reason (small / big / flat), white line = center, dashed lines = ALIGN stop
window, pink = pixels inside the color range.
  drag          pick the color from the box (into the selected range, or a new one)
  Shift+click   widen the range to include that spot
  Ctrl+click    narrow the range to drop that spot
Rejected labels are small / big / flat (작음 / 큼 / 납작).
Right tabs: 색 / 크기·모양 / 카메라 / 프리셋·도구. "저장" writes
config/tower_hsv.json and config/camera.json; the boot sender uses both.
"""
from __future__ import annotations

import json
import os
import queue
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import cv2
import numpy as np

import camera_settings
from tower_detector_hsv import (HSV_HIGH, HSV_LOW, MAX_AREA_FRACTION, MIN_AREA_FRACTION, RANGE_FILE,
                                TALL_RATIO, save_settings)

VIEW = (640, 360)
REFRESH_MS = 66                      # ~15 view updates per second; detection stays at 50 fps
ALIGN_WINDOW = 200 / 1920            # ESP_MAIN/include/bench/bench_config.hpp ALIGN_DEADBAND
H_MARGIN, SV_MARGIN = 4, 30          # widen a dragged color a little for light changes
PRESET_FILE = RANGE_FILE.parent / "tower_presets.json"
CAPTURE_DIR = Path.home() / "c_robot_captures"
RECORD_S = 10
# BGR tint per color range in the overlay, so you see which range takes what.
RANGE_TINTS = ((255, 0, 255), (255, 255, 0), (0, 200, 255), (0, 128, 255), (255, 128, 128), (128, 255, 128))


def pick_range(hsv_pixels: np.ndarray) -> tuple[tuple, tuple]:
    """Range from the dragged pixels: 5th-95th percentile, widened; S/V max stay
    open so highlights on the tower still count."""
    pixels = hsv_pixels.reshape(-1, 3).astype(np.int32)
    lo = np.percentile(pixels, 5, axis=0)
    hi = np.percentile(pixels, 95, axis=0)
    low = (max(0, int(lo[0]) - H_MARGIN), max(0, int(lo[1]) - SV_MARGIN), max(0, int(lo[2]) - SV_MARGIN))
    high = (min(179, int(hi[0]) + H_MARGIN), 255, 255)
    return low, high


def widen(low, high, hsv) -> tuple[tuple, tuple]:
    """Smallest range change that includes this HSV (plus a margin)."""
    margin = (H_MARGIN, SV_MARGIN, SV_MARGIN)
    tops = (179, 255, 255)
    new_low = tuple(min(l, max(0, v - m)) for l, v, m in zip(low, hsv, margin))
    new_high = tuple(max(h, min(t, v + m)) for h, v, m, t in zip(high, hsv, margin, tops))
    return new_low, new_high


def narrow(low, high, hsv) -> tuple[tuple, tuple]:
    """Drop this HSV by moving the one bound that is closest to it (relative to
    the range width), so the rest of the range changes as little as possible."""
    low, high = list(low), list(high)
    if not all(l <= v <= h for l, v, h in zip(low, hsv, high)):
        return tuple(low), tuple(high)
    best = None
    for c in range(3):
        width = max(1, high[c] - low[c])
        for side, gap in (("low", hsv[c] - low[c]), ("high", high[c] - hsv[c])):
            if best is None or gap / width < best[0]:
                best = (gap / width, c, side)
    _, c, side = best
    if side == "low":
        low[c] = min(high[c], hsv[c] + 1)
    else:
        high[c] = max(low[c], hsv[c] - 1)
    return tuple(low), tuple(high)


def pi_power() -> str:
    """EXT5V, throttle flags, temperature; empty off the Pi."""
    def run(*args):
        return subprocess.run(args, capture_output=True, text=True, timeout=2).stdout.strip()
    try:
        volt = run("vcgencmd", "pmic_read_adc", "EXT5V_V").split("=")[-1].rstrip("V")
        thr = run("vcgencmd", "get_throttled").split("=")[-1]
        temp = run("vcgencmd", "measure_temp").split("=")[-1]
        warn = "" if thr == "0x0" else "  ⚠ 저전압/속도저하 기록"
        return f"5V {float(volt):.2f}V  {temp}  throttled {thr}{warn}"
    except Exception:
        return ""


class Recorder(threading.Thread):
    """Writes view-size frames to an AVI without slowing the detection thread."""

    def __init__(self, path: Path, fps: float) -> None:
        super().__init__(daemon=True)
        self.frames: queue.Queue = queue.Queue(maxsize=60)
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, VIEW)
        self.until = time.monotonic() + RECORD_S

    def offer(self, image) -> None:
        try:
            self.frames.put_nowait(image)
        except queue.Full:
            pass   # drop a frame rather than stall detection

    def run(self) -> None:
        while time.monotonic() < self.until or not self.frames.empty():
            try:
                self.writer.write(self.frames.get(timeout=0.2))
            except queue.Empty:
                pass
        self.writer.release()

    @property
    def active(self) -> bool:
        return time.monotonic() < self.until


class App:
    def __init__(self, root, stop, tower, source, sender, stats) -> None:
        self.root, self.stop, self.tower, self.source = root, stop, tower, source
        self.sender, self.stats = sender, stats
        self.pending = None            # (view image, scale, candidates, rejected) from the detect thread
        self.full = None               # last full frame, for snapshots
        self.frame_no = 0
        self.raw = None                # view without drawings, for color picks
        self.drag = None
        self.recorder: Recorder | None = None
        self.power = ""
        self.cam = camera_settings.load()
        self.cam_sent = None
        self.mode = tk.StringVar(value="겹쳐 보기")
        self.message = tk.StringVar(value="타워 위를 드래그하면 그 색을 기준으로 잡습니다")
        root.title("C_ROBOT 타워 설정")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self._build()
        self._load_into_vars()
        threading.Thread(target=self._power_loop, daemon=True).start()
        root.after(REFRESH_MS, self.refresh)

    # ----- layout -----
    def _build(self) -> None:
        left = ttk.Frame(self.root, padding=6)
        left.grid(row=0, column=0, sticky="n")
        self.canvas = tk.Canvas(left, width=VIEW[0], height=VIEW[1], bg="black", highlightthickness=0)
        self.canvas.pack()
        self.image_id = self.canvas.create_image(0, 0, anchor="nw")
        self.canvas.bind("<ButtonPress-1>", self._drag_start)
        self.canvas.bind("<B1-Motion>", self._drag_move)
        self.canvas.bind("<ButtonRelease-1>", self._drag_end)
        self.canvas.bind("<Shift-Button-1>", lambda e: self._click(e, widen))
        self.canvas.bind("<Control-Button-1>", lambda e: self._click(e, narrow))
        bar = ttk.Frame(left)
        bar.pack(fill="x", pady=4)
        for name in ("겹쳐 보기", "원본", "마스크"):
            ttk.Radiobutton(bar, text=name, value=name, variable=self.mode).pack(side="left")
        ttk.Label(left, textvariable=self.message, foreground="#0a5").pack(anchor="w")
        self.status = ttk.Label(left, text="", font=("TkFixedFont", 9))
        self.status.pack(anchor="w")
        self.power_label = ttk.Label(left, text="", font=("TkFixedFont", 9))
        self.power_label.pack(anchor="w")

        right = ttk.Frame(self.root, padding=6)
        right.grid(row=0, column=1, sticky="n")
        tabs = ttk.Notebook(right)
        tabs.pack(fill="both")
        self._color_tab(self._tab(tabs, "색"))
        self._shape_tab(self._tab(tabs, "크기·모양"))
        self._camera_tab(self._tab(tabs, "카메라"))
        self._tools_tab(self._tab(tabs, "프리셋·도구"))
        buttons = ttk.Frame(right)
        buttons.pack(fill="x", pady=6)
        ttk.Button(buttons, text="저장 (부팅 송신도 사용)", command=self.save).pack(side="left")
        ttk.Button(buttons, text="저장값으로 되돌리기", command=self.revert).pack(side="left", padx=4)
        ttk.Button(buttons, text="기본값", command=self.defaults).pack(side="left")

    def _tab(self, tabs, title):
        frame = ttk.Frame(tabs, padding=8)
        tabs.add(frame, text=title)
        return frame

    def _row(self, parent, label, var, low, high, step=1, row=None, fmt=None):
        r = row if row is not None else parent.grid_size()[1]
        ttk.Label(parent, text=label, width=14).grid(row=r, column=0, sticky="w")
        tk.Scale(parent, variable=var, from_=low, to=high, resolution=step, orient="horizontal",
                 length=230, showvalue=False).grid(row=r, column=1)
        extra = {"format": fmt} if fmt else {}
        ttk.Spinbox(parent, textvariable=var, from_=low, to=high, increment=step, width=7,
                    **extra).grid(row=r, column=2, padx=4)

    def _color_tab(self, f) -> None:
        top = ttk.Frame(f)
        top.grid(row=0, column=0, columnspan=3, sticky="w")
        ttk.Label(top, text="색 범위 (켜진 것을 모두 합쳐서 찾음)").grid(row=0, column=0, columnspan=4, sticky="w")
        self.range_list = tk.Listbox(top, height=4, width=26, exportselection=False)
        self.range_list.grid(row=1, column=0, rowspan=3, sticky="w")
        self.range_list.bind("<<ListboxSelect>>", self._range_selected)
        ttk.Button(top, text="추가", width=6, command=self.range_add).grid(row=1, column=1, padx=2)
        ttk.Button(top, text="삭제", width=6, command=self.range_delete).grid(row=1, column=2)
        ttk.Button(top, text="켜기/끄기", command=self.range_toggle).grid(row=2, column=1, columnspan=2, sticky="ew", padx=2)
        self.range_name = tk.StringVar()
        ttk.Entry(top, textvariable=self.range_name, width=12).grid(row=3, column=1)
        ttk.Button(top, text="이름", width=6, command=self.range_rename).grid(row=3, column=2)
        self.drag_new = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="드래그하면 새 범위로 추가", variable=self.drag_new).grid(
            row=4, column=0, columnspan=3, sticky="w", pady=2)
        self.hsv_vars = [tk.IntVar() for _ in range(6)]   # Hmin Hmax Smin Smax Vmin Vmax (selected range)
        names = ("H 최소", "H 최대", "S 최소", "S 최대", "V 최소", "V 최대")
        for i, (name, var) in enumerate(zip(names, self.hsv_vars)):
            self._row(f, name, var, 0, 179 if i < 2 else 255, row=1 + i)
        ttk.Label(f, text="드래그 = 그 색으로 · Shift+클릭 = 넓히기 · Ctrl+클릭 = 빼기 (선택한 범위)",
                  foreground="#666").grid(row=7, column=0, columnspan=3, sticky="w", pady=4)
        self.hist = tk.Canvas(f, width=360, height=96, bg="white", highlightthickness=1)
        self.hist.grid(row=8, column=0, columnspan=3, pady=4)
        ttk.Label(f, text="드래그한 영역의 H·S·V 분포 (빨간 선 = 선택한 범위)",
                  foreground="#666").grid(row=9, column=0, columnspan=3, sticky="w")

    # ----- color range list -----
    def _ranges_refresh(self) -> None:
        self.range_list.delete(0, "end")
        for i, r in enumerate(self.tower.ranges):
            self.range_list.insert("end", f"{'●' if r['on'] else '○'} {r['name']}  H{r['low'][0]}-{r['high'][0]}")
            b, g, rr = RANGE_TINTS[i % len(RANGE_TINTS)]
            self.range_list.itemconfig(i, foreground=f"#{rr:02x}{g:02x}{b:02x}" if r["on"] else "#999")
        self.range_list.selection_clear(0, "end")
        self.range_list.selection_set(self.tower.active)
        self.range_name.set(self.tower.ranges[self.tower.active]["name"])

    def _select_range(self, index: int) -> None:
        self._read_vars()                      # keep slider edits in the range being left
        self.tower.active = max(0, min(index, len(self.tower.ranges) - 1))
        self._set_range(self.tower.low, self.tower.high)
        self._ranges_refresh()

    def _range_selected(self, _event=None) -> None:
        picked = self.range_list.curselection()
        if picked and picked[0] != self.tower.active:
            self._select_range(picked[0])

    def range_add(self, low=None, high=None) -> None:
        self._read_vars()
        t = self.tower
        t.ranges.append({"name": f"범위 {len(t.ranges) + 1}", "low": tuple(low or t.low),
                         "high": tuple(high or t.high), "on": True})
        t.active = len(t.ranges) - 1
        self._set_range(t.low, t.high)
        self._ranges_refresh()

    def range_delete(self) -> None:
        t = self.tower
        if len(t.ranges) <= 1:
            self.message.set("범위는 하나 이상 있어야 합니다 (끄기는 켜기/끄기)")
            return
        del t.ranges[t.active]
        t.active = min(t.active, len(t.ranges) - 1)
        self._set_range(t.low, t.high)
        self._ranges_refresh()

    def range_toggle(self) -> None:
        r = self.tower.ranges[self.tower.active]
        r["on"] = not r["on"]
        self._ranges_refresh()

    def range_rename(self) -> None:
        name = self.range_name.get().strip()
        if name:
            self.tower.ranges[self.tower.active]["name"] = name
            self._ranges_refresh()

    def _shape_tab(self, f) -> None:
        self.min_size = tk.DoubleVar()
        self.max_size = tk.DoubleVar()
        self.tall = tk.DoubleVar()
        self.track = tk.BooleanVar()
        self._row(f, "최소 크기 (%)", self.min_size, 0, 3, 0.01, fmt="%.2f")
        self._row(f, "최대 크기 (%)", self.max_size, 1, 100, 1)
        self._row(f, "납작함 기준 (높이/폭)", self.tall, 0, 2, 0.05, fmt="%.2f")
        ttk.Checkbutton(f, text="추적: 직전 위치 근처의 타워 우선", variable=self.track).grid(
            row=3, column=0, columnspan=3, sticky="w", pady=6)
        ttk.Label(f, text="화면 면적 대비 %. 회색 박스에 '작음/큼/납작'이 뜨면 이 값들을 조정하세요.",
                  foreground="#666", wraplength=380).grid(row=4, column=0, columnspan=3, sticky="w")

    def _camera_tab(self, f) -> None:
        v = self.cam_vars = {k: (tk.BooleanVar() if isinstance(d, bool) else
                                 tk.StringVar() if isinstance(d, str) else tk.DoubleVar())
                             for k, d in camera_settings.DEFAULTS.items()}
        ttk.Label(f, text="해상도·프레임").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=v["mode"], values=list(camera_settings.MODES), width=8,
                     state="readonly").grid(row=0, column=1, sticky="w")
        ttk.Label(f, text="(저장 후 창을 다시 열면 적용)", foreground="#666").grid(row=0, column=2, sticky="w")
        ttk.Label(f, text="노출").grid(row=1, column=0, sticky="w")
        self.exposure_names = {"short": "셔터 고정 + 자동 게인 (기본)", "auto": "자동 (60Hz 플리커 방지, 번짐 큼)",
                               "manual": "셔터·게인 모두 수동"}
        self.exposure_label = tk.StringVar()
        box = ttk.Combobox(f, textvariable=self.exposure_label, values=list(self.exposure_names.values()),
                           width=28, state="readonly")
        box.grid(row=1, column=1, columnspan=2, sticky="w")
        box.bind("<<ComboboxSelected>>", lambda _e: v["exposure_mode"].set(
            next(k for k, n in self.exposure_names.items() if n == self.exposure_label.get())))
        self._row(f, "셔터 (µs)", v["exposure_us"], 100, 33000, 100, row=2)
        self._row(f, "게인 (수동일 때)", v["gain"], 1, 16, 0.1, row=3, fmt="%.1f")
        ttk.Button(f, text="지금 값으로 고정", command=self._lock_exposure).grid(row=1, column=3, sticky="w")
        ttk.Checkbutton(f, text="자동 화이트밸런스", variable=v["awb"]).grid(row=4, column=0, sticky="w")
        ttk.Button(f, text="지금 값으로 고정", command=self._lock_awb).grid(row=4, column=1, sticky="w")
        self._row(f, "빨강 게인", v["red_gain"], 0.5, 8, 0.05, row=5, fmt="%.2f")
        self._row(f, "파랑 게인", v["blue_gain"], 0.5, 8, 0.05, row=6, fmt="%.2f")
        self._row(f, "밝기", v["brightness"], -1, 1, 0.05, row=7, fmt="%.2f")
        self._row(f, "대비", v["contrast"], 0, 4, 0.05, row=8, fmt="%.2f")
        self._row(f, "채도", v["saturation"], 0, 4, 0.05, row=9, fmt="%.2f")
        self._row(f, "선명도", v["sharpness"], 0, 8, 0.1, row=10, fmt="%.1f")
        ttk.Checkbutton(f, text="연속 자동 초점", variable=v["af_continuous"]).grid(row=11, column=0, sticky="w")
        ttk.Button(f, text="지금 초점으로 고정", command=self._lock_focus).grid(row=11, column=1, sticky="w")
        self._row(f, "초점 (디옵터)", v["lens"], 0, 15, 0.1, row=12, fmt="%.1f")
        ttk.Label(f, text="경기장에서는 화이트밸런스를 고정하세요. 자동이면 조명에 따라 타워 색(H)이 바뀝니다. "
                          "셔터는 짧을수록 회전 중 번짐이 적습니다 (기본 1000µs = 1ms). 어두우면 게인이 올라갑니다.",
                  foreground="#a40", wraplength=380).grid(row=13, column=0, columnspan=3, sticky="w", pady=6)

    def _tools_tab(self, f) -> None:
        ttk.Label(f, text="프리셋 (색·크기·카메라 함께)").grid(row=0, column=0, columnspan=3, sticky="w")
        self.preset = tk.StringVar()
        self.preset_box = ttk.Combobox(f, textvariable=self.preset, width=18)
        self.preset_box.grid(row=1, column=0, sticky="w")
        ttk.Button(f, text="불러오기", command=self.preset_load).grid(row=1, column=1)
        ttk.Button(f, text="이 이름으로 저장", command=self.preset_save).grid(row=1, column=2)
        ttk.Button(f, text="삭제", command=self.preset_delete).grid(row=2, column=2, pady=2)
        self._presets_refresh()
        ttk.Separator(f).grid(row=3, column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Button(f, text="사진 찍기", command=self.snapshot).grid(row=4, column=0, sticky="w")
        ttk.Button(f, text=f"{RECORD_S}초 녹화", command=self.record).grid(row=4, column=1, sticky="w")
        ttk.Label(f, text=f"저장 위치: {CAPTURE_DIR}", foreground="#666").grid(
            row=5, column=0, columnspan=3, sticky="w", pady=4)

    # ----- settings <-> vars -----
    def _load_into_vars(self) -> None:
        t = self.tower
        for var, value in zip(self.hsv_vars, (t.low[0], t.high[0], t.low[1], t.high[1], t.low[2], t.high[2])):
            var.set(int(value))
        self.min_size.set(round(t.min_area * 100, 2))
        self.max_size.set(round(t.max_area * 100))
        self.tall.set(t.tall_ratio)
        self.track.set(t.track)
        for key, var in self.cam_vars.items():
            var.set(self.cam[key])
        self.exposure_label.set(self.exposure_names[self.cam["exposure_mode"]])
        self._ranges_refresh()

    def _read_vars(self) -> None:
        """Push the panel into the live detector and camera (safe on bad typing)."""
        try:
            v = [int(x.get()) for x in self.hsv_vars]
            self.tower.low = (v[0], v[2], v[4])
            self.tower.high = (max(v[0], v[1]), max(v[2], v[3]), max(v[4], v[5]))
            low = min(max(float(self.min_size.get()), 0.0), 3.0) / 100
            self.tower.min_area = low
            self.tower.max_area = max(low + 1e-4, min(max(float(self.max_size.get()), 1.0), 100.0) / 100)
            self.tower.tall_ratio = min(max(float(self.tall.get()), 0.0), 5.0)
            self.tower.track = bool(self.track.get())
            cam = camera_settings.clean({k: var.get() for k, var in self.cam_vars.items()})
        except (tk.TclError, ValueError):
            return   # half-typed number in a box; keep the last good values
        self.cam = cam
        if cam != self.cam_sent and hasattr(self.source, "camera"):
            camera_settings.apply(self.source.camera, cam)
        self.cam_sent = cam

    def _lock_exposure(self) -> None:
        md = getattr(self.source, "metadata", {})
        if "ExposureTime" in md:
            self.cam_vars["exposure_us"].set(int(md["ExposureTime"]))
            self.cam_vars["gain"].set(round(float(md.get("AnalogueGain", 1.0)), 1))
            self.cam_vars["exposure_mode"].set("manual")
            self.exposure_label.set(self.exposure_names["manual"])
            self.message.set("노출을 지금 값으로 고정했습니다")

    def _lock_awb(self) -> None:
        md = getattr(self.source, "metadata", {})
        if "ColourGains" in md:
            red, blue = md["ColourGains"]
            self.cam_vars["red_gain"].set(round(float(red), 2))
            self.cam_vars["blue_gain"].set(round(float(blue), 2))
            self.cam_vars["awb"].set(False)
            self.message.set("화이트밸런스를 지금 값으로 고정했습니다")

    def _lock_focus(self) -> None:
        md = getattr(self.source, "metadata", {})
        if "LensPosition" in md:
            self.cam_vars["lens"].set(round(float(md["LensPosition"]), 1))
            self.cam_vars["af_continuous"].set(False)
            self.message.set("초점을 지금 값으로 고정했습니다")

    def save(self) -> None:
        self._read_vars()
        save_settings(self.tower)
        camera_settings.save(self.cam)
        self.message.set("저장했습니다. 부팅 때 자동 송신도 이 설정을 씁니다")

    def revert(self) -> None:
        self.tower.load()
        self.cam = camera_settings.load()
        self._load_into_vars()
        self.message.set("저장된 설정으로 되돌렸습니다")

    def defaults(self) -> None:
        self.tower.ranges = [{"name": "범위 1", "low": HSV_LOW, "high": HSV_HIGH, "on": True}]
        self.tower.active = 0
        self.tower.min_area, self.tower.max_area = MIN_AREA_FRACTION, MAX_AREA_FRACTION
        self.tower.tall_ratio, self.tower.track = TALL_RATIO, True
        self.cam = dict(camera_settings.DEFAULTS)
        self._load_into_vars()
        self.message.set("기본값으로 바꿨습니다 (저장 전에는 파일이 그대로입니다)")

    # ----- presets -----
    def _presets(self) -> dict:
        try:
            return json.loads(PRESET_FILE.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def _presets_refresh(self) -> None:
        self.preset_box["values"] = sorted(self._presets())

    def _write_presets(self, presets: dict) -> None:
        PRESET_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = PRESET_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(presets, ensure_ascii=False, indent=1), encoding="utf-8")
        os.replace(tmp, PRESET_FILE)
        self._presets_refresh()

    def preset_save(self) -> None:
        name = self.preset.get().strip()
        if not name:
            self.message.set("프리셋 이름을 먼저 적으세요")
            return
        self._read_vars()
        t = self.tower
        presets = self._presets()
        presets[name] = {"tower": {"ranges": [{"name": r["name"], "low": list(r["low"]), "high": list(r["high"]),
                                               "on": r["on"]} for r in t.ranges],
                                   "min_area": t.min_area,
                                   "max_area": t.max_area, "tall_ratio": t.tall_ratio, "track": t.track},
                         "camera": self.cam}
        self._write_presets(presets)
        self.message.set(f"프리셋 '{name}' 저장")

    def preset_load(self) -> None:
        p = self._presets().get(self.preset.get().strip())
        if not p or "tower" not in p or "camera" not in p:
            self.message.set("그 이름의 프리셋이 없거나 깨졌습니다")
            return
        t, tw = self.tower, p["tower"]
        saved = tw.get("ranges") or [{"name": "범위 1", "low": tw["low"], "high": tw["high"], "on": True}]
        t.ranges = [{"name": r["name"], "low": tuple(r["low"]), "high": tuple(r["high"]), "on": r["on"]}
                    for r in saved]
        t.active = 0
        t.min_area, t.max_area = tw["min_area"], tw["max_area"]
        t.tall_ratio, t.track = tw["tall_ratio"], tw["track"]
        self.cam = camera_settings.clean(p["camera"])
        self._load_into_vars()
        self.message.set(f"프리셋 '{self.preset.get()}' 적용 (저장을 눌러야 부팅 송신에도 반영)")

    def preset_delete(self) -> None:
        presets = self._presets()
        if presets.pop(self.preset.get().strip(), None) is not None:
            self._write_presets(presets)
            self.message.set("프리셋 삭제")

    # ----- tools -----
    def snapshot(self) -> None:
        if self.raw is None:
            return
        CAPTURE_DIR.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(str(CAPTURE_DIR / f"{stamp}_view.jpg"), self.raw)
        if self.full is not None:
            cv2.imwrite(str(CAPTURE_DIR / f"{stamp}_full.jpg"), self.full)
        self.message.set(f"사진 저장: {stamp}")

    def record(self) -> None:
        if self.recorder and self.recorder.active:
            return
        CAPTURE_DIR.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.recorder = Recorder(CAPTURE_DIR / f"{stamp}.avi", max(10.0, self.stats["fps"] or 50.0))
        self.recorder.start()
        self.message.set(f"녹화 중 {RECORD_S}초: {stamp}.avi")

    # ----- detect thread hook (runs every frame there; keep it cheap) -----
    def on_frame(self, frame, search_frame, result) -> None:
        self.frame_no += 1
        rec = self.recorder if self.recorder and self.recorder.active else None
        if self.frame_no % 3 and rec is None:
            return
        view = search_frame if search_frame is not None else frame
        if (view.shape[1], view.shape[0]) != VIEW:
            view = cv2.resize(view, VIEW, interpolation=cv2.INTER_AREA)
        else:
            view = view.copy()
        if rec:
            rec.offer(view)
        if self.frame_no % 3 == 0:
            self.full = frame
            self.pending = (view, VIEW[0] / frame.shape[1], list(result.candidates),
                            list(getattr(self.tower, "rejected", [])))

    # ----- mouse -----
    def _hsv_at(self, x, y):
        if self.raw is None:
            return None
        x0, x1 = max(0, x - 2), min(VIEW[0], x + 3)
        y0, y1 = max(0, y - 2), min(VIEW[1], y + 3)
        patch = cv2.cvtColor(self.raw[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        return tuple(int(v) for v in np.median(patch.reshape(-1, 3), axis=0))

    def _set_range(self, low, high) -> None:
        for var, value in zip(self.hsv_vars, (low[0], high[0], low[1], high[1], low[2], high[2])):
            var.set(int(value))

    def _click(self, event, change) -> str:
        hsv = self._hsv_at(event.x, event.y)
        if hsv:
            self._read_vars()
            self._set_range(*change(self.tower.low, self.tower.high, hsv))
            self.message.set(f"{'넓힘' if change is widen else '좁힘'}: 클릭한 곳 HSV {hsv}")
        return "break"   # keep the plain click from starting a drag

    def _drag_start(self, event) -> None:
        self.drag = (event.x, event.y, event.x, event.y)

    def _drag_move(self, event) -> None:
        if self.drag:
            self.drag = (*self.drag[:2], event.x, event.y)

    def _drag_end(self, event) -> None:
        if not self.drag or self.raw is None:
            return
        x0, x1 = sorted((self.drag[0], event.x))
        y0, y1 = sorted((self.drag[1], event.y))
        self.drag = None
        if (x1 - x0) * (y1 - y0) < 9:
            return
        region = cv2.cvtColor(self.raw[max(0, y0):y1, max(0, x0):x1], cv2.COLOR_BGR2HSV)
        low, high = pick_range(region)
        if self.drag_new.get():
            self.range_add(low, high)
        else:
            self._set_range(low, high)
        self._draw_hist(region)
        self.message.set("드래그한 색으로 맞췄습니다. 마음에 들면 '저장'")

    def _draw_hist(self, region) -> None:
        c = self.hist
        c.delete("all")
        rows = ((0, 180, "#c80"), (1, 256, "#08c"), (2, 256, "#555"))
        for i, (ch, top, color) in enumerate(rows):
            counts = np.bincount(region[..., ch].ravel(), minlength=top)[:top]
            peak = max(1, counts.max())
            y = 30 * (i + 1)
            for x in range(top):
                if counts[x]:
                    px = 4 + x * 350 / top
                    c.create_line(px, y, px, y - 26 * counts[x] / peak, fill=color)
            lo, hi = (self.tower.low[ch], self.tower.high[ch])
            for b in (lo, hi):
                px = 4 + b * 350 / top
                c.create_line(px, y - 28, px, y, fill="red")
            c.create_text(352, y - 14, text="HSV"[ch], anchor="e")

    # ----- periodic view/status refresh -----
    def _power_loop(self) -> None:
        while not self.stop.is_set():
            self.power = pi_power()
            time.sleep(2)

    def refresh(self) -> None:
        if self.stop.is_set():
            self.root.destroy()
            return
        self._read_vars()
        pending, self.pending = self.pending, None
        if pending:
            self._render(*pending)
        s = self.stats
        age = f"{s['age_ms']:.0f}ms" if s["age_ms"] is not None else "-"
        self.status.config(text=f"감지 {s['fps']:4.1f}fps  {s['detect_ms']:4.1f}ms  영상지연 {age}  "
                                f"송신 {self.sender.sent} (누락 {self.sender.dropped})  타워 {s['targets']}")
        self.power_label.config(text=self.power)
        self.root.after(REFRESH_MS, self.refresh)

    def _render(self, view, k, candidates, rejected) -> None:
        self.raw = view
        t = self.tower
        hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
        mode = self.mode.get()
        if mode == "마스크":
            shown = cv2.cvtColor(t.range_mask(hsv), cv2.COLOR_GRAY2BGR)
        elif mode == "원본":
            shown = view.copy()
        else:
            tint = view.copy()
            for i, r in enumerate(list(t.ranges)):
                if r["on"]:
                    tint[cv2.inRange(hsv, r["low"], r["high"]) > 0] = RANGE_TINTS[i % len(RANGE_TINTS)]
            shown = cv2.addWeighted(view, 0.6, tint, 0.4, 0)
        for (x, y, w, h), why in rejected:
            x, y, w, h = (int(v * k) for v in (x, y, w, h))
            cv2.rectangle(shown, (x, y), (x + w, y + h), (150, 150, 150), 1)
            # English: cv2.putText cannot draw Hangul (small / big / flat = 작음 / 큼 / 납작)
            cv2.putText(shown, why, (x, max(10, y - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        for i, c in enumerate(candidates[:2]):
            x, y, w, h = (int(v * k) for v in c["bbox"])
            cv2.rectangle(shown, (x, y), (x + w, y + h), (0, 255, 0) if i == 0 else (0, 200, 255), 2)
        cx = VIEW[0] // 2
        cv2.line(shown, (cx, 0), (cx, VIEW[1]), (255, 255, 255), 1)
        band = int(ALIGN_WINDOW * VIEW[0])
        for x in (cx - band, cx + band):
            for y in range(0, VIEW[1], 12):
                cv2.line(shown, (x, y), (x, y + 6), (255, 255, 255), 1)
        if self.recorder and self.recorder.active:
            cv2.circle(shown, (VIEW[0] - 14, 14), 6, (0, 0, 255), -1)
        rgb = cv2.cvtColor(shown, cv2.COLOR_BGR2RGB)
        self.photo = tk.PhotoImage(data=b"P6 %d %d 255\n" % VIEW + rgb.tobytes(), format="PPM")
        self.canvas.itemconfig(self.image_id, image=self.photo)
        self.canvas.delete("drag")
        if self.drag:
            self.canvas.create_rectangle(*self.drag, outline="yellow", tags="drag")

    def close(self) -> None:
        self.stop.set()
        self.root.destroy()


def run_app(detect_loop, stop, tower, source, sender, stats) -> None:
    root = tk.Tk()
    app = App(root, stop, tower, source, sender, stats)
    worker = threading.Thread(target=detect_loop, args=(app.on_frame,), daemon=True)
    worker.start()
    root.mainloop()
    stop.set()
    worker.join(timeout=2)
