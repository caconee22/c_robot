"""TowerDetector on synthetic frames: picks the tall tower over a larger mat."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import tempfile  # noqa: E402

import cv2  # noqa: E402

import tower_detector_hsv as T  # noqa: E402
from tower_detector_hsv import TRACK_FRAMES  # noqa: E402
from tower_settings_app import pick_range  # noqa: E402

NO_FILE = Path(tempfile.gettempdir()) / "c_robot_no_such_range.json"


def TowerDetector():  # fallback range, whatever is saved on this machine
    return T.TowerDetector(NO_FILE)

GREEN = (46, 88, 49)  # BGR of the real tower on the Pi camera 11/04 (H~58 S~122)


def frame(*boxes):
    image = np.full((360, 640, 3), 30, np.uint8)
    for x, y, w, h in boxes:
        image[y:y + h, x:x + w] = GREEN
    return image


class TowerDetectorTests(unittest.TestCase):
    def test_tall_tower_beats_larger_wide_mat(self):
        r = TowerDetector().detect(frame((50, 250, 300, 60), (450, 100, 40, 120)))
        self.assertEqual(r.candidates[0]["bbox"], (450, 100, 40, 120))
        self.assertEqual(len(r.candidates), 1)   # the mat is not a candidate

    def test_wide_box_at_the_edge_is_kept(self):
        r = TowerDetector().detect(frame((0, 100, 200, 100)))  # close tower clipped by the frame
        self.assertEqual(r.candidates[0]["bbox"], (0, 100, 200, 100))

    def test_tracking_keeps_the_same_tower(self):
        d = TowerDetector()
        d.detect(frame((300, 100, 30, 80)))
        r = d.detect(frame((310, 100, 30, 80), (500, 50, 40, 100)))  # bigger tall object far away
        self.assertEqual(r.candidates[0]["bbox"], (310, 100, 30, 80))
        for _ in range(TRACK_FRAMES + 1):
            self.assertEqual(d.detect(frame()).candidates, [])
        r = d.detect(frame((310, 100, 30, 80), (500, 50, 40, 100)))  # memory expired: largest
        self.assertEqual(r.candidates[0]["bbox"], (500, 50, 40, 100))

    def test_far_tower_found_but_specks_ignored(self):
        r = TowerDetector().detect(frame((320, 150, 6, 10)))  # 60 px > 640x360 x 1e-4
        self.assertTrue(r.candidates)
        self.assertEqual(TowerDetector().detect(frame((320, 150, 4, 4))).candidates, [])



class RangeUnionTests(unittest.TestCase):
    def test_lit_and_shadow_ranges_make_one_tower(self):
        image = np.full((360, 640, 3), 30, np.uint8)
        image[100:150, 300:340] = (46, 88, 49)      # lit half  (H~58)
        image[150:200, 300:340] = (20, 60, 15)      # shadow half (H~65, darker)
        d = T.TowerDetector(NO_FILE)
        d.ranges = [{"name": "lit", "low": (52, 80, 70), "high": (62, 255, 255), "on": True},
                    {"name": "shadow", "low": (60, 80, 40), "high": (72, 255, 69), "on": False}]
        self.assertEqual(d.detect(image).candidates[0]["bbox"], (300, 100, 40, 50))   # half only
        d.ranges[1]["on"] = True
        self.assertEqual(d.detect(image).candidates[0]["bbox"], (300, 100, 40, 100))  # whole tower
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "tower_hsv.json"
            T.save_settings(d, path)
            self.assertEqual([r["name"] for r in T.load_ranges(path)], ["lit", "shadow"])


class CalibratorTests(unittest.TestCase):
    def test_drag_on_the_room_tower_covers_its_color(self):
        tower = np.full((20, 20, 3), (46, 88, 49), np.uint8)  # BGR seen on the Pi 11/04
        tower[::3] = (40, 80, 44)                               # a little shading
        hsv = cv2.cvtColor(tower, cv2.COLOR_BGR2HSV)
        low, high = pick_range(hsv)
        d = T.TowerDetector(NO_FILE)
        d.low, d.high = low, high
        image = np.full((360, 640, 3), 30, np.uint8)
        image[100:200, 300:340] = (46, 88, 49)
        self.assertEqual(d.detect(image).candidates[0]["bbox"], (300, 100, 40, 100))
        old = T.TowerDetector(NO_FILE)
        old.low, old.high = (45, 138, 61), (56, 255, 255)   # pre-11/04 range
        self.assertEqual(old.detect(image).candidates, [])

    def test_save_load_and_broken_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "tower_hsv.json"
            T.save_range((50, 90, 40), (66, 255, 255), path)
            self.assertEqual(T.load_range(path), ((50, 90, 40), (66, 255, 255)))
            self.assertEqual(T.TowerDetector(path).low, (50, 90, 40))
            path.write_text("{broken", encoding="utf-8")
            self.assertEqual(T.load_range(path), (T.HSV_LOW, T.HSV_HIGH))
            with self.assertRaises(ValueError):
                T.save_range((70, 0, 0), (60, 255, 255), path)  # min above max
            self.assertEqual(T.load_sizes(path), (T.MIN_AREA_FRACTION, T.MAX_AREA_FRACTION))  # old file
            T.save_range((50, 90, 40), (66, 255, 255), path, min_area=0.002, max_area=0.3)
            self.assertEqual(T.load_sizes(path), (0.002, 0.3))

    def test_size_limits(self):
        d = T.TowerDetector(NO_FILE)
        image = frame((300, 100, 40, 100))            # 4000 px = 1.7 % of 640x360
        self.assertTrue(d.detect(image).candidates)
        d.min_area = 0.02                              # smallest allowed 2 %
        self.assertEqual(d.detect(image).candidates, [])
        d.min_area, d.max_area = 1e-4, 0.01            # largest allowed 1 %
        self.assertEqual(d.detect(image).candidates, [])

    def test_zero_min_area_skips_a_one_pixel_line(self):
        # 11/09 Pi: min_area 0.0 saved from the app, a 1 px line (area 0) crashed detect
        d = T.TowerDetector(NO_FILE)
        d.min_area = 0.0
        image = frame()
        image[100:200, 300] = GREEN
        small = cv2.resize(frame((290, 100, 20, 100)), (320, 180), interpolation=cv2.INTER_AREA)
        self.assertEqual(d.detect(image, small).candidates, [])


if __name__ == "__main__":
    unittest.main()
