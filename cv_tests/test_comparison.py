"""Synthetic regression checks, not substitutes for real target annotations."""
import unittest

import cv2
import numpy as np

from comparison_engine import METHODS, detect, render
from analyze_comparison import iou, matches, longest_run


class DetectorTests(unittest.TestCase):
    def test_blank_and_gray(self):
        for value in (0, 35, 128, 255):
            frame = np.full((120, 160, 3), value, np.uint8)
            for method in METHODS:
                with self.subTest(value=value, method=method):
                    self.assertIsNone(detect(frame, method)[2])

    def test_green_largest_center_and_input_unchanged(self):
        frame = np.zeros((200, 300, 3), np.uint8)
        cv2.rectangle(frame, (80, 60), (110, 120), (45, 150, 40), -1)
        cv2.rectangle(frame, (200, 50), (208, 60), (45, 150, 40), -1)
        original = frame.copy()
        for method in METHODS:
            with self.subTest(method=method):
                _, mask, target = detect(frame, method)
                self.assertIsNotNone(target)
                self.assertAlmostEqual(target["center"][0], 95, delta=1)
                self.assertAlmostEqual(target["center"][1], 90, delta=1)
                self.assertEqual(mask.shape, frame.shape[:2])
                self.assertTrue(np.array_equal(frame, original))

    def test_no_stale_box_and_unused_filters(self):
        green = np.full((101, 151, 3), (45, 150, 40), np.uint8)
        blank = np.zeros_like(green)
        for method, (names, _) in METHODS.items():
            masks, fused, target = detect(green, method)
            self.assertEqual(set(masks), set(names))
            self.assertIsNotNone(target)
            self.assertIsNone(detect(blank, method)[2])
            output = render(green, masks, fused, target, method, 0, 1, 30)
            self.assertEqual(output.shape, (960, 810, 3))

    def test_negative_colors(self):
        for color in ((0, 0, 255), (255, 0, 0), (0, 255, 255)):
            for method in METHODS:
                with self.subTest(color=color, method=method):
                    self.assertIsNone(detect(np.full((80, 80, 3), color, np.uint8), method)[2])

    def test_normalized_equivalent_ratios(self):
        rng = np.random.default_rng(5)
        frame = rng.integers(0, 256, (100, 100, 3), dtype=np.uint8)
        b, g, r = [frame[:, :, i].astype(np.int32) for i in range(3)]
        total = b + g + r
        expected = ((g >= 35) & (100 * g >= 42 * total)
                    & (100 * (g - r) >= 8 * total) & (100 * (g - b) >= 8 * total))
        actual = detect(frame, "06_normalized")[0]["Normalized"] > 0
        self.assertTrue(np.array_equal(expected, actual))

    def test_evaluation_does_not_reward_background(self):
        truth = [100, 100, 20, 40]
        correct = {"box": truth, "center": [110, 120]}
        wrong = {"box": [0, 0, 300, 300], "center": [110, 120]}
        self.assertEqual(iou(truth, truth), 1)
        self.assertTrue(matches(correct, truth))
        self.assertFalse(matches(wrong, truth))
        self.assertFalse(matches(None, truth))
        self.assertFalse(matches(correct, None))
        self.assertEqual(longest_run([False, True, True, False, True]), 2)

    def test_invalid_method(self):
        with self.assertRaises(ValueError):
            detect(np.zeros((10, 10, 3), np.uint8), "does_not_exist")


if __name__ == "__main__":
    unittest.main()
