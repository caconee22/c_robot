"""Offline checks for the production CV algorithms, not a camera benchmark."""
import unittest

import cv2
import numpy as np

import raspberry_pi_green_tracker_single as single
from scripts.color_detection_core import DEFAULT_PARAMS, detect_frame
from scripts.color_detection_lightweight import (
    LightweightOptions, _merge_overlapping_rois, _offset_candidate,
    detect_frame_lightweight,
)


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.params = dict(DEFAULT_PARAMS)
        self.frame = np.zeros((1080, 1920, 3), dtype=np.uint8)

    def detectors(self, frame):
        return (detect_frame(frame, self.params),
                detect_frame_lightweight(frame, self.params))

    def test_empty_and_non_green(self):
        for color in ((0, 0, 0), (0, 0, 255), (255, 0, 0)):
            self.frame[:] = color
            for result in self.detectors(self.frame):
                self.assertEqual(result.candidates, [])
            self.assertEqual(single.detect(self.frame)[0], [])

    def test_global_coordinates_and_masks(self):
        cv2.rectangle(self.frame, (1300, 600), (1399, 799), (40, 180, 100), -1)
        for result in self.detectors(self.frame):
            self.assertEqual(len(result.candidates), 1)
            self.assertAlmostEqual(result.best['centroid'][0], 1349.5, delta=2)
            self.assertAlmostEqual(result.best['bbox_center'][0], 1350, delta=2)
            self.assertEqual(result.masks['selected'].shape, (1080, 1920))
        candidates, _ = single.detect(self.frame)
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0]['centroid'][0], 1349.5, delta=2)

    def test_two_targets_and_no_full_output_allocation(self):
        cv2.rectangle(self.frame, (200, 200), (299, 399), (40, 180, 100), -1)
        cv2.rectangle(self.frame, (1300, 600), (1399, 799), (40, 180, 100), -1)
        result = detect_frame_lightweight(self.frame, self.params,
                                         build_output_masks=False, debug_masks=True)
        self.assertEqual(len(result.candidates), 2)
        self.assertEqual(result.masks['selected'].shape, (540, 960))
        candidates, _ = single.detect(self.frame)
        self.assertEqual(len(candidates), 2)

    def test_roi_offset_does_not_modify_input(self):
        candidate = {'bbox': (2, 3, 10, 20), 'centroid': (7., 13.),
                     'bbox_center': (7., 13.),
                     'contour': np.array([[[2, 3]]], dtype=np.int32)}
        mapped = _offset_candidate(candidate, 100, 200)
        self.assertEqual(mapped['bbox'], (102, 203, 10, 20))
        self.assertEqual(mapped['bbox_center'], (107., 213.))
        self.assertEqual(mapped['centroid'], (107., 213.))
        self.assertEqual(mapped['contour'].tolist(), [[[102, 203]]])
        self.assertEqual(candidate['contour'].tolist(), [[[2, 3]]])

    def test_overlapping_roi_chain_and_frame_edges(self):
        self.assertEqual(_merge_overlapping_rois(
            [(0, 0, 10, 10), (18, 0, 10, 10), (9, 0, 10, 10)]), [(0, 0, 28, 10)])
        for left, top in ((0, 0), (1820, 880)):
            self.frame[:] = 0
            cv2.rectangle(self.frame, (left, top), (left+99, top+199), (0, 255, 0), -1)
            result = detect_frame_lightweight(self.frame, self.params, LightweightOptions())
            self.assertTrue(result.candidates)
            x, y, w, h = result.best['bbox']
            self.assertTrue(0 <= x < x+w <= 1920)
            self.assertTrue(0 <= y < y+h <= 1080)


if __name__ == '__main__':
    unittest.main()
