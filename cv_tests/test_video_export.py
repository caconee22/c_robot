"""Shared-mask export must preserve every original detector's output."""
import unittest
import cv2
import numpy as np
from comparison_engine import METHODS, detect, render
from export_videos_only import shared_detections

class ExportTests(unittest.TestCase):
    def test_shared_masks_equal_independent_detectors(self):
        rng=np.random.default_rng(34)
        frames=[rng.integers(0,256,(121,163,3),dtype=np.uint8),np.zeros((120,160,3),np.uint8)]
        cv2.rectangle(frames[1],(30,40),(60,90),(40,150,45),-1)
        for frame in frames:
            results=shared_detections(frame)
            for name in METHODS:
                with self.subTest(method=name):
                    expected=detect(frame,name)
                    masks,final,target=results[name]
                    self.assertTrue(np.array_equal(final,expected[1]))
                    self.assertEqual(target,expected[2])
                    for key in masks:
                        self.assertTrue(np.array_equal(masks[key],expected[0][key]))

    def test_render_without_benchmark(self):
        frame=np.zeros((120,160,3),np.uint8)
        masks,final,target=detect(frame,"05_hsv")
        preview=render(frame,masks,final,target,"05_hsv",0,0,60,show_timing=False)
        self.assertEqual(preview.shape,(960,810,3))

if __name__=="__main__": unittest.main()
