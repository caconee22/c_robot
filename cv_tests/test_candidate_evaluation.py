"""Regression checks for background-independent candidate recall."""
import unittest
import cv2
import numpy as np
from evaluate_candidates import candidate_hit
from comparison_engine import METHODS, detect
from analyze_comparison import matches

class CandidateTests(unittest.TestCase):
    def test_large_background_does_not_hide_valid_tower_candidate(self):
        frame=np.zeros((200,300,3),np.uint8)
        cv2.rectangle(frame,(10,10),(120,170),(45,150,40),-1)
        cv2.rectangle(frame,(230,80),(250,120),(45,150,40),-1)
        truth=[230,80,21,41]
        for name in METHODS:
            with self.subTest(method=name):
                _,mask,target=detect(frame,name)
                self.assertTrue(candidate_hit(mask,truth))
                self.assertFalse(matches(target,truth))

    def test_background_only_is_not_tower_recall(self):
        mask=np.zeros((200,300),np.uint8)
        cv2.rectangle(mask,(10,10),(120,170),255,-1)
        self.assertFalse(candidate_hit(mask,[230,80,21,41]))

    def test_pixel_noise_is_not_a_candidate(self):
        mask=np.zeros((100,100),np.uint8)
        mask[50,50]=255
        self.assertFalse(candidate_hit(mask,[48,48,5,5]))

if __name__=="__main__":
    unittest.main()
