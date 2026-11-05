"""Pi -> ESP32 packet V1 encoder. Byte layout and rules from ESP_MAIN README section 10."""
import unittest

from scripts import vision_packet as vp


class VisionPacketTests(unittest.TestCase):
    def test_readme_example(self):
        packet = vp.encode(1, 0x31, 960, 540, 100, 200)
        self.assertEqual(packet.hex(" ").upper(), "AA 55 01 0A 01 31 C0 03 1C 02 64 00 C8 00")

    def test_no_target_is_zeros_with_camera_flags(self):
        packet = vp.from_candidates(7, [], (1920, 1080), 0)
        self.assertEqual(packet, bytes((0xAA, 0x55, 1, 10, 7, 0x30)) + bytes(8))

    def test_rejects_what_the_esp32_rejects(self):
        for args in ((256, 0x30), (1, 0x40), (1, 0x01, 1, 1, 1, 1),  # reserved bit, target without camera ok
                     (1, 0x31, 1920, 0, 1, 1), (1, 0x31, 0, 0, 0, 1), (1, 0x32),
                     (1, 0x30, 1, 0, 0, 0)):
            with self.assertRaises(ValueError, msg=args):
                vp.encode(*args)

    def test_target_scaling_and_flags(self):
        one = [{"bbox": (100, 50, 200, 300), "centroid": (200.0, 200.0)}]
        packet = vp.from_candidates(9, one, (960, 540), 0)  # half-size frame -> doubled
        self.assertEqual(packet[5], vp.CAMERA_OK | vp.PIPELINE_OK | vp.TARGET_VALID)
        self.assertEqual(packet[6:], bytes.fromhex("9001 9001 9001 5802".replace(" ", "")))
        stable = vp.from_candidates(9, one * 2, (960, 540), vp.STABLE_FRAMES)
        self.assertEqual(stable[5] & (vp.TRACK_STABLE | vp.MULTIPLE_TARGETS), vp.TRACK_STABLE | vp.MULTIPLE_TARGETS)

    def test_edge_box_is_clipped_and_clamped(self):
        edge = [{"bbox": (1800, 1000, 120, 80), "centroid": (1919.6, 1079.7)}]
        packet = vp.from_candidates(0, edge, (1920, 1080), 0)
        self.assertTrue(packet[5] & vp.BOX_CLIPPED)
        self.assertEqual(int.from_bytes(packet[6:8], "little"), 1919)
        self.assertEqual(int.from_bytes(packet[8:10], "little"), 1079)


if __name__ == "__main__":
    unittest.main()
