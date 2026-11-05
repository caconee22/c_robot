"""Wi-Fi profile backup/restore (scripts/pi_netfix.py) on temp folders."""
import importlib
import os
import tempfile
import unittest
from pathlib import Path

GOOD = b"network:\n  version: 2\n  wifis:\n    NM-1:\n      access-points:\n        home: {}\n"


class NetfixTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.net, self.bak, self.keys = root / "netplan", root / "backup", root / "keyfiles"
        self.net.mkdir()
        self.keys.mkdir()
        os.environ["NETFIX_NETPLAN_DIR"], os.environ["NETFIX_BACKUP_DIR"] = str(self.net), str(self.bak)
        os.environ["NETFIX_KEYFILE_DIR"] = str(self.keys)
        import scripts.pi_netfix as netfix
        self.netfix = importlib.reload(netfix)

    def tearDown(self):
        self.tmp.cleanup()

    def test_save_skips_empty_and_restore_refills_it(self):
        (self.net / "90-NM-a.yaml").write_bytes(GOOD)
        (self.net / "90-NM-b.yaml").write_bytes(b"")
        self.netfix.save()
        self.assertEqual((self.bak / "90-NM-a.yaml").read_bytes(), GOOD)
        self.assertFalse((self.bak / "90-NM-b.yaml").exists())  # never back up an empty file
        (self.net / "90-NM-a.yaml").write_bytes(b"")              # power cut empties it
        self.netfix.restore()
        self.assertEqual((self.net / "90-NM-a.yaml").read_bytes(), GOOD)

    def test_restore_brings_back_a_missing_file_and_keeps_good_ones(self):
        (self.net / "90-NM-a.yaml").write_bytes(GOOD)
        self.netfix.save()
        (self.net / "90-NM-a.yaml").unlink()
        newer = GOOD.replace(b"home", b"hotspot")
        (self.net / "90-NM-c.yaml").write_bytes(newer)
        self.netfix.restore()
        self.assertTrue((self.net / "90-NM-a.yaml").exists())
        self.assertEqual((self.net / "90-NM-c.yaml").read_bytes(), newer)

    def test_keyfile_profile_from_nmcli_is_kept_too(self):
        good = b"[connection]\nid=caconee22\ntype=wifi\n"
        (self.keys / "caconee22.nmconnection").write_bytes(good)
        self.netfix.save()
        (self.keys / "caconee22.nmconnection").write_bytes(b"")
        self.netfix.restore()
        self.assertEqual((self.keys / "caconee22.nmconnection").read_bytes(), good)

    def test_broken_yaml_is_neither_saved_nor_restored_over(self):
        (self.net / "90-NM-a.yaml").write_bytes(b"network: [unclosed")
        self.netfix.save()
        self.assertFalse(any(self.bak.glob("*.yaml")))


if __name__ == "__main__":
    unittest.main()
