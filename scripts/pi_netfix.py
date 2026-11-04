"""Raspberry Pi: keep the Wi-Fi settings alive across sudden power cuts.

On 11/04 two power cuts left /etc/netplan/90-NM-*.yaml at 0 bytes and the Pi
lost Wi-Fi. NetworkManager keeps profiles there (netplan backend) or, for
profiles added later with nmcli, in /etc/NetworkManager/system-connections.
Both places are backed up.

  pi_netfix.py restore   before NetworkManager: put back any profile file
                         that is missing or empty, then regenerate
  pi_netfix.py save      after the network is up: back up profile files
                         that are valid (written to a temp file, fsynced,
                         then renamed, so a power cut cannot empty a backup)

To forget a Wi-Fi network for good, delete it in NetworkManager and also
delete its file in the backup folder, or it comes back on the next boot.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import yaml

NETPLAN_DIR = Path(os.environ.get("NETFIX_NETPLAN_DIR", "/etc/netplan"))
KEYFILE_DIR = Path(os.environ.get("NETFIX_KEYFILE_DIR", "/etc/NetworkManager/system-connections"))
BACKUP_ROOT = Path(os.environ.get("NETFIX_BACKUP_DIR", "/var/lib/c-robot/netplan-backup"))


def valid_netplan(path: Path) -> bool:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError):
        return False
    return isinstance(data, dict) and isinstance(data.get("network"), dict)


def valid_keyfile(path: Path) -> bool:
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return False
    return "[connection]" in text and "id=" in text


# (live folder, file pattern, validity check, backup folder)
SOURCES = (
    (NETPLAN_DIR, "90-NM-*.yaml", valid_netplan, BACKUP_ROOT),
    (KEYFILE_DIR, "*.nmconnection", valid_keyfile, BACKUP_ROOT / "keyfiles"),
)


def write_atomic(path: Path, data: bytes) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)
    fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def save() -> int:
    saved = 0
    for live, pattern, valid, backup_dir in SOURCES:
        backup_dir.mkdir(parents=True, exist_ok=True)
        for path in sorted(live.glob(pattern)):
            if not valid(path):
                print(f"skip (empty or broken): {path.name}")
                continue
            backup = backup_dir / path.name
            data = path.read_bytes()
            if not backup.exists() or backup.read_bytes() != data:
                write_atomic(backup, data)
                saved += 1
    print(f"saved {saved} changed profile file(s) to {BACKUP_ROOT}")
    return 0


def restore() -> int:
    restored = []
    netplan_restored = False
    for live, pattern, valid, backup_dir in SOURCES:
        for backup in sorted(backup_dir.glob(pattern)):
            target = live / backup.name
            if target.exists() and valid(target):
                continue
            if not valid(backup):
                print(f"backup also broken, not used: {backup.name}")
                continue
            live.mkdir(parents=True, exist_ok=True)
            write_atomic(target, backup.read_bytes())
            restored.append(backup.name)
            netplan_restored |= live == NETPLAN_DIR
    if restored:
        print("restored: " + ", ".join(restored))
        if netplan_restored and NETPLAN_DIR == Path("/etc/netplan"):
            # The netplan generator already ran at boot; redo it for NetworkManager.
            subprocess.run(["netplan", "generate"], check=False)
    else:
        print("all profile files fine")
    return 0


if __name__ == "__main__":
    action = sys.argv[1] if len(sys.argv) > 1 else ""
    if action == "save":
        sys.exit(save())
    if action == "restore":
        sys.exit(restore())
    print(__doc__)
    sys.exit(2)
