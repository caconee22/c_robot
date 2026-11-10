#!/usr/bin/env bash
# Pull a full backup of the C_ROBOT Raspberry Pi to this PC (Git Bash on
# Windows or any Linux). Needs the passwordless SSH key (PI_RECOVERY.md).
#   bash scripts/pi_backup_pull.sh [pi address]
# Output: D:/c_robot_pi_backup/<date_time>/ (outside Git: it holds Wi-Fi
# passwords and the Tailscale key). Copy the Git-safe parts into pi_backup/
# by hand afterwards (PI_RECOVERY.md section 5).
# Streams straight to the PC (nothing written on the Pi), because the Pi
# switched off once mid-copy and /tmp is lost on reboot.
set -euo pipefail

PI="${1:-100.115.150.119}"
SSH="ssh -o BatchMode=yes -o ConnectTimeout=15 caconee@$PI"
OUT="/d/c_robot_pi_backup/$(date +%Y%m%d_%H%M)"
mkdir -p "$OUT"
echo "== home (everything but .cache) -> $OUT/home_caconee.tgz"
# A log file still being written makes tar exit 1; the archive is still good.
$SSH 'cd ~ && nice -n 10 tar czf - --exclude=.cache .' > "$OUT/home_caconee.tgz" || true
echo "== system settings -> $OUT/system.tgz"
$SSH 'sudo -n tar czf - /etc/systemd/system/c-robot-* /etc/sudoers.d/010_caconee-nopasswd \
  /boot/firmware/config.txt /boot/firmware/cmdline.txt /etc/NetworkManager/system-connections \
  /etc/netplan /var/lib/c-robot /etc/hostname /etc/hosts /etc/apt/sources.list.d /var/lib/tailscale 2>/dev/null' \
  > "$OUT/system.tgz"
echo "== package lists"
$SSH 'apt-mark showmanual' > "$OUT/apt_manual.txt"
$SSH 'pip3 list 2>/dev/null' > "$OUT/pip_system.txt"
$SSH '~/esptool-venv/bin/pip freeze' > "$OUT/esptool_venv_freeze.txt"
$SSH 'systemctl list-unit-files --state=enabled --no-pager' > "$OUT/enabled_units.txt"
$SSH 'cd ~/c_robot && git status --short && git log --oneline -1' > "$OUT/pi_repo_status.txt"
tar tzf "$OUT/home_caconee.tgz" > /dev/null && echo "home archive OK"
ls -la "$OUT"
