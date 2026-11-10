#!/usr/bin/env bash
# Rebuild the C_ROBOT Raspberry Pi 5 after a fresh Raspberry Pi OS install.
# Steps and reasons: PI_RECOVERY.md (repo root). Run as user caconee from
# ~/c_robot after `git clone`:   bash scripts/pi_restore.sh
# Safe to run again; every step only adds or overwrites C_ROBOT files.
# It does NOT set Wi-Fi passwords or log in to Tailscale (you type those).
set -euo pipefail

REPO="$HOME/c_robot"
cd "$REPO"
echo "== 1/7 packages"
sudo apt update
# Same list as the old Pi (pi_backup/apt_manual.txt has the full list).
sudo apt install -y git git-lfs python3-opencv python3-numpy python3-picamera2 \
  python3-serial python3-yaml python3-smbus2 python3-venv lxterminal

echo "== 2/7 esptool for flashing the ESP32 from the Pi (flash_from_pi.sh)"
if [ ! -x "$HOME/esptool-venv/bin/esptool.py" ]; then
  python3 -m venv "$HOME/esptool-venv"
  "$HOME/esptool-venv/bin/pip" install esptool==4.8.1
fi
# Old Pi: /usr/local/bin/esptool -> ~/esptool-venv/bin/esptool.py
sudo ln -sf "$HOME/esptool-venv/bin/esptool.py" /usr/local/bin/esptool

echo "== 3/7 UART0 on GPIO14/15 for the ESP32 link (/dev/ttyAMA0)"
CFG=/boot/firmware/config.txt
grep -q '^dtparam=uart0=on' "$CFG" || printf '\n[all]\ndtparam=uart0=on\n' | sudo tee -a "$CFG" >/dev/null
# The serial login console must stay off on that UART.
sudo sed -i 's/console=serial0,[0-9]* //' /boot/firmware/cmdline.txt
sudo usermod -aG dialout,video,i2c,gpio "$USER"

echo "== 4/7 sudo without password (remote flashing / USB reset over SSH)"
echo "$USER ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010_caconee-nopasswd >/dev/null
sudo chmod 440 /etc/sudoers.d/010_caconee-nopasswd

echo "== 5/7 Pi-only settings (tower colors, camera) from the backup"
mkdir -p config
for f in tower_hsv.json camera.json; do
  if [ ! -f "config/$f" ] && [ -f "pi_backup/config/$f" ]; then
    cp "pi_backup/config/$f" "config/$f"; echo "restored config/$f"
  fi
done

echo "== 6/7 systemd services"
for f in c-robot-vision.service c-robot-esplog.service c-robot-netfix.service \
         c-robot-netsave.service c-robot-wifi-prefer.service c-robot-wifi-prefer.timer; do
  sudo cp "scripts/$f" /etc/systemd/system/
done
sudo systemctl daemon-reload
sudo systemctl enable c-robot-vision.service c-robot-esplog.service \
  c-robot-netfix.service c-robot-netsave.service c-robot-wifi-prefer.timer

echo "== 7/7 desktop icons"
mkdir -p "$HOME/Desktop"
for f in scripts/c-robot-*.desktop; do
  cp "$f" "$HOME/Desktop/"; chmod +x "$HOME/Desktop/$(basename "$f")"
done

echo
echo "Done. Still by hand (PI_RECOVERY.md section 3):"
echo "  - Wi-Fi caconee22 / 307 passwords, Tailscale login, VNC on"
echo "  - reboot, then: systemctl status c-robot-vision c-robot-esplog"
