#!/usr/bin/env bash
# Flash the ESP32-S3 from the Raspberry Pi (remote work over Tailscale).
# Build on a PC with PlatformIO, copy the four .bin files into one folder on
# the Pi, then:  bash ESP_MAIN/tools/flash_from_pi.sh ~/fw [/dev/ttyACM0]
# Offsets and flash settings are the ones `pio run -t upload -v` prints for
# robot_bench_v2_manual / yd_esp32_s3_v2 (8MB, dio, 80m).
set -euo pipefail

FW_DIR="${1:?usage: flash_from_pi.sh <folder with bootloader/partitions/boot_app0/firmware .bin> [port]}"
PORT="${2:-/dev/ttyACM0}"
for f in bootloader.bin partitions.bin boot_app0.bin firmware.bin; do
  [ -f "$FW_DIR/$f" ] || { echo "missing $FW_DIR/$f"; exit 1; }
done

# The vision sender uses /dev/ttyAMA0, not this USB port, so it can keep running.
esptool --chip esp32s3 --port "$PORT" --baud 921600 \
  --before default_reset --after hard_reset \
  write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x0000 "$FW_DIR/bootloader.bin" \
  0x8000 "$FW_DIR/partitions.bin" \
  0xe000 "$FW_DIR/boot_app0.bin" \
  0x10000 "$FW_DIR/firmware.bin"
