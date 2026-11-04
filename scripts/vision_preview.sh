#!/bin/bash
# Desktop launcher (C_ROBOT 카메라 보기): live camera window with the tower box.
# The boot-time sender (c-robot-vision) holds the camera, so it is stopped here
# and started again when this ends (q in the window, Ctrl+C, or closing the
# terminal). Packets keep going to the ESP32 while the window is open.
#
# 11/04: closing the terminal used to restart the service while the preview
# process was still exiting; that service instance sent bytes the ESP32 never
# received until it was restarted. So the preview is stopped and waited for
# before the service starts again.
cd "$(dirname "$0")/.." || exit 1
sudo -n systemctl stop c-robot-vision
python3 scripts/pi_vision_sender.py --preview --detector hsv &
pid=$!
# TERM, not INT: bash starts background jobs with SIGINT ignored.
trap 'kill -TERM "$pid" 2>/dev/null' HUP INT TERM
wait "$pid"
wait "$pid" 2>/dev/null   # the first wait returns early when a signal arrives
sudo -n systemctl start c-robot-vision
echo "끝. 자동 송신을 다시 켰습니다. 창을 닫아도 됩니다."
sleep 2
