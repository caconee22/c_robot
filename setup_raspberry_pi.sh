#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-picamera2
chmod +x run_pi_tracker.sh

echo "설치 완료"
echo "카메라 시험: rpicam-hello -t 3000"
echo "추적 실행: ./run_pi_tracker.sh"
