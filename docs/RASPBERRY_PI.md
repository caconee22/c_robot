# 라즈베리파이 5 — 영상 검출·송신·로그

라즈는 타워를 찾아 좌표만 보낸다. 주행 판단과 안전은 전부 ESP32가 한다. 라즈가 꺼지거나 끊겨도
ESP32는 120ms 뒤 영상을 "없음"으로 보고, 최종 경기 FSM(`AlignFsm`)은 마지막으로 본 쪽으로 제자리 회전하며 찾는다
(바닥 회피와 안전 기능은 그대로 동작). 라즈는 주행 필수 장치가 아니다.

같은 UART1의 반대 방향(ESP32 TX → 라즈 RX)으로는 ESP32가 실행 로그와 부팅·리셋 보고를 보낸다(11/10, 11/13).

```text
Camera Module 3 Wide (720p 60fps, 셔터 1ms, 11/12부터 화면 위쪽 ROI)
  → tower_detector_hsv.py (작은 화면에서 후보 → 원본에서 HSV 확인 → 납작한 것·크기 밖 제외 → 타워 고르기)
  → vision_packet.py (14바이트 패킷 V1, 1920×1080 좌표로 환산)
  → pi_vision_sender.py (최신 결과를 20ms마다 = 50Hz, 새 프레임일 때만 sequence 증가)
  → /dev/ttyAMA0 (GPIO14 TX) → ESP32 UART1 (115200bps, CRC 없음은 의도)
```

## 부팅 때 자동으로 도는 것 (systemd)

| 서비스 | 하는 일 |
|---|---|
| `c-robot-vision.service` | `pi_vision_sender.py --quiet` 자동 실행. 카메라·검출이 실패하면 끝나고 자동 재시작하지 않음 |
| `c-robot-netfix.service` | NetworkManager 전에 비어 버린 Wi-Fi 설정 파일 복구 (11/4 전원 차단으로 0바이트가 된 사고) |
| `c-robot-netsave.service` | 네트워크가 올라오면 Wi-Fi 설정 백업 |
| `c-robot-wifi-prefer.timer` | 30초마다 caconee22가 보이면 그쪽으로 옮김 (없으면 307) |
| `c-robot-esplog.service` | ESP32 주행 로그(UART RX) → `logs/runs/run_*.csv` |

## 바탕화면 아이콘

| 아이콘 | 하는 일 |
|---|---|
| C_ROBOT 타워 설정 | 자동 송신 서비스를 잠시 멈추고 `tower_settings_app.py` 실행. 드래그로 타워 색 잡기, 크기·모양·카메라 설정, 저장. 창을 닫으면 서비스 다시 시작. 열려 있는 동안에도 ESP32로 송신은 계속됨 |
| C_ROBOT 벤치 GUI | `ESP_MAIN/tools/bench_gui.py` |
| C_ROBOT 카메라 보기 (이전) | 예전 단순 미리보기 |
| C_ROBOT 추적 화면 (11/10) | 송신을 멈추지 않고 검출 결과를 보는 창 (`scripts/vision_viewer.py`, `view_tap.py`). 검출 60fps 유지 |

## 라즈 쪽 파일

| 파일 | 역할 |
|---|---|
| `scripts/pi_vision_sender.py` | 카메라 → 검출 → 패킷 50Hz 송신. `--dry-run`(UART 없이 출력), `--preview`(설정 앱), `--detector legacy` |
| `scripts/tower_detector_hsv.py` | 지금 쓰는 검출기. 색 범위 여러 개 합치기, 납작한 녹색 매트 제외, 최근 위치 추적 |
| `scripts/tower_settings_app.py` | 타워 설정 앱 (색 / 크기·모양 / 카메라 / 프리셋·도구 탭) |
| `scripts/vision_preview.py`, `vision_preview.sh` | 예전 미리보기 창과 아이콘 실행 스크립트 |
| `scripts/camera_settings.py` | `config/camera.json` 읽기·쓰기. 기본 720p 60fps, 셔터 1ms(회전 중 번짐), 게인 자동 |
| `scripts/esp_log_receiver.py` | ESP32 실행 로그(UART1 RX, 버전 0x81) → `logs/runs/run_*.csv`, 부팅·리셋 원인 → `logs/runs/boots.csv` (11/10, 11/13) |
| `scripts/vision_packet.py` | 패킷 V1 인코더. ESP32 파서와 같은 규칙, 잘못된 패킷은 만들지 않음 |
| `scripts/pi_netfix.py`, `pi_wifi_prefer.sh` | Wi-Fi 설정 복구·백업, 선호 Wi-Fi로 복귀 |
| `scripts/*.service`, `*.timer`, `*.desktop` | 위 서비스·아이콘 정의 |
| `setup_raspberry_pi.sh` | OpenCV·NumPy·Picamera2 시스템 패키지 설치 (예전 것. 새 OS에는 `scripts/pi_restore.sh`) |
| `scripts/pi_restore.sh`, `pi_backup_pull.sh` | 새 OS에 전부 다시 설정 / 노트북으로 라즈 전체 백업 ([PI_RECOVERY.md](../PI_RECOVERY.md)) |

**라즈에만 있는 설정 (Git에 없음):** `config/tower_hsv.json`(타워 색 범위), `config/camera.json`(카메라 설정).
11/10에 `pi_backup/config/`로 백업했다. SD 카드가 망가지면 [PI_RECOVERY.md](../PI_RECOVERY.md) 순서대로 새로 깔고 `scripts/pi_restore.sh`가 되돌린다.

## 라즈 사용 주의

- 끌 때는 전원 버튼 한 번(정상 종료, 약 10초). 켜다 꺼지기를 반복하면 SD 카드가 상할 수 있다.
- 전원은 USB-C PD 모듈로만. 예전 XL 벅으로 GPIO 5V에 넣으면 부팅 부하에서 전압이 무너져 꺼진다(11/4).
- 카메라는 한 프로그램만 열 수 있다. 설정 앱은 아이콘으로 열어야 서비스를 알아서 멈추고 다시 켠다.

## 라즈 시험 (PC에서도 실행 가능)

```powershell
cd D:\C_ROBOT
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

`test_vision_packet.py`(패킷 바이트·거부 규칙), `test_tower_detector.py`(검출기), `test_pi_netfix.py`(Wi-Fi 복구),
`test_vision.py`(예전 검출기). 실제 카메라 성능이나 경기장 조명은 이 시험으로 확인되지 않는다.


## 대회 중 추가된 것 (11/10–11/13)

- **ROI (11/12, 경기장 현장):** 화면 위쪽에 선을 긋고, 그 위에서는 일정 크기 이상인 녹색만 타워로 본다. 관중석·화면 밖 녹색 오검출을 막기 위함. 설정 앱에서 켜고 끈다.
- **실행 로그 (11/10):** 출발 버튼에서 파일을 열고 비상정지에서 닫는다. 상태·판단·0.2초 생존·자이로 방향을 기록한다. 분석은 `python ESP_MAIN/tools/run_log_stats.py`.
- **리셋 보고 (11/13):** ESP32가 부팅할 때 리셋 원인과 직전 상태를 보내고, 라즈가 `boots.csv`에 한 줄씩 남긴다.
- **EMI (11/13):** 라즈 쪽 전자파 문제로 ESP32가 죽는 현상을 현장에서 해결했다. 방법은 팀원 기록 참고([회고](../IRC2026_RETROSPECTIVE.md) §4).
