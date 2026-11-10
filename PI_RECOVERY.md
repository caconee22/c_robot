# 라즈베리파이 복구·백업 안내

라즈(Raspberry Pi 5, 이름 `caconee`)의 SD 카드가 망가지거나 OS를 새로 깔았을 때, 처음 상태로 되돌리는 순서다.
2026-11-10 기준 라즈 전체를 백업하면서 작성했다. 라즈 쪽 동작 설명은 [README.md §6](README.md),
ESP32 원격 업로드는 [README.md §5](README.md)에 있다.

> 라즈는 주행 필수 장치가 아니다. 라즈가 죽어도 ESP32는 영상 없이 달린다(타워 추적만 안 됨).
> 경기 직전이라면 복구보다 먼저 ESP32가 혼자 시작·정지되는지 확인한다.

## 0. 라즈가 죽었을 때 바로 할 일 (요약)

| 순서 | 할 일 | 걸리는 시간 | 절 |
|---|---|---|---|
| 1 | 새 SD 카드에 Raspberry Pi OS 설치 (Imager에서 사용자·Wi-Fi·SSH 미리 설정) | 15분 | §1 |
| 2 | 부팅 → 터미널에서 저장소 받기 | 5분 | §2 |
| 3 | `bash scripts/pi_restore.sh` 한 번 실행 (패키지, UART, 서비스, 아이콘, 타워 색 설정) | 10분 | §2 |
| 4 | 손으로: Wi-Fi 307, Tailscale 로그인, VNC, 노트북 SSH 키 | 10분 | §3 |
| 5 | 재부팅 → 확인표대로 점검 | 5분 | §4 |

## 1. OS 설치 (Raspberry Pi Imager)

예전 라즈와 같게 맞춘다. 이름이 다르면 서비스 파일(`/home/caconee/...`)이 안 맞으니 **사용자 이름은 반드시 `caconee`**.

| 항목 | 값 |
|---|---|
| 장치 / OS | Raspberry Pi 5 / Raspberry Pi OS (64-bit) 데스크톱 — 예전 것은 Debian 13 trixie |
| 호스트 이름 | `caconee` |
| 사용자 | `caconee` (비밀번호는 팀이 아는 것으로, 문서에 적지 않음) |
| Wi-Fi | `caconee22` (핫스폿), 국가 KR |
| 지역 | Asia/Seoul |
| SSH | 켜기 (비밀번호 인증) |

## 2. 저장소 받고 자동 설정

라즈 터미널(또는 모니터 연결)에서:

```bash
cd ~
git clone https://github.com/caconee22/c_robot.git
cd c_robot
bash scripts/pi_restore.sh
```

`scripts/pi_restore.sh`가 하는 일 (여러 번 실행해도 됨):

1. 패키지: `python3-opencv python3-numpy python3-picamera2 python3-serial python3-yaml` 등
   (예전 목록 전체: [pi_backup/lists/apt_manual.txt](pi_backup/lists/apt_manual.txt))
2. `~/esptool-venv`에 esptool 4.8.1 설치, `/usr/local/bin/esptool`로 연결 → `ESP_MAIN/tools/flash_from_pi.sh`가 씀
3. `/boot/firmware/config.txt`에 `dtparam=uart0=on` (GPIO14/15 = `/dev/ttyAMA0`, ESP32 UART1과 연결),
   직렬 콘솔 끄기, `dialout` 등 그룹 추가
4. `sudo` 비밀번호 없이 (`/etc/sudoers.d/010_caconee-nopasswd`) — 원격 업로드·USB 재시작에 필요
5. `pi_backup/config/`의 `tower_hsv.json`(타워 색 범위), `camera.json`(카메라 설정)을 `config/`로 복사
6. systemd 서비스 설치·켜기:
   `c-robot-vision`(영상→ESP32 송신), `c-robot-esplog`(ESP32 주행 로그 → `logs/runs/`),
   `c-robot-netfix`/`c-robot-netsave`(Wi-Fi 설정 파일 0바이트 사고 복구·백업), `c-robot-wifi-prefer.timer`(30초마다 caconee22로 복귀)
7. 바탕화면 아이콘: 벤치 GUI, 타워 설정, 카메라 보기(이전), 추적 화면

## 3. 손으로 해야 하는 것 (비밀번호가 필요해서 스크립트에 없음)

**Wi-Fi 307 추가** (학교 Wi-Fi, 비밀번호 없음, caconee22보다 낮은 우선순위):

```bash
nmcli con add type wifi con-name 307 ssid 307 connection.autoconnect-priority 5
nmcli con modify caconee22 connection.autoconnect-priority 10
```

예전 설정 파일(비밀번호만 지운 것): [pi_backup/system/](pi_backup/system/)

**Tailscale** (노트북에서 원격 접속용):

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

나오는 주소를 브라우저에서 열어 팀 계정으로 로그인한다. **새로 로그인하면 Tailscale 주소(예전 `100.115.150.119`)가 바뀐다.**
노트북 쪽 스크립트·명령의 주소를 새 주소로 바꾼다. 같은 주소를 유지하려면 로컬 전체 백업의
`system.tgz` 안 `var/lib/tailscale/`을 `tailscaled` 정지 상태에서 그대로 되돌린다(§5).

**VNC** (원격 화면): `sudo raspi-config` → Interface Options → VNC → 켜기.

**노트북 SSH 키** (비밀번호 없이 원격 업로드): 노트북 PowerShell에서

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh caconee@<라즈 주소> "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```

비밀번호를 한 번 입력하면 그다음부터 `ssh -o BatchMode=yes caconee@<주소>`가 바로 된다.

## 4. 재부팅 후 확인표

```bash
sudo reboot
# 다시 접속한 뒤
systemctl status c-robot-vision c-robot-esplog --no-pager
ls /dev/ttyAMA0            # ESP32 연결 UART
rpicam-hello -t 3000       # 카메라 (서비스가 카메라를 잡고 있으면 먼저 sudo systemctl stop c-robot-vision)
esptool version
```

| 확인 | 정상 |
|---|---|
| `c-robot-vision` | active (running). 실패하면 `journalctl -u c-robot-vision -n 50` |
| `c-robot-esplog` | active. SW2로 출발하면 `~/c_robot/logs/runs/run_*.csv`가 생김 |
| 바탕화면 "C_ROBOT 추적 화면" | 타워에 박스가 그려짐 (송신은 계속됨) |
| 타워 색 | 경기장 조명에서 "C_ROBOT 타워 설정"으로 다시 확인 |
| 원격 업로드 | 노트북에서 경기용 `.bin` 업로드 → `Hash of data verified` |

## 5. 백업 방법과 위치

**백업 두 곳:**

| 위치 | 내용 | Git |
|---|---|---|
| `pi_backup/` (이 저장소) | 라즈에만 있던 설정(타워 색·카메라), 주행 로그, 시스템 설정 파일(비밀번호 지움), 패키지 목록 | 올림 |
| `D:\c_robot_pi_backup\<날짜>\` (노트북) | 홈 폴더 전체 압축(`home_caconee.tgz`), 시스템 설정 원본(`system.tgz`: Wi-Fi 비밀번호, Tailscale 키 포함) | **안 올림** (비밀번호 포함) |

**새로 백업하기** (노트북 Git Bash, 라즈가 켜져 있고 SSH 키가 있을 때):

```bash
bash scripts/pi_backup_pull.sh
```

라즈에 파일을 만들지 않고 노트북으로 바로 받는다(11/10에 백업 도중 라즈가 꺼져 `/tmp`에 만든 파일이 사라졌음).
그 뒤 바뀐 타워 색·카메라 설정과 새 주행 로그는 `pi_backup/config/`, `pi_backup/runs/`에 복사해 커밋한다.

**코드는 백업할 필요 없음:** 2026-11-10 비교 결과 라즈의 코드(`scripts/`, `tests/`, `ESP_MAIN/tools/`)는
노트북 저장소와 같다(줄바꿈만 다름). 라즈에만 있던 것은 `config/tower_hsv.json`, `config/camera.json`,
`logs/runs/`, `~/power_log.py`뿐이었고 모두 `pi_backup/`에 있다.

## 6. pi_backup 폴더 내용 (2026-11-10)

| 경로 | 내용 |
|---|---|
| `config/tower_hsv.json`, `camera.json` | 지금 쓰는 타워 색 범위, 카메라(720p 60fps, 셔터 1ms) 설정 |
| `runs/run_*.csv` | 11/10 주행 로그 9판 (ESP32 → 라즈 UART) |
| `system/config.txt`, `cmdline.txt` | 부트 설정 (`dtparam=uart0=on`이 핵심) |
| `system/*.nmconnection` | Wi-Fi caconee22(우선 10), 307(우선 5). 비밀번호는 `<비밀번호 직접 입력>`으로 바꿈 |
| `system/c-robot-powerlog.service`, `home/power_log.py` | 라즈 전원 꺼짐 원인 찾기용 1초 전원 기록(진단용, 복구 때는 안 켜도 됨) |
| `system/hostname`, `hosts`, `tailscale.list` | 호스트 이름, Tailscale apt 저장소 |
| `lists/` | apt 수동 설치 목록, pip 목록, esptool venv, 켜져 있던 서비스 목록 |

## 7. 알아 둘 문제

- **라즈 USB가 주행 뒤 죽음:** ESP32 USB(`/dev/ttyACM0`)가 사라지면
  `echo xhci-hcd.0 | sudo tee /sys/bus/platform/drivers/xhci-hcd/unbind` 다음 같은 명령의 `bind`로 살린다.
- **전원:** USB-C PD 모듈로만 넣는다. 끌 때는 전원 버튼 한 번(정상 종료). 갑자기 끊기면 Wi-Fi 설정 파일이
  0바이트가 될 수 있다(그래서 `c-robot-netfix`가 있음).
- **대기 전력:** 대기 1시간이면 배터리가 거의 바닥난다. 경기 전에는 예비 배터리를 준비한다.
