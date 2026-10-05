# C_ROBOT — IRC 씨름 로봇

ESP32-S3 주행 제어, 라즈베리파이 영상 처리, 회로 설계를 관리하는 저장소다.
현재 주행 펌웨어는 **V1 하드웨어**를 대상으로 한다.

처음 사용하는 사람은 [펌웨어 구조·사용법 전체 설명](ESP_MAIN/README.md)을 먼저 읽는다.
파일별 책임, 실제 FSM/안전 흐름, 센서 설정·교정, UART 규격, 모터 API, 주기와 실물 시험 순서를 설명한다.
별도 벤치의 콘솔/CSV는 [BENCH_TESTS.md](ESP_MAIN/BENCH_TESTS.md), 검토 근거는
[VERIFICATION.md](ESP_MAIN/VERIFICATION.md)에 있다. 아래는 저장소 전체와 영상 프로그램 안내다.

## 현재 진행 상황

2026-10-05 기준, ESP32 펌웨어는 부팅부터 센서·통신·안전·FSM·모터 출력까지 연결했다.
하드웨어 없는 자동 시험과 ESP32용 빌드를 통과했으며, 실제 기체 시험과 튜닝은 남아 있다.
컴파일/PC 시험 통과를 실제 차량의 완전한 동작 보증으로 해석하면 안 된다.

| 항목 | 진행 상태 |
|---|---|
| PlatformIO / Arduino / ESP32-S3 프로젝트 | 구성 완료 |
| V1 핀 정의 및 초기 안전 출력 | 구현 완료, V2 핀 배치는 미적용 |
| 부팅·리셋 원인·시작 대기 | 구현 완료 |
| SW1 인터럽트 비상정지 | 영구 래치, 브레이크 요청, 재부팅 전까지 구동 금지 |
| SW2 인터럽트 시작 버튼 | 정상 눌림·해제 각각 40ms 확인 후 한 번 시작 |
| BTS7960 좌우 모터 공용 API | 출력 제한, 정지/브레이크, 방향 설정, 상태 조회 구현 |
| 모터 명령 만료 감시 | 공용 API 기본 100ms, FSM 80ms, 별도 5ms 감시 |
| AS7341 / TCS34725 네 모서리 센서 | 빌드 시 한 종류 선택, TCA9548A 경유 읽기 구현 |
| 바닥색 판별 | 빨강·노랑·파랑·검정, 새 측정의 동일 색 3회 확인 |
| 컬러 캘리브레이션 | UART 안내식 48단계, 자세별 3초 측정, NVS 저장 구현 |
| 라즈 UART1 | 115200bps, 14바이트, CRC 없음, sequence/신선도 검사 구현 |
| 이벤트 로그 / WS2812 LED | 변경 감지·중복 억제·우선순위 표시 구현 |
| 공격 FSM | 탐색·목표 확인·정렬·접근·돌진·급정거·삽입·밀기·회피 연결 |
| PC 자동 검증 | 두 센서 구성 각각 12개 시험 그룹 통과 |
| ESP32 빌드 | 생산/센서 전용/벤치 × 두 센서 구성, 여섯 환경 통과 |
| 별도 실물 시험 펌웨어 | 모터/UART 수동/데이터/회전 정렬/추적/바닥 회피 구현, 두 센서 구성 빌드·PC 검사 통과 |
| 실물 성능·브레이크·색 임계값·전략 튜닝 | 미완료, 하드웨어 확보 후 진행 |

V1에는 IR, 거리센서, IMU, 전류센서, 엔코더, 배터리 ADC를 사용하지 않는다.
V2 회로 확장 논의와 V1 펌웨어를 혼동하지 않는다.

### 바닥 센서 최적화와 고정 기체 조건

AS7341을 약 5ms 단일 수광(F2/F3/F5/F6/F7/Clear) 연속 측정으로 변경했다.
새 측정 3회 확인은 유지한다. 수광·게인·센서별 LED·검정 임계값·비율/밝기 판별 허용 범위를
`ESP_MAIN/include/sensors/sensor_config.hpp`에 모았다. 실물 성능은 아직 검증되지 않았다.
기체는 추가 가림막 없이 자체 LED를 사용하며 10mm 단차에서 센서-바닥 간격이 3/10/20mm로 변한다.
근접 포화, 먼 거리에서 검정 오인, 색 경계 혼합을 반드시 고려한다.
교정 높이 안내와 튜닝 방법은 [센서 조정 안내](ESP_MAIN/README.md#바닥센서-조정--2026-10-05)를 읽는다.

## 저장소 안내

- [`ESP_MAIN/`](ESP_MAIN/): 주행 펌웨어와 하드웨어 없는 자동 시험.
- [`ESP_MAIN/README.md`](ESP_MAIN/README.md): 펌웨어의 의도·파일별 역할·안전/FSM·센서·통신·사용법 전체 설명.
- [`ESP_MAIN/BENCH_TESTS.md`](ESP_MAIN/BENCH_TESTS.md): 별도 시험 펌웨어의 모터/수동 UART/데이터/회전 정렬/추적/구역 회피 모드.
- [`ESP_MAIN/VERIFICATION.md`](ESP_MAIN/VERIFICATION.md): 요구사항 대응, 실행 흐름, 수정 내역, 시험 범위와 한계.
- [`ESP_MAIN/AGENTS.md`](ESP_MAIN/AGENTS.md): 사용자 요구사항과 다른 AI/개발자를 위한 작업 기준.
- [`scripts/`](scripts/): 영상 처리·시험 영상 준비 등의 파이썬 코드.
- [`COLOR_DETECTOR.md`](COLOR_DETECTOR.md): 기존 영상 검출기 실행·튜닝 안내.
- [`PCB/`](PCB/): 회로 설계 자료. 펌웨어 핀의 기준은 `ESP_MAIN/include/pins.hpp`다.
- [`PCBV2/`](PCBV2/): V2 회로/PCB, 프로젝트 라이브러리, 3D 모델, 제작 자료와 설계 백업.
  V2 자료를 보관하지만 ESP32 펌웨어는 여전히 V1 핀 배치다.

영상 처리 파이썬 코드의 존재와 현재 ESP32 패킷 규격의 종단간 호환성 검증은 별개다.
실제 라즈 송신기와 UART 연결 시험은 남아 있다.

## 펌웨어 구조

```text
ESP_MAIN/
  include/
    pins.hpp             V1 GPIO
    config.hpp           UART/공통 주기
    motor/motor_config.hpp
    sensors/sensor_config.hpp
    control/control_config.hpp
  src/
    main.cpp             전체 실행 연결
    app/                 부팅
    hardware/            초기 핀 상태/리셋 원인
    interrupts/          안전/시작 버튼
    sensors/             측정/색 판별/캘리브레이션
    communication/       라즈 UART 패킷
    system/              안전관리/로그/LED
    control/             FSM/좌우 구동 변환
    motor/               공용 제어/BTS7960 출력
    sensor_serial_test.cpp
  tests/                 생산 소스를 사용하는 PC 자동 검증
```

기본 실행은 메인 루프이며, 모터 명령 만료만 작은 FreeRTOS 태스크로 별도 감시한다.
일반 제어 루프에서 수광 시간 전체를 기다리지 않고 센서 읽기를 짧은 단계로 진행한다.
LED와 이벤트 로그는 제어 뒤에 처리한다.

```text
부팅 → 시작 대기 → SW2 눌림·해제 → 탐색
  → 목표 프레임 2회 확인 → 정렬/접근
  → 돌진 220ms → 브레이크 90ms → 스포크 삽입 350ms → 밀기
  → 목표 상실 시 제한된 시간만 밀기 → 후진 260ms → 탐색

위험색 확인 → 공격보다 먼저 위험구역 회피
SW1 입력   → 일반 실행 종료 → ESTOP만 3초마다 출력 → 재부팅 필요
```

기동 시간과 출력은 실측 전의 초기값이다. 실제 접촉이나 스포크 삽입 성공을 측정하는 제어는 아니다.

## 빌드와 업로드

PlatformIO가 설치된 터미널에서 실행한다.
현재 보드 설정은 `esp32-s3-devkitc-1`의 8MB 플래시이며 PSRAM은 사용하지 않는다.
실제 YD-ESP32-S3 사양과 업로드 포트는 실물에서 확인해야 한다.

```powershell
cd D:\C_ROBOT\ESP_MAIN
pio run -e yd_esp32_s3
```

| 환경 | 용도 | UART0 모니터 |
|---|---|---:|
| `yd_esp32_s3` | 설정 파일에서 선택한 센서의 생산 펌웨어 | 57600 |
| `yd_esp32_s3_tcs34725` | TCS34725를 지정한 생산 펌웨어 | 57600 |
| `sensor_serial_test` | 설정 파일에서 선택한 센서 전용 점검 | 115200 |
| `sensor_serial_test_tcs34725` | TCS34725 센서 전용 점검 | 115200 |
| `robot_bench` | 설정 파일의 센서로 저출력 단계별 실물 시험 | 57600 |
| `robot_bench_tcs34725` | TCS34725 실물 시험 | 57600 |

업로드와 생산 펌웨어 모니터:

```powershell
pio run -e yd_esp32_s3 -t upload
pio device monitor -b 57600
```

센서 전용 점검은 모터 핀을 초기화하거나 변경하지 않고 약 100ms마다 버튼/컬러 원시값을 출력한다.
**이 환경을 사용할 때는 모터 전원을 분리한다.** 생산 펌웨어의 안전 제어가 실행되는 환경이 아니다.

## 설정 변경 위치

| 수정할 내용 | 파일 |
|---|---|
| V1 GPIO 배치 | `ESP_MAIN/include/pins.hpp` |
| UART/로그/버튼 주기 | `ESP_MAIN/include/config.hpp` |
| PWM, 출력 상한, 모터 방향, 브레이크 조합 | `ESP_MAIN/include/motor/motor_config.hpp` |
| 센서 종류, TCA 채널, 수광 시간, 게인, 임시 색 임계값 | `ESP_MAIN/include/sensors/sensor_config.hpp` |
| 탐색·공격·회피 출력/시간, 목표 박스 기준 | `ESP_MAIN/include/control/control_config.hpp` |

기본 환경의 센서 종류는 `sensor_config.hpp`의 다음 한 줄로 선택한다.

```cpp
#define COLOR_SENSOR_MODEL COLOR_SENSOR_AS7341
// TCS34725 차량: 위 값을 COLOR_SENSOR_TCS34725로 변경
```

TCS34725 전용 환경은 빌드 옵션으로 종류를 지정한다.
종류·수광·게인·채널 설정을 바꾸면 이전 교정 데이터가 호환되지 않아 임시 임계값으로 동작한다.
센서 위치는 전방 기준 1=왼쪽 위, 2=오른쪽 위, 3=왼쪽 아래, 4=오른쪽 아래다.

## 교정과 오류 처리

시작 대기 중 UART0에 `CAL START`를 입력한다.
센서 네 개 × 바닥색 네 개 × 평행/들림/눌림 세 자세를 각각 안내받아 측정한다.
`READY` 후 3초 카운트다운과 3초 측정, `ACCEPT`로 다음 단계, 완료 후 `CAL SAVE`로 저장한다.
`RETRY`, `BACK`, `ABORT`, `STATUS`, `CAL ERASE`, `CAL MARGIN <색비율%> <밝기%>`도 지원한다.
교정 중에는 모터가 정지하며, SW2로 중단하면 새로운 눌림·해제 없이 출발하지 않는다.

- 라즈가 없거나 끊겨도 시스템 고장으로 만들지 않고 탐색 등 비전 없는 동작으로 전환한다.
- 센서 부팅 실패는 부팅을 막지 않는다. 실패 채널은 Unknown으로 남긴다.
- 센서 I2C/수광 타임아웃은 채널을 제외하고 오류를 기록한다. 자동 재초기화하지 않는다.
- 모터/애플리케이션 고장은 즉시 정지한다. 오류가 해제되었다고 자동 출발하지 않는다.
- SW1은 영구 구동 금지다. 스위치 해제나 일반 래치 해제 API로 복구할 수 없다.

## 검증 결과와 재실행

2026-10-05 코드 검토 후 결과:

| 검증 | 결과 |
|---|---|
| AS7341 PC 생산 시험 | 12그룹, assert 4,994회 통과 |
| TCS34725 PC 생산 시험 | 12그룹, assert 4,555회 통과 |
| PC 벤치 시험 | 두 센서 구성 각각 67회 통과 |
| ESP32 여섯 환경 빌드 | 모두 성공 |
| Python 핵심 시험 | 가상 이미지 기반 5개 테스트 통과 |
| 두 Pi 추적기 | 로컬 영상 각각 20프레임, 화면 없이 실행·종료 확인 |

assert 횟수에는 반복 측정 확인이 포함되며 서로 다른 시나리오 개수는 아니다.
실제 생산 소스와 `setup()/loop()`를 가짜 GPIO/UART/I2C/NVS에 연결해 검증한다.
네이티브 g++가 필요하며 PATH 또는 PlatformIO의 `toolchain-gccmingw32`를 사용한다.

```powershell
cd D:\C_ROBOT\ESP_MAIN
.\tests\run_host_tests.ps1
.\tests\run_host_tests.ps1 -Demo
.\tests\run_host_tests.ps1 -Bench
```

`-Demo`는 공격 상태와 좌우 출력을 가상 입력으로 보여주는 텍스트 데모다.
시험 상세와 수정한 오류는 [검증 기록](ESP_MAIN/VERIFICATION.md)에 정리했다.

## 영상 코드의 구성과 의도

바닥의 네 색 판별은 ESP32의 광학 센서 코드다. 아래 Python 코드의 녹색 타워 검출과
교정 데이터를 공유하지 않는다. 라즈는 영상만 처리하고, 주행 판단/안전은 ESP32에서 수행하는 구조다.

```text
카메라 또는 영상 파일
  → 저해상도 BGR/ExG 후보 → 후보 ROI 확대 → 원본 해상도 BGR/ExG/HSV/Lab 검증
  → 마스크 정리 → 연결 성분 / 후보 선택(최대2개) → 전역 좌표
  → 텍스트 / JSON / 화면 / 성능 보고서
  → [미구현: 14바이트 UART 패킷 송신] → ESP32 RaspberryLink → FSM → 모터
```

현재 Pi 추적기는 pyserial/UART 송신을 하지 않는다. `--json`은 좌표를 표준출력에 쓰는 옵션이다.
그 JSON에는 현재 박스 폭/높이 등 ESP 패킷 필드가 전부 들어 있지도 않다.
실물 ALIGN/FOLLOW/AVOID에는 규격 송신기가 필요하며, JSON을 그대로 UART1에 보내면 안 된다.
50fps 카메라 설정도 실제 처리/송신 50Hz를 증명하지 않는다.

| 파일 | 역할과 사용 의도 |
|---|---|
| `scripts/color_detection_core.py` | 원본 전체 프레임 검출 기준. BGR/ExG/HSV/Lab, 융합, 모폴로지, 연결 성분, 후보 자료형 |
| `scripts/color_detection_lightweight.py` | 작은 후보 영상에서 ROI를 찾고 원본 ROI만 정밀 검증. 전역 bbox/centroid/contour 변환으로 계산량 감소 |
| `scripts/pi_green_tracker.py` | 공유 검출기를 파일/USB카메라/Picamera2에 연결. JSON/미리보기/파일 반복/측정/내보내기 UI |
| `raspberry_pi_green_tracker_single.py` | 복사하기 쉬운 단일 파일 Pi 검출기. 상단 고정 녹색 임계값과 미소 타워의 엄격한 필터 |
| `scripts/color_detector_ui.py` | 실행 입구. 기본 단계별 wizard, --advanced-ui 전체 설정 UI, --benchmark 화면 없는 측정 |
| `scripts/color_detector_wizard.py` | 순서대로 BGR→ExG→HSV→Lab→융합→후처리 튜닝, ROI 자동 조정, 설정/스냅샷/내보내기 |
| `scripts/benchmark_lightweight_compare.py` | 동일 영상의 전체 프레임/ROI 검출 시간·마스크·후보 비교. 실행 순서를 번갈아 편향 완화 |
| `scripts/color_detector_export.py` | 기준 검출기의 원본/결과/각 마스크를 분석 영상으로 저장 |
| `scripts/export_lightweight_full_video.py` | ROI 검출기의 전체 분석 영상 저장. 실제 제어 루프에서 인코딩하지 않고 별도 실행 |
| `scripts/prepare_test_video.py` | 시험 영상을1920×1080/50fps로 리사이즈·시간 기반 프레임 반복, 메타데이터 검증 |
| `scripts/green_cylinder_video_live.py` | 초기 녹색 검출기 재생/슬라이더/마스크/샘플 저장. 현재 공유 검출기와 별도 실험 구현 |
| `scripts/green_cylinder_image_batch.py` | img 이미지의 초기 녹색 후보를 일괄 검출해 주석 이미지/CSV 생성 |
| `scripts/preprocess_green_images.py` | 초기 영상 검출기 기본 설정으로 이미지·마스크·검출 데이터를 준비 |
| `scripts/opencv_green_cylinder_demo.py` | 합성 도형으로 OpenCV 검출/마스크 저장의 간단한 설치 점검 |
| `scripts/face_tracking_demo.py` | Haar 얼굴 검출 설치 점검. 경기 타워 검출이나 FSM에 사용하지 않음 |
| `cv_live_view.py` | 별도 합성 영상 data/synthetic.avi의 루프 표시 예제. 그 입력 파일이 있어야 하며 주요 Pi 실행 입구가 아님 |
| `scripts/generate_htd5m_20t_pulley.py` | CadQuery/DXF 기반 기구 STEP/STL 생성. 주행/CV와 무관, 별도 CadQuery 환경 필요 |
| `tests/test_vision.py` | 가상 대상/비대상색·좌표·ROI 병합·경계·마스크 형식 검증. 실제 카메라 성능 시험은 아님 |
| `setup_raspberry_pi.sh` | Raspberry Pi OS의 OpenCV/NumPy/Picamera2 시스템 패키지 설치 |
| `run_pi_tracker.sh` | 저장소 위치로 이동 후 Pi 카메라 입력50fps로 공유 추적기 실행 |

공유 검출기는 `config/color_detector_last.json`을 기본값 위에 덮어쓴다.
단일 파일 추적기는 이 JSON을 읽지 않는다. 따라서 UI 설정을 저장했다고 두 추적기 임계값이
동시에 바뀌지는 않는다. 단일 파일은 상단 상수를 조정해야 한다.
예를 들어 순수(0,255,0) 녹색도 단일 파일의 좁은 HSV/Lab 범위 밖이면 거부될 수 있다.
저해상도 후보를 놓친 대상은 ROI 정밀 검증에 들어오지 않는다. 두 방식의 검출 결과가 항상 같지는 않다.

### 로컬 Windows에서 실행

프로젝트 루트에서 가상환경 Python을 사용한다. 기존 `.venv`가 없다면 생성 후 의존성을 준비한다.
핵심 CV 실행에는 OpenCV/NumPy가 필요하고 저장소 전체 `requirements.txt`는 다른 실험용 패키지도 포함한다.

```powershell
cd D:\C_ROBOT
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install opencv-python==4.12.0.88 numpy
# 기존 환경을 쓰면 위 생성/설치는 생략
.\.venv\Scripts\python.exe scripts/pi_green_tracker.py --source dataset/videos/irc_highlight_1080p50.mp4 --preview --realtime
```

Q 종료, M 마스크 표시, E 전체 파일 내보내기. `--realtime`은 파일 원래 FPS에 맞춰 기다리므로
순수 처리 성능 시험에서는 빼야 한다. `--loop`는 파일을 반복한다.
GUI는 실제 데스크톱 세션이 필요하다. 파일/카메라 해상도와 처리 시간은 별도 확인한다.

화면 유무 비교와 좌표 출력:

```powershell
.\.venv\Scripts\python.exe scripts/pi_green_tracker.py --quiet --benchmark --max-frames 300
.\.venv\Scripts\python.exe scripts/pi_green_tracker.py --quiet --benchmark --max-frames 300 --preview
.\.venv\Scripts\python.exe scripts/pi_green_tracker.py --json --max-frames 20
.\.venv\Scripts\python.exe raspberry_pi_green_tracker_single.py --source dataset/videos/irc_highlight_1080p50.mp4 --preview
.\.venv\Scripts\python.exe scripts/color_detector_ui.py
```

벤치 보고서는 `output/color_detector/benchmarks`에 저장된다. 기본10프레임 예열은 통계에서 제외한다.
검출 시간과 입력/표시/대기 포함 loop 시간을 구분한다. 일반 실행의 FPS 이력은60프레임으로 제한하고,
명시적 --benchmark는 전체 통계를 위한 목록을 보관하므로 장시간 운전 옵션이 아니라 유한 시험용으로 쓴다.
전체 프레임/ROI 비교가 필요하면 benchmark_lightweight_compare.py를 실행한다.
각 보조 도구의 옵션은 `--help`, 튜닝·GUI·내보내기의 상세 키는 [COLOR_DETECTOR.md](COLOR_DETECTOR.md).

### 라즈베리파이에서 실행

Raspberry Pi OS의 Picamera2는 시스템 패키지와 카메라 드라이버가 필요하다.
일반 PC에 pip로 설치한 OpenCV만으로 카메라 입력이 되지는 않는다.

```bash
bash setup_raspberry_pi.sh
python3 -m venv --system-site-packages .venv
.venv/bin/python scripts/pi_green_tracker.py --source camera --fps 50 --preview
# 또는 시스템 Python 실행 래퍼:
bash run_pi_tracker.sh --preview
```

이미 설정한 환경이라면 설치를 반복할 필요 없다. 현재 Windows 검토는 라즈에 원격 접속하거나
패키지를 변경하지 않았다. 실행 중인 카메라/GUI/50Hz UART의 실측과 종단간 검증은 남아 있다.

### Python 검증 재실행

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m compileall -q scripts tests cv_live_view.py raspberry_pi_green_tracker_single.py
```

이 검증은 주요 공유/단일 검출기의 오프라인 논리를 검사한다. 모든 GUI/내보내기/CAD 경로,
실제 영상의 정답 정확도, 카메라 성능과 UART 송신까지 보증하는 시험은 아니다.
영상·출력·가상환경·빌드 결과물은 Git에 올리지 않으므로 새 clone에는 기본 시험 영상이 없을 수 있다.
영상이 없으면 별도로 준비하거나 --source로 실제 파일을 지정한다.

## 다음 단계

1. 모터 전원을 분리하고 센서 종류·TCA 채널·원시값부터 확인한다.
2. 출력 상한을 낮추고 바퀴를 띄워 모터 방향, SW1 브레이크, SW2 시작을 확인한다.
3. 실제 바닥색과 자세별로 수광·게인·임계값을 교정한다.
4. 라즈 UART 송신·분리·재부팅을 종단간 시험한다.
5. 위험색 회피를 먼저 시험한 뒤 공격 출력을 올리고 기동 시간을 튜닝한다.

센서 100Hz는 목표이지 현재 실측 달성 결과가 아니다. AS7341 전체 스펙트럼은 두 번의 수광이 필요하다.
실제 브레이크 효과, 시간 기반 기동의 기체 안정성, 바닥과 상대 로봇 표면 구분은 실물 검증이 필요하다.
영상·가상환경·빌드 산출물은 `.gitignore`에 따라 제외한다.
2026-10-05 추가 작업 반영에는 남은 PCBV2 자료와 파이썬 코드, PNG 실행 화면도 함께 포함했다.
