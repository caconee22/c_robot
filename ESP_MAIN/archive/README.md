# 보관 (2026-10-31)

학현 결정으로 쓰지 않게 된 펌웨어 진입 파일과 빌드 환경을 모아 둔 곳이다.
이 폴더는 `src/` 밖에 있어서 빌드되지 않는다. 지우지 않고 기록만 남긴다.

지금 쓰는 펌웨어는 두 개다 (`platformio.ini`).

| 환경 | 용도 |
|---|---|
| `yd_esp32_s3_v2` | 경기용 구동 FSM |
| `robot_bench_v2_manual` | 벤치 시험 + 바닥 색 교정 (예찬 작성 벤치에 교정 통합, PC 쪽 `tools/bench_gui.py`) |

## 보관한 것

| 파일 | 원래 위치 | 내용 |
|---|---|---|
| `src/bench_main.cpp` | `src/` | 기존 벤치 (`robot_bench`, `robot_bench_v2`) |
| `src/sensor_serial_test.cpp` | `src/` | 센서 원시값 점검 (`sensor_serial_test(_v2)`) |
| `src/calibration_main.cpp` | `src/` | 단독 색 교정 펌웨어 (`color_calibration(_v2)`). 교정은 이제 벤치 안에서 함 |
| `tools/bench_console.py` | `tools/` | 기존 벤치용 콘솔. `bench_gui.py`가 대신함 |
| `platformio_full.ini` | `platformio.ini` | 보관 전 전체 환경 정의 (V1, TCS34725 변형 포함) |

V1 핀(`include/pins.hpp`)과 TCS34725 코드는 공용 모듈 안에 있어서 그대로 두었다.
PC 회귀 시험(`tests/run_host_tests.ps1`, `-V2` 없이)은 여전히 V1 핀으로도 컴파일한다.

## 되살리는 법

1. 파일을 원래 위치로 `git mv` 한다.
2. `platformio_full.ini`에서 필요한 `[env:...]` 블록을 `platformio.ini`로 옮긴다.
   V1 환경은 `[env]`의 `-D HW_VERSION=2`와 `qio_qspi`를 빼야 한다.
3. `pio run -e <환경>`과 PC 시험으로 확인한다.
