# ESP_MAIN — 펌웨어 구조와 사용법

ESP32-S3 / Arduino / PlatformIO 기반 씨름 로봇 펌웨어다.
이 문서는 작업 이력이 아니라 **모듈을 나눈 의도, 실행 순서, 설정 방법과 사용법**을 설명한다.
**2026-11-08 기준**으로 고쳤다.

- 지금 쓰는 펌웨어는 두 개다: 경기용 `yd_esp32_s3_v2`, 벤치·색 교정 `robot_bench_v2_manual`. 둘 다 V2 기판용이다.
- V1·TCS34725·예전 벤치·센서 점검·단독 교정 환경은 10/31에 `archive/`로 보관했다 ([archive/README.md](archive/README.md)).
  공용 모듈 안의 V1 핀(`pins.hpp`)과 TCS34725 코드는 그대로 있고, PC 시험은 V1 핀으로도 컴파일한다.
- 부팅·버튼·센서·교정·라즈 수신·안전관리·IMU 뒤집힘·FSM·모터가 연결돼 있고, PC 회귀 시험과 두 환경 빌드는 통과했다.
- 라즈 → ESP32 패킷 수신은 11/4 실물에서 확인했다(초당 50패킷, 오류 0).
- **경기용 펌웨어는 아직 보드에 올려 주행하지 않았다.** 수광·브레이크·제동거리·색 임계값·공격 전략은 미검증이다.
  빌드 성공을 실제 차량의 동작 보증으로 해석하지 않는다.

먼저 읽을 것: 구조 요약 [MAP.md](MAP.md), 규칙 [AGENTS.md](AGENTS.md), 현장 팀원용 [TEAM_GUIDE.md](TEAM_GUIDE.md),
실물 시험 [BENCH_TESTS.md](BENCH_TESTS.md), 검증 기록 [VERIFICATION.md](VERIFICATION.md).

## 1. 하드웨어 범위

지금 기체는 **V2 기판**이다. 빌드 플래그 `-D HW_VERSION=2`가 있으면 `include/pins.hpp`가 `include/pins_v2.hpp`를 불러오고,
없으면 V1 핀이다. 같은 하드웨어 버전 안에서 핀은 바꾸지 않는다.
**Wi-Fi, 외부 플래시·PSRAM, 네이티브 USB는 V1·V2 모두 사용 금지다.** 업로드와 로그는 UART0만 쓴다.

| 장치 | V2 경기용에서 쓰는 방식 |
|---|---|
| 바닥 컬러 센서 AS7341 ×4 | TCA9548A 채널 0~3, 바닥 색 판별 |
| IMU MPU6050 | I2C 본선 직결. 경기용은 뒤집힘 비상정지(TiltGuard)에만 사용 |
| 배터리 분압(×11) | 전원 투입 3초 동안 3S·4S 판정 (NVS로 켜고 끔) |
| BTS7960 ×2 | 모터 1 = 왼쪽, 2 = 오른쪽. PWM 명령 구동(엔코더·PID 없음), 모터당 EN 하나 |
| 전류 감지(IS), IR 입력 6개 | 핀만 있음, 경기용에서 쓰지 않음 (벤치 GUI 핀 상태 판에만 표시) |
| 엔코더 핀 | 사용 안 함, 초기화도 하지 않음 |
| WS2812 | 내장 LED 하나(GPIO48). V2에는 외부 LED가 없다 (GPIO13은 IR6) |

```text
             기체 전방 / 카메라
           S1(FL)       S2(FR)
           왼쪽 모터    오른쪽 모터
           S3(RL)       S4(RR)
```

S1..S4는 사용자 표시 번호다. 배열 인덱스와 TCA 채널은 0..3이다.
동일 주소 센서의 충돌을 막기 위해 매 거래 전에 TCA 채널 하나만 선택한다.

| 신호 | V2 GPIO | 의미 |
|---|---:|---|
| SW1 / SW2 | 15 / 16 | GND 연결 능동 LOW, 내부 풀업 |
| UART0 TX / RX | 43 / 44 | 업로드, 이벤트 로그(57600), 벤치 콘솔 |
| UART1 TX / RX | 17 / 18 | 라즈 UART (115200), 상대 TX→ESP RX, 공통 GND |
| I2C SDA / SCL | 11 / 12 | TCA9548A(0x70), MPU6050(0x68) |
| 왼쪽 LPWM / RPWM / EN | 42 / 41 / 40 | BTS7960 모터 1 |
| 오른쪽 LPWM / RPWM / EN | 47 / 21 / 35 | BTS7960 모터 2 |
| 배터리 ADC | 14 | 분압 ×11 |

원본은 `include/pins_v2.hpp`이며 표와 다르면 코드가 맞다. 오른쪽 EN(GPIO35)은 PSRAM 핀과 겹칠 수 있어서
메모리 타입을 `qio_qspi`로 고정했다. 전원 투입 때 튀는지 확인하는 방법과 대체 핀은 [AGENTS.md](AGENTS.md) "V2 오른쪽 모터 EN".
V1 핀(`include/pins.hpp`)은 보관된 V1 환경과 PC 시험용으로 남아 있다.

## 2. 펌웨어 환경

보드에는 아래 중 하나만 업로드한다. 여러 `setup()/loop()`를 같이 실행하지 않는다.
`platformio.ini`의 `build_src_filter`가 진입점을 나누며 공용 센서·모터 코드를 복제하지 않는다.

| 환경 | 진입 파일 | 용도 | UART0 |
|---|---|---|---:|
| `yd_esp32_s3_v2` (기본) | `src/main.cpp` | 경기용 FSM | 57600 |
| `robot_bench_v2_manual` | `src/bench_v2_main.cpp` → `src/bench_v2/` | 저출력(최대 40%) 시험·진단·12단계 색 교정 | 57600 |
| `i2c_scanner_v2` | `src/i2c_scanner_main.cpp` | I2C 주소·풀업 진단. 모터 출력 LOW 유지 | 57600 |
| `raw_motor_v2` | `src/raw_motor_main.cpp` | 두 모터를 20%로 **자동** 구동. 안전 기능 없음 | 57600 |

- 경기용은 벤치 컨트롤러와 교정 코드를 빌드하지 않는다. 벤치는 경기용 FSM을 실행하지 않는다.
- 교정값은 NVS에 저장되므로 벤치에서 교정하고 경기용을 올려도 남는다.
- `raw_motor_v2`는 바퀴를 띄우고 모터 전원을 뽑아서 멈춘다. SW1도 듣지 않는다.
- `i2c_scanner_v2`와 `raw_motor_v2`는 `upload_port = COM9`가 박혀 있으니 다른 PC에서는 고친다.

## 3. 파일을 나눈 의도와 책임

`include/경로.hpp`는 공용 API·자료형, `src/경로.cpp`는 실제 구현이다.
설정은 `constexpr` 중심이다. ISR에서는 플래그와 즉시 EN 차단만 수행한다.
모든 모듈이 각자 운전 허용 상태를 만드는 구조를 피했다.

| 파일 / 모듈 | 만든 이유와 실제 책임 |
|---|---|
| `platformio.ini` | 플랫폼/보드/라이브러리 버전 고정, 진입점 선택. 현재 8MB DevKitC, PSRAM 미사용 |
| `include/pins.hpp`, `include/pins_v2.hpp` | 배선의 유일한 코드 기준. `HW_VERSION=2`면 V2. PSRAM·USB CDC 설정이면 빌드 중단. ISR EN 마스크도 이 정의 사용 |
| `include/config.hpp` | UART 속도·신선도·버튼·LED·로그·배터리 셀 수의 공통 시간/용량 |
| `include/robot_types.hpp` | 안전·센서·비전 스냅샷, FSM 상태, 주행 의도, 모터 명령 연결 자료형 |
| `src/main.cpp` | 생산 실행 순서 연결. 레지스터/PWM 계산은 직접 구현하지 않음 |
| `app/boot_manager` | 초기 핀 안전부터 필수 모듈 초기화까지 순서 보장, 필수 모터 실패 시 구동 금지 |
| `hardware/board_io` | 부팅 초기에 PWM/EN LOW 설정, 일반 핀모드 초기화 |
| `hardware/reset_manager` | ESP 리셋 원인과 재무장 필요 플래그 제공. 자동 리셋은 실행하지 않음 |
| `interrupts/button_interrupts` | SW1 영구 래치, SW2 에지·디바운스·해제 이벤트, ESTOP 영구 루프 |
| `system/system_manager` | SW2 이벤트 단독 소비자. 운전 요청·집계 고장을 합쳐 motorAllowed 결정 |
| `system/event_logger` | 번호/문자열 고정 큐, 변경·중복 억제, 일반 UART 쓰기 지연 |
| `system/status_led` | 주체별 요청 보존, 우선순위·TTL로 WS2812 표시 결정 (V2는 내장 LED 하나) |
| `system/battery_monitor` | V2: 전원 투입 3초 동안 3S·4S 판정, 판정 전 SW2 시작 보류, 실패 시 시작 금지 |
| `system/battery_setting` | 배터리 판정 켜기/끄기 NVS 값 (벤치 `BAT CHECK ON/OFF`) |
| `sensors/imu_monitor` | V2: MPU6050 원시값 10ms마다 읽기. 융합·위치 추정 없음. 고장 시 FAULT로 멈추고 자동 재시도 없음 |
| `system/tilt_guard` | V2: 정지 중 중력 방향을 수평 기준으로 학습, 구동 중 80° 넘게 3초면 영구 비상정지 |
| `sensors/color_sensor_types` | 모델·모서리·색·원시값·교정 기준·판정 결과와 출력 이름 |
| `include/sensors/sensor_config.hpp` | 모델, 수광·게인·조명, TCA 채널, 확정 횟수, 임시 임계값, 교정 여유 |
| `sensors/color_sensor_manager` | Wire/TCA/레지스터 담당. 수광 완료 확인과 새 원시 측정 발행. 꺼진 채널은 최대 3번, 1초 간격 재초기화 |
| `sensors/floor_color_classifier` | 원시값을 네 바닥색/Unknown으로 판별. I2C·UART·모터를 건드리지 않음 |
| `sensors/calibration_store` | NVS 교정값 읽기·검증·저장·삭제. 생산은 부팅 때 읽기만 함 |
| `calibration/color_calibration` | UART 안내식 12단계 교정. 벤치 펌웨어에만 링크되고 경기용에는 없음 |
| `sensors/sensor_manager` | 새 sequence만 판별, 3회 확정, 무효/오래된 판정 폐기, 오류 변경 기록, snapshot |
| `include/communication/vision_protocol.hpp` | 패킷·상태 비트·범위·오류번호. Pi 송신기와 맞춰야 하는 계약 |
| `communication/raspberry_link` | UART1 재조립, 헤더 재동기화, 중복 sequence와 영상 신선도 처리 |
| `include/control/control_config.hpp` | 생산 출력·중심 기준·공격/탈출 시간·위험색 마스크 |
| `control/robot_fsm` | 안전/바닥/비전으로 무엇을 할지 판단하고 DriveIntent 반환. GPIO 직접 제어 없음 |
| `control/wander` | 라즈가 1초 넘게 끊겼을 때 무작위 회전·전진 호 (같은 동작 반복 = 고장 판정 실점 회피) |
| `control/drive_logic` | 의도를 좌우 MotorCommand와 80ms 기한으로 변환 |
| `include/motor/motor_config.hpp` | PWM·방향 반전·출력 상한·명령 만료·브레이크 전기 조합 |
| `motor/motor_controller` | 공용 API, 안전 검사, 제한, 뮤텍스, 만료 감시 태스크, 상태 반환 |
| `motor/bts7960_driver` | BTS7960 하나의 실제 PWM/EN 쓰기, 방향 전환 deadtime, 영구 금지 확인 |
| `bench_v2/bench_v2_app` | 벤치 진입. 콘솔·안전·센서/IMU/배터리·라즈·핀 상태 출력·교정 연결 |
| `bench_v2/software_switches` | 벤치 전용 콘솔 `SW1`/`SW2` (물리 버튼과 같은 규칙, 경기용에는 없음) |
| `bench_v2/i2c_scan` | 벤치 콘솔 `I2C SCAN` |
| `bench/bench_controller` | MOTOR/MANUAL/DATA/ALIGN/FOLLOW/AVOID의 시간 기반 명령 생성 |
| `include/bench/bench_config.hpp` | 시험 출력/유지시간/추적·회피 조건. 생산 전략 설정과 분리 |
| `tools/bench_gui.py` | PC·라즈용 벤치 GUI (시험 / 색 교정 / 라즈 카메라 탭), `tools/test_bench_gui.py`로 시험 |
| `tools/flash_from_pi.sh` | 라즈에 꽂힌 ESP32에 PC에서 빌드한 `.bin` 올리기 (원격 업로드) |
| `tests/run_host_tests.ps1` | 실제 소스를 PC 가짜 하드웨어에 연결해 컴파일/실행 (`-V2`, `-Bench`) |
| `tests/host/test_firmware.cpp` | 생산 모터·안전·FSM·UART·센서·교정·로그·메인 연결 회귀 시험 |
| `tests/host/test_bench.cpp` | 모드 시간·수동 만료·추적/회피·콘솔·벤치 메인 회귀 시험 |
| `tests/host/fake_hardware.cpp` 및 shim 헤더 | 가상 시간/GPIO/UART/I2C/NVS/태스크. 생산 알고리즘은 대체하지 않음 |

`ContactCheck`, `UndercutDrive`, `ScoreHold`, `WallEscape`, `StuckRecovery`, `Stop` 등
일부 `RobotState` 이름은 현재 생산 FSM에서 진입하지 않는 예약 상태다.
접촉·스포크 삽입·고착을 센서로 확인하는 기능이 있다고 해석하지 않는다.

## 4. 부팅과 메인 루프 워크플로우

```text
전원/리셋
  → PWM/EN LOW → 리셋 원인 → UART0 → 모터/PWM/만료 감시 태스크
  → 버튼 ISR → LED → 컬러 센서/NVS 교정값 → UART1 → 시스템 관리자
  → 배터리 판정 시작(V2, 켜져 있으면 약 3초, LED 흰색 깜빡) → IMU·TiltGuard(V2)
  → 시작 대기 (LED 초록 깜빡) → SW2 0.5초 눌렀다 떼기 → Running

경기용 loop() (src/main.cpp)
  SystemManager::update      ① 안전/버튼
  RaspberryLink::poll        ② 라즈 UART1 최대 128바이트
  SensorManager::update      ③ 센서 짧은 I2C 단계 (꺼진 채널 재시도 한 단계 포함)
  BatteryMonitor::update     ④ 전원 투입 3초 안에서만 판정
  ImuMonitor::update + TiltGuard::update   ④ 뒤집힘 3초면 triggerEmergencyStop
  SystemManager::update      ⑤ 안전 재확인 (의도된 반복)
  RobotFsm::update           ⑥ 센서·비전 snapshot → DriveIntent
  DriveLogic::makeCommand    ⑦ 좌우 출력, 80ms 기한
  MotorController::apply / brake   ⑧ 허용될 때만 출력, 상한 710
  StatusLed::update, EventLogger::process, delay(1)   ⑨ 낮은 우선순위

독립 경로
  SW1: ISR은 에지만 표시 → 메인 루프와 만료 감시 태스크가 0.5초 연속 눌림 확인
       → 영구 구동 금지 + EN LOW → 일반 문맥 브레이크 → ESTOP만 3초마다 → 재부팅 필요
  모터 만료 감시 태스크(5ms) → 마지막 명령 기한 확인 → 만료 시 브레이크
```

컬러 센서 초기화 실패는 부팅 실패로 만들지 않는다. 라즈와 IMU도 필수 부팅 장치가 아니다.
모터 초기화/감시 태스크 생성 실패는 구동을 막고(LED 빨강·흰색 교대), `BootFailed`를 한 번 기록한다.
각 `begin()`은 부팅용이며 운전 중 반복 초기화나 SW1 래치 우회용으로 호출하지 않는다.
예외는 꺼진 센서 채널의 최대 3번 재초기화뿐이다(11/08 결정).

## 5. 안전 조건과 의도된 예외

`SystemManager`는 **구동해도 되는지**, `RobotFsm`은 허용된 상태에서 **어떻게 움직일지** 결정한다.

| 상황 | 생산 펌웨어 | 벤치 펌웨어 |
|---|---|---|
| 전원 직후 | SW2 대기, 무구동 | STOP 모드, 무구동 |
| 부팅부터 SW2 눌림 | 첫 해제 무시, 새 눌림/해제 필요 | 동일 |
| SW2 0.5초 이상 눌림/해제 | Running, FSM 시작 | 선택 모드 허용 |
| 0.5초 미만 버튼 눌림 | 무시 (충격으로 눌린 것) | 동일 |
| 구동 중 SW2 0.5초 눌림 | 영구 정지 (SW1과 같음) | 동일 (소프트웨어 SW2는 구동 중 거부) |
| STOP/모드 변경 | API로 운전 요청 폐기 | STOP/MODE 콘솔, 새 SW2 필요 |
| 정지 전에 누른 SW2 해제 | 출발에 사용하지 않음 | 동일 |
| SW1 0.5초 눌림 | 영구 정지, 해제해도 재부팅 전 금지 | 동일 |
| 구동 중 출발 자세에서 80° 넘게 기운 채 3초 (V2 IMU) | 영구 정지 (SW1과 같음, 11/07) | 해당 없음 |
| IMU 없음/고장/값 정지 | 뒤집힘 판정만 안 함, 주행 계속 | 해당 없음 |
| 라즈 없음/영상 오래됨 | 1초 넘으면 무작위 순회(Wander), 짧은 공격은 지속 가능 | 추적 정지; AVOID 진행 중 탈출은 우선 |
| 센서 하나/전부 없음 | Unknown, 주행 시작 허용 | AVOID는 정지, 나머지 주행 모드는 가능 |
| 색 변경 후 3회 확정 중 | Unknown은 위험색이 아님 | AVOID는 Unknown 때문에 정지 |
| 센서 I2C/변환 고장 | 채널 제외, 1초 간격 최대 3번 재초기화, 그래도 안 되면 재부팅 전까지 꺼짐 | 동일 |
| 무수광/포화 | 그 측정 무효, 채널 유지 | 동일 |
| 모터/애플리케이션 집계 고장 | 즉시 정지·운전 요청/버튼 제스처 폐기 | 동일 |
| 명령 만료 | 브레이크, 새 유효 명령은 적용 가능 | 동일 |

생산에서 Unknown 전체를 정지로 바꾸지 않은 것은 센서 부팅 실패에도 진행하라는 요구에 따른다.
따라서 생산은 센서 고장 시 바닥 회피를 보장하지 않는다. 시험 AVOID는 의도적으로 더 보수적이다.
`setSensorHealthy(false)` 집계 API는 있지만 개별 컬러 센서 실패에 자동 연결하지 않는다.
하드웨어 제동/출력 풀다운/전원 상실 중 안전은 소프트웨어만으로 보장하지 않는다.

## 6. 생산 FSM이 실제로 하는 일

```text
WaitStart → Search → TargetConfirm → TrackAlign / Approach
                                  → DiveReady(돌진 220ms)
                                  → DiveBrake(90ms)
                                  → SpokeEngaged(전진 350ms) → Push
                                                              ↓ 목표 상실
                                                  Backoff(260ms) → Search
어느 구동 단계든 확인된 위험색 → ZoneEscape
운전 금지 → WaitStart / EmergencyStop
```

- Search: 300 천분율 제자리 회전, 1.2초마다 방향 변경. 첫 방향은 타워를 마지막으로 본 쪽(모르면 왼쪽).
  (11/04 벤치: 250 미만은 제자리 회전이 안 되고 260은 한 바퀴만 돌아 앞뒤로 흐름 → 회전 출력은 300 이상)
- TargetConfirm: 서로 다른 새 목표 프레임 2회 필요. 확인 중에도 중앙이면 300으로 접근,
  벗어나면 회전한다. 정지한 채로만 확인하지는 않는다.
- TrackAlign: 중심 X=960, 오차 ±200 밖이면 회전. 출력은 창 바로 밖 280 → 700px 이상 380으로 비례
  (11/05, 벤치 ALIGN에서 ±100 고정 출력은 좌우로 흔들림). 중앙이면 접근450과 비례 조향.
- 공격 진입: 확인 후 X 오차 ±150 이내, 박스 높이420 이상이면 시간 기반 공격 시작.
  박스 높이 420은 예전 카메라 가정값이라 실제 카메라로 공격 거리에서 재야 한다.
- DiveReady→DiveBrake→SpokeEngaged: 650 돌진, 브레이크, 480 삽입 전진.
  짧은 카메라 가림에도 진행하지만 SW1/위험색/운전 금지는 먼저 처리한다.
- Push: 650 전진과 작은 조향. 보이는 박스 높이210 미만 또는 X 오차600 초과이면 재확인.
  상실 시 마지막 **새 목표 프레임 시각**부터 500ms 이내만 직진 밀기 후 후진한다.
  앞의 공격이 시간을 소비했다면 유지 시간이 짧아지거나 바로 후진할 수 있다.
- 일반 목표 상실: 마지막 새 목표부터 1.5초 동안 마지막 방향 회전, 이후 Search(같은 쪽부터).
- Wander (11/07): 쓸 수 있는 라즈 프레임이 1초(`config::wander::VISION_LOST_MS`) 넘게 없으면 제자리 회전 대신
  회전(300~380, 0.2~0.7초, 방향 무작위)과 전진 호(380~480, 조향 ±90, 0.3~0.9초)를 번갈아 하고 길이·출력·방향을
  매번 무작위로 바꾼다(같은 동작 반복은 고장 판정 실점). 위험색 회피와 운전 금지가 항상 먼저다. 라즈가 살아
  있고 타워만 안 보이면 기존 Search다. 값은 실물 미검증 시작값이다.
- 뒤집힘 (11/07, V2): `TiltGuard`가 정지 중 IMU 중력 방향을 수평 기준으로 배우고, 구동 중 80° 넘게 기운
  상태가 3초 이어지면 `SystemManager::triggerEmergencyStop()`으로 SW1과 같은 영구 정지에 들어간다.
  장착 방향과 무관하다. IMU 데이터가 없으면 판정하지 않는다. 들어 올리기 공격이 3초를 넘기지 않는지 실물 확인 필요.
- 바닥 접촉: 센서가 바닥에 닿아 어둡거나 포화되면 측정이 무효가 되어 색이 바로 지워진다. 직전에
  위험색이던 센서는 HAZARD_CONTACT_HOLD_MS(300ms, 실측 전) 동안 위험으로 유지한다. 처음부터 어두운
  채로 위험구역에 들어가면 볼 수 없다(센서 높이·마운트로 해결해야 함).
- ZoneEscape: 빨강/파랑 위험 (노랑은 위험 아님, 11/04). 전방이면360ms 후진, 후방이면 전진,
  이어 반대 방향 회전320ms. 여전히 위험하면 탈출 재판단.
  이동 방향에 새 위험이면 브레이크 한 번 후 방향을 새로 판단한다.
- 홈 안 갇힘(전후방 동시 위험): BOTH_HAZARD_BRAKE_MS 동안 브레이크 후, 마지막으로 한쪽 끝만
  위험색이었던 방향의 반대로 ZONE_ESCAPE_DUTY 직진한다(뒤가 마지막이면 전진, 앞이면 후진).
  기록이 없으면 Backoff 중 진입은 전진, 그 외는 후진. GROOVE_EXIT_FLIP_MS마다 방향을 뒤집는다.
  한쪽 끝이 빠지면 일반 ZoneEscape로 이어간다. 시간 값은 실물 미검증 시작값이다.
  회전 단계는 현재 위험 방향을 반영하며, 위험이 없으면 짧은 예정 탈출 후 Search로 돌아간다.

박스 높이는 거리계가 아니며 스포크 삽입 성공을 확인하지 않는다.
`trackStable/multipleTargets/boxClipped`는 수신하지만 생산 공격을 모두 차단하는 조건은 아니다.
벤치 FOLLOW는 중앙 근처 잘린 박스에서 전진을 멈춘다. 전략의 세부 튜닝은 실물에서 진행한다.

## 7. 센서 수광과 새 측정 구분

네 센서는 순서대로 방문하지만 수광 자체는 독립적이다. 수광 시간을 delay로 기다리지 않는다.
각 Wire 거래는 동기식이고 최대5ms 타임아웃이 있으므로 전체 처리가 완전히 무대기는 아니다.
기본 I2C는400kHz다.

AS7341은 부팅 때 F2/F3/F5/F6/F7/Clear를 ADC6개에 연결하고 연속 수광한다.
완료 AINT를 확인·해제한 뒤 ASTATUS부터 버스트 읽기한다. AVALID만 계속 읽어 동일 결과를
새 측정으로 세지 않는다. 최소 수광 간격도 확인한다. F1/F4/F8/NIR는 측정하지 않는다.
공통10칸 배열의 미사용 칸은0, 유색 featureMask는0x76, Clear는 밝기와 비율의 분모다.

TCS34725는 AEN 켜기→수광 대기 상태→유효 RGBC 읽기→AEN 끄기로 측정한다.
코드 INTEGRATION_MS는2.4ms를 올림한3ms이고 읽기 전1ms를 더 기다린다.
한 센서씩 최소1ms 슬롯에 방문하므로 재방문은 이상적으로4ms다.
TCS의 3회 확정 지연을 단순히2.4ms×3으로 계산하면 안 된다.

원시 sequence는 새 결과 발행 때 증가한다. SensorManager는 새 sequence만 판별하고
같은 색3회를 확인한다. 같은 snapshot을100번 읽어도 횟수는 늘지 않는다.
색 변경·무효·오래된 값은 이전 reliable 판정을 취소한다. 교정 변경도 확정 필터를 초기화한다.

readyMask는 통신 가능한 채널이며 색 신뢰도가 아니다. raw.valid, floor.reliable,
consecutiveMatches, timestampMs를 구분해서 읽는다.
오류번호0=None,1=Initialization,2=I2c,3=Timeout,4=Range.
I2C/Timeout은 채널 제외, Range는 현재 결과만 무효다.

## 8. 바닥센서 조정 — 2026-10-05

모든 값은 `include/sensors/sensor_config.hpp`에서 수정 후 다시 빌드한다.
자동 게인·자동 노출은 없다. 통신이 끊긴 센서는 최대 3번, 1초에 한 번만 다시 초기화한다(11/08).

```cpp
#define COLOR_SENSOR_MODEL COLOR_SENSOR_AS7341
// TCS 차량이면 위 값을 COLOR_SENSOR_TCS34725로 변경
```

TCS 전용 환경은 빌드 옵션 `COLOR_SENSOR_MODEL=2`로 기본값을 덮어쓴다.

| 조정 항목 | 설정 / 현재 시작값 |
|---|---|
| AS 수광 | AS7341_EXPOSURE_US=5000, ATIME=0/ASTEP=1797, 약4998.44us |
| AS 게인 | AS7341_GAIN=X16, 선택 X0.5..X512 |
| AS 자체 LED | AS7341_LED_ENABLED=true, LED_CURRENT_MA={20,20,20,20} |
| AS LED 전류 | 4..150mA 짝수, 모듈 LED가 AS LDR에 연결된 경우만 적용 |
| AS 내부 보정 | AS7341_AUTOZERO_INTERVAL=255, 첫 수광 전 보정/시작 지연 |
| TCS 수광 | TCS34725_INTEGRATION=Ms2_4, 선택2.4/24/50/101/154/614ms 계열 |
| TCS 게인 | TCS34725_GAIN=X16, 선택 X1/X4/X16/X60 |
| TCS 조명 | 별도 LED 제어 API 없음, 모듈 배선 확인 필요 |
| 확인 횟수 | COLOR_CONFIRM_COUNT=3, 3 미만 빌드 거부 |
| 신선도 | SAMPLE_STALE_MS, AS255ms / TCS253ms 기본 |
| 무교정 검정 한계 | 센서별 AS Clear180 / TCS Clear120 배열 |
| 무교정 유색 | 빨강 우세1.20, 파랑1.18, 노랑/파랑1.30, R/G0.55..1.80 |
| 교정 여유 | DEFAULT_RATIO_MARGIN_PERMILLE=150 / BRIGHTNESS=300 =15/30% |
| 교정 매칭 | 최소 비율폭.005, 최대점수1.0, 타색 점수차.08 |
| 밝기 가중치 | MATCH_COLOR_BRIGHTNESS_WEIGHT=.20 / BLACK=.55 |

무교정은 Clear 검정 한계 이하부터 검정으로 보고 나머지는 비율로 유색 판별, 애매하면 Unknown이다.
교정은 센서별 각 색의 세 자세 중 가까운 기준을 선택한다. 비율 허용 폭은 평균×여유,
3×표준편차, 최소폭 중 큰 값이다. 밝기는 min/max에 여유를 더한다.
타색과 너무 비슷하거나 점수가 나쁘면 Unknown이다. 같은 색의 다른 자세는 경쟁 색으로 세지 않는다.

### 반드시 유지할 실제 기체 조건

- 추가 가림막/구조물 없이 로봇 밑 그늘에서 자체 LED로 바닥을 비춘다.
- 10mm 단차 때문에 센서 간격은 근접3mm / 내려간 상태10mm / 걸친 상태20mm다.
  20mm 조건을 제거하거나 고정 높이로 가정하지 않는다.
- 3mm 포화, 20mm 약한 유색의 검정 오인, 경계 혼합색을 각각 확인한다.
- V1은 높이를 측정하지 않는다. matchedPose는 가까운 교정 기준이며 실제 높이/IMU 측정값이 아니다.
- 검정 바닥·공중·상대 로봇 표면을 항상 구분할 수 없다.
- 다음 개발자도 이 조건과 미검증 사항을 사용자에게 짚어준다.

조정 순서: 원시값→3mm 포화 방지→20mm 유색 신호 확인→필요시 수광 연장→세 높이 교정
→낮은 출력 경계 시험. 수광/게인/조명/채널 변경은 설정 서명을 바꾸므로 이전 NVS 교정 대신
임시 임계값을 사용한다. 자동 NVS 재작성이나 재교정은 하지 않는다.

## 9. 컬러 교정 사용법

교정은 **벤치 펌웨어** `robot_bench_v2_manual` 안에서 한다. 경기용에는 교정 콘솔이 없고 부팅 때 NVS 값만 읽는다.
보통은 `tools/bench_gui.py`의 "색 교정" 탭이 아래 명령을 대신 보내고 단계마다 안내한다.
교정 중에는 모든 콘솔 줄이 교정으로 가고(`MODE`·`DRIVE` 무시), 모터는 매 루프 브레이크, 시험 로그는 멈추며
250ms마다 `CAL,…` 상태 줄을 낸다. 그래도 모터 전원은 분리하는 것을 권한다. 네 채널 모두 준비돼야 시작한다.

```text
CAL START
READY      ← 배치 안내에 맞춰 놓고 보냄. 카운트다운 1초 + 측정 1초
ACCEPT     ← 결과 수락, 다음 단계
...        ← 12단계 반복
CAL SAVE   ← NVS 저장. SensorManager가 새 값을 바로 씀
```

순서는 검정 → 빨강 → 파랑, 색마다 평평 10mm(4개 동시) → 들림 20mm(4개 동시) → 눌림 3mm 앞 1·2 → 뒤 3·4.
노랑은 교정하지 않는다(단차일 뿐 위험구역이 아님, 11/04 결정). 높이는 측정하지 않고 사람이 맞춘다.
IMU는 교정에 쓰지 않는다. 유효한 새 측정으로 평균·표준편차·Clear min/max를 구한다.
GUI "색 교정" 탭의 센서별 스펙트럼 막대로 포화(3mm)와 어두움(20mm)을 눈으로 확인할 수 있다.

| 명령 | 조건 / 의미 |
|---|---|
| CAL START | 새 교정 시작, 기존 저장 기준은 SAVE 전까지 유지 |
| READY | 배치 완료 후 준비/측정 |
| ACCEPT | 결과 수락, 다음 단계 |
| RETRY | 현재 결과 폐기, 다시 READY 대기 |
| BACK | 배치 대기/결과 확인 중 이전 단계 |
| STATUS / CAL STATUS | 상태·센서·색·자세·샘플 수·여유 표시 (CAL STATUS는 저장 여부만, 모터 안 멈춤) |
| CAL MARGIN 20 40 | 교정 중 비율 20% / 밝기 40% 여유, 두 정수 1..100만 허용 |
| ABORT | 작업 중단, 기존 저장 교정 유지 |
| CAL SAVE | 전체 완료 후 NVS 저장, 실패는 오류 후 수동 재시도 가능 |
| CAL ERASE | 저장 기준 삭제, 다음 부팅부터 임시 기준 사용 |

측정 시간 안에 유효 샘플이 모자라면 그 단계를 다시 안내한다. NVS `color-cal/data` 한 블롭의 길이·버전·모델·
설정 서명·유한값을 확인한다(노랑 칸은 건너뜀). 저장/삭제 실패 시 기존 RAM 기준을 보존한다.
SAVE 전 중간 측정은 재부팅하면 사라진다. 수광·게인·LED 전류·채널을 바꾸면 설정 서명이 달라져 이전 교정은
무효가 되고 임시 임계값으로 동작한다. 바퀴를 바꿔 차체 높이가 달라지거나 센서를 교체하면 다시 교정한다.

## 10. 라즈 UART 규격과 수신 의미

UART1 115200/8N1, CRC 없음, 고정14바이트, 16bit는 little-endian이다.
좌표는1920×1080 기준. 다른 영상 크기는 송신 측에서 변환해야 한다.

| 바이트 | 내용 |
|---:|---|
| 0..3 | AA 55 01 0A: 헤더/버전/payload 길이 |
| 4 | 새 처리 프레임 sequence, 255→0 순환 |
| 5 | 상태 비트 |
| 6..7 / 8..9 | 중심 X/Y, 0..1919 / 0..1079 |
| 10..11 / 12..13 | 박스 폭/높이, 목표 시1..1920 / 1..1080 |

bit0=TargetValid,1=TrackStable,2=MultipleTargets,3=BoxClipped,4=CameraOk,5=PipelineOk.
bit6/7은0. 목표 유효면 camera/pipeline도 유효해야 한다.
목표가 없으면 좌표/폭/높이와 bit1/2/3도0. 정상 카메라에서 목표만 없으면 flags0x30이다.

sequence1, flags0x31, 중심(960,540), 폭100/높이200 예:

```text
AA 55 01 0A 01 31 C0 03 1C 02 64 00 C8 00
```

한 루프 최대128바이트, 손상 후보 안의 다음 헤더 보존, 부분 입력20ms 끊김은 폐기.
정상 패킷/새 영상 수명은 각각120ms다. 동일 sequence/내용 재전송은 링크만 갱신하고
영상 수명을 늘리지 않는다. 같은 sequence/다른 내용은 오류다.
링크 끊김 후 첫 정상 패킷은 새 기준으로 받아 라즈 재부팅/sequence 초기화를 허용한다.
sequence 차이 크기로 역순/손실을 엄격 판정하지는 않는다.

50Hz는 송신 측 요구다. ESP가 그 주기를 만들거나 재전송을 요청하지 않는다.
14×50×10bit=7000bit/s로115200bps 대역 안이다. CRC가 없으므로 정상 형식의 비트 손상은
검출 보장이 없다. 송신기는 `scripts/pi_vision_sender.py`(+ `vision_packet.py`)이고 11/4 수신을 확인했다.
예전 `pi_green_tracker.py --json` 출력은 패킷이 아니므로 UART1에 그대로 넣으면 동작하지 않는다.

## 11. 공용 모터 API

일반 태스크/루프에서 MotorController를 사용하고 ISR에서 호출하지 않는다.
setTank는 좌우 독립, setArcade의 양의 steering은 왼쪽 출력 증가=오른쪽 선회다.
방향 반전은 motor_config.hpp에서만 설정한다.

```cpp
#include "motor/motor_controller.hpp"
#include "system/system_manager.hpp"

// 기존 부팅과 안전 update가 실행되는 일반 루프 안에서:
const SafetyStatus safety = SystemManager::safetyStatus();
MotorController::setTank(150, 150, safety); // 15%, 기본100ms
const MotorControllerStatus motor = MotorController::status();
// appliedLeftPermille: 제한 적용 후 논리 출력
// lastResult: Accepted / SafetyBlocked / CommandExpired 등
```

이 코드를 생산 main에 덧붙이면 FSM과 경쟁한다. 실행 모드당 명령 생성자가 한 곳이 되게 연결한다.
caller가 임의 Running 안전 스냅샷을 꾸며서 우회하지 않는다.

- 단위 -1000..1000 천분율, 생산 기본±710(11/08, 밀기 650+조향 60), 벤치±400.
- setOutputLimit 변경은 기존 출력을 정지한다. 새 명령이 필요하다.
- setTank/setArcade/forward, apply(FSM 명령), stop/brake, status/outputLimit 제공 (reverse·rotate·coast는 11/08 삭제).
- timeoutMs=0은 기본100ms. INT32_MAX 초과는 InvalidArgument와 브레이크로 거부한다.
- FSM/벤치 기한80ms. 직접 기한을 만들면 미래 간격은 INT32_MAX 이하로 사용한다.
  millis 순환으로 기한0이 된 경우도 실제 명시 기한으로 처리한다.
- 만료는 정지일 뿐 보드 리셋/영구 고장이 아니다. 다음 새 유효 명령은 적용 가능하다.
- requested는 마지막 요청, applied는 제한/정지 후 값. 정지 후에도 요청값은 남을 수 있다.
  outputsActive=false는 구동 명령이 없다는 뜻이지 브레이크 EN도 반드시 LOW라는 뜻은 아니다.
- 모터 API는 뮤텍스 보호. sensor/FSM/logger/system은 현재 메인 문맥용으로
  추후 여러 태스크에서 자동으로 thread-safe하다고 가정하면 안 된다.

PWM20kHz(50us)/8bit(0..255), 방향 전환 deadtime100us.
기본 브레이크 BothPwmLowEnableHigh는 실측 전 선택값이다.
SW1 ISR EN 차단 후 일반 문맥에서 선택한 브레이크를 적용·유지한다.
clearEmergencyStop은 일반 래치만 안전 조건에서 해제하며 실제 SW1 영구 래치는 해제하지 못한다.

## 12. 로그와 LED

일반 UART0 57600 로그 형식:

```text
EV,시간ms,이벤트번호,심각도,소스,value1,value2[,문자열]
```

심각도 D/I/W/E/F, 소스 SYS/STATE/SAFE/MOTOR/SENSOR/COM/USER.
100번대 부팅/시스템, 200 상태/모드, 300 통신, 400 센서, 500 모터, 600 안전, 900 사용자.
같은 번호·소스·value1·문자열은 기본 1초 제한. value2는 반복 키에서 제외한다.
상태/모드/오류는 변경 API로 바뀔 때만 기록. 큐 32개, 한 번 최대 2개, 송신 공간 부족은
다음 루프로 미룬다. 큐가 차면 드롭, `status().dropped`로 확인하며 주행을 막지 않는다.

```cpp
#include "system/event_logger.hpp"
#include "system/status_led.hpp"

EventLogger::log(EventId::InvalidPacket, 6);
EventLogger::logText(900, LogSeverity::Info, LogSource::User, "test_begin");
EventLogger::faultChanged(LogSource::Sensor, FaultCode::Range, true, 1);
StatusLed::setBoth(LedOwner::User, LedState::Tracking, 500);
```

LED 요청은 즉시 WS2812를 쓰지 않고 `update()`에서 최소 20ms마다 반영한다. 주체별로 요청 하나씩 유지하고,
높은 우선순위가 이기며 같은 우선순위는 최신 요청, `clear(owner)`는 그 주체만 지운다.
TTL=0은 계속 유지, 짧은 TTL 만료 후 다른 요청이 다시 표시된다. V2는 내장 LED 하나에만 출력한다.

| 우선순위 | 상태 | 색 · 깜빡임 | 뜻 |
|---:|---|---|---|
| 255 | EmergencyStop | 빨강 계속 | SW1 0.5초, 구동 중 SW2 0.5초, 뒤집힘 80° 3초. 재부팅해야 풀림 |
| 240 | Fault | 빨강↔흰색 교대 (150ms) | 모터 드라이버·부팅 실패, 배터리 판정 실패(켜져 있을 때) |
| 200 | EscapeBlue / EscapeRed | 파랑 / 빨강 빠른 깜빡 (120ms) | 홈 탈출 중. 처음 본 바닥 색, 탈출이 끝날 때까지 유지 |
| 150 | Attacking | 보라 계속 | 돌진 → 브레이크 → 삽입 → 밀기 → 후진 |
| 120 | Tracking | 노랑 계속 | 타워 확인 → 정렬 → 접근 |
| 100 | Searching | 노랑 깜빡 (400ms) | 탐색, 놓친 직후 회전, 무작위 순회 |
| 70 | Ready | 초록 계속 | 구동 허용 (경기용은 출발 순간에만 잠깐) |
| 50 | WaitingForStart | 초록 깜빡 (700ms) | SW2 기다림. 벤치에서 색 교정 중에도 이 색 |
| 40 | Booting | 흰색 깜빡 (700ms) | 부팅·배터리 판정 중. SW2 무시 |

LED로 알 수 없는 것: 센서 하나 끊김(그 모서리만 Unknown), 라즈 끊김(찾는 중과 같은 노랑 깜빡),
IMU 고장(뒤집힘 판정만 꺼짐). 벤치 GUI의 센서·라즈·IMU 칸이나 로그로 확인한다.
CAL 안내와 벤치 ACK/HELP는 사람이 요청한 동기 콘솔이며 일반 이벤트 큐와 다르다.
SW1 영구 루프 진입 후 일반 큐를 버리고 ESTOP만 3초마다 출력한다.

## 13. 현재 주기와 스케줄링

| 처리 | 실행 방식 | 시간의 의미 |
|---|---|---|
| Arduino loop | 기본 loopTask 우선순위1, 코어는 Arduino 설정 | 매 반복 delay(1), 전체1kHz 보장은 아님 |
| 버튼 | ISR 에지 + 루프/태스크 확인 | 모든 버튼 0.5초 유지, SW2 눌림/해제 디바운스 각각40ms |
| 모터 만료 | FreeRTOS 태스크 우선순위2 | 5ms 간격, 실제 제동 지연은 실측 |
| 센서 방문 | 메인 최소1ms 슬롯 | AS 매번4개, TCS1개. 샘플 Hz와 다름 |
| AS 수광 | 내부 연속 약5ms | 이론 약200회/s, I2C/보정/누락으로 실제 낮아질 수 있음 |
| UART1 | 메인 제한 처리 | 송신 목표50Hz, 프레임 신선도120ms |
| FSM/명령 생성 | 메인 반복마다 | 별도 고정주기 태스크 아님 |
| LED | 최소20ms | 최대 갱신 시도50Hz, 변경색만 전송 |
| 이벤트 | 루프 끝 최대2개 | 사건/변경 시 기록, 낮은 처리 순서 |
| 벤치 데이터 | 100ms마다5행 시도 | 최대10Hz, 80ms 내 못 보내면 프레임 생략 |
| IMU 읽기 (V2) | 메인, 10ms마다 | 원시값만, 융합·위치 추정 없음 |
| 센서 재시도 | 메인, 한 번에 한 단계 | 켜기 → 5ms 뒤 설정, 채널당 1초 간격 최대 3번 |

FreeRTOS는 Arduino 내부에도 사용된다. 직접 분리한 기능 태스크는 모터 만료뿐이다.
I2C·FSM·UART·로그가 모두 독립 태스크라는 의미는 아니다.
I2C가 밀리면 만료 감시가 모터를 정지할 수 있고 다음 새 명령으로 다시 적용된다.
0.5m/s에서 순수5ms 수광 이동은2.5mm다. 색 변경 직후 혼합된 첫 결과를 버리는 이상적 모델의
3회 확정은 약20ms/10mm이지만 최대 반응거리 보장이 아니다.
초기 보정·폴링/읽기 지연·측정 누락·경계 혼합·제동거리는 별도다.

## 14. 빌드·업로드·실물 시험 순서

PlatformIO CLI 또는 VS Code PlatformIO가 필요하다. 플랫폼 espressif32 6.11.0,
Arduino core 2.0.17, NeoPixel 1.15.5. 빌드 폴더는 한글 경로 문제 때문에 `%USERPROFILE%/.platformio/build/c_robot_current`.

```powershell
cd D:\C_ROBOT\ESP_MAIN
pio run -e robot_bench_v2_manual -t upload
pio run -e yd_esp32_s3_v2 -t upload
pio device monitor -b 57600
# 필요하면 upload에 --upload-port COM번호, monitor에 -p COM번호 추가
```

실물 첫 시험 순서 (건너뛰지 않는다):

1. 모터 전원 분리 → 벤치 업로드 → GUI 연결. 배터리 값, 센서 1–4 응답, IMU, 핀 상태 판 확인.
2. SW1 0.5초 → ESTOP, 놓아도 복구 안 됨 확인. 재부팅.
3. 바퀴 띄우기 → "주행 준비" → 방향키로 좌우 정역, 저출력부터. 드라이버·커넥터 발열 확인.
4. 12단계 색 교정 → 저장 → "색 확인하러 가기"로 네 센서 색 확인 (3mm 포화, 20mm 파랑↔검정, 경계 혼합).
5. 라즈 연결 → `MODE ALIGN`(또는 SW2 3초) → `MODE FOLLOW` → `MODE AVOID`. 앞의 두 모드는 바닥 보호가 없다.
6. 경기용 업로드 → 바퀴 띄우고 LED 흰색→초록 깜빡, SW2 출발, SW1 정지 확인 → 전원·리셋 때 바퀴가 움찔하지 않는지.
7. 바닥에서 홈 회피 → 탐색 → 밀기 → 리허설. 출력과 시간은 `control_config.hpp`에서 고친다.

벤치 명령, CSV 형식, 각 모드 조건, GUI 사용법은 [BENCH_TESTS.md](BENCH_TESTS.md)의 "V2 전용 실물 시험" 절.
문제가 생기면 GUI가 `ESP_MAIN/logs/`에 남긴 세션 로그를 본다.

## 15. 검증 재실행과 한계

```powershell
cd D:\C_ROBOT\ESP_MAIN
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1 -V2
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1 -Bench
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1 -Demo
python tools/test_bench_gui.py
pio run -e yd_esp32_s3_v2
pio run -e robot_bench_v2_manual
cd D:\C_ROBOT
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

PC 펌웨어 시험은 PATH의 native g++ 또는 PlatformIO toolchain-gccmingw32가 필요하다.
실제 `setup()/loop()`와 모듈을 연결해 시간 순환·채터링·ISR 경합·UART 무작위 입력·센서 고장과 재시도·
교정 저장 실패·만료·뒤집힘·무작위 순회·벤치 콘솔을 시험한다. `.pio/host-tests` 결과는 Git에서 제외한다.
`-Demo`는 공격 상태와 좌우 출력을 가상 입력으로 보여주는 텍스트 데모다.

최신(11/08): V2 18,516·18,051회 검사, V1 18,490회, 벤치 1,026 + 4회, 두 환경 빌드 성공.
반복 assert는 독립 시나리오 개수가 아니다. GUI·실제 카메라·ESP 동시성·광학/전기 성능은
이 시험으로 증명하지 않는다.

상세 검토/실물 목록은 [VERIFICATION.md](VERIFICATION.md), 다음 개발 기준은
[AGENTS.md](AGENTS.md), 라즈 쪽 설명은 [루트 README](../README.md)를 읽는다.
공격 전략의 세부 루틴은 실제 차량에서 튜닝하는 다음 단계다.
