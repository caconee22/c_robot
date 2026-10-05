# ESP_MAIN — V1 펌웨어 구조와 사용법

ESP32-S3 / Arduino / PlatformIO 기반 씨름 로봇 제어 코드다.
이 문서는 작업 이력이 아니라 **모듈을 나눈 의도, 실행 순서, 설정 방법과 사용법**을 설명한다.
최신 검토 기준은 2026-10-05다. 부팅·버튼·센서·교정·비전 수신·안전관리·FSM·모터가 연결돼 있다.
생산/벤치 코드의 PC 시험과 여섯 ESP32 빌드는 통과했지만 실제 수광·브레이크·제동거리·
색 임계값·공격 전략은 미검증이다. 빌드 성공을 실제 차량의 완전한 동작 보증으로 해석하지 않는다.

중요한 연결 공백: 저장소의 라즈 영상 프로그램은 현재 화면/텍스트/JSON 출력만 한다.
**ESP32용 14바이트 UART 송신기는 아직 연결되지 않았다.** ESP32 수신부 구현과
카메라부터 모터까지의 종단간 동작 완료는 다르다.

## 1. 하드웨어 범위

현재 코드는 **V1**이다. PCBV2 핀을 적용하지 않는다.
네 모서리 컬러 센서만 사용하며 IR·IMU·전류·엔코더·배터리 ADC는 없다.
한 차량에 AS7341 네 개 또는 TCS34725 네 개를 사용한다. 두 종류를 동시에 섞는 구성은 아니다.
모터 1은 왼쪽, 2는 오른쪽이며 엔코더 속도/PID가 아닌 PWM 명령으로 구동한다.

```text
             기체 전방 / 카메라
           S1(FL)       S2(FR)
           왼쪽 모터    오른쪽 모터
           S3(RL)       S4(RR)
```

S1..S4는 사용자 표시 번호다. 배열 인덱스와 TCA 기본 채널은 0..3이다.
동일 주소 센서의 충돌을 막기 위해 매 거래 전에 TCA 채널 하나만 선택한다.

| 신호 | GPIO | 의미 |
|---|---:|---|
| SW1 / SW2 | 15 / 16 | GND 연결 능동 LOW, 내부 풀업 |
| UART0 TX / RX | 43 / 44 | 이벤트·교정 또는 벤치 콘솔, COM/SiK 공유 |
| UART1 TX / RX | 17 / 18 | 라즈 UART, 상대 TX→ESP RX, 공통 GND |
| I2C SDA / SCL | 11 / 12 | TCA9548A 기본 0x70 |
| 내장 / 외부 WS2812 | 48 / 13 | 각각 1개 |
| 왼쪽 LPWM / RPWM | 42 / 41 | BTS7960 Motor 1 |
| 왼쪽 LEN / REN | 40 / 39 | V1 독립 EN 핀 |
| 오른쪽 LPWM / RPWM | 38 / 47 | BTS7960 Motor 2 |
| 오른쪽 LEN / REN | 21 / 14 | V1 독립 EN 핀 |

`include/pins.hpp`는 커넥터 순번이 아니라 Arduino에 직접 넣는 GPIO 번호다.
UART0 RX에 COM/SiK의 TX 두 개를 동시에 구동하지 않는다. USB CDC 대신 GPIO UART0를 사용한다.

## 2. 세 종류의 펌웨어를 분리한 이유

보드에는 아래 중 하나만 업로드한다. 여러 `setup()/loop()`를 같이 실행하지 않는다.

| PlatformIO 환경 | 진입 파일 | 용도 | UART0 |
|---|---|---|---:|
| `yd_esp32_s3` | `src/main.cpp` | 설정 파일의 센서로 생산 FSM | 57600 |
| `yd_esp32_s3_tcs34725` | 같은 파일 | TCS34725 강제 지정 생산 FSM | 57600 |
| `robot_bench` | `src/bench_main.cpp` | 저출력 모터·수동·데이터·비전 시험 | 57600 |
| `robot_bench_tcs34725` | 같은 파일 | TCS34725 벤치 | 57600 |
| `sensor_serial_test` | `src/sensor_serial_test.cpp` | 설정 파일의 센서 원시값만 확인 | 115200 |
| `sensor_serial_test_tcs34725` | 같은 파일 | TCS34725 원시값만 확인 | 115200 |

생산 환경은 벤치 컨트롤러를 빌드하지 않는다. 벤치는 생산 FSM 메인을 실행하지 않는다.
센서 전용은 모터·버튼 ISR·시스템 안전관리도 실행하지 않는다.
**센서 전용 시험에서는 모터 전원을 물리적으로 분리한다.**
`platformio.ini`의 source filter가 진입점을 나누며 공용 센서/모터 코드를 복제하지 않는다.

## 3. 파일을 나눈 의도와 책임

`include/경로.hpp`는 공용 API·자료형, `src/경로.cpp`는 실제 구현이다.
설정은 `constexpr` 중심이다. ISR에서는 플래그와 즉시 EN 차단만 수행한다.
모든 모듈이 각자 운전 허용 상태를 만드는 구조를 피했다.

| 파일 / 모듈 | 만든 이유와 실제 책임 |
|---|---|
| `platformio.ini` | 플랫폼/보드/라이브러리 버전 고정, 진입점 선택. 현재 8MB DevKitC, PSRAM 미사용 |
| `include/pins.hpp` | V1 배선의 유일한 코드 기준. ISR EN 마스크도 이 정의 사용 |
| `include/config.hpp` | UART 속도·신선도·버튼·LED·로그의 공통 시간/용량 |
| `include/robot_types.hpp` | 안전·센서·비전 스냅샷, FSM 상태, 주행 의도, 모터 명령 연결 자료형 |
| `src/main.cpp` | 생산 실행 순서 연결. 레지스터/PWM 계산은 직접 구현하지 않음 |
| `app/boot_manager` | 초기 핀 안전부터 필수 모듈 초기화까지 순서 보장, 필수 모터 실패 시 구동 금지 |
| `hardware/board_io` | 부팅 초기에 PWM/EN LOW 설정, 일반 핀모드 초기화 |
| `hardware/reset_manager` | ESP 리셋 원인과 재무장 필요 플래그 제공. 자동 리셋은 실행하지 않음 |
| `interrupts/button_interrupts` | SW1 영구 래치, SW2 에지·디바운스·해제 이벤트, ESTOP 영구 루프 |
| `system/system_manager` | SW2 이벤트 단독 소비자. 운전 요청·교정·집계 고장을 합쳐 motorAllowed 결정 |
| `system/event_logger` | 번호/문자열 고정 큐, 변경·중복 억제, 일반 UART 쓰기 지연 |
| `system/status_led` | 주체별 요청 보존, 우선순위·TTL로 두 WS2812 표시 결정 |
| `sensors/color_sensor_types` | 모델·모서리·색·원시값·교정 기준·판정 결과와 출력 이름 |
| `include/sensors/sensor_config.hpp` | 모델, 수광·게인·조명, TCA 채널, 확정 횟수, 임시 임계값, 교정 여유 |
| `sensors/color_sensor_manager` | Wire/TCA/레지스터 담당. 수광 완료 확인과 새 원시 측정 발행 |
| `sensors/floor_color_classifier` | 원시값을 네 바닥색/Unknown으로 판별. I2C·UART·모터를 건드리지 않음 |
| `sensors/color_calibration` | 시작 전 UART 안내식 교정과 NVS 저장. 판별 알고리즘과 분리 |
| `sensors/sensor_manager` | 새 sequence만 판별, 3회 확정, 무효/오래된 판정 폐기, 오류 변경 기록, snapshot |
| `include/communication/vision_protocol.hpp` | 패킷·상태 비트·범위·오류번호. Pi 송신기와 맞춰야 하는 계약 |
| `communication/raspberry_link` | UART1 재조립, 헤더 재동기화, 중복 sequence와 영상 신선도 처리 |
| `include/control/control_config.hpp` | 생산 출력·중심 기준·공격/탈출 시간·위험색 마스크 |
| `control/robot_fsm` | 안전/바닥/비전으로 무엇을 할지 판단하고 DriveIntent 반환. GPIO 직접 제어 없음 |
| `control/drive_logic` | 의도를 좌우 MotorCommand와 80ms 기한으로 변환 |
| `include/motor/motor_config.hpp` | PWM·방향 반전·출력 상한·명령 만료·브레이크 전기 조합 |
| `motor/motor_controller` | 공용 API, 안전 검사, 제한, 뮤텍스, 만료 감시 태스크, 상태 반환 |
| `motor/bts7960_driver` | BTS7960 하나의 실제 PWM/EN 쓰기, 방향 전환 deadtime, 영구 금지 확인 |
| `src/bench_main.cpp` | 벤치 콘솔·안전·센서/비전·CSV 연결. 교정 콘솔은 실행하지 않음 |
| `bench/bench_controller` | MOTOR/MANUAL/DATA/ALIGN/FOLLOW/AVOID의 시간 기반 명령 생성 |
| `include/bench/bench_config.hpp` | 시험 출력/유지시간/추적·회피 조건. 생산 전략 설정과 분리 |
| `src/sensor_serial_test.cpp` | 모터를 건드리지 않고 100ms마다 버튼/센서 원시값 출력 |
| `tests/run_host_tests.ps1` | 실제 소스를 PC 가짜 하드웨어에 연결해 두 모델 컴파일/실행 |
| `tests/host/test_firmware.cpp` | 생산 모터·안전·FSM·UART·센서·교정·로그·메인 연결 회귀 시험 |
| `tests/host/test_bench.cpp` | 모드 시간·수동 만료·추적/회피·콘솔·벤치 메인 회귀 시험 |
| `tests/host/fake_hardware.cpp` 및 shim 헤더 | 가상 시간/GPIO/UART/I2C/NVS/태스크. 생산 알고리즘은 대체하지 않음 |

`ContactCheck`, `UndercutDrive`, `ScoreHold`, `WallEscape`, `StuckRecovery`, `Stop` 등
일부 `RobotState` 이름은 현재 생산 FSM에서 진입하지 않는 예약 상태다.
접촉·스포크 삽입·고착을 센서로 확인하는 기능이 있다고 해석하지 않는다.

## 4. 부팅과 메인 루프 워크플로우

```text
전원/리셋
  → PWM/EN LOW → 리셋 원인 → UART0 → 모터/PWM/만료 감시
  → 버튼 ISR → LED → 컬러 센서/NVS → UART1 → 시스템 관리자
  → 시작 대기
       ├─ CAL START → 48단계 측정 → CAL SAVE → 시작 대기
       └─ SW2 정상 눌림/해제 → Running

생산 loop()
  안전/버튼 → 라즈 수신 → 센서 짧은 I2C 단계 → 안전 재확인
  → 교정 콘솔 (교정 중이면 모터 정지, FSM 생략)
  → 센서/비전 snapshot → FSM 의도 → 좌우 명령 → 제한/PWM
  → LED → 이벤트 큐 출력 → delay(1)

독립 경로
  SW1 ISR → 영구 구동 금지 + EN 즉시 LOW
           → 일반 문맥 브레이크 → ESTOP만 3초마다 → 재부팅 필요
  모터 만료 태스크 → 마지막 명령 기한 확인 → 만료 시 브레이크
```

컬러 센서 초기화 실패는 부팅 실패로 만들지 않는다. 라즈도 필수 부팅 장치가 아니다.
모터 초기화/감시 태스크 생성 실패는 구동을 막는다.
각 `begin()`은 부팅용이며 운전 중 반복 초기화/자동 복구나 SW1 래치 우회용으로 호출하지 않는다.

## 5. 안전 조건과 의도된 예외

`SystemManager`는 **구동해도 되는지**, `RobotFsm`은 허용된 상태에서 **어떻게 움직일지** 결정한다.

| 상황 | 생산 펌웨어 | 벤치 펌웨어 |
|---|---|---|
| 전원 직후 | SW2 대기, 무구동 | STOP 모드, 무구동 |
| 부팅부터 SW2 눌림 | 첫 해제 무시, 새 눌림/해제 필요 | 동일 |
| 정상 SW2 눌림/해제 | Running, FSM 시작 | 선택 모드 허용 |
| SW2 다시 조작 | 정지 버튼이 아님 | 정지 버튼이 아님 |
| STOP/모드 변경 | API로 운전 요청 폐기 | STOP/MODE 콘솔, 새 SW2 필요 |
| 정지 전에 누른 SW2 해제 | 출발에 사용하지 않음 | 동일 |
| SW1 눌림 | 영구 정지, 해제해도 재부팅 전 금지 | 동일 |
| 라즈 없음/영상 오래됨 | 탐색/상실 처리, 짧은 공격은 지속 가능 | 추적 정지; AVOID 진행 중 탈출은 우선 |
| 센서 하나/전부 없음 | Unknown, 주행 시작 허용 | AVOID는 정지, 나머지 주행 모드는 가능 |
| 색 변경 후 3회 확정 중 | Unknown은 위험색이 아님 | AVOID는 Unknown 때문에 정지 |
| 센서 I2C/변환 고장 | 채널 제외, 재부팅 전 자동 초기화 없음 | 동일 |
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

- Search: 300 천분율 제자리 회전, 절대 시간 기준 1.2초마다 방향 변경.
- TargetConfirm: 서로 다른 새 목표 프레임 2회 필요. 확인 중에도 중앙이면 240으로 접근,
  벗어나면 회전한다. 정지한 채로만 확인하지는 않는다.
- TrackAlign: 중심 X=960, 오차 ±140 밖이면 회전280. 중앙이면 접근450과 비례 조향.
- 공격 진입: 확인 후 X 오차 ±75 이내, 박스 높이420 이상이면 시간 기반 공격 시작.
- DiveReady→DiveBrake→SpokeEngaged: 650 돌진, 브레이크, 480 삽입 전진.
  짧은 카메라 가림에도 진행하지만 SW1/위험색/운전 금지는 먼저 처리한다.
- Push: 650 전진과 작은 조향. 보이는 박스 높이210 미만 또는 X 오차600 초과이면 재확인.
  상실 시 마지막 **새 목표 프레임 시각**부터 500ms 이내만 직진 밀기 후 후진한다.
  앞의 공격이 시간을 소비했다면 유지 시간이 짧아지거나 바로 후진할 수 있다.
- 일반 목표 상실: 마지막 새 목표부터 260ms 이내는 마지막 방향 회전, 이후 Search.
- ZoneEscape: 빨강/노랑/파랑 위험. 전방이면360ms 후진, 후방이면 전진,
  이어 반대 방향 회전320ms. 여전히 위험하면 탈출 재판단.
  이동 방향에 새 위험이면 브레이크 한 번 후 방향을 새로 판단한다. 전후방 동시 위험은 정지한다.
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
자동 게인·자동 노출·센서 재초기화는 없다.

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

**생산 펌웨어** 시작 대기 중 UART0 57600에서 실행한다. 벤치는 NVS를 불러오지만 교정 콘솔은
실행하지 않는다. 센서 전용도 교정이 없다. 네 채널 모두 초기화돼야 CAL START를 받는다.
SW1 래치 후에는 교정도 불가능하다. LF/CRLF 줄바꿈, 소문자는 대문자로 변환한다.

```text
CAL START
READY
# 준비3초 + 측정3초. 주석은 보내지 않는다.
ACCEPT
# 다음 배치 안내에 맞춰 READY→ACCEPT 반복
CAL SAVE
```

순서는 S1 빨강→노랑→파랑→검정, 각 색마다 Level→Lifted→Pressed, 이어 S2/S3/S4다.
4×4×3=48단계, 간격 안내10/20/3mm는 사람이 맞춘다. 준비/측정만 최소288초, 배치/응답은 추가다.
IMU는 사용하지 않는다. 유효한 새 측정으로 평균·표준편차·Clear min/max를 구한다.

| 명령 | 조건 / 의미 |
|---|---|
| CAL START | 운전 전 새 교정, 기존 저장 기준은 SAVE 전까지 유지 |
| READY | 배치 완료 후 준비/측정 |
| ACCEPT | 결과 수락, 다음 단계 |
| RETRY | 현재 결과 폐기, 다시 READY 대기 |
| BACK | 배치 대기/결과 확인 중 이전 단계 |
| STATUS / CAL STATUS | 상태·센서·색·자세·샘플 수·여유 표시 |
| CAL MARGIN 20 40 | 교정 중 비율20% / 밝기40% 여유, 두 정수1..100만 허용 |
| ABORT | 작업 중단, 기존 저장 교정 유지 |
| CAL SAVE | 전체 완료 후 NVS 저장, 실패는 오류 후 수동 재시도 가능 |
| CAL ERASE | 운전 전 저장 기준 삭제, 임시 기준 전환 |

3초 내 유효5샘플 미만이면 그 단계를 재안내한다. 긴 수광은 측정 시간도 조정한다.
NVS color-cal/data 한 블롭의 길이·버전·모델·설정 서명·유한값을 확인한다.
저장/삭제 실패 시 기존 RAM 기준을 보존한다. SAVE 전 중간 측정은 재부팅하면 사라진다.
교정 중 SW2는 교정을 중단하고 제스처를 버린다. 완전히 놓은 뒤 새로 눌렀다 놓아야 출발한다.

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
검출 보장이 없다. 현재 Pi JSON을 UART1에 그대로 넣으면 동작하지 않는다.

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

- 단위 -1000..1000 천분율, 생산 기본±650, 벤치±200.
- setOutputLimit 변경은 기존 출력을 정지한다. 새 명령이 필요하다.
- forward/reverse/rotateLeft/rotateRight, stop/coast/brake, status/outputLimit 제공.
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
100번대 부팅/시스템,200 상태/모드,300 통신,400 센서,500 모터,600 안전,900 사용자.
같은 번호·소스·value1·문자열은 기본1초 제한. value2는 반복 키에서 제외한다.
상태/모드/오류는 변경 API로 바뀔 때만 기록. 큐32개, 한 번 최대2개, 송신 공간 부족은
다음 루프로 미룬다. 큐가 차면 드롭, status().dropped로 확인하며 주행을 막지 않는다.

```cpp
#include "system/event_logger.hpp"
#include "system/status_led.hpp"

EventLogger::log(EventId::InvalidPacket, 6);
EventLogger::logText(900, LogSeverity::Info, LogSource::User, "test_begin");
EventLogger::faultChanged(LogSource::Sensor, FaultCode::Range, true, 1);
StatusLed::setBoth(LedOwner::User, LedState::Tracking, 500);
```

LED 요청은 즉시 WS2812를 쓰지 않고 update에서 최소20ms마다 반영한다.
내장 밝기32/255, 외부64/255, 대상별/주체별 요청 하나씩 유지한다.
높은 상태 우선순위가 이기고 같은 우선순위는 최신 요청, clear(owner)는 그 주체만 제거한다.
TTL=0은 계속 유지, 짧은 TTL 만료 후 다른 요청이 다시 표시된다.

| 우선순위 | 표시 |
|---:|---|
| 255/240/230/220 | ESTOP 빨강 / 일반 고장 빨강·흰색 / 모터 빨강·주황 / 센서 노랑 점멸 |
| 200/190/180 | 구역 주황 / 벽 자홍(예약) / 상실 보라 점멸 |
| 150/140/130/120 | 돌진 흰색 점멸 / 밀기 파랑 / 정렬 노랑 / 추적 초록 |
| 110/100/70/50/40 | 접근 청록 / 탐색 빨강 점멸 / 준비 파랑 / 대기 주황 / 부팅 파랑 점멸 |

개별 센서 오류 로그가 자동으로 전체 SensorFault LED를 만들지는 않는다.
벤치는 생산 FSM LED 갱신이 없으므로 시스템 대기/Ready 표시가 중심이다.
CAL 안내와 벤치 ACK/HELP는 사람이 요청한 동기 콘솔이며 일반 이벤트 큐와 다르다.
SW1 영구 루프 진입 후 일반 큐를 버리고 ESTOP만3초마다 출력한다.

## 13. 현재 주기와 스케줄링

| 처리 | 실행 방식 | 시간의 의미 |
|---|---|---|
| Arduino loop | 기본 loopTask 우선순위1, 코어는 Arduino 설정 | 매 반복 delay(1), 전체1kHz 보장은 아님 |
| 버튼 | ISR + 루프 디바운스 | SW1 즉시 래치, SW2 눌림/해제 각각40ms |
| 모터 만료 | FreeRTOS 태스크 우선순위2 | 5ms 간격, 실제 제동 지연은 실측 |
| 센서 방문 | 메인 최소1ms 슬롯 | AS 매번4개, TCS1개. 샘플 Hz와 다름 |
| AS 수광 | 내부 연속 약5ms | 이론 약200회/s, I2C/보정/누락으로 실제 낮아질 수 있음 |
| UART1 | 메인 제한 처리 | 송신 목표50Hz, 프레임 신선도120ms |
| FSM/명령 생성 | 교정 아닌 메인 반복 | 별도 고정주기 태스크 아님 |
| LED | 최소20ms | 최대 갱신 시도50Hz, 변경색만 전송 |
| 이벤트 | 루프 끝 최대2개 | 사건/변경 시 기록, 낮은 처리 순서 |
| 벤치 데이터 | 100ms마다5행 시도 | 최대10Hz, 80ms 내 못 보내면 프레임 생략 |
| 센서 전용 출력 | 약100ms | 약10Hz, 수광 주기와 다름 |

FreeRTOS는 Arduino 내부에도 사용된다. 직접 분리한 기능 태스크는 모터 만료뿐이다.
I2C·FSM·UART·교정·로그가 모두 독립 태스크라는 의미는 아니다.
I2C가 밀리면 만료 감시가 모터를 정지할 수 있고 다음 새 명령으로 다시 적용된다.
0.5m/s에서 순수5ms 수광 이동은2.5mm다. 색 변경 직후 혼합된 첫 결과를 버리는 이상적 모델의
3회 확정은 약20ms/10mm이지만 최대 반응거리 보장이 아니다.
초기 보정·폴링/읽기 지연·측정 누락·경계 혼합·제동거리는 별도다.

## 14. 빌드·업로드·실물 시험 순서

PlatformIO CLI 또는 VS Code PlatformIO가 필요하다. 플랫폼 espressif32 6.11.0,
Arduino core2.0.17, NeoPixel1.15.5. 실제 보드 플래시/전원/포트는 따로 확인한다.

```powershell
cd D:\C_ROBOT\ESP_MAIN
pio run -e robot_bench
pio run -e robot_bench -t upload
pio device monitor -e robot_bench -b 57600
# 필요하면 upload에 --upload-port COM번호, monitor에 -p COM번호 추가
```

1. 모터 전원 분리→sensor_serial_test 업로드→UART0 115200에서 네 센서/SW 원시값 확인.
2. 바퀴 띄우기→robot_bench 업로드→UART0 57600, 전원 직후 무구동 확인.
3. MODE MOTOR→SW2→좌/우 정역 각3초(12%), 사이0.5초 정지 확인.
4. SW1 차단·브레이크·ESTOP·해제 후 미복구 확인. 다음 시험은 재부팅.
5. MODE MANUAL→SW2→DRIVE 100 100 등을250ms보다 빠르게 반복, STOP/입력 상실 정지 확인.
6. 생산 환경으로 수광/게인 조정·교정 저장 후 벤치로 돌아와 NVS 기준 사용.
7. 규격 송신기 연결 후 ALIGN→FOLLOW→AVOID. 처음 두 모드는 바닥 보호가 없다.
8. 낮은 생산 출력으로 회피/비전 분리 시험 후 공격 시간/출력 조정.

벤치는 대문자·줄바꿈 필수. MODE는 기존 출발/수동 명령 폐기, DATA는 SW2 없이 무구동 출력.
HELP는 정지 후 도움말, STATUS는 지속 로그 켜기, LOG OFF로 끈다.
잘못된/긴/250ms 미완성 줄은 정지 후 줄 끝까지 버린다. 교정 명령은 받지 않는다.
정확한 명령·CSV·각 모드 조건은 [BENCH_TESTS.md](BENCH_TESTS.md)를 읽는다.

## 15. 검증 재실행과 한계

```powershell
cd D:\C_ROBOT\ESP_MAIN
.\tests\run_host_tests.ps1
.\tests\run_host_tests.ps1 -Bench
.\tests\run_host_tests.ps1 -Demo
pio run -e yd_esp32_s3 -e yd_esp32_s3_tcs34725 -e sensor_serial_test -e sensor_serial_test_tcs34725 -e robot_bench -e robot_bench_tcs34725
cd D:\C_ROBOT
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

PC 펌웨어 시험은 PATH의 native g++ 또는 PlatformIO toolchain-gccmingw32가 필요하다.
실제 setup/loop와 모듈을 연결해 시간 순환·채터링·ISR 경합·UART 무작위 입력·센서 고장·
교정 저장 실패·만료·공격/벤치 콘솔을 시험한다. .pio/host-tests 결과는 Git에서 제외한다.
이번에는 정지 중 버튼/새 진행 방향 위험/교정 숫자 검사의 실패를 먼저 재현하고 수정했다.

최신: 생산 AS12그룹/4,994 검사, TCS12그룹/4,555 검사, 벤치 각각67 검사,
여섯 ESP32 빌드, Python 핵심5테스트 통과. 반복 assert는 독립 시나리오 개수가 아니다.
라즈 두 추적기는 로컬 파일20프레임씩 실행했다. GUI·실제 카메라·ESP 동시성·광학/전기 성능은
이 시험으로 증명하지 않는다. 보조 UI/영상 변환/CAD의 전체 경로도 실물 검증 범위 밖이다.

상세 검토/실물 목록은 [VERIFICATION.md](VERIFICATION.md), 다음 개발 기준은
[AGENTS.md](AGENTS.md), 영상 코드 설명은 [루트 README](../README.md)를 읽는다.
공격 전략의 세부 루틴은 실제 차량에서 튜닝하는 다음 단계다.
