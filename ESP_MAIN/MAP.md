# ESP_MAIN 지도 — 작업 전에 이것부터 읽는다

이 문서는 펌웨어 구조를 한 장으로 요약한다. 규칙은 [AGENTS.md](AGENTS.md),
설명과 사용법은 [README.md](README.md), 검증 이력은 [VERIFICATION.md](VERIFICATION.md)에 있다.
각 모듈의 역할·호출 위치·안전 여부는 해당 헤더 맨 위 `// Contract` 주석에 있으니
`.cpp`를 열기 전에 헤더부터 읽는다.

## 1. 작업별로 읽을 파일

| 작업 | 먼저 읽을 것 | 필요할 때만 |
|---|---|---|
| FSM 전략·출력·시간 | `include/control/robot_fsm.hpp`, `include/control/control_config.hpp`, `src/control/robot_fsm.cpp` | README §6 |
| 바닥 센서 튜닝 | `include/sensors/sensor_config.hpp`, `src/sensors/floor_color_classifier.cpp` | README §7·§8, AGENTS "컬러 센서" |
| 센서 판정 흐름 | `include/sensors/sensor_manager.hpp`, `src/sensors/sensor_manager.cpp` | `color_sensor_manager.cpp` |
| 모터·브레이크 | `include/motor/motor_controller.hpp`, `include/motor/motor_config.hpp` | README §11 |
| 안전·시작 절차 | `include/system/system_manager.hpp`, `include/interrupts/button_interrupts.hpp` | README §5, AGENTS "안전" |
| 라즈 UART | `include/communication/vision_protocol.hpp`, `include/communication/raspberry_link.hpp` | README §10 |
| 실물 벤치 시험 | [BENCH_TESTS.md](BENCH_TESTS.md) | README §14 |
| 센서 교정 (별도 펌웨어) | `src/calibration_main.cpp`, `include/calibration/color_calibration.hpp` | README §9 |
| 교정값 저장 형식 | `include/sensors/calibration_store.hpp` | |

## 2. 값의 원본 (문서에 숫자를 복사하지 않는다)

| 내용 | 원본 |
|---|---|
| GPIO 핀 (V1 확정, 변경 금지) | `include/pins.hpp` |
| UART 속도·타임아웃, 버튼, LED, 로그 크기 | `include/config.hpp` |
| 모터 PWM·출력 상한·명령 만료·브레이크 조합 | `include/motor/motor_config.hpp` |
| FSM 출력·시간·중심 기준·위험색 | `include/control/control_config.hpp` |
| 센서 종류·수광·게인·LED·임계값·확인 횟수 | `include/sensors/sensor_config.hpp` |
| 라즈 패킷 형식·상태 비트·오류 번호 | `include/communication/vision_protocol.hpp` |
| 모듈 간 자료형 (스냅샷, 의도, 명령, 상태) | `include/robot_types.hpp` |
| 벤치 시험 출력·시간 | `include/bench/bench_config.hpp` |

## 3. loop() 한 바퀴 (`src/main.cpp`)

```text
SystemManager::update      안전 상태 갱신
RaspberryLink::poll        UART1 최대 128바이트
SensorManager::update      센서 짧은 I2C 단계
SystemManager::update      안전 재확인 (의도된 반복)
SensorManager::snapshot + RaspberryLink::latest
RobotFsm::update           → DriveIntent
DriveLogic::makeCommand    → MotorCommand (80ms 기한)
motorAllowed ? MotorController::apply : MotorController::brake
StatusLed::update, EventLogger::process, delay(1)
```

메인 루프 밖의 독립 경로:

- SW1 ISR: 영구 구동 금지 래치와 BTS7960 EN 즉시 LOW. 재부팅 전 복구 없음.
- 모터 만료 감시 태스크: 5ms마다 명령 기한을 확인하고 만료 시 브레이크.

## 4. 모듈 표

안전 열이 "예"인 모듈은 AGENTS.md의 안전 코드 규칙을 따른다. 정리·통합 목적으로 수정하지 않는다.

| 모듈 | 헤더 | 핵심 API | 호출하는 곳 | 안전 |
|---|---|---|---|---|
| 부팅 | `app/boot_manager.hpp` | `begin`, `staySafe` | `setup` | 예 |
| 핀 초기화 | `hardware/board_io.hpp` | `setEarlySafeState`, `configurePins` | BootManager | 예 |
| 리셋 원인 | `hardware/reset_manager.hpp` | `reason`, `requiresRearm` | BootManager, SystemManager | |
| SW1/SW2 | `interrupts/button_interrupts.hpp` | `update`, `consumeStartEvent`, `enterEmergencyStopLoop` | ISR, SystemManager | 예 |
| 안전 관리 | `system/system_manager.hpp` | `update`, `status`, `requestStop`, `set*Healthy` | loop | 예 |
| 모터 | `motor/motor_controller.hpp` | `apply`, `brake`, `update`, `status` | loop, 감시 태스크 | 예 |
| BTS7960 | `motor/bts7960_driver.hpp` | `drive`, `brake`, `latch*DriveInhibit` | MotorController, SW1 ISR | 예 |
| FSM | `control/robot_fsm.hpp` | `update`, `state` | loop | |
| 구동 변환 | `control/drive_logic.hpp` | `makeCommand` | loop | |
| 라즈 UART | `communication/raspberry_link.hpp` | `poll`, `latest`, `status` | loop | |
| 센서 판정 | `sensors/sensor_manager.hpp` | `update`, `snapshot` | loop | |
| 센서 I2C | `sensors/color_sensor_manager.hpp` | `update`, `sample` | SensorManager | |
| 색 판별 | `sensors/floor_color_classifier.hpp` | `classify` | SensorManager | |
| 교정값 저장소 | `sensors/calibration_store.hpp` | `load`, `data`, `valid` | SensorManager | |
| 로그 | `system/event_logger.hpp` | `log`, `faultChanged`, `process` | 모든 모듈 | |
| LED | `system/status_led.hpp` | `setBoth`, `update` | 모든 모듈 | |

의존 방향: 센서·라즈·FSM·구동 변환은 다른 기능 모듈을 호출하지 않고 값만 돌려준다.
모터를 멈추는 경로는 SystemManager, 만료 감시 태스크, SW1 ISR 세 곳이다.
구조 분석에서 로그·LED는 무시해도 흐름이 바뀌지 않는다.

## 5. 기능을 추가·제거하는 방법

1. 새 기능은 `include/<module>/이름.hpp` + `src/<module>/이름.cpp`로 만든다.
2. 헤더 맨 위에 다른 모듈과 같은 형식의 `// Contract` 주석을 쓴다.
3. 설정값은 해당 `*_config.hpp`에 둔다.
4. `src/main.cpp`의 `loop()`에는 호출 한 줄만 추가한다. 모터 출력은 FSM 경로 하나로만 낸다.
5. 이 문서의 §1·§2·§4 표에 한 줄씩 추가한다.
6. `tests/run_host_tests.ps1`로 기존 회귀 시험을 실행한다.

제거는 역순이다. `loop()`의 한 줄과 표의 한 줄을 지우면 연결이 끊긴다.

교정·벤치·센서 점검은 생산 `main.cpp`에서 진입하지 않는다. 각각 별도 PlatformIO 환경으로 업로드한다.

| 환경 | 진입 파일 | 용도 |
|---|---|---|
| `yd_esp32_s3` | `src/main.cpp` | 경기용 생산 펌웨어 |
| `robot_bench` | `src/bench_main.cpp` | 저출력 실물 시험 |
| `sensor_serial_test` | `src/sensor_serial_test.cpp` | 센서 원시값 확인 (모터 미사용) |
| `color_calibration` | `src/calibration_main.cpp` | 48단계 색 교정, NVS 저장 (모터 미사용) |

## 6. 빌드와 시험

```powershell
pio run -e yd_esp32_s3          # 생산 펌웨어
.\tests\run_host_tests.ps1      # PC 회귀 시험 (실물 검증 아님)
```

전체 환경 목록과 업로드 순서는 README §2·§14에 있다.
