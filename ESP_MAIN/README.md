# ESP_MAIN — V1 주행 펌웨어

PlatformIO / ESP32-S3 / Arduino 기반 씨름 로봇 펌웨어다.
현재 부팅, 버튼 안전, 컬러 센서, 교정, 라즈 UART, 로그/LED, 공격 FSM, 모터 출력까지 연결되어 있다.
PC 검증과 ESP32 빌드는 통과했으며 실물 시연과 튜닝은 남아 있다.

## 먼저 읽을 문서

- [프로젝트 README와 진행 상황](../README.md)
- [요구사항·워크플로우·검증·실물 확인 사항](VERIFICATION.md)
- [사용자 요구사항 및 작업 기준](AGENTS.md)

## 빠른 실행

이 폴더에서:

```powershell
pio run -e yd_esp32_s3
pio run -e yd_esp32_s3 -t upload
pio device monitor -b 57600
```

TCS34725 차량은 `yd_esp32_s3_tcs34725` 환경을 사용하거나
`include/sensors/sensor_config.hpp`의 `COLOR_SENSOR_MODEL` 한 줄을 바꾼다.
수광 시간과 게인도 같은 설정 파일에 있다.

센서만 점검하려면 `sensor_serial_test` / `sensor_serial_test_tcs34725` 환경과 UART0 115200을 사용한다.
센서 전용 환경은 모터 핀과 안전 제어를 실행하지 않으므로 모터 전원을 분리한다.

하드웨어 없는 검증과 텍스트 데모:

```powershell
.\tests\run_host_tests.ps1
.\tests\run_host_tests.ps1 -Demo
```

## 바닥센서 조정 — 2026-10-05

설정은 `include/sensors/sensor_config.hpp` 한곳에서 바꾼다. 런타임 자동 노출/복구는 추가하지 않았다.

| 조정할 내용 | 설정 | 기본값 |
|---|---|---|
| 센서 종류 | `COLOR_SENSOR_MODEL` | AS7341 |
| AS7341 수광 | `AS7341_EXPOSURE_US` | 5000us (실제 4998.44us) |
| AS7341 게인 | `AS7341_GAIN` | X16 |
| 자체 백색 LED | `AS7341_LED_ENABLED`, `AS7341_LED_CURRENT_MA[4]` | 켬, 센서별 20mA |
| 완료 확인 | `COLOR_SENSOR_SLOT_PERIOD_MS`, `COLOR_SENSOR_VISITS_PER_UPDATE` | 1ms마다 네 센서 확인 |
| 색 확정 | `COLOR_CONFIRM_COUNT` | 서로 다른 새 측정 3회 |
| 너무 약한 수광 제외 | `MINIMUM_CLEAR_COUNTS` | 1 |
| 센서별 임시 검정 임계값 | `FALLBACK_*_BLACK_CLEAR_BY_SENSOR[4]` | 실측 전 임시값 |
| 색 비율 임계값 | `FALLBACK_RED_*`, `FALLBACK_BLUE_*`, `FALLBACK_YELLOW_*` | 실측 후 조정 |
| 교정값 판별 허용 범위 | `DEFAULT_RATIO_MARGIN_PERMILLE`, `DEFAULT_BRIGHTNESS_MARGIN_PERMILLE` | 150 / 300 |
| 밝기 영향·혼합색 거부 | `MATCH_*` | 색/검정 밝기 가중치 0.20 / 0.55 |
| 교정 높이 안내 | `CALIBRATION_HEIGHT_MM` | LEVEL=10, LIFTED=20, PRESSED=3mm |

AS7341은 F2(445nm), F3(480nm), F5(555nm), F6(590nm), F7(630nm), Clear를
한 번에 측정하며, SMUX 설정은 시작할 때만 한다. 이후 수광을 켜둔 채 최신 데이터를 읽는다.
F1/F4/F8/NIR은 이 모드에서 미측정이며 원시 배열의 0을 실제 측정값으로 해석하면 안 된다.
매 수광 완료 이벤트(AINT, APERS=0)를 읽고 소거하며, 최소 수광 간격도 확인해 같은 값을
3회 확정으로 중복 계산하지 않는다. 지연 시 지나간 모든 샘플을 재생하지 않고 최신 결과만 읽는다.
포화/무수광은 해당 샘플만 무효화하고, I2C/완료 타임아웃은 기존대로 해당 센서를 재부팅까지 제외한다.
TCS34725는 기존 단발 RGBC 읽기 방식을 유지한다. 단일 묶음 채널 분리와 연속 동작은 실물 검증이 필요하다.

LED 설정은 AS7341의 LDR에 연결된 모듈에만 적용된다. 외부 고정 LED는 제어되지 않는다.
현재 드라이버는 4~150mA, 2mA 단위만 허용한다. 실제 모듈 LED 정격을 먼저 확인한다.
수광/게인/채널 구성/LED 설정이 달라지면 기존 NVS 교정은 사용하지 않고 임시 임계값으로 동작하므로 재교정한다.
`CAL MARGIN <비율%> <밝기%>`는 저장된 교정의 허용 범위를 조절한다.

### 다음 작업에서도 반드시 고려할 기체 조건

- 추가 가림막이나 구조물을 설치할 수 없다. 센서는 차체 아래에서 자체 LED로 바닥을 비춘다.
- 색 구역에는 10mm 단차가 있다. 센서-바닥 간격은 근접 약 3mm, 내려간 상태 약 10mm,
  단차에 걸친 상태 약 20mm로 변한다. 20mm 조건을 제거하거나 고정 높이라고 가정하지 않는다.
- V1에 거리센서/IMU가 없으므로 높이는 측정하지 않는다. 교정 시 사람이 간격을 맞춘다.
- 센서별 네 색 × 세 높이에서 3초씩 측정한다. 비율 중심의 판별과 검정 밝기 조건을 함께 튜닝한다.
- 3mm에서 반사광 포화, 20mm에서 약한 유색 바닥의 검정 오인, 경계에서 혼합색을 검증한다.
- 5ms/16배/20mA는 시험 시작값이지 거리 전체에서 검증된 값이 아니다. 자동 게인 변경은 하지 않는다.
- 0.5m/s에서 순수 5ms 수광은 2.5mm 이동이다. 3회 확정의 이상적 위상 계산은 약 20ms/10mm이지만
  내부 보정·I2C·스케줄링·측정 누락·경계 혼합·제동거리는 별도다. 최대 반응거리 보장으로 쓰지 않는다.
- 다음 AI/개발자도 센서나 주행을 수정할 때 이 조건과 미검증 사항을 사용자에게 짚어준다.

## 기존 안전 주의

- V1 핀 배치를 유지한다. V2 회로의 핀 번호를 적용하지 않는다.
- 네 모서리 컬러 센서만 사용한다. V1에는 IMU/IR/전류/엔코더/배터리 ADC가 없다.
- SW1은 영구 정지이며 재부팅해야 한다. 라즈 미연결은 시스템 고장이 아니다.
- 기본 출력 상한은 65%다. 실물 첫 시험 전에 낮추고 바퀴를 띄운다.
- 브레이크 조합과 바닥색 임계값은 실측 전이다. 100Hz 수광도 아직 검증되지 않았다.
