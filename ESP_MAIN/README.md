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

## 주의

- V1 핀 배치를 유지한다. V2 회로의 핀 번호를 적용하지 않는다.
- 네 모서리 컬러 센서만 사용한다. V1에는 IMU/IR/전류/엔코더/배터리 ADC가 없다.
- SW1은 영구 정지이며 재부팅해야 한다. 라즈 미연결은 시스템 고장이 아니다.
- 기본 출력 상한은 65%다. 실물 첫 시험 전에 낮추고 바퀴를 띄운다.
- 브레이크 조합과 바닥색 임계값은 실측 전이다. 100Hz 수광도 아직 검증되지 않았다.
