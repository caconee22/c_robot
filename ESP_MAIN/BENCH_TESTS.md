# V1 실물 시험 펌웨어

> **2026-10-31 정리:** 지금 쓰는 펌웨어는 `yd_esp32_s3_v2`(경기용)와 `robot_bench_v2_manual`(벤치 + 색 교정) 두 개뿐이다. 아래에 나오는 V1·TCS34725·`robot_bench`·`robot_bench_v2`·`sensor_serial_test`·`color_calibration` 환경과 `bench_console.py`는 `archive/`로 보관했다 (`archive/README.md`). 색 교정은 `tools/bench_gui.py`의 "색 교정" 탭.

생산 공격 FSM과 별도인 테스트 코드다. V1 핀·기존 센서 설정·SW1 영구 비상정지·모터 명령
만료 감시는 그대로 사용한다. 돌진/급정거/스포크 기동은 실행하지 않는다.

## 업로드 및 연결

`D:\C_ROBOT\ESP_MAIN`에서:

```powershell
pio run -e robot_bench -t upload
pio device monitor -e robot_bench -b 57600
```

TCS34725 기체는 두 명령의 환경 이름을 `robot_bench_tcs34725`로 바꾼다.
명령은 UART0(COM/SiK 공용), 57600bps, 대문자, 줄바꿈으로 전송한다.
PlatformIO 모니터는 기본적으로 로컬 입력이 안 보일 수 있다. Enter로 전송하거나
터미널에서 줄바꿈(LF/CRLF)을 설정한다. UART1은 기존 라즈 115200bps/14바이트 패킷용이다.
UART0의 COM/SiK 수신 출력 둘을 동시에 구동하지 않는다.

## 모드와 사용 순서

부팅 시 `STOP`으로 시작한다. `MODE ...`는 항상 먼저 정지하고 기존 수동 명령을 폐기한다.
주행 모드는 선택한 뒤 **SW2를 눌렀다 놓아야** 시작한다. UART 명령으로 시작 안전을 우회하지 않는다.
`STOP`으로 언제든 소프트웨어 정지한다. 다시 움직이려면 SW2를 새로 눌렀다 놓는다.
SW1은 누르는 즉시 영구 비상정지이며 해제 후에도 재부팅해야 한다.

| 명령 | 시험 내용 |
|---|---|
| `MODE MOTOR` | 왼쪽 정방향 3초 → 0.5초 정지 → 왼쪽 역방향 3초 → 정지 → 오른쪽 정/역 각각 3초. 한 번만 실행 후 정지 |
| `MODE MANUAL` | SW2 시작 후 `DRIVE <왼쪽> <오른쪽>`으로 독립 제어 |
| `MODE DATA` | 움직이지 않고 센서·라즈·모터 상태 로그 자동 출력. SW2 불필요 |
| `MODE ALIGN` | 타워 중심 X가 960±100 밖이면 제자리 회전, 중앙에 들어오면 정지. 전진하지 않음 |
| `MODE FOLLOW` | 중앙 타워를 추적하며 전진. 크게 벗어나면 회전만. 박스 높이 420 이상 또는 중앙 근처의 잘린 박스에서는 정지 |
| `MODE AVOID` | 추적 + 빨강/노랑/파랑 바닥 회피. 검정만 정상 바닥으로 취급 |
| `MODE STOP` | 모드 선택 없이 움직이지 않음 |

`ALIGN/FOLLOW`는 **바닥을 검사하지 않는다.** 안전한 넓은 곳에서 시험한다.
`FOLLOW/AVOID`의 박스 높이는 거리계가 아니라 임시 접근 정지 기준이다.
타워 박스가 커도 크게 옆에 있으면 먼저 회전 정렬할 수 있다. 공격/밀기/목표 수색은 하지 않는다.
타워가 없거나 UART/프레임/카메라/처리 상태가 유효하지 않으면 추적은 정지한다.
AVOID에서 이미 진행 중인 짧은 바닥 회피는 타워 정보보다 우선하며 계속할 수 있다.

`AVOID`는 네 바닥센서가 모두 신뢰 가능하고 최신이어야 움직인다.
하나라도 오류/Unknown/포화/오래된 판정이면 정지한다. 색 변경 후 3회 확정 중에도 잠시 멈출 수 있다.
전방 위험은 후진, 후방 위험은 전진 250ms 후 위험 방향 반대쪽으로 회전 250ms 한다.
전후방에 동시에 위험색이 있거나 회피 진행 방향에 위험이 생기면 정지한다.
현재 위험 구역을 물리적으로 벗어났다고 보장하는 로직은 아니며 추가 기구/거리센서가 없다.

## UART 수동 제어

```text
MODE MANUAL
# 여기서 SW2를 눌렀다 놓는다. 이 주석 줄은 전송하지 않는다.
DRIVE 100 100
DRIVE -100 100
STOP
```

숫자는 퍼센트가 아니라 **천분율**이다. `100`은 10%, 허용 범위는 `-200..200`(±20%).
`DRIVE`는 유지 스위치처럼 **250ms 안에 반복 전송**해야 한다. 새 명령이 없으면 정지한다.
단일 명령을 계속 래치하지 않는다. 안전이 풀리기 전에 입력한 명령도 저장하지 않는다.
잘못된 명령/범위 초과/지나치게 긴 줄/250ms 넘게 끊긴 미완성 줄은 정지 처리한다.
`HELP`도 안전하게 정지한 뒤 도움말을 출력한다. 입력이 끊긴 줄은 다음 줄바꿈까지 버린다.

## 데이터 출력

`LOG ON` / `LOG OFF`로 어느 모드에서든 켜고 끈다. `STATUS`는 로그 출력을 켜고 다음 프레임을 요청한다.
`EV,...`는 기존 이벤트 로그, `D,...`는 로봇 상태, `S,...`는 센서별 데이터다.
100ms 주기로 출력 시도하되 UART 공간이 부족하면 기다리지 않고 일부 프레임을 생략한다.
출력 주기는 센서 수광 주기가 아니며, 각 행은 읽는 시점이 조금 다를 수 있다.

```text
D,시간ms,모드,모터허용,좌출력,우출력,라즈연결,프레임최신,타워유효,X,박스높이,위험센서비트
S,시간ms,센서번호,sequence,원시값유효,판정색,색확정,Clear,값1,값2,값3,값4,값5,센서오류번호
```

AS7341의 값1..5는 F2/F3/F5/F6/F7이다. TCS34725는 R/G/B/Clear/Clear이며 마지막 Clear는
고정 출력 형식을 위한 중복 칸이다. 센서 순서는 전방 왼쪽/전방 오른쪽/후방 왼쪽/후방 오른쪽.
위험센서 비트 0..3은 같은 순서다. 출력은 분류 실패/포화 때도 원시값을 보여준다.

## 조정과 검증

- 시험 출력·수동 명령 유효시간·회전/추적/회피 시간: `include/bench/bench_config.hpp`
- 바닥 수광·게인·LED·임계값: `include/sensors/sensor_config.hpp`
- 타워 기준은 1920×1080 좌표이며 라즈 송신 규격은 기존 펌웨어와 같다.
- 바닥은 구조물 없이 3/10/20mm 간격, 자체 LED, 10mm 단차 조건이다. 근접 포화/먼 거리 검정 오인/
  경계 혼합은 실측해야 한다. 회피 모드라고 가장자리에서 안전하다고 보장할 수 없다.
- 처음에는 바퀴를 띄우고 MOTOR/MANUAL부터 확인한다. 12% 출력에서도 실제 속도/토크는 기체에 따라 다르다.
- 실제 구동 모터·UART·카메라·바닥에 대한 실물 검증은 아직 하지 않았다.

```powershell
.\tests\run_host_tests.ps1 -Bench
.\tests\run_host_tests.ps1
pio run -e robot_bench -e robot_bench_tcs34725
```

테스트 코드는 `src/bench_main.cpp`(입출력/안전 연결), `src/bench/bench_controller.cpp`(모드별 주행)로 나뉜다.
테스트 환경은 기존 `src/main.cpp`를 빌드하지 않는다. 생산 환경은 테스트 메인과 컨트롤러를 빌드하지 않는다.

2026-10-05 최신 PC 검증: AS7341/TCS34725 각각 67개 검사 통과. 모터 시간 순서, 수동 명령 만료와
millis 순환, 정렬/추적/회피, UART 패킷 수신과 끊김, 모드 변경 정지, 잘못된/긴/끊긴 명령,
UART 출력 공간 부족, SW1 영구 정지를 확인했다. 생산 펌웨어의 기존 12개 시험 그룹도 통과했다.
모드 변경 직전에 누르기 시작한 SW2의 해제는 새 모드 출발에 사용하지 않는다.
잘못된 공개 모드 값은 STOP으로 처리하고, 회피 회전 중에도 현재 위험 방향을 다시 반영한다.
생산/센서 전용/벤치 × 두 센서 구성의 여섯 PlatformIO 환경 빌드 성공. 실물 시험은 별도다.

## V2 전용 실물 시험 (2026-10-31)

새 환경은 `robot_bench_v2_manual`이며 `src/bench_v2_main.cpp`에서
`include/bench_v2/bench_v2_app.hpp`와 `src/bench_v2/bench_v2_app.cpp`를 호출한다.
기존 `robot_bench_v2`는 V1 벤치 진입점을 HW_VERSION=2로 빌드하는 환경으로 보존했다.
두 환경의 V2 핀맵과 공용 API는 같다. 확장 진단 출력은 새 환경에만 있다.
서로 다른 진입점을 별도로 빌드하므로 중복 링크되지 않는다.
V1 벤치 파일, 기존 환경 정의, 핀 상수 및 안전 모듈은 변경하지 않았다.

```powershell
pio run -e robot_bench_v2_manual
pio run -e robot_bench_v2
pio run -e robot_bench
powershell -File tests/run_host_tests.ps1 -V2 -Bench   # 기존·새 V2 벤치 둘 다 시험
```

UART0 Serial 57600bps로 부팅부터 진단 출력을 켠다.
LOG ON/OFF, STATUS, MODE, DRIVE, STOP, HELP는 위 V1 명령 체계를 따른다.
MODE MANUAL → SW2 눌림·해제 → DRIVE 순서로 시험한다.
출력 상한은 ±200 천분율(±20%)이며 마지막 DRIVE 후 250ms에 정지한다.
모드 변경·STOP 이후에는 새 SW2 제스처가 필요하다.
SW1 영구 비상정지 래치, 독립 모터 명령 만료 감시 및 안전 확인 순서는 기존대로다.
SW1 이후에는 일반 진단을 중단하고 기존 ESTOP 전용 루프로 진입한다.

BatteryMonitor를 그대로 사용한다. 전원 투입 후 3초 판정 창 안에서 3S/4S 범위의
연속 정상 샘플을 확인하고 성공하면 측정을 끝낸다. 배터리 없음·범위 미달·초과는
애플리케이션 고장으로 시작을 막는다. Checking 중 처리도 기존 모듈과 같다.
출력 전압은 연속 배터리 감시값이 아니라 마지막 부팅 판정값이다.
분압비와 전압 판정은 멀티미터로 비교해야 한다.

D 행과 네 S 행은 기존 형식 그대로다. 프레임마다 여섯 번째 행 하나가 붙고,
그 행은 돌아가며 바뀐다: I(IMU)는 두 프레임마다, 나머지는 여덟 프레임(0.8초)마다.
UART0에는 128바이트 하드웨어 FIFO만 있고 TX 링버퍼가 없어서(`availableForWrite()` ≤ 128)
128바이트가 넘는 줄은 영영 나가지 못한다. 그래서 진단을 기존 줄 끝에 붙이지 않고
따로 짧은 줄로 낸다(PC 시험이 모든 줄 < 128바이트를 확인). 숫자 enum은 해당 API 헤더 정의를 따른다.
EV 프레임은 기존 이벤트만 출력하며 실시간 수치를 EventLogger로 보내지 않는다.

```text
I,ms,IMU상태,WHO_AM_I(hex),dataFresh,samples,busErrors,ax,ay,az,gx,gy,gz
SYSTEM,ms,SystemState,모터비상래치,runRequested,motorAllowed,rearmRequired,startEventCount,resetReason,faultFlags
MOTOR,ms,요청좌,요청우,outputsActive,lastResult,stopMode,commandValidUntilMs,watchdogStopCount
LINK,ms,lastSequence,lastPacketMs,lastFrameMs,validPackets,invalidPackets,duplicatePackets
POWER_LED,ms,BatteryCheckResult,packMillivolts,cells,okSamples,LedState,outputOn,R,G,B
```

적용 좌우 출력과 라즈 연결·최신 여부는 기존 D 필드에서 확인한다.
100ms마다 여섯 행을 순서대로 시도하며 각 행은 해당 루프 시점의 스냅샷이다.
57600bps에서 80ms 동안 보낼 수 있는 양은 약 460바이트라 한 프레임은 그 안에 들어가게 했다.
80ms 프레임 제한과 UART 여유 공간 확인을 유지한다.
공간이 부족하면 기다리지 않고 진단을 생략하며 명령 응답도 생략될 수 있다.
PC 시험은 실제 최대 호출 시간을 보장하지 않는다.
실물에서는 로그 ON/OFF 각각 UART1 50Hz 수신 누락, 최악 loop 호출 시간,
모터 명령 갱신·안전 확인 간격 및 독립 만료 지연을 계측한다.

### PC 시험 GUI (`tools/bench_gui.py`)

포트 선택·연결, 로그(저장: `ESP_MAIN/logs/`), 상태판(배터리·모드·모터·ESTOP·센서 1–4·IMU), 명령 버튼,
수동 명령 입력, 누르는 동안만 구동하는 주행 버튼(방향키 가능, Esc·Space는 언제나 STOP)이 있다.
주행은 "바퀴를 바닥에서 띄웠음" 체크 뒤에만 된다. 맨 위 띠가 지금 상태(연결·SW2 대기·움직임 허용·구동 중·ESTOP·배터리 실패)를 한 줄로 알려 주고, 오른쪽 "일요일 시험 순서"는 로그로 확인되는 항목(BAT 값, 응답한 센서 번호, IMU, ESTOP)을 자동으로 채운다. "시험 결과 복사"로 결과를 붙여 넣을 수 있다. 포트 목록의 DEMO를 고르면 보드 없이 화면을 미리 볼 수 있다. tkinter가 있는 일반 Python으로 실행한다
(PlatformIO 내장 Python에는 tkinter가 없음): `python tools/bench_gui.py`.
창 없이 쓰는 모드: `python tools/bench_gui.py --cmd COM5 "STATUS" "MODE DATA" --listen 3`
(DRIVE는 `--allow-drive`가 있어야 하고 한 번에 최대 3초 뒤 STOP). 해석 자체 시험: `python tools/test_bench_gui.py`.

### 색 교정 (벤치 펌웨어 안, GUI "색 교정" 탭)

`CAL START` → 48단계(센서 4 × 빨강·노랑·파랑·검정 × 평평 10mm·들림 20mm·눌림 3mm) → `CAL SAVE`.
교정 중에는 모든 콘솔 줄이 교정으로 가고(`MODE`·`DRIVE` 무시), 모터는 매 루프 브레이크, D/S 로그는 멈춘다.
펌웨어는 250ms마다 `CAL,ms,state,step,sensor,color,pose,samples,stored`(0부터 셈)를 내고,
GUI는 이 줄과 `[CAL][RESULT]`·`[CAL][PREVIEW]`로 화면을 그린다. 측정 결과마다 포화·어두움·흔들림·샘플 수를
참고용으로 판정한다(임계값은 실물에서 확인 전). `CAL STATUS`는 저장 여부만 묻고 모터를 멈추지 않는다.
저장하면 `SensorManager`가 새 값을 바로 쓰므로, GUI의 "색 확인하러 가기"(LOG ON + MODE DATA)로 곧장 확인한다.
교정 중 GUI 연결을 끊으면 `ABORT`를 보내 보드를 일반 벤치 상태로 되돌린다.

### 세션 로그 (문제 생기면 Claude·Codex에 이 파일을 보여 주기)

GUI와 `--cmd` 모드는 `ESP_MAIN/logs/bench_날짜_시간.log`에 모든 줄을 시각과 함께 남긴다.
`<` 보드에서 받은 줄, `>` 보낸 명령, `#` GUI 동작(버튼, 체크, 교정 단계 판정), `!` 경고·오류·Python 예외.
연결 전 GUI 오류는 `logs/bench_gui_errors.log`. `logs/`는 git에 올라가지 않는다.

### IMU(MPU6050) 확인

`ImuMonitor`(`include/sensors/imu_monitor.hpp`)가 부팅 때 IMU를 리셋하고(DEVICE_RESET → 100ms →
깨우기 → WHO_AM_I 확인 → 설정) 10ms마다 데이터 준비 플래그와 값 14바이트를 한 번에 읽는다.
가속·자세 계산은 하지 않고 원시값만 낸다. 모터와 안전 상태에는 영향이 없다.

| 확인 | 정상 |
|---|---|
| `I` 줄 상태 | `READY` |
| WHO_AM_I | `68` (호환 칩은 `70`·`72`·`98`도 허용, 그 밖이면 `FAULT`) |
| dataFresh | `1` = 최근 100ms 안에 새 샘플이 있고 0.5초 안에 값이 변함 |
| samples | 계속 증가 (약 100/초) |
| 기체를 손으로 기울이기 | ax·ay·az가 바뀜 (±8g 설정, 1g ≈ 4096) |

`IMU RESET` 명령으로 다시 리셋한다(모터는 멈추지 않음). 버스 오류가 연속 5번이면 `FAULT`로 멈추고
스스로 다시 시도하지 않는다. 레지스터 설정(±8g, ±1000dps, DLPF 44Hz, 100Hz)과 주소 0x68은 실물에서 확인 전이다.

미구현 목록: IR, 모터 전류(IS), 엔코더.
해당 센서는 읽거나 초기화하지 않으며 엔코더 핀도 초기화하지 않는다.

V2 핀맵 검증·GPIO48 내장 LED 존재는 **사용자 진술 기준(실측하지 않음)**이다.
이번 작업에서 실물 검증을 완료한 것으로 표현하지 않는다.

**GPIO35(EN_2) PSRAM 충돌 가능성:** 모듈 내부 옥탈 PSRAM과 충돌할 수 있다.
PSRAM 설정을 켜지 않는다. 첫 시험은 모터 전원을 분리하고 전원 투입·리셋 순간
EN_2 파형, 부팅 로그, EN 명령 반영 및 SW1 시 양쪽 EN 차단을 확인한다.

**UART0 COM/SiK 동시 연결 주의:** COM과 SiK의 TX 두 개가 ESP32 RX를
동시에 구동하지 않도록 한다.

바퀴를 띄워 SW2·DRIVE·입력 중단·STOP·SW1 순서를 확인한 뒤 바닥 시험을 한다.
컬러 센서는 네 모서리 × 네 색 × 3/10/20mm에서 근접 포화,
먼 유색의 검정 오인 및 경계 혼합을 확인한다.
수광·게인·LED·판정 임계값은 변경하지 않았다.
컴파일·PC 시험 통과는 전기적 브레이크 효과, 제동거리 및 실제 차량 동작의 보증이 아니다.
