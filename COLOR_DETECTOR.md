# 초록색 상대 로봇 검출기

## 실행

프로젝트 루트 `D:\C_ROBOT`에서 실행한다.

```powershell
.\.venv\Scripts\python.exe scripts\color_detector_ui.py
```

기본 입력은 `dataset\videos\irc_highlight_1080p50.mp4`이다.

기본 실행은 단계별 튜닝 UI이다. 화면은 다음 세 창으로 분리된다.

- `Green detector tuning wizard`: 원본, 현재 마스크, 단계별 가이드
- `1 Playback`: 영상 위치
- `2 Current stage controls`: 현재 필터에 필요한 슬라이더만 표시

영상은 초록 원통이 보이는 92% 지점에서 일시정지된 상태로 시작한다.

기존 전체 설정 UI가 필요하면 다음처럼 실행한다.

```powershell
.\.venv\Scripts\python.exe scripts\color_detector_ui.py --advanced-ui
```

## 시험 영상 다시 만들기

```powershell
.\.venv\Scripts\python.exe scripts\prepare_test_video.py
```

원본은 1280x720 29.97FPS이므로 출력의 추가 프레임은 타임스탬프에 맞춰 반복된다. 이 영상은 1080p 처리 부하와 UI 시험용이며, 실제 Camera Module 3 Wide의 세부 묘사를 재현하지 않는다.

## 단계별 튜닝 순서

1. BGR의 G-R, G-B 차이
2. ExG (`2G-R-B`)
3. HSV 초록색 범위
4. Lab 초록색 범위
5. 네 방식 중 3개 이상이 동의한 픽셀 유지
6. 타원형 3x3 Closing 1회
7. 최대 2개 후보의 면적, 채움률, 색상 신뢰도 조정
8. 최종 영상 확인 및 설정 저장

기본 융합 방식은 `Vote 3`이다. `Core+Support`는 더 넓은 색 영역을 복원하지만 장면이 복잡하면 느려질 수 있다.

## 융합 모드

`Fusion 0-8` 트랙바를 사용한다.

| 값 | 방식 |
|---:|---|
| 0 | BGR |
| 1 | ExG |
| 2 | HSV |
| 3 | Lab |
| 4 | OR |
| 5 | AND |
| 6 | Vote 2 |
| 7 | Vote 3 |
| 8 | Core+Support |

## 스무딩 모드

`Smooth 0-3` 트랙바를 사용한다.

| 값 | 방식 |
|---:|---|
| 0 | 없음 |
| 1 | Gaussian |
| 2 | Median |
| 3 | Bilateral |

기본값은 스무딩 없음이다.

## 단계형 UI 키보드

| 키 | 동작 |
|---|---|
| `[` / `]` | 이전/다음 튜닝 단계 |
| M | 재생 모드와 단일 필터 튜닝 모드 전환 |
| Space | 재생 또는 일시정지 |
| N | 다음 프레임 |
| P | 이전 프레임 |
| 마우스 드래그 | 초록 원통 자동 튜닝 영역 선택 |
| A | 현재 BGR/ExG/HSV/Lab 필터 자동 튜닝 |
| C | 자동 튜닝 영역 삭제 |
| S | 원본, 결과, 모든 마스크 저장 |
| W | 현재 설정 저장 |
| L | 저장한 설정 불러오기 |
| R | 기본 설정 복원 |
| B | 벤치마크 기록 시작 또는 종료 |
| E | 전체 9분할 분석 영상 내보내기 시작 |
| X | 진행 중인 영상 내보내기 중단 |
| Q | 종료 |

현재 설정은 `config\color_detector_last.json`에 저장된다. 스냅샷과 벤치마크 CSV는 `output\color_detector` 아래에 저장된다.

메인 창 오른쪽의 `EXPORT FULL VIDEO` 버튼을 누르면 현재 슬라이더 설정으로 전체 영상을 만든다. `STOP EXPORT`는 진행 중인 출력을 중단하고, `SAVE SNAPSHOT`과 `SAVE SETTINGS`는 각각 현재 화면과 현재 설정을 저장한다. 영상 출력 중에도 UI에서 재생하거나 설정을 확인할 수 있다. 다만 출력 작업은 시작 시점의 설정 복사본을 사용하므로, 슬라이더를 바꾼 뒤에는 다시 출력해야 변경값이 반영된다.

## 모든 마스크 영상 출력

내보낸 영상은 1920x1080, 50FPS의 3x3 비교 화면이다.

| 위치 | 내용 |
|---|---|
| 왼쪽 위 | 원본 위 최종 검출 박스와 중심점 |
| 가운데 위 | 융합과 모폴로지를 마친 최종 마스크 |
| 오른쪽 위 | 최대 2개로 선택된 후보만 표시한 마스크 |
| 가운데 줄 | BGR, ExG+색 우세 조건, HSV 마스크 |
| 아래 줄 | Lab, 엄격한 공통 영역, 필터 투표 수 |

UI 없이 저장된 설정으로 직접 출력하려면 다음 명령을 사용한다.

```powershell
.\.venv\Scripts\python.exe scripts\color_detector_export.py --config config\color_detector_last.json --output output\color_detector\exports\green_detector_all_masks.mp4
```

영상과 함께 같은 이름의 `.json` 파일도 생성된다. 이 파일에는 사용한 입력 영상, 설정, 처리 시간과 출력 프레임 수가 기록된다.

자동 튜닝할 때는 녹색 원통을 가능한 한 여백 없이 마우스로 감싼 뒤 `A`를 누른다. 자동 계산 결과의 원통 유지율과 화면 바깥 흰색 비율이 가이드 아래에 표시된다. 자동 결과가 완벽하지 않으면 현재 단계의 작은 슬라이더 창에서 값만 조금씩 보정한다.

## UI 없이 벤치마크

```powershell
.\.venv\Scripts\python.exe scripts\color_detector_ui.py --benchmark --start-frame 12900 --max-frames 200
```

평균, p95, p99 처리시간과 50FPS, 30FPS, 24FPS 주기 초과 횟수를 출력한다.

## 라즈베리파이용 경량 검출

`scripts\color_detection_lightweight.py`에는 다음 순서의 경량 파이프라인이 구현되어 있다.

1. 1920x1080 입력을 960x540 탐색 영상으로 축소
2. 탐색 영상에서 BGR과 ExG를 한 번의 채널 분리로 계산
3. 초록색 후보를 최대 8개 ROI로 변환
4. 원본 해상도의 ROI 안에서만 BGR, ExG, HSV, Lab Vote 3 재검증
5. 기존과 같은 후보 조건으로 최대 2개 선택

기존 방식과 경량 방식을 같은 프레임으로 비교하려면 다음 명령을 사용한다.

```powershell
.\.venv\Scripts\python.exe scripts\benchmark_lightweight_compare.py --start-frame 12900 --max-frames 200
```

영상 전체를 일정 간격으로 표본 검사하려면 다음처럼 실행한다.

```powershell
.\.venv\Scripts\python.exe scripts\benchmark_lightweight_compare.py --start-frame 0 --max-frames 200 --frame-step 70
```

비교 결과 CSV와 JSON은 `output\color_detector\benchmarks`에 저장된다. 속도뿐 아니라 검출 유무 일치율, 후보 개수 일치율, 중심점 거리, 선택 마스크 IoU도 함께 기록한다.

현재 PC에서 저장된 튜닝값으로 측정한 결과는 다음과 같다. 이 수치는 Raspberry Pi 5 실측값이 아니라 두 알고리즘의 상대적인 연산량 비교다.

| 시험 | 기존 1080p 전체 처리 | 경량 ROI 처리 | 향상 |
|---|---:|---:|---:|
| 12900번부터 연속 200프레임 | 31.08ms, 32.18FPS | 8.11ms, 123.28FPS | 3.83배 |
| 전체 영상 70프레임 간격 200장 | 31.29ms, 31.96FPS | 7.50ms, 133.28FPS | 4.17배 |

두 시험 모두 기존 방식과 검출 유무 및 후보 개수가 100% 일치했고, 동시에 검출된 주 후보의 평균 중심점 오차는 0픽셀이었다. 최종 판단은 Camera Module 3 Wide가 연결된 Raspberry Pi 5에서 다시 측정해야 한다.

## Raspberry Pi 5에서 간단히 실행

라즈베리파이에는 프로젝트 폴더를 복사한 뒤 터미널에서 다음 명령만 실행한다.

```bash
cd C_ROBOT
bash setup_raspberry_pi.sh
./run_pi_tracker.sh
```

기본 설정은 Camera Module 3 Wide, 1920x1080, 50FPS, 960x540 탐색 스트림이다. 화면 표시와 영상 저장은 하지 않고 다음과 같이 표적 좌표만 출력한다.

```text
frame=120 process=9.42ms targets=2 #1 x=1041.2 y=451.8 area=3247 conf=0.96 #2 x=252.4 y=547.1 area=834 conf=0.93
```

검출 화면을 직접 보려면 다음처럼 실행한다.

```bash
./run_pi_tracker.sh --preview
```

로봇 프로그램에서 읽기 쉬운 JSON Lines 형식은 다음과 같다.

```bash
./run_pi_tracker.sh --json
```

출력이 너무 많으면 10프레임마다 한 번만 표시한다.

```bash
./run_pi_tracker.sh --print-every 10
```

실전에서는 `--preview`를 사용하지 않는 것이 가장 빠르다. 종료는 `Ctrl+C`를 누른다. 카메라 없이 PC 영상으로 같은 실행 파일을 시험할 수도 있다.

```powershell
.\.venv\Scripts\python.exe scripts\pi_green_tracker.py --source dataset\videos\irc_highlight_1080p50.mp4 --start-frame 13091 --max-frames 20
```

Windows에서 `--source`를 생략하면 기본 시험 영상 `dataset\videos\irc_highlight_1080p50.mp4`를 사용한다. 따라서 다음 명령만 실행해도 Picamera2 없이 영상 처리가 시작된다.

```powershell
.\.venv\Scripts\python.exe scripts\pi_green_tracker.py
```

검출 화면까지 보려면 `--preview`를 추가한다.

미리보기 창 아래의 `Time` 막대를 움직이면 원하는 영상 프레임으로 즉시 이동한다. 세팅 슬라이더는 표시되지 않는다. 영상 끝에서 다시 반복하면서 성능을 계속 보려면 다음처럼 실행한다.

```powershell
.\.venv\Scripts\python.exe scripts\pi_green_tracker.py --start-frame 12900 --preview --quiet --loop
```

미리보기 위쪽의 버튼과 단축키는 다음과 같다.

| 버튼 | 키 | 동작 |
|---|---|---|
| `MASKS OFF/ON` | M | 오른쪽에 Proposal, BGR, ExG, HSV, Lab, Core, Support, Fused, Cleaned, Selected 마스크 표시 |
| `EXPORT FULL` | E | 원본 영상의 처음부터 끝까지 마스크 화면을 포함한 MP4 하나로 자동 저장 |

전체 출력 파일은 `output\color_detector\exports`에 1280x720, 50FPS MP4로 저장된다. 버튼은 한 번만 누르면 되고 현재 재생 위치와 관계없이 0프레임부터 마지막 프레임까지 별도 작업으로 처리한다. 마스크를 켠 상태의 현재 PC 측정은 검출 평균 8.92ms, 화면 전체 반복 평균 19.65ms, 처리량 50.89FPS였다.

## 화면 표시 성능 비교

미리보기 화면에는 최근 60프레임 기준 검출 시간과 전체 화면 반복 FPS가 표시된다.

화면 없이 벤치마크:

```powershell
.\.venv\Scripts\python.exe scripts\pi_green_tracker.py --start-frame 12900 --max-frames 210 --warmup-frames 10 --benchmark --quiet
```

1280x720 검출 화면을 띄운 상태의 벤치마크:

```powershell
.\.venv\Scripts\python.exe scripts\pi_green_tracker.py --start-frame 12900 --max-frames 210 --warmup-frames 10 --benchmark --quiet --preview
```

현재 PC에서 동일한 200프레임을 측정한 결과다.

| 모드 | 검출 평균 | 전체 반복 평균 | 실제 처리량 | 20ms 초과 |
|---|---:|---:|---:|---:|
| 화면 없음 | 8.09ms | 11.95ms | 83.70FPS | 1회 |
| 1280x720 화면 표시 | 8.08ms | 16.28ms | 61.43FPS | 2회 |

화면 표시 때문에 평균 4.33ms가 추가됐지만 50FPS 처리 예산인 20ms 안에 들어왔다. 초기화 또는 영상 디코딩으로 생긴 단발성 지연도 초과 횟수에 포함된다. 결과 JSON은 `output\color_detector\benchmarks`에 저장된다.
