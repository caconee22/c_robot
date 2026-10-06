# 컬러타워 필터 비교 테스트

기존 라즈 추적기와 분리된 새 실험입니다. 기존 CV/ESP 코드와 UART 통신을 바꾸지 않습니다.
모든 방식은 같은 영상을 입력받고, 현재 프레임의 가장 큰 색 영역 하나를 선택합니다.
이전 박스를 유지하거나 예측하는 기능은 없습니다. 따라서 검출이 없는 프레임을 숨기지 않습니다.

## 비교 방식

| 별도 실행 파일 | 계산하는 필터 | 최종 판정 |
|---|---|---|
| `test_01_four_vote.py` | BGR, ExG, HSV, Lab | 4개 중 3개 이상 |
| `test_02_no_exg.py` | BGR, HSV, Lab | 3개 중 2개 이상 |
| `test_03_no_lab.py` | BGR, ExG, HSV | 3개 중 2개 이상 |
| `test_04_no_bgr.py` | ExG, HSV, Lab | 3개 중 2개 이상 |
| `test_05_hsv.py` | HSV | HSV 통과 |
| `test_06_normalized.py` | 정규화 RGB 초록 비율 | 비율 조건 통과 |
| `test_07_hsv_normalized.py` | HSV, 정규화 RGB | 둘 다 통과 |
| `test_08_hsv_cascade.py` | 절반 해상도 HSV, 원본 ROI HSV | 축소 후보를 원본에서 재검사 |

각 테스트는 별도 파일로 직접 실행됩니다. 공통 판정·영상 저장 코드만 `comparison_engine.py`에 둬서
버전별로 최소 면적이나 박스 계산이 달라지는 문제를 막았습니다. 빠진 필터는 실제로 계산하지 않습니다.

3/4와 2/3은 판정 규칙이 다릅니다. 따라서 이 비교는 방식별 정확도/속도 비교이지,
정확도가 완전히 고정된 순수 필터 제거 실험은 아닙니다.
특히 BGR와 ExG는 같은 채널 차이를 이용하므로 독립적인 두 표가 아닙니다.

## 공통 설정

`comparison_engine.py` 상단에서 수정합니다.

- HSV: H 35~90, S 60~255, V 35~255. OpenCV H 범위는 0~179입니다.
- Lab: L 15~255, a 0~125, b 135~255. OpenCV 8비트 인코딩 기준입니다.
- BGR: G≥40, G−R≥20, G−B≥20.
- ExG: 2G−R−B≥45, G≥35 및 위의 두 채널 차이 조건.
- 정규화: G/(R+G+B)≥0.42, (G−R)/(합)≥0.08, (G−B)/(합)≥0.08, G≥35.
  나눗셈 대신 동등한 선형 부등식을 OpenCV 변환으로 계산합니다.
- 최종 마스크에만 3×3 닫힘 처리. 작은 먼 표적이 사라질 수 있는 열림/침식은 하지 않습니다.
- 최소 윤곽 면적 8픽셀. 가장 큰 외곽 윤곽의 모멘트 중심과 축 정렬 박스를 반환합니다.
- 원본 1080×1920에서 검사. 08번만 탐색을 540×960에서 하고 후보 주변을 원본으로 확인합니다.
- 08번은 면적 1 이상인 축소 윤곽을 모두 확인하고, 원본 기준 12픽셀 여유를 줍니다.
  후보가 많고 영역이 겹치면 반복 검사 비용이 증가할 수 있습니다. 작은/흐린 타워가 축소에서
  없어지면 복구할 수 없다는 한계도 있습니다.

기존 `config/color_detector_last.json`의 좁은 임계값과 면적 300은 가져오지 않았습니다.
먼 타워를 놓치지 않는 접근을 비교하기 위한 넓은 공통 범위입니다.

## 실행

프로젝트 루트 `D:\C_ROBOT`에서 실행합니다. Python 환경은 이미 있는 `.venv`를 사용합니다.
필요 패키지는 OpenCV와 NumPy뿐입니다.

```powershell
# 한 방식만 실행하고 영상 저장
.\.venv\Scripts\python.exe cv_tests\test_05_hsv.py

# 화면도 표시; Q 또는 ESC 종료
.\.venv\Scripts\python.exe cv_tests\test_02_no_exg.py --show

# 다른 영상과 별도 결과 경로
.\.venv\Scripts\python.exe cv_tests\test_01_four_vote.py "D:\videos\input.mp4" --output "D:\videos\four.mp4"

# 회귀 테스트
.\.venv\Scripts\python.exe cv_tests\test_comparison.py
.\.venv\Scripts\python.exe cv_tests\test_pipeline.py

# 전체 8방식 비교. 재실행은 새 출력 폴더 사용
.\.venv\Scripts\python.exe cv_tests\run_comparison.py --output outputs\cv_comparison_new

# 원본만 담은 독립 정답 검토용 표본 이미지
.\.venv\Scripts\python.exe cv_tests\run_comparison.py --references --output outputs\cv_comparison_new

# 평가/스트레스 테스트/보고서 생성 (기본 휴대폰 영상용 주석 기준)
.\.venv\Scripts\python.exe cv_tests\analyze_comparison.py --output outputs\cv_comparison_new

# 저장한 박스를 같은 시점에 8개 나란히 비교하는 종합 영상
.\.venv\Scripts\python.exe cv_tests\build_overview.py --output outputs\cv_comparison_new

# 저장 영상의 모든 프레임을 다시 디코딩해서 확인
.\.venv\Scripts\python.exe cv_tests\verify_videos.py --output outputs\cv_comparison_new
```

개별 실행은 출력 파일을 덮어쓸 수 있으므로 보존할 결과에는 다른 `--output`을 사용합니다.
전체 비교 실행은 `metrics.csv`가 있는 폴더에 다시 저장하지 않도록 보호합니다.
중도 종료한 비교를 완료 결과로 사용하지 않습니다. `summary.json`이 생성되고
각 영상의 프레임 수와 처음/마지막 디코딩 검사가 성공한 결과만 완료입니다.

## 영상 구성과 시간 해석

각 방식은 810×960, 3열×2행 MP4로 저장합니다. 원본 FPS와 전체 프레임 수를 유지합니다.

```text
원본 + 박스/중심 | 개별 마스크 1 | 개별 마스크 2
개별 마스크 3   | 개별 마스크 4 | 최종 마스크 + 박스
```

사용하지 않는 칸에는 `UNUSED - NOT COMPUTED`가 표시됩니다. 개별 마스크는 닫힘 처리 전,
최종 마스크는 투표/교집합과 닫힘 처리 후입니다. 축소된 마스크 화면은 검토용이며 판정은
원본 마스크에서 수행합니다. 원본의 소리는 저장하지 않습니다.

`detect_ms`에는 색 변환, 마스크 생성/합성, 닫힘 처리, 윤곽 추출, 최대 표적 선택이 포함됩니다.
디코딩·화면 합성·인코딩은 제외합니다. `render_encode_ms`는 영상 합성/저장 호출 시간입니다.
비교에서는 각 방법을 독립 계산하고 매 프레임 실행 순서를 회전합니다.
OpenCV 1스레드로 고정하고 첫 30프레임을 속도 통계에서 제외합니다.
단독 실행은 컴퓨터의 OpenCV 기본 스레드 설정을 사용하므로 시간이 다를 수 있습니다.

`1000/연산시간`은 CV-only 처리 가능 FPS이지 카메라 포함 실제 FPS가 아닙니다.
영상 재생은 원본 약 29.979fps이고, 저장 영상을 30fps로 본다고 검출기가 30fps라는 의미도 아닙니다.
50Hz 목표는 검출기 p95/p99와 20ms 초과 비율로 비교합니다. 라즈5에서는 다시 측정해야 합니다.

## 자체 평가 기준

1. 전체 4,280프레임: 시간, 검출 유무, 박스, 중심, 면적을 기록합니다.
2. 수작업 표본 48개: 순차 디코딩한 원본만 보고 타워 위치를 정합니다. 45개는 타워가 보이고 3개는 화면 밖입니다.
3. 정답 박스 15% 여유 범위 안에 중심이 있고 IoU≥0.25면 정답입니다.
   표본 박스는 축소 이미지의 육안 추정이므로 픽셀 단위 정밀 정답이 아닙니다.
4. 정답/다른 물체 선택/검출 없음/타워 없는 장면의 오검출을 구분합니다.
5. 45개 타워 표본에서 타워 주변을 회색으로 지워 초록 배경을 잡는 성향을 검사합니다.
   실제 촬영한 타워 부재 영상이 아니므로 합성 오검출 진단으로 따로 집계합니다.
6. 표본 12개에 밝기 0.5배/1.4배, 노이즈 σ12, 5×5 Gaussian 흐림을 각각 적용한 48개 스트레스 검사를 합니다.
7. 현재의 재현율 우선 요청에 따라 표본 정답 수 → 스트레스 정답 수 → p95 시간 순으로 정렬합니다.
   충분한 실제 음성 영상이 없어 이 순위를 대회용 오검출 안전성 순위로 해석하면 안 됩니다.
8. 별도로 p99≤20ms인 방식만 남겨 50Hz 연산 예산 안의 후보를 정합니다.
   이 PC의 검출만 측정한 조건이므로 카메라·UART·라즈 처리까지 보장하지 않습니다.

전체 프레임을 사람이 모두 정답 표시하지 않았으므로 **한 번도 놓치지 않았다는 보장은 할 수 없습니다.**
검출률이 높아도 배경의 초록 매트/식물을 잡으면 실패입니다. 색 필터만으로 같은 색의 다른 물체와
원통을 완전히 구분할 수 없습니다. 박스 흔들림은 카메라 자체 기동과 섞이므로 정확도 점수에
무조건 넣지 않습니다. 다음 단계에서 형태/시간적 일관성을 별도로 비교할 수 있습니다.

휴대폰 영상에서 `CAP_PROP_POS_FRAMES`로 이동해 읽은 프레임과 순차 디코딩 프레임이
다른 경우를 실제로 발견했습니다. 따라서 평가용 표본/스트레스/후보 진단은 `video_frames.py`로
처음부터 순서대로 읽어 같은 번호의 프레임을 사용합니다. seek 기반 중간 점수는 최종 결과가 아닙니다.
영상에 표시한 초 단위 시간은 프레임 번호/명목 FPS이며 휴대폰 가변 FPS의 정확한 촬영 시각이 아닙니다.

## 구성 파일

- `comparison_engine.py`: 새 필터, 최종 판정, 박스/중심 계산, 공통 영상 합성.
- `video_frames.py`: 표본과 순차 처리 CSV 사이의 프레임 번호를 일치시키는 순차 디코딩 유틸리티.
- `test_01_*.py` ~ `test_08_*.py`: 각각 독립 실행 가능한 실험 진입점.
- `run_comparison.py`: 전체 프레임 비교, 8개 마스크 영상, CSV, 속도 요약, 출력 영상 검사.
- `annotations_phone.json`: 독립 수작업 정답 표본. 다른 영상에는 새 정답 파일을 만들어 사용합니다.
- `analyze_comparison.py`: 표본 평가, 합성 스트레스/부재 검사, 순위와 보고서.
- `build_overview.py`: 원본과 저장 CSV로 전체 8방식 종합 비교 영상 생성. 재검출하지 않습니다.
- `finalize_results.py`: 저장 CSV와 영상을 검증하고 요약 재생성. 집계만 중단된 경우 검출 재실행 없이 사용합니다.
- `test_pipeline.py`: 5프레임 영상을 만들어 EOF까지 처리하고 8개 출력 영상/CSV/요약/검출 잔류를 검사합니다.
- `verify_videos.py`: 개별 8영상과 종합 영상의 모든 프레임을 실제로 디코딩해 누락/잘림/크기를 검사합니다.
- `diagnose_candidates.py`: 색 마스크에서 타워 후보가 사라진 것인지, 후보는 있지만 큰 배경을 선택한 것인지 구분합니다.
- `test_comparison.py`: 빈 화면/회색/다른 색 배제, 최대 표적과 중심, 입력 변경 방지,
  사용하지 않는 필터 미계산, 이전 박스 잔류 방지, 홀수 해상도와 출력 화면 크기 검사.

최종 실험 결과는 `outputs/cv_comparison_phone/REPORT.md`에 있습니다.
[최종 해석과 추천](RESULTS_PHONE_20261006.md)에는 전체 영상 링크, 실패 원인, 순위의 한계가 있습니다.
`outputs/cv_comparison_phone_draft`는 합성부 최적화 전 중단한 초안이며 최종 비교에 사용하지 않습니다.

## 새 영상 재실험: 210419904

`KakaoTalk_20261006_210419904.mp4`는 별도 결과 폴더에 저장합니다. 기존 결과를 덮어쓰지 않습니다.
사용자 요청에 따라 큰 초록 배경 선택은 순위에서 제외하고, **타워 후보 자체가 최종 마스크에 남는지**를
우선 비교합니다. 최대영역 박스가 타워를 선택했는지는 참고 항목으로 분리합니다.

```powershell
.\.venv\Scripts\python.exe cv_tests\run_comparison.py --video img\KakaoTalk_20261006_210419904.mp4 --output outputs\cv_comparison_210419904
.\.venv\Scripts\python.exe cv_tests\evaluate_candidates.py --video img\KakaoTalk_20261006_210419904.mp4 --annotations cv_tests\annotations_210419904.json --output outputs\cv_comparison_210419904
.\.venv\Scripts\python.exe cv_tests\build_overview.py --output outputs\cv_comparison_210419904 --annotations cv_tests\annotations_210419904.json
.\.venv\Scripts\python.exe cv_tests\verify_videos.py --output outputs\cv_comparison_210419904
```

- `annotations_210419904.json`: 새 원본의 독립 육안 표본 48개. 영상 SHA256으로 잘못된 정답 파일 사용을 차단합니다.
- `evaluate_candidates.py`: 모든 윤곽 중 타워와 대응하는 후보를 찾습니다. 48개 표본 및 합성 스트레스 48건을 평가합니다.
  전체 비교가 끝난 뒤 실행하면 원본 순차 재검출 박스와 저장 CSV 박스도 대조합니다.
- `grid_references.py`: 타워 정답 박스를 육안으로 정할 때 축소 원본에 좌표 안내선을 추가합니다. 검출값은 사용하지 않습니다.
- `test_candidate_evaluation.py`: 큰 배경/진짜 타워 분리, 배경만 있는 장면, 작은 픽셀 잡음을 검사합니다.
- 새 결과: `outputs/cv_comparison_210419904/REPORT.md`, 종합 영상 1개, 개별 마스크 영상 8개.

표본 48개가 모두 성공하더라도 전체 4,297프레임 무손실 검출 보장은 아닙니다.
배경을 평가에서 제외하는 것과 실제 최대영역 검출기의 배경 선택을 고치는 것은 다릅니다.
이번 실험은 필터 비교이며 기존 ESP/라즈 제어 코드는 바꾸지 않습니다.
