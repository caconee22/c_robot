"""Evaluate saved detections against independent sparse annotations and stress probes."""

import argparse
import csv
import json
from pathlib import Path
import platform

import cv2
import numpy as np

from comparison_engine import METHODS, ROOT, VIDEO, detect
from video_frames import selected_frames


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    intersection = max(0, min(ax + aw, bx + bw) - max(ax, bx)) * max(0, min(ay + ah, by + bh) - max(ay, by))
    union = aw * ah + bw * bh - intersection
    return intersection / union if union else 0.0


def matches(target, truth):
    if target is None or truth is None:
        return False
    x, y, w, h = truth
    cx, cy = target["center"]
    # Approximate manual boxes: modest center tolerance plus overlap requirement.
    return (x - .15 * w <= cx <= x + 1.15 * w
            and y - .15 * h <= cy <= y + 1.15 * h
            and iou(target["box"], truth) >= .25)


def row_target(row):
    if int(row["found"]) == 0:
        return None
    return {"box": [float(row[key]) for key in ("x", "y", "w", "h")],
            "center": [float(row[key]) for key in ("cx", "cy")]}


def longest_run(flags):
    longest = current = 0
    for value in flags:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def read_annotations(path, source_width, source_height):
    data = json.loads(path.read_text(encoding="utf-8"))
    width, height = data["coordinate_size"]
    sx, sy = source_width / width, source_height / height
    labels = {}
    for entry in data["annotations"]:
        box = entry["box"]
        if box:
            box = [box[0] * sx, box[1] * sy, box[2] * sx, box[3] * sy]
        labels[entry["frame"]] = {**entry, "box": box}
    return labels


def label_sheets(video, labels, output):
    output.mkdir(parents=True, exist_ok=True)
    sheet = None
    for number, (index, frame) in enumerate(selected_frames(video, labels)):
        label = labels[index]
        if number % 6 == 0:
            sheet = np.zeros((1280, 1080, 3), np.uint8)
        if label["box"]:
            x, y, w, h = map(round, label["box"])
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 4)
        preview = cv2.resize(frame, (360, 640))
        cv2.putText(preview, f"GT frame={index} {'visible' if label['box'] else 'absent'}",
                    (5, 24), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1, cv2.LINE_AA)
        row, col = (number % 6) // 3, number % 3
        sheet[row * 640:(row + 1) * 640, col * 360:(col + 1) * 360] = preview
        if number % 6 == 5 or number == len(labels) - 1:
            cv2.imwrite(str(output / f"labels_{number // 6:02d}.jpg"), sheet)


def probes(video, labels, threads):
    """Synthetic target removal and perturbations; never label these real-world accuracy."""
    cv2.setNumThreads(threads)
    result = {name: {"removed_fp": 0, "removed_n": 0, "stress_hits": 0, "stress_n": 0,
                     "stress_by_type": {}} for name in METHODS}
    rng = np.random.default_rng(72388)
    stress_indices = {0, 182, 546, 819, 1183, 1547, 2458, 2822, 3459, 3732, 4096, 4279}
    for index, frame in selected_frames(video, [i for i, item in labels.items() if item["box"]]):
        label = labels[index]
        x, y, w, h = label["box"]
        erased = frame.copy()
        x0, y0 = max(0, int(x - .2 * w - 9)), max(0, int(y - .2 * h - 9))
        x1 = min(frame.shape[1], int(x + 1.2 * w + 9))
        y1 = min(frame.shape[0], int(y + 1.2 * h + 9))
        erased[y0:y1, x0:x1] = (100, 100, 100)
        for name in METHODS:
            target = detect(erased, name)[2]
            result[name]["removed_fp"] += target is not None
            result[name]["removed_n"] += 1
        if index not in stress_indices:
            continue
        perturbations = {
            "dark_0.5": np.clip(frame.astype(np.float32) * .5, 0, 255).astype(np.uint8),
            "bright_1.4": np.clip(frame.astype(np.float32) * 1.4, 0, 255).astype(np.uint8),
            "noise_sigma12": np.clip(frame.astype(np.float32) + rng.normal(0, 12, frame.shape), 0, 255).astype(np.uint8),
            "blur5": cv2.GaussianBlur(frame, (5, 5), 0),
        }
        for kind, perturbed in perturbations.items():
            for name in METHODS:
                hit = matches(detect(perturbed, name)[2], label["box"])
                result[name]["stress_hits"] += hit
                result[name]["stress_n"] += 1
                item = result[name]["stress_by_type"].setdefault(kind, {"hit": 0, "n": 0})
                item["hit"] += hit
                item["n"] += 1
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "cv_comparison_phone")
    parser.add_argument("--annotations", type=Path, default=Path(__file__).with_name("annotations_phone.json"))
    parser.add_argument("--labels-only", action="store_true")
    args = parser.parse_args()
    if args.labels_only:
        labels = read_annotations(args.annotations, 1080, 1920)
        label_sheets(VIDEO, labels, args.output / "label_review")
        return
    info = json.loads((args.output / "summary.json").read_text(encoding="utf-8"))
    annotation_info = json.loads(args.annotations.read_text(encoding="utf-8"))
    if annotation_info.get("source_sha256") not in (None, info["source_sha256"]):
        raise RuntimeError("Annotations belong to a different source video")
    labels = read_annotations(args.annotations, info["source_width"], info["source_height"])
    video = Path(info["source"])
    rows = {name: [] for name in METHODS}
    with (args.output / "metrics.csv").open(encoding="utf-8") as file:
        for row in csv.DictReader(file):
            rows[row["method"]].append(row)
    scores = {}
    review_rows = []
    for name, method_rows in rows.items():
        if [int(row["frame"]) for row in method_rows] != list(range(info["processed_frames"])):
            raise RuntimeError(f"Duplicate, missing or reordered metrics: {name}")
        tp = wrong = empty = tn = fp = 0
        overlaps = []
        lookup = {int(row["frame"]): row for row in method_rows}
        for index, label in labels.items():
            if index not in lookup:
                raise RuntimeError(f"Annotated frame {index} missing for {name}")
            target = row_target(lookup[index])
            truth = label["box"]
            overlap = iou(target["box"], truth) if target and truth else 0
            if truth:
                hit = matches(target, truth)
                tp += hit
                empty += target is None
                wrong += target is not None and not hit
                overlaps.append(overlap)
                status = "correct" if hit else "empty" if target is None else "wrong"
            else:
                tn += target is None
                fp += target is not None
                status = "true_negative" if target is None else "false_positive"
            review_rows.append({"frame": index, "method": name, "status": status, "iou": overlap})
        visible = tp + empty + wrong
        scores[name] = {"visible_n": visible, "correct": tp, "miss": empty, "wrong": wrong,
                        "absent_n": tn + fp, "true_negative": tn, "false_positive": fp,
                        "recall_percent": 100 * tp / visible,
                        "median_iou": float(np.median(overlaps)),
                        "longest_empty_frames": longest_run([int(row["found"]) == 0 for row in method_rows])}
    print("Running target-removal and light/noise/blur probes...", flush=True)
    stress = probes(video, labels, info["opencv_threads"])
    # Recall first, then correct detections under perturbation, then latency.
    # This ranking intentionally does NOT claim adequate competition false-positive safety.
    ranking = sorted(METHODS, key=lambda name: (-scores[name]["correct"],
                                               -stress[name]["stress_hits"],
                                               info["methods"][name]["p95_ms"]))
    realtime = [name for name in ranking if info["methods"][name]["p99_ms"] <= 20]
    result = {"machine": platform.platform(), "ranking_rule": "Sparse true-target recall, stress recall, p95 latency",
              "ranking": ranking, "realtime_p99_under20": realtime,
              "realtime_recommendation": realtime[0] if realtime else None,
              "scores": scores, "synthetic_probes": stress,
              "limitations": ["48 approximate manual boxes, not full-video ground truth.",
                              "Only 3 naturally absent sampled frames; false-positive estimates are weak.",
                              "Target removal and perturbations are synthetic, not independent competition footage.",
                              "PC timings are not Raspberry Pi 5 timings."]}
    (args.output / "evaluation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    with (args.output / "annotation_results.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=("frame", "method", "status", "iou"))
        writer.writeheader()
        writer.writerows(review_rows)
    visible_n = sum(item["box"] is not None for item in labels.values())
    absent_n = len(labels) - visible_n
    stress_n = next(iter(stress.values()))["stress_n"]
    lines = ["# HSV / 다중 필터 비교 결과", "", "같은 원본의 모든 프레임을 검사하고 마스크 포함 영상 8개를 저장했습니다.", "",
             f"원본: {info['source_frames']}프레임, {info['source_fps']:.3f}fps, "
             f"{info['source_width']}×{info['source_height']}. OpenCV {info['opencv']}, "
             f"{info['opencv_threads']}스레드. 첫 30프레임은 시간 통계에서 제외.", "",
             "## 공통 평가 기준", "",
             f"원본을 순서대로 읽어 정한 {len(labels)}개 표본(타워 있음 {visible_n}개, 없음 {absent_n}개)을 사용합니다. "
             "중심점이 정답 박스의 15% 여유 범위 안에 있고 IoU≥0.25이면 타워를 맞혔다고 봅니다. "
             "작고 흐린 표적의 수작업 박스 오차를 허용한 기준이며 정밀 분할 정확도가 아닙니다.", "",
             "검출 없음과 다른 물체 검출은 구분합니다. 전체 검출률은 정답률이 아닙니다. "
             "이전 박스를 유지하거나 추정해서 검출 성공으로 세지 않습니다.", "",
             "## 결과", "",
             f"| 방식 | 평균 ms | p95 ms | p99 ms | 20ms 초과 % | 실제 타워 / {visible_n} | 오선택 | 미검출 | 스트레스 / {stress_n} |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name in METHODS:
        speed, quality, probe = info["methods"][name], scores[name], stress[name]
        lines.append(f"| {name} | {speed['mean_ms']:.2f} | {speed['p95_ms']:.2f} | {speed['p99_ms']:.2f} | "
                     f"{speed['over20_percent']:.2f} | {quality['correct']} | {quality['wrong']} | "
                     f"{quality['miss']} | {probe['stress_hits']} |")
    lines += ["", "## 자동 순위", "", " → ".join(ranking), "",
              "현재 요청의 우선순위인 놓치지 않기를 반영해 정답 표본 수, 스트레스 정답 수, p95 시간 순으로 정렬했습니다. "
              "이 순위는 대회 투입 승인이나 통계적 유의성 판정이 아닙니다.", "",
              "## 50Hz 연산 예산을 적용한 후보", "",
              "p99≤20ms인 방식만 남긴 순위: " + (" → ".join(realtime) or "없음"), "",
              "이 기준은 PC의 검출 연산만 봅니다. 카메라 입력·UART·라즈 처리까지 50Hz가 보장되는 것은 아닙니다. "
              "20ms를 모두 검출에 쓰는 방식은 주변 처리에 여유가 없으므로 추가 여유가 필요합니다.", "",
              "## 거짓 검출과 끊김", "",
              f"| 방식 | 전체 검출 없음 | 최장 검출 없음 프레임 | 자연 부재 오검출 / {absent_n} | 합성 타워 제거 오검출 / {visible_n} |",
              "|---|---:|---:|---:|---:|"]
    for name in METHODS:
        lines.append(f"| {name} | {info['methods'][name]['empty']} | {scores[name]['longest_empty_frames']} | "
                     f"{scores[name]['false_positive']} | {stress[name]['removed_fp']} |")
    lines += ["", "합성 타워 제거는 타워 주변을 회색으로 지운 실험입니다. "
              "남은 초록 배경을 잡는 성향을 보는 진단일 뿐, 실제 대회 오검출률이 아닙니다. "
              "최장 검출 없음에는 타워가 실제로 화면 밖에 있는 구간도 포함됩니다.", "",
              "## 해석에 필요한 조건", "",
              "- 4필터는 3/4 투표, 3필터는 2/3 투표입니다. 따라서 차이는 순수 계산량 제거뿐 아니라 판정 규칙 변화도 포함합니다.",
              "- 모든 방식의 개별 HSV/Lab/BGR/ExG 조건, 최소 면적 8, 3×3 닫힘 처리는 공통입니다. 기존 좁은 저장 설정은 가져오지 않았습니다.",
              "- 타워 후보는 가장 큰 색 영역입니다. 색만으로 배경의 같은 초록색과 원통을 확실히 구별할 수 없습니다.",
              "- 연산시간은 색 처리·합성·윤곽·표적 선택을 포함합니다. 디코딩, 화면 합성, 저장은 제외하고 별도 CSV 열에 기록합니다.",
              "- CV capacity는 연산시간의 역수이며 카메라 포함 실제 처리 FPS가 아닙니다. 저장 영상은 원본 FPS이고 모든 프레임을 유지합니다.",
              "- 프레임마다 실행 순서를 회전했습니다. 프레임 간 색공간이나 마스크를 공유하지 않았습니다.",
              "- 휴대폰 영상의 프레임 이동(seek)이 순차 디코딩과 다른 장면을 반환하는 것을 발견해, 정답 표본/스트레스/실패 진단을 모두 순차 디코딩으로 수정했습니다. 기존 seek 기반 중간 평가는 폐기했습니다.",
              "- 합성 스트레스는 표본 12개에 밝기 0.5배/1.4배, 노이즈 σ12, Gaussian 5×5를 각각 적용했습니다.",
              "- 각 출력 영상은 프레임 수·해상도·첫/마지막 프레임 디코딩을 검사했습니다.", "",
              "## 파일", "",
              "- `01_*.mp4` ~ `08_*.mp4`: 원본 검출 화면 + 개별 마스크 + 최종 마스크.",
              "- `metrics.csv`: 모든 프레임의 시간, 중심점, 박스, 면적, 검출 유무.",
              "- `annotation_results.csv`: 정답 표본별 맞음/오선택/미검출과 IoU.",
              "- `summary.json`, `evaluation.json`: 기계가 다시 분석할 수 있는 원자료와 요약.",
              "- `references/`, `label_review/`: 무표시 원본 표본 및 독립 정답 표시 화면.", ""]
    (args.output / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    label_sheets(video, labels, args.output / "label_review")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
