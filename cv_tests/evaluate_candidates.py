"""Score actual tower candidates; unrelated larger green backgrounds are not penalized."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from comparison_engine import METHODS, MIN_AREA, detect
from analyze_comparison import matches, read_annotations, label_sheets, row_target
from video_frames import selected_frames

def candidate_hit(mask, truth):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) < MIN_AREA:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        candidate = {"box": list(cv2.boundingRect(contour)),
                     "center": [moments["m10"]/moments["m00"], moments["m01"]/moments["m00"]]}
        if matches(candidate, truth):
            return True
    return False

def report(output):
    info = json.loads((output/"summary.json").read_text(encoding="utf-8"))
    evaluation = json.loads((output/"candidate_evaluation.json").read_text(encoding="utf-8"))
    if info["source_sha256"] != evaluation["source_sha256"]:
        raise RuntimeError("Evaluation and metrics sources differ")
    ranked = sorted(METHODS, key=lambda n:(-evaluation["methods"][n]["candidate_hits"],
                        -evaluation["methods"][n]["stress_hits"],info["methods"][n]["p95_ms"]))
    lines = ["# 새 영상 필터 비교 — 210419904", "",
        f"원본: `{info['source']}`", "",
        f"{info['source_width']}×{info['source_height']}, {info['source_fps']:.2f}fps, "
        f"{info['processed_frames']}/{info['source_frames']}프레임. 모든 프레임을 8개 방식에 입력했습니다.", "",
        "## 결과", "", "큰 초록 배경을 선택한 것은 순위에서 제외합니다. 실제 타워에 대응하는 후보가 최종 마스크에 남았는지 평가합니다.", "",
        "| 방식 | 타워 후보/48표본 | 합성 스트레스/48 | 최대영역 선택 정답(참고) | 평균 ms | p99 ms | 전체 검출없음 프레임 |",
        "|---|---:|---:|---:|---:|---:|---:|"]
    for name in ranked:
        e,t=evaluation["methods"][name],info["methods"][name]
        lines.append(f"| {name} | {e['candidate_hits']}/{e['visible']} | {e['stress_hits']}/{e['stress_n']} | "
                     f"{e['largest_hits']}/{e['visible']} | {t['mean_ms']:.2f} | {t['p99_ms']:.2f} | {t['empty']} |")
    fast = [n for n in ranked if info["methods"][n]["p99_ms"] <= 20]
    baseline = info["methods"]["01_four_vote"]["mean_ms"]
    lines += ["", f"표본 후보 유지 → 스트레스 후보 유지 → p95 시간 순 후보: `{ranked[0]}`.",
              f"이 PC에서 p99 검출시간 20ms 이하인 후보 중 위 기준 선두: `{fast[0] if fast else '없음'}`.","",
              "4필터 대비 평균 검출시간 감소:", ""]
    for name in ("02_no_exg","03_no_lab","04_no_bgr"):
        lines.append(f"- {name}: {(1-info['methods'][name]['mean_ms']/baseline)*100:.1f}%")
    lines += ["", "## 평가 범위와 사용법", "",
        "- 48개 정답 표본은 순차 디코딩 원본만 보고 육안으로 정한 대략적인 박스입니다. 중심이 정답 박스 ±15% 범위 안에 있고 IoU≥0.25면 일치합니다.",
        "- 스트레스는 표본 중 매 4번째(12개)에 밝기 0.5배/1.4배, 노이즈 σ12, Gaussian 5×5를 가한 합성 48건입니다. 실전 정확도가 아닙니다.",
        "- 4필터는 3/4 투표, 3필터는 2/3 투표입니다. 속도는 제거한 계산량을 비교하지만 정확도 차이는 투표 기준 변화의 영향도 포함합니다.",
        "- 전체 프레임에 수작업 정답은 없습니다. 48/48이어도 영상 전체에서 한 번도 놓치지 않았음을 증명하지 않습니다.",
        "- 전체 검출없음 프레임 수는 실제 놓침 수가 아닙니다. 타워가 화면 밖인 경우도 포함하며, 배경 검출은 검출있음으로 집계됩니다.",
        f"- 표본의 대략적인 정답 박스 폭은 원본 기준 최소 {evaluation.get('smallest_annotated_width_px', float('nan')):.0f}px입니다. 약 10px 폭 극소 표적의 성능은 별도 영상으로 검증해야 합니다.",
        "- 타워 후보가 존재하는 것과 UART로 보낼 최대영역 박스가 정확한 것은 다릅니다. 최대영역 선택 정답은 참고용으로 함께 기록했습니다.",
        "- 시간은 현재 Windows PC의 필터·마스크·윤곽·선택 계산만 측정한 값이며 디코딩/표시/저장 제외입니다. 최초 30프레임은 시간 집계에서 제외했습니다.",
        "- 실행 순서는 프레임마다 회전합니다. 실행 중 일부 평가/회귀 검사도 함께 수행했으므로 CPU가 완전히 독점된 벤치마크는 아닙니다.",
        "- 휴대폰 입력은 약 30fps입니다. 50Hz 카메라 입력, 라즈베리파이 실시간 성능, UART 주기는 검증하지 않았습니다.",
        "- 종합 영상은 원본+8개 박스 비교, 개별 영상은 원본/성분 마스크/최종 마스크 6칸입니다. 초 단위 표시에는 명목 FPS를 사용합니다.", "",
        "## 영상", "", "[8방식 종합 비교](00_all_methods_overview.mp4)", ""]
    for name in METHODS:
        lines.append(f"- [{name} — 성분 및 최종 마스크]({name}.mp4)")
    (output/"REPORT.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    # Keep terminal output ASCII; Windows cp949 cannot print every Markdown symbol.
    print(f"Saved report: {output / 'REPORT.md'}",flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video",type=Path)
    parser.add_argument("--annotations",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--report-only",action="store_true")
    args=parser.parse_args()
    if args.report_only:
        report(args.output)
        return
    if not args.video or not args.annotations:
        parser.error("--video and --annotations are required for evaluation")
    data=json.loads(args.annotations.read_text(encoding="utf-8"))
    digest=hashlib.sha256(args.video.read_bytes()).hexdigest()
    if digest != data["source_sha256"]:
        raise RuntimeError("Annotations belong to another video")
    cap=cv2.VideoCapture(str(args.video))
    width,height=int(cap.get(3)),int(cap.get(4))
    cap.release()
    labels=read_annotations(args.annotations,width,height)
    visible=[i for i,item in labels.items() if item["box"]]
    stress_indices=set(visible[::4])
    cv2.setNumThreads(1)
    counts={name:{"visible":0,"candidate_hits":0,"largest_hits":0,"missing_frames":[],
                  "stress_hits":0,"stress_n":0,"stress_by_type":{}} for name in METHODS}
    records=[]
    rng=np.random.default_rng(72388)
    csv_targets={}
    # Reading a running metrics file is unsafe. Cross-check saved targets only after completion.
    if (args.output/"summary.json").exists():
        with (args.output/"metrics.csv").open(encoding="utf-8") as file:
            for row in csv.DictReader(file):
                if int(row["frame"]) in labels:
                    csv_targets[(int(row["frame"]),row["method"])]=row_target(row)
    for index,frame in selected_frames(args.video,visible):
        truth=labels[index]["box"]
        variants={"original":frame}
        if index in stress_indices:
            floating=frame.astype(np.float32)
            variants.update({"dark_0.5":np.clip(floating*.5,0,255).astype(np.uint8),
                "bright_1.4":np.clip(floating*1.4,0,255).astype(np.uint8),
                "noise_sigma12":np.clip(floating+rng.normal(0,12,frame.shape),0,255).astype(np.uint8),
                "blur5":cv2.GaussianBlur(frame,(5,5),0)})
        for kind,probe in variants.items():
            for name in METHODS:
                _,mask,target=detect(probe,name)
                hit=candidate_hit(mask,truth)
                largest=matches(target,truth)
                item=counts[name]
                if kind=="original":
                    item["visible"]+=1
                    item["candidate_hits"]+=int(hit)
                    item["largest_hits"]+=int(largest)
                    if not hit: item["missing_frames"].append(index)
                    if csv_targets:
                        saved=csv_targets[(index,name)]
                        if (saved is None)!=(target is None) or (saved and saved["box"]!=target["box"]):
                            raise RuntimeError(f"Sequential evaluation differs from export: {index}/{name}")
                else:
                    item["stress_n"]+=1
                    item["stress_hits"]+=int(hit)
                    detail=item["stress_by_type"].setdefault(kind,{"hits":0,"n":0})
                    detail["n"]+=1
                    detail["hits"]+=int(hit)
                records.append({"frame":index,"variant":kind,"method":name,
                                "candidate_hit":int(hit),"largest_correct":int(largest)})
    args.output.mkdir(parents=True,exist_ok=True)
    result={"source_sha256":digest,"annotations":str(args.annotations.resolve()),
            "scope":"Sparse candidate recall; unrelated larger background selection excluded from ranking",
            "smallest_annotated_width_px":min(labels[i]["box"][2] for i in visible),
            "csv_cross_checked":bool(csv_targets),"methods":counts}
    (args.output/"candidate_evaluation.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    with (args.output/"candidate_evaluation.csv").open("w",newline="",encoding="utf-8") as file:
        writer=csv.DictWriter(file,fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    label_sheets(args.video,labels,args.output/"labels")
    print(json.dumps(counts,indent=2),flush=True)
    if (args.output/"summary.json").exists(): report(args.output)

if __name__=="__main__":
    main()
