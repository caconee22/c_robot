"""Fast shared-mask export, without benchmarks, annotations, CSV or reports."""
import argparse
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor
import cv2
import numpy as np
from comparison_engine import METHODS, KERNEL, color_masks, largest_target, detect, render

def shared_detections(frame):
    shared=color_masks(frame,("BGR","ExG","HSV","Lab","Normalized"))
    results={}
    for name,(components,required) in METHODS.items():
        if name=="08_hsv_cascade":
            results[name]=detect(frame,name)
            continue
        masks={key:shared[key] for key in components}
        if len(components)==1:
            fused=masks[components[0]].copy()
        else:
            votes=np.zeros(frame.shape[:2],np.uint8)
            for mask in masks.values(): votes+=mask//255
            fused=cv2.inRange(votes,required,len(components))
        fused=cv2.morphologyEx(fused,cv2.MORPH_CLOSE,KERNEL)
        results[name]=(masks,fused,largest_target(fused))
    return results

def overview(frame,results,index,fps):
    canvas=np.zeros((960,960,3),np.uint8)
    scale=min(240/frame.shape[1],425/frame.shape[0])
    source=cv2.resize(frame,None,fx=scale,fy=scale,interpolation=cv2.INTER_LINEAR)
    for number,(name,(_,_,target)) in enumerate(results.items()):
        tx,ty=number%4*240,number//4*480
        image=source.copy()
        if target:
            x,y,w,h=[round(v*scale) for v in target["box"]]
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.drawMarker(image,tuple(round(v*scale) for v in target["center"]),(0,0,255),cv2.MARKER_CROSS,8,1)
        canvas[ty+55:ty+55+image.shape[0],tx:tx+image.shape[1]]=image
        for line,text in enumerate((name,f"#{index} {index/fps:.2f}s | input {fps:.2f}fps")):
            cv2.putText(canvas,text,(tx+4,ty+20+line*20),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1,cv2.LINE_AA)
    return canvas

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise SystemExit("Use an empty output directory to preserve earlier videos")
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    cap=cv2.VideoCapture(str(args.video))
    if not cap.isOpened(): raise RuntimeError("Cannot open source video")
    fps=cap.get(cv2.CAP_PROP_FPS) or 60
    total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    writers={}
    index=0
    started=time.perf_counter()
    pool=ThreadPoolExecutor(max_workers=4)
    def save_method(name,frame,masks,fused,target,index):
        writers[name].write(render(frame,masks,fused,target,name,index,0,fps,show_timing=False))
    def save_overview(frame,results,index):
        writers["00_all_methods_overview"].write(overview(frame,results,index,fps))
    try:
        for name in [*METHODS,"00_all_methods_overview"]:
            size=(960,960) if name.startswith("00_") else (810,960)
            writer=cv2.VideoWriter(str(args.output/f"{name}.mp4"),cv2.VideoWriter_fourcc(*"mp4v"),fps,size)
            writers[name]=writer
            if not writer.isOpened(): raise RuntimeError(f"Cannot create {name}")
        while True:
            ok,frame=cap.read()
            if not ok: break
            results=shared_detections(frame)
            # Separate writers may encode concurrently. Complete this frame before the next,
            # so one writer is never accessed concurrently and output order is preserved.
            futures=[pool.submit(save_method,name,frame,masks,fused,target,index)
                     for name,(masks,fused,target) in results.items()]
            futures.append(pool.submit(save_overview,frame,results,index))
            for future in futures: future.result()
            index+=1
            if index%300==0: print(f"Export {index}/{total} | {time.perf_counter()-started:.1f}s",flush=True)
    finally:
        pool.shutdown(wait=True)
        cap.release()
        for writer in writers.values(): writer.release()
    if index!=total or index==0: raise RuntimeError(f"Incomplete source decode: {index}/{total}")
    for name in writers:
        check=cv2.VideoCapture(str(args.output/f"{name}.mp4"))
        count=int(check.get(cv2.CAP_PROP_FRAME_COUNT))
        first,_=check.read()
        check.set(cv2.CAP_PROP_POS_FRAMES,index-1)
        last,_=check.read()
        check.release()
        if count!=index or not first or not last: raise RuntimeError(f"Incomplete output: {name}")
    print(f"Saved 9 videos, {index} frames each: {args.output}",flush=True)

if __name__=="__main__": main()
