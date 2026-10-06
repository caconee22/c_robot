"""Coordinate guides for independent visual annotation of contact sheets."""
import argparse
from pathlib import Path
import cv2

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    for path in sorted(args.folder.glob("reference_??.jpg")):
        frame = cv2.imread(str(path))
        for row in range(2):
            for col in range(3):
                tile = frame[row*640:(row+1)*640, col*360:(col+1)*360]
                for y in range(80, 640, 80):
                    cv2.line(tile, (0,y), (359,y), (150,150,150), 1)
                    cv2.putText(tile, str(y), (1,y-2), cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
                for x in range(80,360,80):
                    cv2.line(tile,(x,36),(x,639),(150,150,150),1)
                    cv2.putText(tile,str(x),(x+1,52),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
        cv2.imwrite(str(path.with_name("grid_"+path.name)),frame)

if __name__ == "__main__":
    main()
