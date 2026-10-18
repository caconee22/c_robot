"""index.html과 로봇 그림을 한 파일로 합친다 (메신저·클로드 아티팩트로 공유할 때).

사용: python tools/build_artifact.py  ->  dist/sumo-sim.html
"""
import base64
import pathlib

root = pathlib.Path(__file__).resolve().parent.parent
html = (root / "index.html").read_text(encoding="utf-8")
png = base64.b64encode((root / "assets" / "robot_top.png").read_bytes()).decode()
old = "ROBOT_IMG.src = 'assets/robot_top.png';"
assert old in html, "index.html의 로봇 그림 경로가 바뀌었습니다"
out = root / "dist" / "sumo-sim.html"
out.parent.mkdir(exist_ok=True)
out.write_text(html.replace(old, f"ROBOT_IMG.src = 'data:image/png;base64,{png}';"), encoding="utf-8")
print(out)
