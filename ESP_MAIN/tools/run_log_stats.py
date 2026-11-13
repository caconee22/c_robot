"""Summarize Pi run logs (logs/runs/run_*.csv from scripts/esp_log_receiver.py).

Written after the 2026 IRC competition (11/13) for the retrospective:
time share by state, escapes (color, first side, opponent distance, repeats),
circle bypass episodes, fork attack outcomes, search stretches.

  python tools/run_log_stats.py [folder with run_*.csv]   (default: pi_backup/runs)
"""
import csv, glob, os, re, collections, sys
sys.stdout.reconfigure(encoding="utf-8")
FOLDER = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "pi_backup", "runs")

runs = []
for path in sorted(glob.glob(os.path.join(FOLDER, "run_*.csv"))):
    rows = list(csv.reader(open(path, encoding="utf-8")))[1:]
    if not rows:
        continue
    ev = []
    for r in rows:
        if len(r) < 4:
            continue
        try:
            t = float(r[0])
        except ValueError:
            continue
        ev.append((t, r[2], r[3]))
    if not ev:
        continue
    name = os.path.basename(path)
    day = name[4:12]
    dur = ev[-1][0]
    end = ev[-1][2] if ev[-1][1] == "끝" else ""
    start = next((e[2] for e in ev if e[1] == "시작"), "")
    st_time = collections.Counter()
    cur, since = None, 0.0
    trans = 0
    for t, k, c in ev:
        if k == "상태":
            m = re.match(r"(\S+) -> (\S+)", c)
            if m:
                if cur is not None:
                    st_time[cur] += t - since
                cur, since = m.group(2), t
                trans += 1
    if cur is not None:
        st_time[cur] += dur - since
    esc = [c for t, k, c in ev if k == "회피"]
    colors = collections.Counter(re.search(r"색=(\S+)", c).group(1) for c in esc if "색=" in c)
    sides = collections.Counter(re.search(r"먼저=(\S+)", c).group(1) for c in esc if "먼저=" in c)
    straight = sum("일자후진" in c for c in esc)
    charging = sum("돌진중=1" in c for c in esc)
    bypass = sum(1 for t, k, c in ev if k == "원돌기")
    fork = sum(1 for t, k, c in ev if k == "상태" and "-> 포크브레이크" in c)
    forkpush = sum(1 for t, k, c in ev if k == "상태" and "-> 포크밀기" in c)
    duties = collections.Counter()
    for t, k, c in ev:
        if k == "생존":
            m = re.search(r"동작=(\S+) (\d+)", c)
            if m:
                duties[m.group(1)] += 1
    # longest continuous stay in one state
    longest = max(((v, k) for k, v in st_time.items()), default=(0, ""))
    runs.append(dict(name=name, day=day, dur=dur, end=end, start=start, st=st_time, esc=len(esc),
                     colors=colors, sides=sides, straight=straight, charging=charging, bypass=bypass,
                     fork=fork, forkpush=forkpush, duties=duties, trans=trans))

print("runs:", len(runs))
byday = collections.defaultdict(list)
for r in runs:
    byday[r["day"]].append(r)
for d, rs in sorted(byday.items()):
    tot = sum(r["dur"] for r in rs)
    print(f"{d}: {len(rs)} runs, {tot/60:.1f} min driving")

real = [r for r in runs if r["dur"] >= 10]
T = sum(r["dur"] for r in real)
print(f"\nruns >=10 s: {len(real)}, total {T/60:.1f} min")
st = collections.Counter()
for r in real:
    st.update(r["st"])
print("time share by state:")
for k, v in st.most_common():
    print(f"  {k:8s} {v:7.1f}s {100*v/T:5.1f}%")
E = sum(r["esc"] for r in real)
print(f"\nescapes {E} ({E/(T/60):.1f}/min); charging when hit {sum(r['charging'] for r in real)}, straight {sum(r['straight'] for r in real)}")
c = collections.Counter(); s = collections.Counter()
for r in real:
    c.update(r["colors"]); s.update(r["sides"])
print("  colors", dict(c)); print("  first side", dict(s))
print(f"circle bypass starts {sum(r['bypass'] for r in real)}, fork attacks {sum(r['fork'] for r in real)} (reached push {sum(r['forkpush'] for r in real)})")
ends = collections.Counter(r["end"] for r in runs)
print("\nrun ends:", dict(ends))
du = collections.Counter()
for r in real:
    du.update(r["duties"])
tot = sum(du.values())
print("\nalive intents share:")
for k, v in du.most_common():
    print(f"  {k:10s} {100*v/tot:5.1f}%")
print("\nper run (>=10 s): name dur esc bypass fork longest-state")
for r in real:
    lk = max(r["st"].items(), key=lambda kv: kv[1]) if r["st"] else ("", 0)
    print(f"  {r['name'][4:19]} {r['dur']:6.1f}s esc={r['esc']:3d} byp={r['bypass']:2d} fork={r['fork']:2d}/{r['forkpush']:2d} "
          f"search={r['st'].get('탐색',0):5.1f}s circle={r['st'].get('원돌기',0):5.1f}s  {r['start'][:40]}")



byp_d = []; byp_end = collections.Counter(); esc_h = collections.Counter(); quick = 0; esc_n = 0
fork_out = collections.Counter(); open_turn = []
search_runs = []
for path in sorted(glob.glob(os.path.join(FOLDER, "run_*.csv"))):
    ev = []
    for r in list(csv.reader(open(path, encoding="utf-8")))[1:]:
        try: ev.append((float(r[0]), r[2], r[3]))
        except (ValueError, IndexError): pass
    if not ev or ev[-1][0] < 10: continue
    states = [(t, re.match(r"(\S+) -> (\S+)", c), c) for t, k, c in ev if k == "상태"]
    states = [(t, m.group(1), m.group(2), c) for t, m, c in states if m]
    for i, (t, a, b, c) in enumerate(states):
        if b == "원돌기":
            nxt = next(((t2, b2) for t2, a2, b2, c2 in states[i+1:]), None)
            if nxt:
                byp_d.append(nxt[0] - t); byp_end[nxt[1]] += 1
        if b == "회피":
            esc_n += 1
            h = int(re.search(r"h=(\d+)", c).group(1))
            esc_h["상대 가까움 h>=300" if h >= 300 else ("중간 150-299" if h >= 150 else "멀거나 안보임 <150")] += 1
        if a == "포크브레이크" or a == "포크삽입" or a == "포크밀기":
            fork_out[f"{a}->{b}"] += 1
    # quick re-escape: escape again within 2 s of the previous one ending
    last_end = None
    for t, a, b, c in states:
        if a == "회피": last_end = t
        if b == "회피" and last_end is not None and t - last_end < 2.0: quick += 1
    # search stretches
    cur = None
    for t, a, b, c in states:
        if b == "탐색": cur = t
        elif a == "탐색" and cur is not None:
            search_runs.append(t - cur); cur = None

def pct(xs, q):
    xs = sorted(xs); return xs[int(q * (len(xs) - 1))] if xs else 0
print(f"bypass episodes {len(byp_d)}: median {pct(byp_d,.5):.1f}s p90 {pct(byp_d,.9):.1f}s max {max(byp_d):.1f}s; >5s: {sum(d>5 for d in byp_d)}, >10s: {sum(d>10 for d in byp_d)}")
print("  bypass ends ->", dict(byp_end))
print(f"escapes {esc_n} by tower distance when it started:", dict(esc_h))
print(f"re-escape within 2 s of the last one: {quick} ({100*quick/esc_n:.0f}%)")
print("fork transitions:", dict(fork_out))
print(f"search stretches {len(search_runs)}: median {pct(search_runs,.5):.1f}s p90 {pct(search_runs,.9):.1f}s max {max(search_runs):.1f}s")
