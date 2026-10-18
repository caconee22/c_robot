// 시뮬레이터를 브라우저 없이 돌려 경기 지표를 모은다.
// 사용: node run_batch.js '{"ourFsm":"v2","opp":"wander"}' 12
const fs = require('fs'), vm = require('vm'), path = require('path');
const html = fs.readFileSync(path.join(__dirname, '..', 'index.html'), 'utf8');
const js = html.slice(html.indexOf('<script>') + 8, html.lastIndexOf('</script>'));
const defaults = {};
for (const m of html.matchAll(/id="(\w+)"[^>]*value="([^"]+)"/g)) defaults[m[1]] = m[2];
function rng(seed) { let s = seed >>> 0; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }
function runMatch(over, seed, ms = 90000) {
  const els = {}, vals = Object.assign({ preset:'custom', ourFsm:'v1', order:'rec', opp:'wander', startPos:'left', searchMode:'spin', openDir:'left', speed:'1' }, over);
  const ctx2d = new Proxy({}, { get: (o, k) => (k in o ? o[k] : () => {}), set: (o, k, v) => { o[k] = v; return true; } });
  const el = id => els[id] || (els[id] = { id, value: String(vals[id] ?? defaults[id] ?? ''), checked: id === 'flip' || id === 'yellowHazard' || id === 'edgeFollow' ? !!vals[id] : (vals[id] ?? true),
    textContent:'', innerHTML:'', width:900, height:900, listeners:{}, addEventListener(t, f) { this.listeners[t] = f; },
    getContext: () => ctx2d, getBoundingClientRect: () => ({ width:900, height:900, left:0, top:0 }), setPointerCapture() {} });
  let raf = null, now = 0;
  const math = Object.create(Math); math.random = rng(seed);
  const sandbox = { document:{ getElementById: el }, window:{ addEventListener() {}, devicePixelRatio:1 }, Image: function () { return { complete:false, naturalWidth:0 }; },
    requestAnimationFrame: f => { raf = f; }, performance:{ now: () => now }, Math: math, console };
  vm.createContext(sandbox); vm.runInContext(js, sandbox);
  if (over.__v2) Object.assign(sandbox.window.__V2, over.__v2);
  el('btnStart').listeners.click();
  const st = {}, M = { contact:0, escapes:0, dives:0, pushes:0, approachContact:0, pushPts:0 }; let prev = '', lastContact = -1e9, lastUs = 0;
  const trace = [], hist = []; let lastThem = 0;
  for (let f = 0; f * 20 < ms; f++) {
    now += 20; raf(now);
    const s = sandbox.window.__sim(), state = s.ours.fsm.state;
    st[state] = (st[state] || 0) + 20;
    const touching = Math.hypot(s.ours.x - s.opp.x, s.ours.y - s.opp.y) < 340;
    if (s.score[0] > lastUs) { if (s.t - lastContact < 1500) M.pushPts += s.score[0] - lastUs; lastUs = s.score[0]; }
    if (touching) { lastContact = s.t; M.contact += 20; if (state === 'Approach' || state === 'TrackAlign' || state === 'TargetConfirm') M.approachContact += 20; }
    if (state !== prev) { if (['ZoneEscape','Escape','BlueRetreat','GrooveExit'].includes(state)) M.escapes++; if (state === 'DiveReady' || state === 'Charge') M.dives++; if (state === 'Push') M.pushes++; prev = state; }
    hist.push(state + '@' + (s.t/1000).toFixed(1)); if (hist.length > 8) hist.shift();
    if (s.score[1] > lastThem) { trace.push(`실점 ${s.score[1]-lastThem} t=${(s.t/1000).toFixed(1)} [${[...new Set(hist.map(h=>h.split('@')[0]))].join('>')}] 색=${s.ours.sens.map(q=>q.reliable?q.conf:'-').join('')}`); lastThem = s.score[1]; }
    if (!s.started) break;
  }
  const s = sandbox.window.__sim();
  const resets = s.logLines.filter(l => l.includes('처음 위치')).length;
  const oursConceded = s.logLines.filter(l => l.includes('우리 로봇')).length;
  return { score: s.score.slice(), resets, oursConceded, ...M, st, trace };
}
module.exports = { runMatch };
if (require.main === module) {
  const over = JSON.parse(process.argv[2] || '{}'), seeds = +(process.argv[3] || 8);
  const agg = { us:0, them:0, contact:0, approachContact:0, escapes:0, dives:0, pushes:0, resets:0, oursConceded:0, pushPts:0, st:{} };
  for (let i = 1; i <= seeds; i++) {
    const r = runMatch(over, i * 7919);
    agg.us += r.score[0]; agg.them += r.score[1];
    for (const k of ['contact','approachContact','escapes','dives','pushes','resets','oursConceded','pushPts']) agg[k] += r[k];
    for (const [k, v] of Object.entries(r.st)) agg.st[k] = (agg.st[k] || 0) + v;
  }
  const n = seeds, pct = v => (100 * v / (n * 90000)).toFixed(1) + '%';
  console.log(JSON.stringify(over), `| 평균 득점 ${(agg.us/n).toFixed(1)} (밀어서 ${(agg.pushPts/n).toFixed(1)}) : 실점 ${(agg.them/n).toFixed(1)} | 접촉 ${pct(agg.contact)} (공격 없이 밀기 ${pct(agg.approachContact)}) | 돌진 ${(agg.dives/n).toFixed(1)} 밀기 ${(agg.pushes/n).toFixed(1)} 홈탈출 ${(agg.escapes/n).toFixed(1)} | 우리 실점 사건 ${(agg.oursConceded/n).toFixed(1)}`);
  console.log('   상태 시간:', Object.entries(agg.st).sort((a, b) => b[1] - a[1]).map(([k, v]) => `${k} ${pct(v)}`).join(', '));
}
