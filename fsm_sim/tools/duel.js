// v1/v2 FSM과 상대 행동 조합을 좌우 시작 위치를 바꿔 가며 비교한다.
// 사용: node duel.js 6   (조합마다 6 × 2경기)
const { runMatch } = require('./run_batch.js');
const N = +(process.argv[2] || 10);
function batch(label, over) {
  const a = { us:0, them:0, push:0, contact:0, escapes:0, dives:0, st:{} };
  for (let i = 1; i <= N; i++) for (const startPos of ['left', 'right']) {
    const r = runMatch(Object.assign({ startPos }, over), i * 104729 + (startPos === 'left' ? 0 : 7));
    a.us += r.score[0]; a.them += r.score[1]; a.push += r.pushPts; a.contact += r.contact; a.escapes += r.escapes; a.dives += r.dives;
    for (const [k, v] of Object.entries(r.st)) a.st[k] = (a.st[k] || 0) + v;
  }
  const n = 2 * N, pct = v => (100 * v / (n * 90000)).toFixed(0) + '%';
  const top = Object.entries(a.st).sort((x, y) => y[1] - x[1]).slice(0, 6).map(([k, v]) => `${k} ${pct(v)}`).join(', ');
  console.log(`${label.padEnd(34)} 득점 ${(a.us/n).toFixed(1).padStart(5)} : 실점 ${(a.them/n).toFixed(1).padStart(5)} | 밀어서 ${(a.push/n).toFixed(1).padStart(4)} | 접촉 ${pct(a.contact)} | 돌진 ${(a.dives/n).toFixed(1)} | 홈탈출 ${(a.escapes/n).toFixed(1)}\n    우리 상태: ${top}`);
}
const v1 = { preset: 'edge' };
batch('v1(최적+끼고돌기) vs 무작위 배회', { ...v1, ourFsm: 'v1', opp: 'wander' });
batch('v2(새 FSM)       vs 무작위 배회', { ...v1, ourFsm: 'v2', opp: 'wander' });
batch('v1 vs v1', { ...v1, ourFsm: 'v1', opp: 'fsm' });
batch('v2 vs v1 (우리 v2)', { ...v1, ourFsm: 'v2', opp: 'fsm' });
batch('v2 vs v2', { ...v1, ourFsm: 'v2', opp: 'v2' });
