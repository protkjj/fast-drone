// 분할 실행한 표8 스윕 결과를 하나로 합친다.
//
// 왜 필요한가: 본실행은 조건x제어기 1920칸이고 제어기마다 비용이 250배까지 차이난다
// (실측: gslqr 1.4초/런 vs f13 124초/런). 단일 프로세스면 27시간이지만 조건 단위로
// 쪼개 병렬로 돌리면 코어 수만큼 줄어든다. 대신 산출 파일이 나뉘므로 표9/10 통계를
// 내려면 합쳐야 한다.
//
// 요약·통계는 sweep.cjs 의 summarise() 를 **그대로 재사용**한다. 병합기가 자체
// 계산을 갖고 있으면 분할로 만든 표와 단일 실행으로 만든 표가 조용히 달라진다.
//
// 사용법:
//   node merge_sweep.cjs <출력.json> <입력1.json> [입력2.json ...]
//   node merge_sweep.cjs ../results/table8_full.json ../results/part_*.json
const fs = require('node:fs');
const {summarise, CONDITIONS, CONTROLLERS} = require('./sweep.cjs');

function main() {
  const [outFile, ...inputs] = process.argv.slice(2);
  if (!outFile || !inputs.length)
    throw new Error('사용법: node merge_sweep.cjs <출력.json> <입력1.json> [입력2.json ...]');

  const merged = {};
  let seeds = null, seconds = null, profile = null;
  const seenControllers = new Set();
  for (const file of inputs) {
    const part = JSON.parse(fs.readFileSync(file, 'utf8'));
    // 서로 다른 설정으로 돌린 조각을 합치면 표가 거짓이 된다. 합치기 전에 막는다.
    for (const [name, value] of [['seeds', 'seeds'], ['seconds', 'seconds'], ['profile', 'profile']]) {
      const current = {seeds, seconds, profile}[name];
      if (current !== null && current !== part[value])
        throw new Error(`${file}: ${name} 불일치 (${current} vs ${part[value]}) — 같은 설정끼리만 합칠 수 있다`);
    }
    ({seeds, seconds, profile} = {seeds: part.seeds, seconds: part.seconds, profile: part.profile});
    (part.controllers || []).forEach(c => seenControllers.add(c));

    for (const [id, byController] of Object.entries(part.raw || {})) {
      merged[id] = merged[id] || {};
      for (const [controller, rows] of Object.entries(byController)) {
        if (!rows?.length) continue;
        const existing = merged[id][controller];
        if (existing && existing.length !== rows.length)
          throw new Error(`${id}/${controller}: 조각마다 난수 개수가 다르다 (${existing.length} vs ${rows.length})`);
        merged[id][controller] = rows;   // 같은 칸이 겹치면 나중 파일이 이긴다
      }
    }
    console.error(`  읽음 ${file}: ${Object.keys(part.raw || {}).length}조건`);
  }

  const conditionIds = CONDITIONS.map(c => c.id).filter(id => merged[id]);
  const controllers = CONTROLLERS.filter(c => seenControllers.has(c));
  const output = summarise(merged, {conditionIds, controllers, seeds, seconds, profile});

  const missing = [];
  for (const id of conditionIds)
    for (const controller of controllers)
      if (merged[id][controller]?.length !== seeds) missing.push(`${id}/${controller}`);

  fs.writeFileSync(outFile, JSON.stringify(output, null, 2));
  console.error(`written: ${outFile}`);
  console.error(`  조건 ${conditionIds.length} x 제어기 ${controllers.length} x 난수 ${seeds}`);
  console.error(missing.length ? `  ⚠ 미완 ${missing.length}칸: ${missing.slice(0, 8).join(', ')}${missing.length > 8 ? ' …' : ''}`
                               : `  전 칸 완료 (complete=${output.complete})`);
}

if (require.main === module) {
  try { main(); } catch (e) { console.error(e.message); process.exitCode = 1; }
}
module.exports = {main};
