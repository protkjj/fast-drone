// 게인 튜닝 평가기 — 후보 묶음 하나를 받아 튜닝 시나리오에서 점수를 매긴다.
//
// 실제 탐색(표본 추출·예산 배분)은 research/tune_gains.py 가 한다. 이 파일은
// "후보 목록 -> 점수 목록" 만 담당한다. 분리한 이유는 두 가지다.
//   · GSLQR 후보는 Q/R 에서 ARE 를 풀어야 해서 파이썬(scipy)이 필요하다.
//   · casadi-wasm 로드가 비싸므로 후보마다 node 를 새로 띄우면 안 된다.
//     한 프로세스가 묶음 전체를 돌린다.
//
// 지표는 sweep.cjs 의 evaluateRun 을 그대로 재사용한다 — 튜닝 목적함수와
// 본시험 지표가 갈라지면 튜닝이 엉뚱한 것을 최적화하게 된다.
//
// 사용: node tune_eval.cjs <job.json> <out.json>
const fs = require('node:fs');
const path = require('node:path');
const runtime = require('./runtime.js');
const {evaluateRun} = require('./sweep.cjs');

const FAILURE_PENALTY = 1000; // 실패 1건당. 유한해야 탐색이 순위를 매길 수 있다.

async function main() {
  const [jobPath, outPath] = process.argv.slice(2);
  if (!jobPath || !outPath) throw new Error('사용: node tune_eval.cjs <job.json> <out.json>');
  const job = JSON.parse(fs.readFileSync(jobPath, 'utf8'));

  const ca = await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');
  await ca.load_interpolant('linear');
  const base = JSON.parse(fs.readFileSync(
    path.join(__dirname, 'generated', (job.profile || 'selected') + '.json'), 'utf8'));

  const results = [];
  let done = 0;
  const total = job.candidates.length*job.scenarios.length;
  for (const cand of job.candidates) {
    // GSLQR 후보는 게인이 아니라 스케줄 표 자체를 갈아끼운다(런타임은 번들에서 읽는다).
    const data = cand.gslqr_schedule ? {...base, gslqr: cand.gslqr_schedule} : base;
    const runs = [];
    for (const sc of job.scenarios) {
      const opts = {controller: cand.controller, feedback: 'truth', scenario: 'ramp',
                    speed: sc.speed, altitude: job.altitude, ramp_t0: job.ramp_t0,
                    ramp_duration_s: job.ramp_duration_s, seconds: job.seconds,
                    seed: sc.seed, ...(cand.gains ? {gains: cand.gains} : {}), ...sc.opts};
      let row;
      try {
        row = evaluateRun(await runtime.run(ca, data, opts, () => {}), sc.speed, job.altitude);
      } catch (e) {
        row = {failed: true, reason: `threw: ${e.message}`, rmse_v: Infinity, rmse_z: Infinity};
      }
      runs.push({scenario: sc.id, ...row});
      done++;
      if (done % 20 === 0) console.error(`  eval ${done}/${total}`);
    }
    const failures = runs.filter(r => r.failed).length;
    const ok = runs.filter(r => !r.failed);
    // 점수 = 성공 런의 (RMSE_v + RMSE_z) 평균 + 실패 벌점. 두 항 모두 SI 단위의
    // 추종 오차라 그대로 더한다(가중치를 또 하나 도입하면 그것도 튜닝 대상이 된다).
    const base_score = ok.length ? ok.reduce((s, r) => s+r.rmse_v+r.rmse_z, 0)/ok.length : 0;
    results.push({id: cand.id, controller: cand.controller,
                  score: base_score + FAILURE_PENALTY*failures,
                  mean_rmse_v: ok.length ? ok.reduce((s, r) => s+r.rmse_v, 0)/ok.length : null,
                  mean_rmse_z: ok.length ? ok.reduce((s, r) => s+r.rmse_z, 0)/ok.length : null,
                  failures, runs});
  }
  fs.writeFileSync(outPath, JSON.stringify({penalty: FAILURE_PENALTY, results}, null, 2));
  console.error(`written: ${outPath}  (${results.length} candidates)`);
}

main().catch(e => { console.error(e); process.exitCode = 1; });
