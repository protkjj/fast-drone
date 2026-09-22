// 표8 확인 시험 + 표9/10 통계 — §5.8-5.10.
//
// run.cjs 패턴(브라우저 없이 runtime.run()을 직접 호출)을 반복 호출로 확장한
// 것. 한 프로세스 안에서 WASM 모듈과 번들을 한 번만 로드하고 재사용한다.
//
// 사용법:
//   node sweep.cjs [--seeds N] [--conditions Q01,Q04] [--controllers hybrid,nmpc]
//                  [--seconds S] [--out FILE.json]
//
// 기본값은 스모크 테스트 규모(seeds=2)다. 논문 §5.8의 실제 확인 시험
// (12조건×6제어기×32난수=2,304회)은 --seeds 32로 명시적으로 지정해야 돌아가며,
// 규모상 수 시간 걸릴 수 있다 — 이건 의도적으로 기본값에서 뺐다(3단계는
// 스윕 도구를 만드는 것까지이지 본시험 실행이 아니다).
const fs = require('node:fs');
const path = require('node:path');
const runtime = require('./runtime.js');

const CONTROLLERS = ['cpid', 'gslqr', 'nmpc', 'f13', 'hybrid', 'gindi']; // 표5: CPID,GSLQR,M17,F13,V13,GINDI
const RAMP_T0 = 1, RAMP_DURATION_S = 3; // 식(43) 참조: t0=1s에 시작, 3초에 걸쳐 목표 속도로.
const V_L = 8, V_H = 18; // 트림이 실제로 존재하는 0-18 m/s로 재설정(§5.8 규칙:
// "해당 모델에서 유효한 트림이 존재하지 않으면 낮춘다" — 20/40 후보는 20-80 m/s에
// 트림이 아예 없어 그대로 못 씀, research/TRIM_ENVELOPE_AUDIT.md와 2026-09-22
// 재확인 참고).

// 표8. 확인 시험 12조건. wind_angle=90은 순수 측풍(진행방향 vx에 수직).
const CONDITIONS = [
  {id: 'Q01', speed: V_L, opts: {}},
  {id: 'Q02', speed: V_H, opts: {}},
  {id: 'Q03', speed: V_L, opts: {wind_speed: 5, wind_angle: 90}},
  {id: 'Q04', speed: V_H, opts: {wind_speed: 5, wind_angle: 90}},
  {id: 'Q05', speed: V_H, opts: {wind_speed: 10, wind_angle: 90}},
  {id: 'Q06', speed: V_H, opts: {wind_vertical_mps: 5}},
  {id: 'Q07', speed: V_H, opts: {wind_vertical_mps: -5}},
  {id: 'Q08', speed: V_H, opts: {wind_speed: 5, wind_angle: 90, scales: [1.2, 1, 1, 1, 1, 1]}},
  {id: 'Q09', speed: V_H, opts: {wind_speed: 5, wind_angle: 90, scales: [1, 1, 1, 1, 1, 2.0]}}, // 20ms->40ms
  {id: 'Q10', speed: V_H, opts: {wind_speed: 5, wind_angle: 90, initial_soc: 0.2}},
  {id: 'Q11', speed: V_H, opts: {wind_speed: 5, wind_angle: 90, gps_position_std_m: 3.0, gps_delay_ms: 100}},
  {id: 'Q12', speed: V_H, opts: {wind_speed: 5, wind_angle: 90, indi_rpm_desync_ms: 5}},
];

function rmse(values) { return Math.sqrt(values.reduce((s, v) => s+v*v, 0)/values.length); }

// 식(47)-(48) 근사: 램프가 끝난(RAMP_T0+RAMP_DURATION_S 이후) 정착 구간에서
// v/z 오차 RMSE. 논문의 돌풍 전용 12초 평가창(돌풍전3s+돌풍2s+돌풍후7s)은
// gust 시나리오 전용이라 여기 ramp 시나리오엔 안 맞는다 — 대신 램프 종료
// 시점을 명시적으로 아는 걸 이용해 "그 이후"를 평가창으로 잡는다. 정확한
// 평가창 규칙은 실제 §5.8 본시험 설계 시 다시 검토가 필요하다(문서화된
// 단순화이지 §5.9 규칙의 완전한 재현이 아니다).
function evaluateRun(result, speed, altitude) {
  if (result.failure) return {failed: true, reason: result.failure, rmse_v: Infinity, rmse_z: Infinity};
  const tr = result.trace;
  const steady = tr.filter(f => f.t >= RAMP_T0+RAMP_DURATION_S);
  if (!steady.length) return {failed: true, reason: 'no post-ramp samples (seconds too short)', rmse_v: Infinity, rmse_z: Infinity};
  const ev = steady.map(f => Math.hypot(f.v[0]-speed, f.v[1], f.v[2]));
  const ez = steady.map(f => f.z-altitude);
  return {failed: false, rmse_v: rmse(ev), rmse_z: rmse(ez),
          energy_J: result.metrics?.energy_J ?? null};
}

// §5.10 부트스트랩: 제어기 A-B의 짝지어진(같은 난수) RMSE 차이의 중앙값과
// 95% 신뢰구간(10,000 재표집, 복원추출).
function bootstrapPairedMedianCI(diffs, resamples=10000, seed=1) {
  const rand = (() => { let s = seed>>>0; return () => { s=(Math.imul(1664525,s)+1013904223)>>>0; return s/4294967296; }; })();
  const n = diffs.length;
  const sorted = a => a.slice().sort((x,y)=>x-y);
  const median = a => { const s=sorted(a); const m=Math.floor(s.length/2); return s.length%2?s[m]:(s[m-1]+s[m])/2; };
  const point = median(diffs);
  const boots = Array.from({length: resamples}, () => {
    const sample = Array.from({length: n}, () => diffs[Math.floor(rand()*n)]);
    return median(sample);
  });
  const sortedBoots = sorted(boots);
  const lo = sortedBoots[Math.floor(0.025*resamples)], hi = sortedBoots[Math.floor(0.975*resamples)];
  return {median: point, ci95: [lo, hi]};
}

// Holm-Bonferroni step-down 보정 (§5.10 "사전 지정 주 비교엔 Holm 보정 적용").
function holmCorrection(pValues) {
  const m = pValues.length;
  const indexed = pValues.map((p, i) => ({p, i})).sort((a, b) => a.p-b.p);
  const adjusted = new Array(m);
  let runningMax = 0;
  indexed.forEach(({p, i}, rank) => {
    const value = Math.min(1, (m-rank)*p);
    runningMax = Math.max(runningMax, value);
    adjusted[i] = runningMax;
  });
  return adjusted;
}

// 정규분포 근사 없이, 부트스트랩 median 차이의 부호가 0을 넘는 비율로 양측
// p-value를 근사한다(순열/부트스트랩 p-value의 표준적인 근사 방식).
function bootstrapPValue(diffs, resamples=10000, seed=2) {
  const rand = (() => { let s = seed>>>0; return () => { s=(Math.imul(1664525,s)+1013904223)>>>0; return s/4294967296; }; })();
  const n = diffs.length;
  const median = a => { const s=a.slice().sort((x,y)=>x-y); const m=Math.floor(s.length/2); return s.length%2?s[m]:(s[m-1]+s[m])/2; };
  const shifted = diffs.map(d => d-median(diffs)); // 귀무가설(중앙값=0) 아래로 재중심화
  let extreme = 0;
  const observed = Math.abs(median(diffs));
  for (let b = 0; b < resamples; b++) {
    const sample = Array.from({length: n}, () => shifted[Math.floor(rand()*n)]);
    if (Math.abs(median(sample)) >= observed) extreme++;
  }
  return Math.max(1/resamples, extreme/resamples);
}

async function main() {
  const args = process.argv.slice(2);
  const flag = (name, dflt) => { const i = args.indexOf(`--${name}`); return i>=0 ? args[i+1] : dflt; };
  const seeds = Number(flag('seeds', '2'));
  const conditionIds = flag('conditions', null)?.split(',');
  const controllerIds = flag('controllers', null)?.split(',');
  const seconds = Number(flag('seconds', String(RAMP_T0+RAMP_DURATION_S+4))); // 램프 끝난 뒤 정착 4초
  const outFile = flag('out', null);
  const profile = flag('profile', 'selected');

  const conditions = CONDITIONS.filter(c => !conditionIds || conditionIds.includes(c.id));
  const controllers = CONTROLLERS.filter(c => !controllerIds || controllerIds.includes(c));

  console.error(`sweep: ${conditions.length}조건 x ${controllers.length}제어기 x ${seeds}난수 = ${conditions.length*controllers.length*seeds}회`);

  const ca = await require('@casadi/casadi-wasm')();
  await ca.load_nlpsol('ipopt');
  await ca.load_interpolant('linear');
  const data = JSON.parse(fs.readFileSync(path.join(__dirname, 'generated', profile+'.json'), 'utf8'));

  const results = {}; // results[conditionId][controller] = [{seed, rmse_v, rmse_z, failed}, ...]
  let done = 0, total = conditions.length*controllers.length*seeds;
  for (const cond of conditions) {
    results[cond.id] = {};
    for (const controller of controllers) {
      const rows = [];
      for (let s = 0; s < seeds; s++) {
        const seed = 1000+s;
        // 'step'(순간 계단)이 아니라 'ramp'(식43 매끄러운 참조)를 쓴다 -- 논문
        // §5.5 자신의 규칙("계단형은 입력포화·지연 진단 전용, 주 기동 비교는
        // 연속 참조") + 실측: V=8 m/s 순간 스텝은 무풍에서도 hybrid/M17 둘 다
        // 예고(preview)가 과도한 피치 오버슈트를 만들어 추락했다(2026-09-22
        // 재현·원인 확인, ramp로 바꾸자 동일 조건이 정확히 수렴).
        const opts = {controller, feedback: 'truth', scenario: 'ramp', speed: cond.speed,
                     altitude: 20, ramp_t0: RAMP_T0, ramp_duration_s: RAMP_DURATION_S,
                     seconds, seed, ...cond.opts};
        let result;
        try {
          result = await runtime.run(ca, data, opts, () => {});
        } catch (e) {
          rows.push({seed, failed: true, reason: `threw: ${e.message}`, rmse_v: Infinity, rmse_z: Infinity});
          done++; continue;
        }
        rows.push({seed, ...evaluateRun(result, cond.speed, 20)});
        done++;
        if (done%10===0||done===total) console.error(`  ${done}/${total}`);
      }
      results[cond.id][controller] = rows;
    }
  }

  // 표9 형태: 조건x제어기 요약 (실패율, RMSE 평균).
  const summary = {};
  for (const cond of conditions) {
    summary[cond.id] = {};
    for (const controller of controllers) {
      const rows = results[cond.id][controller];
      const ok = rows.filter(r => !r.failed);
      summary[cond.id][controller] = {
        success_fraction: ok.length/rows.length,
        rmse_v_mean: ok.length ? ok.reduce((s,r)=>s+r.rmse_v,0)/ok.length : null,
        rmse_z_mean: ok.length ? ok.reduce((s,r)=>s+r.rmse_z,0)/ok.length : null,
      };
    }
  }

  // 표9 "주 비교": V13(hybrid) 대 나머지 5개. 짝지은(같은 시드) RMSE_v 차이의
  // 부트스트랩 median/95%CI + Holm 보정 p-value.
  const primaryComparisons = controllers.filter(c => c !== 'hybrid').map(other => {
    const a = results[conditions[0]?.id]?.['hybrid'], b = results[conditions[0]?.id]?.[other];
    if (!a || !b) return null;
    const diffs = a.map((r, i) => (b[i].rmse_v-r.rmse_v)).filter(Number.isFinite);
    if (!diffs.length) return {pair: `hybrid-vs-${other}`, note: '유한 표본 없음(둘 다 실패했거나 표본 부족)'};
    return {pair: `hybrid-vs-${other}`, ...bootstrapPairedMedianCI(diffs), p: bootstrapPValue(diffs)};
  }).filter(Boolean);
  const pValues = primaryComparisons.map(c => c.p ?? 1);
  const adjusted = holmCorrection(pValues);
  primaryComparisons.forEach((c, i) => { c.p_holm = adjusted[i]; });

  const output = {conditions: conditions.map(c => c.id), controllers, seeds, seconds, profile,
                  summary, primary_comparisons_first_condition: primaryComparisons, raw: results};
  const text = JSON.stringify(output, null, 2);
  if (outFile) { fs.writeFileSync(outFile, text); console.error(`written: ${outFile}`); }
  else console.log(text);
}

module.exports = {rmse, evaluateRun, bootstrapPairedMedianCI, bootstrapPValue, holmCorrection,
                  CONDITIONS, CONTROLLERS, V_L, V_H, RAMP_T0, RAMP_DURATION_S};

if (require.main === module) main().catch(e => { console.error(e); process.exitCode = 1; });
