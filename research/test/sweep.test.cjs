const {test} = require('node:test'), assert = require('node:assert/strict');
const S = require('../sweep.cjs');

test('bootstrap paired-median CI excludes 0 for a clear shift, includes 0 for none', () => {
  const shifted = Array.from({length: 20}, (_, i) => 2.0+(i%5-2)*0.3);
  const r1 = S.bootstrapPairedMedianCI(shifted, 2000, 7);
  assert.ok(Math.abs(r1.median-2.0) < 1e-9);
  assert.ok(r1.ci95[0] > 0, 'CI should exclude 0 for a clearly positive shift');

  const centered = Array.from({length: 20}, (_, i) => (i%2===0?1:-1)*0.5);
  const r2 = S.bootstrapPairedMedianCI(centered, 2000, 7);
  assert.ok(r2.ci95[0] <= 0 && r2.ci95[1] >= 0, 'CI should contain 0 for no real difference');
});

test('holm correction never sets an adjusted p-value below its raw p-value, and is monotone by rank', () => {
  const raw = [0.01, 0.04, 0.03, 0.20];
  const adj = S.holmCorrection(raw);
  raw.forEach((p, i) => assert.ok(adj[i] >= p-1e-9, `adjusted[${i}] must be >= raw[${i}]`));
  const order = raw.map((_, i) => i).sort((a, b) => raw[a]-raw[b]);
  order.forEach((idx, rank) => { if (rank>0) assert.ok(adj[idx] >= adj[order[rank-1]]-1e-9, 'adjusted p must be non-decreasing in rank order'); });
  // Hand-computed Holm step-down for this exact input (m=4).
  assert.deepEqual(adj.map(v => Number(v.toFixed(4))), [0.04, 0.09, 0.09, 0.2]);
});

test('rmse matches the direct formula', () => {
  assert.ok(Math.abs(S.rmse([3, 4])-Math.sqrt(12.5)) < 1e-12);
  assert.equal(S.rmse([0, 0, 0]), 0);
});

test('evaluateRun reports a failed run as infinite RMSE, not a silently small number', () => {
  const failed = S.evaluateRun({failure: 'ground crossing', trace: []}, 10, 20);
  assert.equal(failed.failed, true);
  assert.equal(failed.rmse_v, Infinity);
  assert.equal(failed.rmse_z, Infinity);
});

test('evaluateRun computes steady-state RMSE only from samples after the ramp completes', () => {
  // 램프 구간(t < RAMP_T0+RAMP_DURATION_S, 과도구간, 큰 오차)과 그 이후
  // (정착, 오차 0)를 섞은 합성 trace.
  const rampEnd = S.RAMP_T0+S.RAMP_DURATION_S;
  const trace = [
    ...Array.from({length: 10}, (_, i) => ({t: i*(rampEnd/10), v: [0, 0, 0], z: 0})),  // 과도구간(t<rampEnd)
    ...Array.from({length: 10}, (_, i) => ({t: rampEnd+1+i, v: [10, 0, 0], z: 20})),   // 정착(목표 v=10,z=20)
  ];
  const result = S.evaluateRun({failure: null, trace}, 10, 20);
  assert.equal(result.failed, false);
  assert.ok(result.rmse_v < 1e-9, `steady-state RMSE_v should be ~0, got ${result.rmse_v}`);
  assert.ok(result.rmse_z < 1e-9, `steady-state RMSE_z should be ~0, got ${result.rmse_z}`);
});

test('evaluateRun fails cleanly (not silently 0) when no sample lies after the ramp window', () => {
  const before = (S.RAMP_T0+S.RAMP_DURATION_S)/2;
  const trace = [{t: 0, v: [0, 0, 0], z: 0}, {t: before, v: [0, 0, 0], z: 0}];
  const result = S.evaluateRun({failure: null, trace}, 10, 20);
  assert.equal(result.failed, true);
});

test('CONDITIONS covers all 12 표8 IDs exactly once, each referencing a valid runtime option', () => {
  const ids = S.CONDITIONS.map(c => c.id);
  assert.deepEqual(ids, ['Q01','Q02','Q03','Q04','Q05','Q06','Q07','Q08','Q09','Q10','Q11','Q12']);
  const known = new Set(['wind_speed','wind_angle','wind_vertical_mps','scales','initial_soc',
    'gps_position_std_m','gps_delay_ms','indi_rpm_desync_ms']);
  for (const c of S.CONDITIONS) for (const key of Object.keys(c.opts)) assert.ok(known.has(key), `${c.id} uses unknown option ${key}`);
});

test('CONTROLLERS lists exactly the 표5 minimum set (CPID,GSLQR,M17,F13,V13,GINDI internal names)', () => {
  assert.deepEqual(S.CONTROLLERS, ['cpid','gslqr','nmpc','f13','hybrid','gindi']);
});

test('V_L/V_H stay inside the confirmed 0-18 m/s level-trim band', () => {
  assert.ok(S.V_L >= 0 && S.V_L <= 18);
  assert.ok(S.V_H >= 0 && S.V_H <= 18);
  assert.ok(S.V_L < S.V_H);
});
