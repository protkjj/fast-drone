const {test} = require('node:test'), assert = require('node:assert/strict');
const R = require('../runtime.js');

test('smoothstep: boundary values, monotone, matches 식43 exactly at u=0.5', () => {
  assert.equal(R.smoothstep(-1), 0);
  assert.equal(R.smoothstep(0), 0);
  assert.equal(R.smoothstep(1), 1);
  assert.equal(R.smoothstep(2), 1);
  let prev = -1;
  for (let u = 0; u <= 1; u += 0.05) {
    const v = R.smoothstep(u);
    assert.ok(v >= prev - 1e-12, `smoothstep must be non-decreasing (u=${u})`);
    prev = v;
  }
  // 식(43) u=0.5 손계산: 126/32-420/64+540/128-315/256+70/512 = 0.5
  assert.ok(Math.abs(R.smoothstep(0.5) - 0.5) < 1e-9);
});

test('ramp scenario: reference holds v0 before t0 and v1 after t0+Tr, smooth in between', () => {
  const cfg = R.validateOptions({scenario: 'ramp', speed: 8, altitude: 20, ramp_t0: 1, ramp_duration_s: 2, seconds: 5});
  assert.deepEqual(R.reference(0, cfg), [0, 0, 0, 20]);
  assert.deepEqual(R.reference(0.9, cfg), [0, 0, 0, 20]);
  const mid = R.reference(2, cfg); // (t-t0)/Tr = 0.5
  assert.ok(Math.abs(mid[0] - 4) < 1e-6, `midpoint vx should be speed*0.5=4, got ${mid[0]}`);
  const end = R.reference(3, cfg);
  assert.ok(Math.abs(end[0] - 8) < 1e-9);
  const after = R.reference(4.5, cfg);
  assert.ok(Math.abs(after[0] - 8) < 1e-9);
});

test('ramp scenario rejects a duration too short to contain the ramp', () => {
  assert.throws(() => R.validateOptions({scenario: 'ramp', speed: 8, ramp_t0: 1, ramp_duration_s: 2, seconds: 2}));
});

test("ramp is NOT the same as an instantaneous step -- no discontinuity at any single tick", () => {
  const cfg = R.validateOptions({scenario: 'ramp', speed: 8, ramp_t0: 1, ramp_duration_s: 2, seconds: 5});
  const dt = 0.001;
  let maxJump = 0;
  for (let t = 0; t < 4; t += dt) {
    const a = R.reference(t, cfg)[0], b = R.reference(t + dt, cfg)[0];
    maxJump = Math.max(maxJump, Math.abs(b - a));
  }
  // 순간 스텝(step 시나리오)이면 한 틱에 8 m/s가 그대로 뛴다. ramp는 2초에 걸쳐
  // 퍼지니 한 틱(1ms) 최대 변화가 그보다 훨씬 작아야 한다.
  assert.ok(maxJump < 0.05, `max single-tick jump should be small for a 2s ramp, got ${maxJump}`);
});
