const {test} = require('node:test'), assert = require('node:assert/strict');
const R = require('../runtime.js');

function fakeFrame(t) {
  return {t, position_m: [1, 2, 3], v: [4, 5, 6], q_xyzw: [0, 0, 0, 1],
    body_rate_rad_s: [.1, .2, .3], rotor_rad_s: [100, 101, 102, 103],
    command_rad_s: [110, 111, 112, 113], reference: [8, 0, 0, 20], soc: .95,
    motor: {voltage_V: 22.1, bus_current_A: 5.5, power_W: 120, thrust_N: [1, 2, 3, 4]}};
}

test('traceToCSV: header matches CSV_COLUMNS and row count matches trace length', () => {
  const result = {trace: [fakeFrame(0), fakeFrame(0.02), fakeFrame(0.04)]};
  const csv = R.traceToCSV(result);
  const lines = csv.trim().split('\n');
  assert.equal(lines.length, 4); // header + 3 rows
  assert.equal(lines[0], R.CSV_COLUMNS.join(','));
});

test('traceToCSV: a data row has the right column count and correct values in order', () => {
  const result = {trace: [fakeFrame(1.5)]};
  const csv = R.traceToCSV(result);
  const row = csv.trim().split('\n')[1].split(',');
  assert.equal(row.length, R.CSV_COLUMNS.length);
  assert.equal(row[0], '1.5');           // t_s
  assert.equal(row[1], '1');             // pos_x_m
  assert.equal(row[4], '4');             // vx_mps
  assert.equal(row[26], '0.95');         // soc
  assert.equal(row[32], '3');            // thrust3_N
});

test('traceToCSV throws a clear error for a sweep summary (no .trace), not a silent empty file', () => {
  assert.throws(() => R.traceToCSV({summary: {}}), /trace/);
});

test('traceToCSV handles a missing motor block without crashing (blank cells, not a thrown error)', () => {
  const frame = fakeFrame(0); delete frame.motor;
  const csv = R.traceToCSV({trace: [frame]});
  const row = csv.trim().split('\n')[1].split(',');
  assert.equal(row.length, R.CSV_COLUMNS.length);
  assert.equal(row[27], ''); // voltage_V blank
});
