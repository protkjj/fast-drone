"""튜닝 기록 승계(control/tuning_carryover.py, arena_tune_repro 승계 모드) — kj 결정(2026-09-28 밤).

  1) 표에 있는 기록(파일 sha256 일치)만 arena.json 해시가 arena_v2에서 통과한다.
  2) 기록 파일이 바뀌면, 표에 없으면, 표가 없으면, 증명이 비트 동일이 아니면 거부한다.
  3) 승계 모드는 보정 대상(M17·F13·GSLQR)이나 모멘트 절 밖이 다른 설정에서는 열리지 않는다.
  4) 튜닝값 로더(tuned_overrides)가 같은 규칙을 쓴다.
"""
from copy import deepcopy
import json
import shutil

import pytest

from control.arena import ROOT, load_config
from control.arena_tune_repro import carryover_allowed
from control.tuning_carryover import load_table, record_hash_problems, DEFAULT_TABLE

RUN_DIR = ROOT/'results'/'arena'/'tuning'/'retune_v3'
V2 = ROOT/'configs'/'arena_v2.json'


@pytest.fixture(scope='module')
def table():
    doc = load_table(DEFAULT_TABLE)
    if doc is None or 'CPID' not in doc['entries']:
        pytest.skip('carry-over table has no CPID entry yet')
    return doc


def _record(label):
    path = RUN_DIR/f'{label}.record.json'
    return json.loads(path.read_text(encoding='utf-8')), path


def test_carried_record_passes_under_v2(table):
    assert record_hash_problems([_record('CPID')], load_config(V2), table) == []


def test_uncarried_old_record_is_refused_under_v2(table):
    problems = record_hash_problems([_record('GSLQR')], load_config(V2), table)
    assert problems and 'not in the carry-over table' in problems[0]


def test_edited_record_file_is_refused(table, tmp_path):
    record, path = _record('CPID')
    copy = tmp_path/'CPID.record.json'
    shutil.copy(path, copy)
    copy.write_text(copy.read_text(encoding='utf-8') + ' ', encoding='utf-8')
    problems = record_hash_problems([(record, copy)], load_config(V2), table)
    assert any('record file sha256' in p for p in problems)


def test_without_a_table_old_hashes_are_refused():
    assert record_hash_problems([_record('CPID')], load_config(V2), None)


def test_failed_proof_is_refused(table):
    bad = deepcopy(table)
    bad['entries']['CPID']['proofs']['best']['bit_identical'] = False
    assert any('bit-identical' in p for p in record_hash_problems([_record('CPID')], load_config(V2), bad))


def test_same_config_needs_no_table():
    assert record_hash_problems([_record('GSLQR')], load_config(), None) == []


def test_carryover_mode_is_closed_for_corrected_controllers_and_other_differences():
    v2 = load_config(V2)
    assert carryover_allowed(v2, 'CPID') is not None and carryover_allowed(v2, 'V13') is not None
    for label in ('M17', 'F13', 'GSLQR'):
        assert carryover_allowed(v2, label) is None
    other = deepcopy(v2)
    other['tuning']['budget'] = 7
    assert carryover_allowed(other, 'CPID') is None
    assert carryover_allowed(load_config(), 'CPID') is None


def test_tuned_overrides_uses_the_same_rule(table):
    from control.arena_design_check import tuned_overrides
    overrides, used = tuned_overrides(load_config(V2), RUN_DIR, ['CPID'])
    assert 'CPID' in used
    with pytest.raises(ValueError, match='not in the carry-over table'):
        tuned_overrides(load_config(V2), RUN_DIR, ['GSLQR'])
