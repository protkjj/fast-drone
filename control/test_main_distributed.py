"""본 실험 분산 실행(`control/main_distributed.py`) — kj 결정(2026-09-28) 시험. 시뮬레이션은 가짜로 바꾼다.

  1) 가드: 튜닝 해시가 비면 조각 만들기·실행 모두 거부.
  2) 조각 배분: 모든 시행이 정확히 한 조각에, 예상 시간 기준으로 고르게(LPT), 같은 입력이면 같은 배분.
  3) 이어 돌리기: 끝난 시행은 건너뛴다, 끊긴 시행(임시 파일)은 지우고 다시 돈다, 해시가 다른 끝난 파일은 거부.
  4) 합치기: 누락·중복·해시 불일치·지문 누락·플랫폼·코드 상태 불일치를 잡고, 문제가 있으면 merged.jsonl을 안 쓴다.
  5) 교차 확인: 같은 컴퓨터 거부, 판정 불일치·rtol 초과면 FAIL, 같으면 PASS.
"""
from copy import deepcopy
import hashlib
import json

import pytest

from control import main_distributed as md
from control import main_experiment as me
from control.arena import ROOT

RUN_DIR = 'results/arena/tuning/retune_v3'


def _spec_file(tmp_path, labels=('GSLQR', 'CPID')):
    """가드를 통과하는 작은 spec — 완료된 retune_v3 GSLQR·CPID 기록을 쓴다(시험·맥 점검 전용)."""
    spec = me.load_spec()
    # 분산 실행 기능만 본다 — 기준을 arena.json으로 두어 retune_v3(arena.json으로 튜닝) 기록이 그대로
    # 통과하게 한다(arena_v2에서는 GSLQR이 재튜닝 대상이라 기록이 거부되는 것이 정상).
    spec['base_config'] = dict(path='configs/arena.json',
                               config_sha256='1376310bd08e4466da61d87b57af9b13274c198cfdfc2e7c3077ceb19bad8903')
    spec.pop('moment_model', None)
    spec.pop('carryover', None)
    spec['controllers'] = list(labels)
    spec['tuned']['run_dir'] = RUN_DIR
    spec['tuned']['records'] = {label: {'sha256': hashlib.sha256(
        (ROOT/RUN_DIR/f'{label}.record.json').read_bytes()).hexdigest()} for label in labels}
    path = tmp_path/'spec.json'
    path.write_text(json.dumps(spec, ensure_ascii=False), encoding='utf-8')
    return path


@pytest.fixture
def fake_trials(monkeypatch):
    """Campaign.run_trial을 가짜로 — 시행 ID로 정해지는 결정적 행. 호출 기록을 돌려준다."""
    calls = []

    def fake(self, trial):
        calls.append(trial['trial_id'])
        row = dict(trial_id=trial['trial_id'], batch=trial['batch'], scenario_id=trial['scenario_id'],
                   controller=trial['controller'], variant=trial['variant'],
                   **self.hashes_for(trial['controller']), skipped=False, skip_reason=None, passed=True,
                   paper_failed=False, paper_reasons=[], stop_reason=None, failure_reasons=[],
                   window_rmse_velocity=0.1, window_rmse_z=0.2, window_max_omega=1.0, max_omega=1.0,
                   trajectory_sha256='x')
        return row, None

    monkeypatch.setattr(md.Campaign, 'run_trial', fake)
    return calls


def test_guard_refuses_to_make_or_run_shards_while_tuning_is_empty(tmp_path):
    with pytest.raises(SystemExit, match='refusing to make shards'):
        md.make_shards(me.DEFAULT_SPEC, 2, tmp_path/'s.json', only_batches=['gust'])
    doc = dict(schema=md.SHARDS_SCHEMA, spec_path=str(me.DEFAULT_SPEC), spec_sha256='',
               shards=[dict(index=0, trials=[])], trials={})
    (tmp_path/'s.json').write_text(json.dumps(doc), encoding='utf-8')
    with pytest.raises(SystemExit, match='refusing to run the shard'):
        md.run_shard(tmp_path/'s.json', 0, tmp_path/'out')
    assert not (tmp_path/'out').exists()


def test_shards_cover_every_trial_once_balanced_and_deterministic(tmp_path):
    spec = _spec_file(tmp_path)
    a = md.make_shards(spec, 3, tmp_path/'a.json', only_batches=['gust', 'reference'])
    b = md.make_shards(spec, 3, tmp_path/'b.json', only_batches=['gust', 'reference'])
    assigned = [t for s in a['shards'] for t in s['trials']]
    assert sorted(assigned) == sorted(a['trials']) and len(assigned) == len(set(assigned))
    assert [s['trials'] for s in a['shards']] == [s['trials'] for s in b['shards']]
    loads = [s['est_hours_mac'] for s in a['shards']]
    longest = max(t['est_seconds'] for t in a['trials'].values())/3600.0
    assert max(loads) - min(loads) <= longest + 1e-12      # LPT: 차이는 가장 긴 시행 하나 이하


def test_resume_skips_finished_reruns_interrupted_and_refuses_mixed(tmp_path, fake_trials):
    spec = _spec_file(tmp_path)
    doc = md.make_shards(spec, 1, tmp_path/'s.json', only_batches=['gust'])
    ids = doc['shards'][0]['trials']
    md.run_shard(tmp_path/'s.json', 0, tmp_path/'out', max_trials=3)
    assert fake_trials == ids[:3]
    folder = tmp_path/'out'/'shard0'/'trials'
    # 끊긴 시행: 4번째가 임시 파일만 남은 상태
    (folder/(md.safe_name(ids[3])+'.json.tmp')).write_text('{"partial": ', encoding='utf-8')
    fake_trials.clear()
    md.run_shard(tmp_path/'s.json', 0, tmp_path/'out')
    assert fake_trials == ids[3:]                           # 끝난 3개는 건너뛰고 끊긴 것부터
    assert not list(folder.glob('*.tmp'))
    # 해시가 다른 끝난 파일은 섞지 않는다
    row = json.loads((folder/(md.safe_name(ids[0])+'.json')).read_text(encoding='utf-8'))
    row['tuned_record_sha256'] = 'other'
    (folder/(md.safe_name(ids[0])+'.json')).write_text(json.dumps(row), encoding='utf-8')
    with pytest.raises(SystemExit, match='different hashes'):
        md.run_shard(tmp_path/'s.json', 0, tmp_path/'out')


def _merged_setup(tmp_path, fake_trials, n=2):
    spec = _spec_file(tmp_path)
    md.make_shards(spec, n, tmp_path/'s.json', only_batches=['gust'])
    for i in range(n):
        md.run_shard(tmp_path/'s.json', i, tmp_path/'out')
    return tmp_path/'s.json', tmp_path/'out'


def test_merge_accepts_complete_results_and_refuses_the_wrong_platform(tmp_path, fake_trials):
    shards, out = _merged_setup(tmp_path, fake_trials)
    import platform
    ok = md.merge(shards, [out], tmp_path/'m', require_platform=platform.system())
    assert ok['problems'] == [] and (tmp_path/'m'/'merged.jsonl').exists()
    bad = md.merge(shards, [out], tmp_path/'m2', require_platform='Windows' if platform.system() != 'Windows' else 'Darwin')
    assert bad['problems'] and all('platform' in p for p in bad['problems'])
    assert not (tmp_path/'m2'/'merged.jsonl').exists()


@pytest.mark.parametrize('tamper, expect', [
    ('delete', 'missing'), ('duplicate', 'duplicate'), ('hash', 'mismatch'),
    ('fingerprint', 'fingerprint missing'), ('revision', 'different code states'),
])
def test_merge_catches_each_problem(tmp_path, fake_trials, tamper, expect):
    import platform
    shards, out = _merged_setup(tmp_path, fake_trials)
    files = sorted((out/'shard0'/'trials').glob('*.json'))
    if tamper == 'delete':
        files[0].unlink()
    elif tamper == 'duplicate':
        (tmp_path/'copy'/'shard9'/'trials').mkdir(parents=True)
        (tmp_path/'copy'/'shard9'/'trials'/files[0].name).write_bytes(files[0].read_bytes())
    else:
        row = json.loads(files[0].read_text(encoding='utf-8'))
        if tamper == 'hash':
            row['base_config_sha256'] = 'other'
        elif tamper == 'fingerprint':
            row['machine'] = {}
        else:
            row['git_revision'] = 'other'
        files[0].write_text(json.dumps(row), encoding='utf-8')
    inputs = [out, tmp_path/'copy'] if tamper == 'duplicate' else [out]
    report = md.merge(shards, inputs, tmp_path/'m', require_platform=platform.system())
    assert any(expect in p for p in report['problems']), report['problems']
    assert not (tmp_path/'m'/'merged.jsonl').exists()


def test_cross_check_refuses_same_machine_and_detects_differences(tmp_path, fake_trials):
    import platform
    shards, out = _merged_setup(tmp_path, fake_trials)
    md.merge(shards, [out], tmp_path/'m', require_platform=platform.system())
    cross = md.cross_select(shards, tmp_path/'m', tmp_path/'cross.json', fraction=0.2, seed=1)
    picked = cross['shards'][0]['trials']
    # 가짜 시행은 건너뛰기를 안 해서 gust 8×2 = 16개 전부가 '돈 시행'이다 → 20%면 ceil(3.2) = 4개
    assert cross['cross_check']['population'] == 16 and len(picked) == 4
    md.run_shard(tmp_path/'cross.json', 0, tmp_path/'rerun')
    same = md.cross_compare(tmp_path/'cross.json', tmp_path/'m', tmp_path/'rerun')
    assert not same['passed'] and all('same computer' in p for p in same['problems'])
    allowed = md.cross_compare(tmp_path/'cross.json', tmp_path/'m', tmp_path/'rerun', allow_same_machine=True)
    assert allowed['passed']
    # 수치 rtol 초과와 판정 불일치
    f = sorted((tmp_path/'rerun'/'shard0'/'trials').glob('*.json'))
    row = json.loads(f[0].read_text(encoding='utf-8')); row['window_rmse_z'] *= 1.01
    f[0].write_text(json.dumps(row), encoding='utf-8')
    row = json.loads(f[1].read_text(encoding='utf-8')); row['passed'] = False
    f[1].write_text(json.dumps(row), encoding='utf-8')
    bad = md.cross_compare(tmp_path/'cross.json', tmp_path/'m', tmp_path/'rerun', allow_same_machine=True)
    assert not bad['passed'] and len(bad['problems']) == 2
