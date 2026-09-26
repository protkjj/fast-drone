"""arena_reference_merge — 나눠 돌린 스모크 실행을 합치는 규칙을 합성 실행으로 확인한다(빠름, 시뮬레이션 없음)."""
import pytest

from control.arena_reference_merge import merge, MergeRefused

ORDER = ('V13', 'M17', 'CPID')          # 설정의 제어기 순서


def fake_run(name, controllers, tuned=None, **changes):
    """validation_suite가 남기는 (manifest, arena_reference)의 필요한 부분만 흉내 낸다. 사례 2개."""
    manifest = dict(status='complete', recorded_trials=2*len(controllers), expected_trials=2*len(controllers),
                    git_dirty=False, config_sha256='c'*64, git_revision='r'*40, environment={'cpu': 'M4'},
                    source_sha256={'control/x.py': '1'}, parameter_sha256='p', controller_model_sha256='m',
                    scenarios=[dict(id='s1', cases=[{}]), dict(id='s2', cases=[{}])],
                    config=dict(controllers={label: {} for label in ORDER}), controllers=list(controllers))
    if tuned is not None:
        manifest['tuned'] = dict(run_dir=tuned, controllers={c: dict(record_sha256=c*4) for c in controllers})
    manifest.update(changes)
    reference = dict(label='SMOKE', created_utc='2026-09-28T00:00:00+00:00',
                     command='python -m control.validation_suite --config configs/arena.json --smoke'
                             ' --only-controllers ' + ' '.join(controllers),
                     controller_settings={c: dict(kind=c) for c in controllers},
                     rows=[dict(scenario_id=s, controller=c, trajectory_sha256=s + c)
                           for s in ('s1', 's2') for c in controllers])
    return name, manifest, reference


def test_merge_orders_rows_like_a_single_run_and_keeps_provenance():
    merged = merge([fake_run('A', ['V13', 'CPID']), fake_run('B', ['M17'])])
    assert [(r['scenario_id'], r['controller']) for r in merged['rows']] == \
        [(s, c) for s in ('s1', 's2') for c in ORDER]
    assert [r['merged_from'] for r in merged['rows'][:3]] == ['A', 'B', 'A']
    assert set(merged['controller_settings']) == set(ORDER)
    assert [m['controllers'] for m in merged['merged_from']] == [['V13', 'CPID'], ['M17']]
    assert merged['command'].count('--only-controllers') == 2       # 명령에 이미 있으면 다시 붙이지 않는다
    assert 'tuned' not in merged


def test_merge_of_tuned_runs_keeps_every_record_hash():
    merged = merge([fake_run('A', ['V13', 'CPID'], tuned='results/arena/tuning/main120'),
                    fake_run('B', ['M17'], tuned='results/arena/tuning/main120')])
    assert merged['tuned']['run_dir'] == 'results/arena/tuning/main120'
    assert set(merged['tuned']['controllers']) == set(ORDER)


@pytest.mark.parametrize('runs, needle', [
    ([fake_run('A', ['V13', 'CPID'], recorded_trials=3), fake_run('B', ['M17'])], 'not complete'),
    ([fake_run('A', ['V13', 'CPID'], git_dirty=True), fake_run('B', ['M17'])], 'git dirty'),
    ([fake_run('A', ['V13', 'CPID']), fake_run('B', ['M17'], git_revision='s'*40)], 'git_revision differs'),
    ([fake_run('A', ['V13', 'CPID']), fake_run('B', ['M17'], environment={'cpu': 'M1'})], 'environment differs'),
    ([fake_run('A', ['V13', 'CPID']), fake_run('B', ['M17', 'CPID'])], 'ran in both'),
    ([fake_run('A', ['V13']), fake_run('B', ['M17'])], 'not covered'),
    ([fake_run('A', ['V13', 'CPID'], tuned='x'), fake_run('B', ['M17'])], 'gain sources differ'),
    ([fake_run('A', ['V13', 'M17', 'CPID'])], 'at least two'),
])
def test_merge_refuses_runs_that_do_not_belong_together(runs, needle):
    with pytest.raises(MergeRefused, match=needle):
        merge(runs)
