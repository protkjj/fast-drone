"""공정 비교 하네스 검증.

여기서 지키려는 것은 성능이 아니라 **경기장이 기울지 않았는지**다.
기울어짐은 양방향 모두 실격이다 — 하이브리드에 유리한 설정도 똑같이 막는다.
"""
import numpy as np
import pytest

from control.fair_compare import (ComparisonSpec, ReferenceNotDelivered,
                                  _build, audit_matched, run_one, smoothstep,
                                  speed_ramp)
from control.vehicle_params import vehicle_params as P

V_TABLE = list(np.arange(0, 90, 10).astype(float))


def _spec(**kw):
    base = dict(params=P, reference=speed_ramp(0., 14., 1., 6.), duration=2.0,
                V_table=V_TABLE)
    base.update(kw)
    return ComparisonSpec(**base)


# ── 참조 함수 ─────────────────────────────────────────────────────────

def test_smoothstep_endpoints_and_flatness():
    """논문 식(32): 양 끝에서 1~4차 도함수가 0이라 이어 붙여도 꺾이지 않는다."""
    assert smoothstep(0.0) == 0.0
    assert smoothstep(1.0) == pytest.approx(1.0)
    assert smoothstep(-5.0) == 0.0 and smoothstep(5.0) == pytest.approx(1.0)
    # 끝단 1차 도함수가 0 인지 수치적으로
    h = 1e-5
    assert abs(smoothstep(h) - smoothstep(0.0))/h < 1e-8
    assert abs(smoothstep(1.0) - smoothstep(1.0 - h))/h < 1e-8


def test_speed_ramp_holds_before_and_after():
    r = speed_ramp(3.0, 9.0, 1.0, 4.0)
    assert r(0.0) == pytest.approx(3.0)
    assert r(1.0) == pytest.approx(3.0)
    assert r(5.0) == pytest.approx(9.0)
    assert r(99.0) == pytest.approx(9.0)
    assert 3.0 < r(3.0) < 9.0


# ── 조건 일치 감사 ────────────────────────────────────────────────────

def test_audit_flags_preview_mismatch():
    """LQR 은 미래 참조를 못 쓰고 V13 은 쓴다 — 섞으면 반드시 알려야 한다.

    이 경고가 없으면 '미리보기 덕분인 우위'가 '아키텍처 덕분인 우위'로
    오인된다. 실측: 고속 램프에서 V13 은 LQR 대비 고도 10.1배 우수하지만,
    미리보기를 빼면 0.26배(= LQR 이 3.8배 더 좋다)로 뒤집힌다.
    """
    warnings = audit_matched(_spec(), ['GSLQR', 'V13'])
    assert any('preview' in w for w in warnings), warnings


def test_audit_passes_when_information_is_matched():
    """미리보기를 양쪽 다 끄면 경고가 없어야 한다."""
    warnings = audit_matched(_spec(), ['GSLQR', 'V13-nopreview'])
    assert warnings == [], warnings


def test_audit_flags_mixed_ladder_flags():
    """표6 사다리의 단을 섞으면 하위 루프가 달라져 비교가 깨진다."""
    spec = _spec()
    v0 = spec.reference(0.0)
    a, _ = _build('V13', spec, v0)
    b, _ = _build('V13-nopreview', spec, v0)
    assert a.alloc_mode == b.alloc_mode
    assert a.time_align == b.time_align
    # 내부 상위 제어기의 설정도 같아야 한다
    assert a.nmpc.dt_ctrl == b.nmpc.dt_ctrl
    assert a.nmpc.N == b.nmpc.N
    assert a.nmpc.cost_spec == b.nmpc.cost_spec
    assert a.nmpc._max_iter == b.nmpc._max_iter


# ── 사고 방지 ─────────────────────────────────────────────────────────

def test_reference_target_is_the_object_that_reads_it():
    """ProperHybrid 는 참조를 안 읽는다 — 내부 상위 제어기가 읽는다.

    바깥에 v_ref 를 꽂으면 새 속성이 조용히 생기고 제어기는 초기 참조에
    머문다. 그 사고로 'V13 이 70 m/s 를 못 따라간다'는 가짜 결과가 나온 적이
    있다(LQR 만 램프를 받았다). 구조적으로 막혀 있어야 한다.
    """
    spec = _spec()
    for name in ('V13', 'V13-nopreview', 'GINDI'):
        controller, target = _build(name, spec, spec.reference(0.0))
        assert target is controller.nmpc, name
        assert not hasattr(controller, 'v_ref'), (
            f"{name}: ProperHybrid 에 v_ref 가 있으면 바깥에 꽂는 실수를 "
            f"구조가 막아 주지 못한다")
    controller, target = _build('GSLQR', spec, spec.reference(0.0))
    assert target is controller


def test_run_one_rejects_a_mismatched_reference_target(monkeypatch):
    """참조를 읽지 않는 객체를 지목하면 조용히 나쁜 숫자 대신 예외가 나야 한다."""
    import control.fair_compare as fc
    real = fc._build

    def decoyed(name, spec, v0):
        controller, _ = real(name, spec, v0)
        return controller, controller      # ProperHybrid 자신을 잘못 지목

    monkeypatch.setattr(fc, '_build', decoyed)
    with pytest.raises(ReferenceNotDelivered):
        fc.run_one('V13-nopreview', _spec(duration=0.1))


def test_gslqr_requires_an_explicit_grid():
    """기본 격자(0~80)는 기체에 따라 트림 없는 점을 포함해 게인을 오염시킨다."""
    spec = ComparisonSpec(params=P, reference=speed_ramp(0., 14., 1., 6.),
                          duration=0.5, V_table=None)
    with pytest.raises(ValueError, match='V_table'):
        _build('GSLQR', spec, 0.0)


# ── 지표 ──────────────────────────────────────────────────────────────

def test_metrics_include_omega_and_failure_flag():
    """MEMORY.md: 모든 평가에 |ω| 기준 포함 — RMSE 만으론 텀블을 못 잡는다."""
    row = run_one('GSLQR', _spec(duration=1.0))
    assert set(row) >= {'RMSE_z', 'RMSE_vx', 'omega_max', 'failed'}
    assert row['omega_max'] >= 0.0
    assert row['failed'] is False


def test_settle_window_is_excluded_from_metrics():
    """전환 과도응답을 평가에서 빼는 구간이 실제로 동작하는지."""
    spec = _spec(duration=1.0, initial_offset={'z': -5.0})
    full = run_one('GSLQR', spec)
    spec_settled = _spec(duration=1.0, initial_offset={'z': -5.0}, settle=0.5)
    late = run_one('GSLQR', spec_settled)
    # 초기 오차를 빼면 RMSE 가 줄어야 한다
    assert late['RMSE_z'] < full['RMSE_z']
