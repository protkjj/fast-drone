"""NMPC 계열(V13·M17·F13)이 **똑같이** 써야 하는 설정을 한 곳에 둔다.

kj 절대 규칙 1: "NMPC 계열의 수정은 전부 동일하게 적용한다." 세 클래스가
각자 IPOPT 옵션과 비용 가중치를 리터럴로 들고 있으면, 한 곳만 바뀌어도
알아채기 어렵다(실제로 M17만 max_iter가 인자가 아니었다). 그래서 옵션과
가중치를 여기서 만들고, 세 클래스가 이것을 받아 쓰며 그대로 인스턴스에
보관한다 — 경기장 불변식 I-3이 그 보관값을 직접 비교한다.
"""

# 논문 v5.3 식(14)·(16)·(17)의 가중치. 속도 5, 고도 Q_z(생성자 인자, 기본 20),
# 각속도 1, 입력 편차 0.02, 입력 변화 0.10, 종말 ×10.
PAPER_COST_WEIGHTS = dict(w_v=5.0, w_omega=1.0, r_dev=0.02, r_rate=0.10, terminal=10.0)


def cost_weights(overrides=None):
    """논문 가중치에 덮어쓸 값만 바꾼 사본. 모르는 키는 오타일 수 있어 막는다."""
    weights = dict(PAPER_COST_WEIGHTS)
    for key, value in (overrides or {}).items():
        if key not in weights:
            raise ValueError(f'unknown NMPC cost weight {key!r}; allowed {sorted(weights)}')
        weights[key] = float(value)
    return weights


def ipopt_options(max_iter=30, tol=1e-4):
    """세 NMPC가 nlpsol에 넘기는 IPOPT 옵션. 시간 제한은 두지 않는다 —
    반복 상한만 쓴다(kj: 결과가 벽시계·병렬 부하에 의존하면 안 된다)."""
    return {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
            'ipopt.max_iter': int(max_iter), 'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': float(tol)}


def solver_settings(nmpc):
    """I-3 비교용 요약 — 인스턴스에 실제로 보관된 값만 읽는다."""
    return dict(N=nmpc.N, dt_pred_s=nmpc.dt_nmpc, dt_ctrl_s=nmpc.dt_ctrl,
                ipopt_options=dict(nmpc.ipopt_options), cost_spec=nmpc.cost_spec,
                cost_weights=dict(nmpc.cost_weights), Q_z=float(nmpc._Q_z),
                preview=nmpc.ref_fn is not None,
                soft_constraints=dict(getattr(nmpc, 'soft_constraints', {})))
