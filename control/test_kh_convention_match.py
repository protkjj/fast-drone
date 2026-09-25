"""좌표 규약 일치 테스트 — kj 지시 1): "규약이 같다고 했으니 규약 일치
테스트 하나만 붙이면 됩니다."

`external/fastdrone_kh`(팀원 저장소, `kh_control`로 이름만 바꾼 벤더 사본)를
우리 어댑터로 감싸 쓰기 전에, 두 코드베이스의 물리적 규약이 실제로 같은지
숫자로 확인한다. 두 저장소가 각자 자기 안에서는 자기완결적이므로, 여기서
검증하는 건 "섞어 써도 되는 지점"이지 "모든 숫자가 같다"가 아니다.

발견한 것 하나: 호버 쿼터니언의 **월드 프레임 표현**이 롤 180°만큼 다르다
(body+y/+z가 가리키는 월드축이 서로 반대). 이건 문제가 아니다 — 우리
쪽 할당·제어효과도 함수(`compute_allocation_matrix`,
`compute_control_effectiveness`)는 전부 **동체 좌표계 안에서만** 계산하고
(로터 위치도 동체좌표, 산출 모멘트도 동체좌표), 월드축과 동체축의 대응은
그때그때의 쿼터니언이 담당한다. 즉 "로터 배열은 팀원 것, 상태는 팀원
플랜트에서" 를 지키는 한(둘을 섞지 않는 한) 이 롤 오프셋은 물리에 영향이
없다 — 이 파일이 그 전제를 확인해 둔다.
"""
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

_KH_ROOT = Path(__file__).resolve().parent.parent / 'external' / 'fastdrone_kh'
if str(_KH_ROOT) not in sys.path:
    sys.path.insert(0, str(_KH_ROOT))

from kh_control.baseline_v2 import baseline_params as kh_baseline_params  # noqa: E402
from kh_control.dynamics import AxialDronePlant as KHPlant  # noqa: E402
from kh_control.geometry import hover_quaternion as kh_hover_quaternion  # noqa: E402
from kh_control.trim import find_trim as kh_find_trim  # noqa: E402

from control.dynamics import (compute_allocation_matrix,  # noqa: E402
                              _quat_to_rotmat)
from control.hybrid_comparison import compute_control_effectiveness  # noqa: E402
from control.dynamics import AxialDronePlant as OurPlant  # noqa: E402
from control.vehicle_params import load_selected_params  # noqa: E402

KH = kh_baseline_params()
OURS = load_selected_params()

# 우리 쪽 함수는 params['thrust_axis']를 문자열 'x'/'z'로 기대하는데, 팀원
# snapshot은 3벡터([1,0,0])로 담는다 — 스키마 차이, 버그 아님. 우리 함수에
# 넘길 때는 이렇게 정규화한다(실제 어댑터에도 그대로 반영할 부분).
KH_FOR_OUR_FUNCS = dict(KH)
KH_FOR_OUR_FUNCS['thrust_axis'] = 'x'


def test_state_layout_matches():
    """17상태 순서 [p(3),v(3),q(4)xyzw,ω(3),n(4)] — 두 쪽이 같은 인덱스 의미."""
    assert KHPlant(KH).nx == 17 == OurPlant(OURS).nx
    assert KHPlant(KH).nu == 4 == OurPlant(OURS).nu


def test_thrust_axis_is_body_plus_x_mapping_to_world_plus_z_at_hover():
    """thrust_axis 규약의 핵심 물리적 사실 — 둘이 반드시 같아야 하는 부분."""
    R_kh = Rotation.from_quat(kh_hover_quaternion()).as_matrix()
    x0_ours = OurPlant.hover_state(OURS)
    R_ours = Rotation.from_quat(x0_ours[6:10]).as_matrix()
    np.testing.assert_allclose(R_kh[:, 0], [0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(R_ours[:, 0], [0, 0, 1], atol=1e-9)


def test_hover_attitude_differs_by_a_roll_about_thrust_axis_not_something_else():
    """차이가 있다는 것 자체를 기록한다 — 이걸 '고쳐야 할 버그'로 오인해
    로터 배열을 섞어 쓰면 안 된다는 걸 다음 사람에게 남기는 회귀 테스트다.
    """
    R_kh = Rotation.from_quat(kh_hover_quaternion()).as_matrix()
    R_ours = Rotation.from_quat(OurPlant.hover_state(OURS)[6:10]).as_matrix()
    # 두 회전이 body+x(추력축) 둘레 회전만큼 다르면, 그 상대회전의 축은
    # ±body+x 여야 한다(다른 축이면 추력 방향 자체가 달라졌다는 뜻이라 문제).
    relative = R_ours.T @ R_kh
    rotvec = Rotation.from_matrix(relative).as_rotvec()
    angle = np.linalg.norm(rotvec)
    assert angle == pytest.approx(np.pi, abs=1e-6)     # 180도
    axis = rotvec / angle
    assert abs(abs(axis[0]) - 1.0) < 1e-6              # 축이 ±(1,0,0)


def test_rotor_layout_has_the_same_index_and_sign_pattern():
    """숫자(팔 길이)는 다른 기체라 다르지만, (부호쌍) 구조는 같아야 우리의
    범용 할당 함수가 팀원 로터 배열을 넣었을 때도 제대로 작동한다."""
    kh_pos = np.asarray(KH['rotor_positions'])
    our_pos = np.asarray(OURS['rotor_positions'])
    # 같은 (±y,±z) 4분면 패턴: 부호만 비교(스케일 무시).
    np.testing.assert_array_equal(np.sign(kh_pos[:, 1:3]), np.sign(our_pos[:, 1:3]))
    assert list(KH['rotor_directions']) == list(OURS['rotor_directions']) == [1, -1, 1, -1]


def test_our_allocation_matrix_is_well_posed_with_their_rotor_geometry():
    """우리 compute_allocation_matrix가 팀원 로터 배열을 받아도 정상(가역)인지.

    이 함수는 params['rotor_positions']/['rotor_directions']/['thrust_axis']에
    대해 범용이다 — 팀원 값을 넣어도 동체좌표계 안에서만 계산하므로 앞서
    확인한 롤 오프셋과 무관하게 작동해야 한다.
    """
    A, A_inv = compute_allocation_matrix(KH_FOR_OUR_FUNCS)
    assert np.linalg.matrix_rank(A) == 4
    cond = np.linalg.cond(A)
    assert np.isfinite(cond) and cond < 1e6


def test_our_control_effectiveness_is_nonsingular_at_their_trim():
    """팀원 플랜트에서 얻은 실제 트림점에서, 우리 INDI의 G행렬(제어효과도)이
    팀원 로터 배열로 계산해도 특이하지 않은지 — 어댑터가 실제로 쓸 조합."""
    trim = kh_find_trim(KH, 70.0)
    n = trim['state'][13:17]
    q = trim['state'][6:10]
    v_world = trim['state'][3:6]
    v_body = Rotation.from_quat(q).as_matrix().T @ v_world
    G = compute_control_effectiveness(KH_FOR_OUR_FUNCS, n, v_body)
    assert np.linalg.matrix_rank(G) == 4
    assert np.isfinite(np.linalg.cond(G))


def test_kh_plant_hover_state_round_trips_through_step():
    """어댑터가 실제로 할 일(팀원 플랜트를 그대로 호출)의 최소 스모크 시험."""
    plant = KHPlant(KH, dt=0.001)
    x0 = KHPlant.hover_state(KH)
    u0 = x0[13:17].copy()
    x1 = plant.step(x0, u0)
    assert np.all(np.isfinite(x1))
    np.testing.assert_allclose(np.linalg.norm(x1[6:10]), 1.0, atol=1e-9)
