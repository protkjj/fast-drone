"""
EKF 통합 시뮬레이션 — 센서 노이즈 하에서 제어기 검증
=====================================================

시뮬레이션 루프:
  Plant(참값) → Sensors(노이즈) → ESKF(추정) → x̂ → Controller → u → Plant

비교 대상:
  1. 참값 제어 (기존): controller(t, x_true)  → 성능 상한
  2. 추정값 제어 (새로): controller(t, x_hat) → 현실적 성능

검증 항목:
  - 추정 오차 (x_true - x_hat) 시계열
  - 제어 성능 저하율 (RMSE 비교)
  - 노이즈 레벨 스윕 (0.5x ~ 2.0x)
  - 제어기별 노이즈 민감도 순위
"""

import numpy as np
from dynamics import AxialDronePlant, NX
from vehicle_params import vehicle_params as P
from sensors import SensorSuite, create_default_sensors
from estimator import ESKF


def simulate_with_ekf(plant, controller, x0, T,
                      sensors=None, noise_level=1.0, seed=42,
                      wind_fn=None):
    """
    센서+EKF 포함 시뮬레이션.

    Parameters
    ----------
    plant : AxialDronePlant
    controller : callable(t, x) → u[4]
    x0 : array(17)
        초기 상태.
    T : float
        시뮬레이션 시간 [s].
    sensors : SensorSuite or None
        None이면 noise_level로 자동 생성.
    noise_level : float
        1.0 = 표준 MEMS. sensors가 None일 때 사용.
    seed : int
        재현성 시드.
    wind_fn : callable(t) → w[3] or None
        바람 함수.

    Returns
    -------
    result : dict
        ts, xs_true, xs_est, us, est_errors
    """
    dt = plant.dt
    N = int(round(T / dt))
    ts = np.linspace(0, T, N + 1)

    # 센서 생성
    if sensors is None:
        sensors = create_default_sensors(dt, noise_level, seed)
    sensors.reset()

    # ESKF 초기화 (초기 상태에 약간의 불확실성 부여)
    rng = np.random.default_rng(seed + 100)
    x0_est = x0.copy()
    x0_est[0:3] += rng.normal(0, 0.5, 3)  # 초기 위치 오차 ~0.5m
    x0_est[3:6] += rng.normal(0, 0.2, 3)  # 초기 속도 오차 ~0.2m/s

    eskf = ESKF(x0_est, g=P['g'])

    # 로깅 배열
    xs_true = np.zeros((N + 1, NX))
    xs_est = np.zeros((N + 1, NX))
    us = np.zeros((N, 4))
    xs_true[0] = x0
    xs_est[0] = eskf.get_state()
    # 로터 속도는 추정기가 모르니 초기값 복사
    xs_est[0, 13:17] = x0[13:17]

    # 제어 출력 추적 (ESKF에 로터 속도 공급용)
    motor_cmd = x0[13:17].copy()

    for k in range(N):
        t = ts[k]
        x_true = xs_true[k]

        # ── 1. 관성 가속도 계산 (센서 모델에 필요) ──
        w = wind_fn(t) if wind_fn is not None else None
        xdot = plant.evaluate_xdot(x_true, motor_cmd, w)
        acc_inertial = xdot[3:6]

        # ── 2. 센서 측정 ──
        sensor_data = sensors.measure(t, x_true, acc_inertial)

        # ── 3. ESKF 예측 (IMU rate = 매 스텝) ──
        eskf.predict(sensor_data.acc_body, sensor_data.gyro_body, dt)

        # ── 4. ESKF 측정 업데이트 ──
        # 가속도계 자세 업데이트: GPS와 같은 주기 (10 Hz)
        # 매 스텝은 너무 빈번 → 기동 중 오보정 위험
        if sensor_data.gps_valid:
            eskf.update_accel(sensor_data.acc_body)
            eskf.update_gps(sensor_data.gps_pos, sensor_data.gps_vel)

        # ── 5. 추정 상태 추출 ──
        x_hat = eskf.get_state()
        # 로터 속도는 우리가 보낸 명령으로 추정 (1차 지연)
        tau_m = P['tau_m']
        alpha = dt / (dt + tau_m)
        motor_est = alpha * motor_cmd + (1 - alpha) * xs_est[k, 13:17]
        x_hat[13:17] = motor_est

        # ── 6. 제어기 호출 (추정값 사용!) ──
        u = controller(t, x_hat)
        u = np.clip(u, P['n_min'], P['n_max'])
        motor_cmd = u

        # ── 7. 플랜트 전파 (참값) ──
        xs_true[k + 1] = plant.step(x_true, u, w)

        # ── 로깅 ──
        us[k] = u
        xs_est[k + 1] = x_hat  # 다음 스텝 시작 시 추정값

    # 추정 오차 계산
    est_errors = xs_true - xs_est

    return {
        'ts': ts,
        'xs_true': xs_true,
        'xs_est': xs_est,
        'us': us,
        'est_errors': est_errors,
    }


def simulate_perfect(plant, controller, x0, T, wind_fn=None):
    """
    참값 제어 시뮬레이션 (기존 방식, 비교 기준).

    controller(t, x_true) → u
    """
    ts, xs, us = plant.simulate(x0, controller, T, wind_fn)
    return {'ts': ts, 'xs_true': xs, 'us': us}


def compute_metrics(result, z_ref, v_ref_x):
    """
    성능 지표 계산.

    Returns
    -------
    metrics : dict
        rmse_z, rmse_vx, max_z_err, max_vx_err
    """
    xs = result['xs_true']
    z = xs[:, 2]
    vx = xs[:, 3]

    return {
        'rmse_z': np.sqrt(np.mean((z - z_ref)**2)),
        'rmse_vx': np.sqrt(np.mean((vx - v_ref_x)**2)),
        'max_z_err': np.max(np.abs(z - z_ref)),
        'max_vx_err': np.max(np.abs(vx - v_ref_x)),
    }


def compute_estimation_metrics(result):
    """
    추정 오차 지표.

    Returns
    -------
    dict : pos_rmse, vel_rmse, att_rmse (각도 [deg])
    """
    err = result['est_errors']

    pos_err = np.sqrt(np.mean(np.sum(err[:, 0:3]**2, axis=1)))
    vel_err = np.sqrt(np.mean(np.sum(err[:, 3:6]**2, axis=1)))

    # 자세 오차: 쿼터니언 차이 → 각도
    att_errs = []
    for k in range(len(err)):
        q_true = result['xs_true'][k, 6:10]
        q_est = result['xs_est'][k, 6:10]
        # 오차 각도 = 2 * arccos(|q_true · q_est|)
        dot = np.abs(np.dot(q_true, q_est))
        dot = min(dot, 1.0)
        att_errs.append(2 * np.arccos(dot))
    att_rmse_deg = np.degrees(np.sqrt(np.mean(np.array(att_errs)**2)))

    return {
        'pos_rmse': pos_err,
        'vel_rmse': vel_err,
        'att_rmse_deg': att_rmse_deg,
    }


# ════════════════════════════════════════════════════
# 검증 스크립트
# ════════════════════════════════════════════════════

def test_eskf_standalone():
    """
    ESKF 단독 테스트: 호버 상태에서 추정이 수렴하는지 확인.

    참값 = 일정한 호버, 센서 = 노이즈 포함
    → ESKF 추정이 참값에 수렴해야 함.
    """
    print("=" * 60)
    print("  ESKF 단독 테스트: 호버 수렴")
    print("=" * 60)

    plant = AxialDronePlant(P, dt=0.001)
    x0 = plant.hover_state(P)
    x0[2] = 10.0  # 고도 10m

    # 참값 제어: 호버 유지 (제어기 없이 트림 입력)
    n_hov = np.sqrt(P['mass'] * P['g'] / (4 * P['k_T']))
    u_hover = np.full(4, n_hov)
    controller = lambda t, x: u_hover

    T_sim = 5.0
    result = simulate_with_ekf(plant, controller, x0, T_sim,
                                noise_level=1.0, seed=42)

    est_met = compute_estimation_metrics(result)

    print(f"\n  시뮬 시간: {T_sim}초")
    print(f"  위치 RMSE: {est_met['pos_rmse']:.4f} m")
    print(f"  속도 RMSE: {est_met['vel_rmse']:.4f} m/s")
    print(f"  자세 RMSE: {est_met['att_rmse_deg']:.4f} deg")

    # 마지막 1초의 오차 (수렴 후)
    N = len(result['ts'])
    last_sec = int(1.0 / 0.001)
    err_last = result['est_errors'][-last_sec:]

    pos_err_last = np.sqrt(np.mean(np.sum(err_last[:, 0:3]**2, axis=1)))
    vel_err_last = np.sqrt(np.mean(np.sum(err_last[:, 3:6]**2, axis=1)))

    print(f"\n  마지막 1초:")
    print(f"    위치 RMSE: {pos_err_last:.4f} m")
    print(f"    속도 RMSE: {vel_err_last:.4f} m/s")

    ok = pos_err_last < 2.0 and vel_err_last < 1.0
    print(f"\n  결과: {'[OK] 수렴 확인' if ok else '[FAIL] 수렴 실패!'}")
    return ok


def test_eskf_with_pid():
    """
    PID + ESKF: 호버 제어를 추정값으로 했을 때 안정적인지.
    """
    print("\n" + "=" * 60)
    print("  PID + ESKF 테스트: 호버 교란 복원")
    print("=" * 60)

    from controller import CascadedPID
    from trim import find_trim

    plant = AxialDronePlant(P, dt=0.001)
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    x0[2] = 10.0
    x0[3] = 2.0   # 수평 교란
    x0[5] = 1.0   # 수직 교란

    pid = CascadedPID(P, v_ref=[0, 0, 0], z_ref=10.0, dt=0.001)

    T_sim = 5.0

    # 참값 제어
    pid.reset()
    res_perf = simulate_perfect(plant, pid, x0.copy(), T_sim)

    # EKF 제어
    pid.reset()
    res_ekf = simulate_with_ekf(plant, pid, x0.copy(), T_sim,
                                 noise_level=1.0, seed=42)

    met_perf = compute_metrics(res_perf, z_ref=10.0, v_ref_x=0.0)
    met_ekf = compute_metrics(res_ekf, z_ref=10.0, v_ref_x=0.0)
    est_met = compute_estimation_metrics(res_ekf)

    print(f"\n  {'지표':>20s}  {'참값제어':>10s}  {'EKF제어':>10s}  {'저하율':>8s}")
    print(f"  {'─'*52}")
    for key in ['rmse_z', 'rmse_vx']:
        v_p = met_perf[key]
        v_e = met_ekf[key]
        deg = ((v_e - v_p) / max(v_p, 1e-6)) * 100
        print(f"  {key:>20s}  {v_p:>10.4f}  {v_e:>10.4f}  {deg:>+7.1f}%")

    print(f"\n  추정 오차:")
    print(f"    위치 RMSE: {est_met['pos_rmse']:.4f} m")
    print(f"    속도 RMSE: {est_met['vel_rmse']:.4f} m/s")
    print(f"    자세 RMSE: {est_met['att_rmse_deg']:.4f} deg")

    # 5초 후 상태
    print(f"\n  5초 후:")
    print(f"    z (참값): {res_perf['xs_true'][-1, 2]:.3f} m "
          f"→ (EKF): {res_ekf['xs_true'][-1, 2]:.3f} m")
    print(f"    |v| (참값): {np.linalg.norm(res_perf['xs_true'][-1, 3:6]):.4f} "
          f"→ (EKF): {np.linalg.norm(res_ekf['xs_true'][-1, 3:6]):.4f} m/s")


def compare_controllers_hover():
    """
    호버 교란: PID vs LQR — 참값 vs EKF 비교.

    "노이즈가 들어오면 제어기 순위가 바뀌나?"의 첫 답.
    """
    print("\n" + "=" * 60)
    print("  제어기 비교: 호버 교란 (참값 vs EKF)")
    print("=" * 60)

    from controller import CascadedPID, LQRController
    from trim import find_trim

    plant = AxialDronePlant(P, dt=0.001)
    trim = find_trim(P, 0.0)
    x0 = trim['state'].copy()
    x0[2] = 10.0
    x0[3] = 2.0
    x0[5] = 1.0

    T_sim = 5.0
    z_ref, v_ref_x = 10.0, 0.0

    controllers = {}

    # PID
    pid = CascadedPID(P, v_ref=[0, 0, 0], z_ref=z_ref, dt=0.001)
    controllers['PID'] = pid

    # LQR
    lqr = LQRController(P, trim['state'], trim['control'])
    lqr.set_position_ref(x0[0:3])
    controllers['LQR'] = lqr

    print(f"\n  {'제어기':>8s}  {'조건':>8s}  {'RMSE z':>8s}  {'RMSE vx':>8s}  "
          f"{'최종 z':>8s}  {'최종 |v|':>8s}")
    print(f"  {'─'*60}")

    for name, ctrl in controllers.items():
        # 참값 제어
        if hasattr(ctrl, 'reset'):
            ctrl.reset()
        res_p = simulate_perfect(plant, ctrl, x0.copy(), T_sim)
        mp = compute_metrics(res_p, z_ref, v_ref_x)

        # EKF 제어
        if hasattr(ctrl, 'reset'):
            ctrl.reset()
        res_e = simulate_with_ekf(plant, ctrl, x0.copy(), T_sim,
                                   noise_level=1.0, seed=42)
        me = compute_metrics(res_e, z_ref, v_ref_x)

        z_final_p = res_p['xs_true'][-1, 2]
        v_final_p = np.linalg.norm(res_p['xs_true'][-1, 3:6])
        z_final_e = res_e['xs_true'][-1, 2]
        v_final_e = np.linalg.norm(res_e['xs_true'][-1, 3:6])

        print(f"  {name:>8s}  {'참값':>8s}  {mp['rmse_z']:>8.4f}  "
              f"{mp['rmse_vx']:>8.4f}  {z_final_p:>8.3f}  {v_final_p:>8.4f}")
        print(f"  {name:>8s}  {'EKF':>8s}  {me['rmse_z']:>8.4f}  "
              f"{me['rmse_vx']:>8.4f}  {z_final_e:>8.3f}  {v_final_e:>8.4f}")


if __name__ == '__main__':
    ok = test_eskf_standalone()
    if ok:
        test_eskf_with_pid()
        compare_controllers_hover()
