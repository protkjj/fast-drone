"""야간 작업 — 표 기반(진짜) 물리로 트림 가지 지도 다시 그리기 (PILOT/진단).

2026-09-25 저녁에 단순화 모델로 찾은 '고속가지'(67.99~90 m/s)는 J_max 결함
(전진비가 아무리 커도 추력이 음수가 안 되는 근사)의 인공물이었다. 이 스크립트가
그 정정의 재현 기록이다 — control/trim_hifi.py 를 써서 저속가지·간극·고속가지
전 구간을 표 기반 물리로 다시 확인하고, 실제 요구 전류(research 교차검증)까지
계산한다.

실행: python3 -m control.night_hifi_branch
"""
import numpy as np

from control.trim_hifi import find_trim_hifi, required_motor_current
from control.vehicle_params import load_selected_params

P = load_selected_params()


def bisect_boundary(lo_feasible, hi_infeasible, tol=1e-3):
    """하강형 경계(작은 V=가능, 큰 V=불가능 — 저속가지 상한 전용)."""
    lo, hi = lo_feasible, hi_infeasible
    while hi - lo > tol:
        mid = (lo + hi)/2
        if find_trim_hifi(P, mid)['converged']:
            lo = mid
        else:
            hi = mid
    return hi


def bisect_boundary_ascending(lo_infeasible, hi_feasible, tol=1e-3):
    """상승형 경계(작은 V=불가능, 큰 V=가능 — 고속가지 재개방 전용).

    저속가지와 반대 방향이라 갱신 규칙도 반대다: mid 가 가능하면 그 위가
    아니라 **hi 를 mid 로 낮추고**(더 작은 가능점을 찾는 중), mid 가
    불가능하면 lo 를 올린다. 반대로 짜면 엉뚱한 값에 수렴한다 — 처음에
    저속가지용 함수를 그대로 재사용하려다 이 실수를 했다.
    """
    lo, hi = lo_infeasible, hi_feasible
    while hi - lo > tol:
        mid = (lo + hi)/2
        if find_trim_hifi(P, mid)['converged']:
            hi = mid
        else:
            lo = mid
    return hi


def main():
    print("=" * 70)
    print("① 저속가지 경계 (하이파이) — research 독립보고 19.63 m/s 와 교차검증")
    lo = None
    for V in np.arange(17.0, 20.01, 0.25):
        if find_trim_hifi(P, V)['converged']:
            lo = V
        elif lo is not None:
            break
    boundary_low = bisect_boundary(lo, lo + 0.25, tol=1e-4)
    print(f"  하이파이 저속가지 상한 = {boundary_low:.4f} m/s  (research: 19.63)")

    print("\n② 간극 재확인 (20~80, 5 m/s 간격 — 저녁 조사보다 넓은 구간)")
    gap_hits = [V for V in np.arange(20.0, 80.01, 5.0)
               if find_trim_hifi(P, V)['converged']]
    print(f"  트림 존재하는 속도: {gap_hits}" if gap_hits else "  간극 전부 미존재 확인(닫힘)")

    print("\n③ 고속가지 재개방 경계 정밀화")
    feasible_lo = None
    for V in np.arange(78.0, 85.01, 0.5):
        if find_trim_hifi(P, V)['converged']:
            feasible_lo = V
            break
    boundary_high = None
    if feasible_lo is not None:
        infeasible_hi = feasible_lo - 0.5
        while find_trim_hifi(P, infeasible_hi)['converged']:
            infeasible_hi -= 0.5
        boundary_high = bisect_boundary_ascending(infeasible_hi, feasible_lo, tol=1e-4)
    print(f"  고속가지 재개방 = {boundary_high:.4f} m/s"
         if boundary_high else "  재개방 지점 못 찾음(격자 조정 필요)")

    print("\n④ 고속가지 상한(재포화로 닫히는 지점)")
    top = None
    for V in np.arange(97.0, 130.01, 1.0):
        if not find_trim_hifi(P, V)['converged']:
            top = V
            break
    print(f"  {top} m/s 부근에서 닫힘" if top else "  130까지 열려있음(더 볼 것)")

    print("\n⑤ 고속가지 전체 요구전류 (I_lim=40A 가정, 클립 없는 실제 요구값)")
    print(f"{'V':>6} {'theta':>7} {'n_front':>8} {'n_rear':>8} "
          f"{'I_req_front':>11} {'I_req_rear':>11} {'초과율%':>8}")
    rows = []
    for V in np.arange(82.0, 96.01, 2.0):
        r = find_trim_hifi(P, V)
        if not r['converged']:
            continue
        n = r['state'][13:17]
        cur = required_motor_current(P, n, V)
        over_pct = 100*(cur['I_uncapped'][2]/P['I_lim'] - 1)
        rows.append((V, np.degrees(r['theta']), n[0], n[2],
                    cur['I_uncapped'][0], cur['I_uncapped'][2], over_pct))
        print(f"{V:6.1f} {np.degrees(r['theta']):7.3f} {n[0]:8.1f} {n[2]:8.1f} "
              f"{cur['I_uncapped'][0]:11.3f} {cur['I_uncapped'][2]:11.3f} {over_pct:8.1f}%")

    print("\n⑥ 85 m/s 근접점 research 교차검증")
    r85 = find_trim_hifi(P, 85.0)
    n85 = r85['state'][13:17]
    cur85 = required_motor_current(P, n85, 85.0)
    T_front = np.maximum(np.interp(cur85['J'][0], P['prop_table']['J'], P['prop_table']['CT']), 0)
    print(f"  theta={np.degrees(r85['theta']):.3f}°  n=({n85[0]:.1f},{n85[2]:.1f})")
    print(f"  요구전류 rear={cur85['I_uncapped'][2]:.3f}A  (research 83.3m/s: 45.61A)")
    # 추력비는 n² 이 아니라 실제 추력(ct(J)·rho·rev²·D⁴)으로 — 앞뒤 rev 가 달라
    # J 도 다르고, ct(J) 는 J 에 매우 민감하다(선형이 아니다).
    rev85 = n85/(2*np.pi)
    ct85 = np.maximum(np.interp(cur85['J'], P['prop_table']['J'], P['prop_table']['CT']), 0.0)
    T85 = ct85*P['rho']*rev85**2*P['D_prop']**4
    # 페어 단위(전방 2개 합 vs 후방 2개 합)로 맞춘다 — research 수치가 그 단위.
    front_pair, rear_pair = 2*T85[0], 2*T85[2]
    print(f"  추력(N, 로터쌍 합) front={front_pair:.3f} rear={rear_pair:.3f}  "
          f"비율 {100*front_pair/(front_pair+rear_pair):.1f}% : "
          f"{100*rear_pair/(front_pair+rear_pair):.1f}%"
          f"  (research 83.3m/s: 2.2%:97.8%)")

    return {'boundary_low': boundary_low, 'boundary_high': boundary_high,
            'top': top, 'rows': rows}


if __name__ == '__main__':
    main()
