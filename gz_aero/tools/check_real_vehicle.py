#!/usr/bin/env python3
"""실기체에 붙인 공력이 **실제로 힘을 걸고 있는지** 판정한다.

겹1·겹2 는 "수식과 좌표변환이 맞나" 를 봤다. 이건 다른 질문이다:
PX4 SITL 로 띄운 진짜 기체에서 플러그인이 살아 돌면서 의미 있는 크기의 힘을
내고 있는가.

    python3 gz_aero/tools/check_real_vehicle.py [로그.csv]

파싱·파생량·판정은 aero_log.py 에 있다. 그림 도구와 **같은 값, 같은 문장**을
쓰기 위해서다.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import aero_log as A   # noqa: E402


def report(f, title=""):
    if title:
        print(f"\n── {title} ──")
    print(f"          {len(f.rows)} 행, t {f.t[0]:.2f} ~ {f.t[-1]:.2f} s")
    print(f"속도      V {min(f.V):.2f} ~ {max(f.V):.2f} m/s "
          f"(최고 {max(f.V) * 3.6:.0f} km/h)")
    print(f"대지속도  {max(f.v_ground):.2f} m/s")
    print(f"받음각    {min(f.alpha):.1f} ~ {max(f.alpha):.1f} deg")
    print(f"바람      {min(f.wind):.1f} ~ {max(f.wind):.1f} m/s"
          f"{'  (매 줄 기록)' if f.wind_per_row else '  (머리말 값만 — 도중 변경 감지 불가)'}")
    print(f"공력      |F| 최대 {max(f.F):.3f} N (무게의 {100 * max(f.F) / f.W:.2f}%)")
    print(f"          |M| 최대 {max(f.M):.4f} N·m")
    print(f"기울임    최대 {max(f.tilt):.1f} deg")
    print(f"자기일관성 |F_body| vs q_bar*S*sqrt(C_A^2+C_N^2) "
          f"최대차 {f.consistency:.3e} N")

    # 최대 하중이 **어느 상태에서** 났는지가 중요하다. 최고속도일 거라고
    # 넘겨짚기 쉬운데, 감속하며 기체를 세울 때 받음각이 커져 더 크게 나온다.
    i = max(range(len(f.F)), key=lambda k: f.F[k])
    print(f"최대 하중  t={f.t[i]:.1f} s 에서 V={f.V[i]:.1f} m/s, "
          f"alpha={f.alpha[i]:.0f} deg, 기울임={f.tilt[i]:.0f} deg")

    print()
    ok, lines = A.verdict(f)
    for line in lines:
        print(line)
    return ok


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else A.DEFAULT_LOG
    f, err = A.open_flight(path)
    if f is None:
        print(err)
        return 1
    if f.bad:
        print(f"⚠ 열 개수가 안 맞는 줄 {f.bad} 개를 건너뜁니다 "
              "(시뮬을 끄면 마지막 줄이 잘립니다. 1~2 개면 정상입니다)")

    print(f"로그      {path}")
    print(f"          열 {len(f.cols)} 개")
    print(f"질량      {f.mass:.4g} kg (무게 {f.W:.1f} N)")
    ok = report(f)

    # 바람 구간과 무풍 구간이 한 로그에 섞여 있으면 따로도 낸다.
    # 안 그러면 측풍 구간의 최대값이 속도 구간을 덮는다.
    calm = f.calm()
    if calm is not None:
        ok = report(calm, "무풍 구간만 (바람 <= 0.5 m/s)") and ok

    tbl = f.table_path
    if tbl and pathlib.Path(tbl).is_file():
        Vs, als, grid = A.table_grid(tbl)
        print(f"\n  이 표({pathlib.Path(tbl).name}) 기준 — 비행이 실제로 지난 "
              "받음각에서의 예상 힘:")
        for a_deg in (0, 15, 30, 45, 60, 90):
            c = A.lookup(Vs, als, grid, max(f.V), math.radians(a_deg))
            mag = math.hypot(c["C_A"], c["C_N"])
            q = 0.5 * f.rho * max(f.V) ** 2
            fN = q * f.S * mag
            print(f"    V={max(f.V):.0f} m/s, alpha={a_deg:3d} deg -> "
                  f"|C|={mag:5.3f}  {fN:7.1f} N ({100 * fN / f.W:5.0f}% of W)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
