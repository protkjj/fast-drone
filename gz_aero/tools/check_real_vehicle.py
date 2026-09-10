#!/usr/bin/env python3
"""실기체에 붙인 공력이 **실제로 힘을 걸고 있는지** 판정한다.

겹1·겹2 는 "수식과 좌표변환이 맞나" 를 봤다. 이건 다른 질문이다:
PX4 SITL 로 띄운 진짜 기체에서 플러그인이 살아 돌면서 의미 있는 크기의 힘을
내고 있는가.

    python3 gz_aero/tools/check_real_vehicle.py [로그.csv]
"""
import math
import pathlib
import sys

DEFAULT = "/tmp/fast_drone_aero/real_vehicle.csv"


def load_table(path):
    """공력표를 읽어 (V, alpha) -> (C_A, C_N) 격자를 돌려준다.

    예상 힘을 손으로 적어두면 표가 바뀔 때 조용히 틀린다. 로그 머리말에 적힌
    표를 그대로 읽어서 계산한다.
    """
    cols, rows = None, []
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        if line.startswith("#"):
            continue
        q = [x.strip() for x in line.split(",")]
        if cols is None:
            cols = q
            continue
        if q and q[0] != "":
            rows.append([float(x) for x in q])
    iV, ia = cols.index("V_mps"), cols.index("alpha_rad")
    iCA, iCN = cols.index("C_A"), cols.index("C_N")
    return [(r[iV], r[ia], r[iCA], r[iCN]) for r in rows]


def load(path):
    meta, cols, rows = {}, None, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                if ":" in line:
                    k, v = line[1:].split(":", 1)
                    meta[k.strip()] = v.strip()
                continue
            p = [x.strip() for x in line.rstrip("\n").split(",")]
            if cols is None:
                cols = p
                continue
            if p and p[0] != "":
                rows.append([float(x) for x in p])
    return meta, cols, rows


def main():
    path = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
    if not path.is_file():
        print(f"로그가 없습니다: {path}")
        print("  model.sdf 에 <debug_csv> 를 넣고 SITL 을 돌렸는지 확인하세요.")
        return 1

    meta, cols, rows = load(path)
    if not rows:
        print(f"로그에 데이터 줄이 없습니다: {path}")
        print("  플러그인이 Configure 는 통과했지만 PreUpdate 가 안 돌았다는 뜻입니다.")
        return 1

    ix = {c: i for i, c in enumerate(cols)}
    mass = float(meta.get("mass", "nan"))
    S, rho = float(meta["S_ref"]), float(meta["rho"])
    W = mass * 9.80665

    def col(c):
        return [r[ix[c]] for r in rows]

    t = col("t")
    V, al, qb = col("V"), col("alpha"), col("q_bar")
    fw = [math.dist((0, 0, 0), (r[ix["fWx"]], r[ix["fWy"]], r[ix["fWz"]])) for r in rows]
    mw = [math.dist((0, 0, 0), (r[ix["mWx"]], r[ix["mWy"]], r[ix["mWz"]])) for r in rows]

    print(f"로그      {path}")
    print(f"          {len(rows)} 행, t {t[0]:.2f} ~ {t[-1]:.2f} s")
    print(f"질량      {mass:.4g} kg (무게 {W:.1f} N)")
    print(f"속도      V {min(V):.2f} ~ {max(V):.2f} m/s")
    print(f"받음각    {math.degrees(min(al)):.1f} ~ {math.degrees(max(al)):.1f} deg")
    print(f"공력      |F| 최대 {max(fw):.3f} N (무게의 {100 * max(fw) / W:.2f}%)")
    print(f"          |M| 최대 {max(mw):.4f} N·m")

    # 자기일관성: 동체력 크기가 q_bar * S * sqrt(C_A^2 + C_N^2) 와 맞아야 한다.
    worst = 0.0
    for r in rows:
        f_b = math.dist((0, 0, 0), (r[ix["Fx"]], r[ix["Fy"]], r[ix["Fz"]]))
        expect = r[ix["q_bar"]] * S * math.hypot(r[ix["C_A"]], r[ix["C_N"]])
        worst = max(worst, abs(f_b - expect))
    print(f"자기일관성 |F_body| vs q_bar*S*sqrt(C_A^2+C_N^2) 최대차 {worst:.3e} N")

    print()
    ok = True
    if max(fw) < 1e-9:
        print("❌ 힘이 0 입니다. 플러그인은 돌지만 공력이 안 나옵니다.")
        print(f"   V 최대가 {max(V):.2f} m/s 입니다. 너무 느리면 원래 힘이 거의 없습니다.")
        ok = False
    elif 100 * max(fw) / W < 1.0:
        print("⚠ 힘이 무게의 1% 미만입니다. 돌고는 있지만 이 속도에선 "
              "기체 거동으로 확인이 안 됩니다.")
        print("   <wind> 로 상대풍을 넣어 크게 만드세요 (아래 표 참고).")
    else:
        print(f"✅ 공력이 무게의 {100 * max(fw) / W:.1f}% 까지 걸리고 있습니다. "
              "기체 거동으로 확인 가능한 크기입니다.")
    if worst > 1e-6:
        print(f"❌ 자기일관성이 깨졌습니다 ({worst:.3e} N).")
        ok = False

    tbl_path = meta.get("table", "")
    if tbl_path and pathlib.Path(tbl_path).is_file():
        tbl = load_table(tbl_path)
        print(f"\n  이 표({pathlib.Path(tbl_path).name}) 기준 예상 — "
              f"기수와 직각 상대풍(alpha=90도):")
        for v in (10, 20, 40, 60):
            near = min(tbl, key=lambda x: (abs(x[0] - v), abs(x[1] - math.pi / 2)))
            # 정지비행이면 축력·법선력이 **둘 다 수평**이라 합성 크기로 기울어진다
            c = math.hypot(near[2], near[3])
            q = 0.5 * rho * v * v
            f = q * S * c
            print(f"    V={v:3d} m/s -> |C|={c:5.3f}  {f:7.2f} N ({100 * f / W:5.1f}%), "
                  f"기울임 {math.degrees(math.atan2(f, W)):5.1f} deg")
    else:
        print(f"\n  표를 못 읽어 예상값을 생략합니다: {tbl_path or '(머리말에 없음)'}")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
