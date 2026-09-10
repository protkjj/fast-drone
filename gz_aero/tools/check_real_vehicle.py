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
    """디버그 로그를 읽는다.

    ⚠ 이 로그는 시뮬이 도는 동안 실시간으로 쓰인다. SITL 을 Ctrl-C 로 끄면
      마지막 줄이 쓰다 말고 잘린다. 그러면 열 개수가 모자라 파싱이 터진다.
      정상 상황이므로 **망가진 줄은 세어서 버리고** 나머지로 판정한다.
    """
    meta, cols, rows, bad = {}, None, [], 0
    with open(path, encoding="utf-8", errors="replace") as f:
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
            if not p or p[0] == "":
                continue
            if len(p) != len(cols):
                bad += 1
                continue
            try:
                rows.append([float(x) for x in p])
            except ValueError:
                bad += 1
    return meta, cols, rows, bad


def main():
    path = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
    if not path.is_file():
        print(f"로그가 없습니다: {path}")
        print("  model.sdf 에 <debug_csv> 를 넣고 SITL 을 돌렸는지 확인하세요.")
        return 1

    meta, cols, rows, bad = load(path)
    if bad:
        print(f"⚠ 열 개수가 안 맞는 줄 {bad} 개를 건너뜁니다 "
              "(시뮬을 끄면 마지막 줄이 잘립니다. 1~2 개면 정상입니다)")
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
    print(f"          열 {len(cols)} 개")
    print(f"          {len(rows)} 행, t {t[0]:.2f} ~ {t[-1]:.2f} s")
    print(f"질량      {mass:.4g} kg (무게 {W:.1f} N)")
    print(f"속도      V {min(V):.2f} ~ {max(V):.2f} m/s")
    print(f"받음각    {math.degrees(min(al)):.1f} ~ {math.degrees(max(al)):.1f} deg")
    # 기울임 = 링크의 +Z 축이 월드 +Z 에서 얼마나 벗어났나.
    #   쿼터니언은 scalar-last (qx,qy,qz,qw), 링크 -> 월드.
    #   R*(0,0,1) 의 z 성분이 1 - 2(qx^2 + qy^2) 이므로 acos 하면 된다.
    #
    # ★ 이게 결정적 증거다. PX4 는 바람을 모른다. 위치 오차만 본다.
    #   공력이 **실제로 걸리면** 기체가 밀리고 PX4 가 버티려고 기운다.
    #   힘을 계산만 하고 안 걸면 밀리지 않으니 기울지도 않는다.
    tilt = [math.degrees(math.acos(max(-1.0, min(1.0,
            1.0 - 2.0 * (r[ix["qx"]] ** 2 + r[ix["qy"]] ** 2)))))
            for r in rows]

    print(f"바람      {meta.get('wind', '(없음)')} m/s (월드 ENU)")
    print(f"공력      |F| 최대 {max(fw):.3f} N (무게의 {100 * max(fw) / W:.2f}%)")
    print(f"          |M| 최대 {max(mw):.4f} N·m")
    print(f"기울임    최대 {max(tilt):.1f} deg  (수평에서 벗어난 각)")

    # 자기일관성: 동체력 크기가 q_bar * S * sqrt(C_A^2 + C_N^2) 와 맞아야 한다.
    worst = 0.0
    for r in rows:
        f_b = math.dist((0, 0, 0), (r[ix["Fx"]], r[ix["Fy"]], r[ix["Fz"]]))
        expect = r[ix["q_bar"]] * S * math.hypot(r[ix["C_A"]], r[ix["C_N"]])
        worst = max(worst, abs(f_b - expect))
    print(f"자기일관성 |F_body| vs q_bar*S*sqrt(C_A^2+C_N^2) 최대차 {worst:.3e} N")

    print()
    ok = True
    frac = 100 * max(fw) / W
    need = math.degrees(math.atan2(max(fw), W))

    if max(fw) < 1e-9:
        print("❌ 힘이 0 입니다. 플러그인은 돌지만 공력이 안 나옵니다.")
        print(f"   V 최대가 {max(V):.2f} m/s 입니다. 너무 느리면 원래 힘이 거의 없습니다.")
        ok = False
    elif frac < 1.0:
        print(f"⚠ 힘이 무게의 {frac:.2f}% 입니다. 플러그인은 정상이지만 이 속도에선")
        print("   기체 거동으로 확인이 안 됩니다. <wind> 로 상대풍을 넣으세요.")
    elif max(tilt) < 2.0:
        print(f"❌ 공력이 무게의 {frac:.1f}% 인데 기체가 {max(tilt):.1f} deg 밖에 안 기울었습니다.")
        print(f"   이 힘이면 {need:.1f} deg 기울어야 버팁니다. 둘 중 하나입니다:")
        print("     · 아직 지상에 있다 (착륙 상태면 바닥이 힘을 받는다)")
        print("     · 힘이 계산만 되고 물리에 안 걸린다")
        print("   commander takeoff 로 띄운 뒤 다시 재세요.")
        ok = False
    else:
        print(f"✅ 공력 {frac:.1f}% 에 기울임 {max(tilt):.1f} deg "
              f"(이 힘이 필요로 하는 각 {need:.1f} deg).")
        print("   PX4 는 바람을 모릅니다. 기울었다는 건 기체가 실제로 밀렸다는 뜻입니다.")

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
