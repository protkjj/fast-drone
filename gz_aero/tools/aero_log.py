#!/usr/bin/env python3
"""공력 로그·표를 읽고 파생량과 판정을 만든다. **여기 한 곳에서만** 계산한다.

이 모듈이 생긴 이유:
  check_real_vehicle.py 와 plot_real_vehicle.py 가 같은 로그에서 같은 파생량
  (alpha, tilt, |F|, |M|, C_A/C_N)을 각자 따로 계산하고 있었다. 그래서 그림에
  ✅/❌ 판정이 안 실렸다. 둘이 어긋나면 어느 쪽이 맞는지도 알 수 없었다.

쓰는 쪽:
  check_real_vehicle.py / plot_real_vehicle.py / build_aero_dashboard.py
"""
import math
import pathlib

G = 9.80665
DEFAULT_LOG = "/tmp/fast_drone_aero/real_vehicle.csv"


# ══════════════════════════════════════════════════════════════════════
def load_table(path):
    """공력표 CSV -> [(V, alpha_rad, C_A, C_N, x_cp, C_lp, C_mq), ...]"""
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
    ix = {c: i for i, c in enumerate(cols)}
    get = lambda r, c: r[ix[c]] if c in ix else float("nan")   # noqa: E731
    return [(r[ix["V_mps"]], r[ix["alpha_rad"]], r[ix["C_A"]], r[ix["C_N"]],
             get(r, "x_cp"), get(r, "C_lp"), get(r, "C_mq")) for r in rows]


def table_grid(path):
    """표를 격자로 -> (V리스트, alpha리스트, {이름: 2차원 배열})

    대시보드가 플러그인과 **같은 양선형 보간**을 하려면 격자 형태가 필요하다.
    """
    rows = load_table(path)
    Vs = sorted({r[0] for r in rows})
    als = sorted({r[1] for r in rows})
    iv = {v: i for i, v in enumerate(Vs)}
    ia = {a: i for i, a in enumerate(als)}
    names = ("C_A", "C_N", "x_cp", "C_lp", "C_mq")
    grid = {n: [[0.0] * len(als) for _ in Vs] for n in names}
    for r in rows:
        i, j = iv[r[0]], ia[r[1]]
        for k, n in enumerate(names):
            grid[n][i][j] = r[2 + k]
    return Vs, als, grid


def table_meta(path):
    """표 머리말의 S_ref/d_ref/rho_ref 등을 읽는다."""
    meta = {}
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        if not line.startswith("#"):
            break
        if ":" in line:
            k, v = line[1:].split(":", 1)
            meta[k.strip()] = v.strip()
    return meta


def lookup(Vs, als, grid, V, alpha):
    """aero_table.hpp:Lookup() 과 **같은** 양선형 + 가장자리 클램프."""
    def locate(g, x):
        if x <= g[0]:
            return 0, 0, 0.0
        if x >= g[-1]:
            return len(g) - 1, len(g) - 1, 0.0
        lo = 0
        hi = len(g) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if g[mid] <= x:
                lo = mid
            else:
                hi = mid
        span = g[hi] - g[lo]
        return lo, hi, (0.0 if span == 0.0 else (x - g[lo]) / span)

    i0, i1, tv = locate(Vs, V)
    j0, j1, ta = locate(als, alpha)
    w = ((1 - tv) * (1 - ta), (1 - tv) * ta, tv * (1 - ta), tv * ta)
    out = {}
    for n, a in grid.items():
        out[n] = (w[0] * a[i0][j0] + w[1] * a[i0][j1]
                  + w[2] * a[i1][j0] + w[3] * a[i1][j1])
    return out


# ══════════════════════════════════════════════════════════════════════
def load(path):
    """디버그 로그를 읽는다.

    ⚠ 시뮬이 도는 동안 실시간으로 쓰인다. Ctrl-C 로 끄면 마지막 줄이 잘려
      열 개수가 모자란다. 정상 상황이므로 **세어서 버리고** 나머지로 판정한다.
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


class Flight:
    """로그 한 벌에서 뽑은 모든 파생량. 계산은 여기서만 한다."""

    def __init__(self, meta, cols, rows, bad=0):
        self.meta, self.cols, self.rows, self.bad = meta, cols, rows, bad
        ix = {c: i for i, c in enumerate(cols)}
        self.ix = ix
        col = lambda c: [r[ix[c]] for r in rows]        # noqa: E731
        has = lambda c: c in ix                         # noqa: E731

        self.mass = float(meta.get("mass", "nan"))
        self.W = self.mass * G
        self.S = float(meta["S_ref"])
        self.rho = float(meta["rho"])
        self.table_path = meta.get("table", "")

        self.t = col("t")
        self.V = col("V")
        self.alpha = [math.degrees(x) for x in col("alpha")]
        self.q_bar = col("q_bar")
        self.C_A, self.C_N, self.x_cp = col("C_A"), col("C_N"), col("x_cp")

        mag = lambda r, a, b, c: math.sqrt(                       # noqa: E731
            r[ix[a]] ** 2 + r[ix[b]] ** 2 + r[ix[c]] ** 2)
        self.F = [mag(r, "fWx", "fWy", "fWz") for r in rows]
        self.M = [mag(r, "mWx", "mWy", "mWz") for r in rows]
        self.F_body = [mag(r, "Fx", "Fy", "Fz") for r in rows]

        # 기울임 = 링크 +Z 가 월드 +Z 에서 벗어난 각.
        #   쿼터니언 scalar-last (qx,qy,qz,qw), 링크 -> 월드.
        #   R*(0,0,1) 의 z 성분이 1 - 2(qx^2+qy^2).
        self.tilt = [math.degrees(math.acos(max(-1.0, min(1.0,
                     1.0 - 2.0 * (r[ix["qx"]] ** 2 + r[ix["qy"]] ** 2)))))
                     for r in rows]
        self.yaw = [math.degrees(math.atan2(
            2 * (r[ix["qw"]] * r[ix["qz"]] + r[ix["qx"]] * r[ix["qy"]]),
            1 - 2 * (r[ix["qy"]] ** 2 + r[ix["qz"]] ** 2))) for r in rows]

        # 대지속도. **시험 종류를 가르는 값**이다 (아래 kind 참고).
        self.v_ground = ([mag(r, "vLx", "vLy", "vLz") for r in rows]
                         if has("vLx") else [0.0] * len(rows))

        # 바람은 시뮬 도중 토픽으로 바뀔 수 있다. 매 줄 값이 있으면 그걸 쓰고,
        # 없으면(옛 로그) 머리말의 처음 설정으로 떨어진다.
        if has("wWx"):
            self.wind = [mag(r, "wWx", "wWy", "wWz") for r in rows]
            self.wind_per_row = True
        else:
            w0 = meta.get("wind_initial", meta.get("wind", "0 0 0")).split()
            w = math.sqrt(sum(float(x) ** 2 for x in w0)) if len(w0) == 3 else 0.0
            self.wind = [w] * len(rows)
            self.wind_per_row = False

        # 자기일관성: |F_body| 가 q_bar * S * sqrt(C_A^2 + C_N^2) 와 같아야 한다
        self.consistency = max(
            (abs(fb - q * self.S * math.hypot(ca, cn))
             for fb, q, ca, cn in zip(self.F_body, self.q_bar,
                                      self.C_A, self.C_N)),
            default=0.0)

    # ── 시험 종류 ────────────────────────────────────────────────────
    @property
    def kind(self):
        """'speed' | 'hover' | 'ground'

        ★ 판정이 갈리는 지점이다. 기울임은 **정지비행일 때만** 공력의 증거다.
          가속 중에는 기체가 추력을 기울여 가속하므로, 공력이 0 이어도 기운다.
          그때 "기울었으니 밀린 것" 이라고 쓰면 인과가 틀린다.
        """
        if max(self.v_ground, default=0.0) > 5.0:
            return "speed"
        if max(self.tilt, default=0.0) > 1.0 or max(self.V, default=0.0) > 2.0:
            return "hover"
        return "ground"

    def calm(self, tol=0.5):
        """바람이 거의 없는 행만 고른 Flight. 없으면 None.

        측풍 시험 뒤에 속도 시험을 이어 돌리면 한 로그에 두 조건이 섞인다.
        max 로 판정하면 측풍 구간이 속도 구간을 덮어 버린다.
        """
        keep = [r for r, w in zip(self.rows, self.wind) if w <= tol]
        if not keep or len(keep) == len(self.rows):
            return None
        return Flight(self.meta, self.cols, keep, self.bad)


def open_flight(path):
    """경로 -> (Flight, 오류메시지). 실패하면 (None, 메시지)."""
    p = pathlib.Path(path)
    if not p.is_file():
        return None, (f"로그가 없습니다: {p}\n"
                      "  model.sdf 에 <debug_csv> 를 넣고 SITL 을 돌렸는지 확인하세요.")
    meta, cols, rows, bad = load(p)
    if not rows:
        return None, (f"로그에 데이터 줄이 없습니다: {p}\n"
                      "  Configure 는 통과했지만 PreUpdate 가 안 돌았다는 뜻입니다.")
    return Flight(meta, cols, rows, bad), ""


# ══════════════════════════════════════════════════════════════════════
def verdict(f):
    """(통과여부, [줄, ...]) — 시험 종류에 맞는 판정.

    그림과 터미널이 **같은 문장**을 쓰게 하려고 문자열로 돌려준다.
    """
    lines = []
    ok = True
    Fmax = max(f.F, default=0.0)
    frac = 100 * Fmax / f.W if f.W else float("nan")
    need = math.degrees(math.atan2(Fmax, f.W)) if f.W else float("nan")
    tilt = max(f.tilt, default=0.0)
    vg = max(f.v_ground, default=0.0)

    if Fmax < 1e-9:
        lines.append("❌ 힘이 0 입니다. 플러그인은 돌지만 공력이 안 나옵니다.")
        lines.append(f"   V 최대가 {max(f.V, default=0):.2f} m/s 입니다. "
                     "너무 느리면 원래 힘이 거의 없습니다.")
        return False, lines

    if f.kind == "speed":
        # 가속 중에는 기울임이 공력의 증거가 못 된다 — 추력을 기울여 가속한다.
        lines.append(f"ℹ 속도 시험입니다 (대지속도 최대 {vg:.1f} m/s "
                     f"= {vg * 3.6:.0f} km/h).")
        lines.append(f"   공력 최대 {Fmax:.2f} N (무게의 {frac:.1f}%), "
                     f"받음각 {min(f.alpha):.0f}~{max(f.alpha):.0f} deg.")
        lines.append("   ⚠ 이 시험에서는 **기울임을 공력의 증거로 쓸 수 없습니다.**")
        lines.append("     가속하려면 추력을 기울여야 하므로 공력이 0 이어도 기웁니다.")
        lines.append("     여기서 보는 것은 공력이 V^2 를 따라 자라는가 입니다.")
    elif f.kind == "ground":
        lines.append(f"⚠ 기체가 뜨지 않았습니다 (기울임 {tilt:.1f} deg, "
                     f"대지속도 {vg:.2f} m/s).")
        lines.append("   지상에서는 바닥이 힘을 받아 공력이 걸려도 안 움직입니다.")
        ok = False
    elif frac < 1.0:
        lines.append(f"⚠ 힘이 무게의 {frac:.2f}% 입니다. 플러그인은 정상이지만")
        lines.append("   이 속도에선 기체 거동으로 확인이 안 됩니다. "
                     "<wind> 로 상대풍을 넣으세요.")
    elif tilt < 2.0:
        lines.append(f"❌ 공력이 무게의 {frac:.1f}% 인데 기체가 {tilt:.1f} deg 만 "
                     "기울었습니다.")
        lines.append(f"   이 힘이면 {need:.1f} deg 기울어야 버팁니다. "
                     "힘이 계산만 되고 물리에 안 걸리는 것일 수 있습니다.")
        ok = False
    else:
        lines.append(f"✅ 공력 {frac:.1f}% 에 기울임 {tilt:.1f} deg "
                     f"(이 힘이 필요로 하는 각 {need:.1f} deg).")
        lines.append("   PX4 는 바람을 모릅니다. 정지비행에서 기울었다는 건 "
                     "기체가 실제로 밀렸다는 뜻입니다.")

    if f.consistency > 1e-6:
        lines.append(f"❌ 자기일관성이 깨졌습니다 ({f.consistency:.3e} N).")
        ok = False
    return ok, lines
