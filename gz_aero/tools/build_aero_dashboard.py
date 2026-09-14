#!/usr/bin/env python3
"""공력 대시보드(자립형 HTML)를 만든다. 우분투 없는 사람도 브라우저로 본다.

    python3 gz_aero/tools/build_aero_dashboard.py
    python3 gz_aero/tools/build_aero_dashboard.py --log /tmp/fast_drone_aero/real_vehicle.csv

왜 손으로 쓴 HTML 이 아니라 생성기인가:
  공력 담당자가 CSV 를 고치면 **다시 돌려서 확인**하는 것이 이 도구의 쓰임이다.
  손으로 쓴 페이지는 표가 바뀌면 조용히 거짓말을 한다.

★ 자기검사
  페이지에 심은 JS 보간이 플러그인(aero_table.hpp:Lookup)과 같은 값을 내야 한다.
  아니면 "대시보드에선 이랬는데" 가 되어 신뢰를 잃는다. 그래서 빌드할 때
  파이썬 기준 구현으로 40 점을 계산해 같이 심고, 페이지가 로드되면 JS 로 다시
  계산해 대조한 뒤 결과를 배지로 띄운다.
"""
import argparse
import json
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import aero_log as A   # noqa: E402

HERE = pathlib.Path(__file__).resolve().parents[1]
SOURCES = [
    ("placeholder", "control/vehicle_params.py 임시값", 8.0),
    ("sized", "팀 사이징 rocket-drone/modules/aero.py", 1.6607780870266748),
]


def sig(x, n=7):
    """표시용 반올림. **표 데이터에는 쓰지 않는다.**

    처음엔 용량을 줄이려고 표 값도 7 자리로 접었다가 자기검사가 2.2e-6 으로
    실패했다. 기준값은 전체 정밀도로 계산했는데 페이지는 접힌 값으로 보간하니
    당연히 어긋난다. 표는 원본 그대로 심는다 — 플러그인이 읽는 값과 같아야
    한다는 것이 이 페이지의 전제다. 200 KB 더 쓰고 신뢰를 산다.
    """
    if x == 0 or not math.isfinite(x):
        return 0.0 if x == 0 else x
    return float(f"{x:.{n}g}")


def pack(grid_2d):
    """전부 같은 값이면 스칼라로 접는다. x_cp·C_lp·C_mq 가 그렇다.

    값 자체는 **반올림하지 않는다** (sig 의 주석 참고).
    """
    flat = {v for row in grid_2d for v in row}
    if len(flat) == 1:
        return flat.pop()
    return [list(row) for row in grid_2d]


def build_table(name):
    path = HERE / "data" / f"aero_{name}.csv"
    if not path.is_file():
        return None
    Vs, als, grid = A.table_grid(path)
    meta = A.table_meta(path)
    return {
        "V": list(Vs),
        "alpha": list(als),
        "C_A": pack(grid["C_A"]), "C_N": pack(grid["C_N"]),
        "x_cp": pack(grid["x_cp"]), "C_lp": pack(grid["C_lp"]),
        "C_mq": pack(grid["C_mq"]),
        "S_ref": float(meta["S_ref"]), "d_ref": float(meta["d_ref"]),
        "rho": float(meta["rho_ref"]),
        "_grid": grid, "_Vs": Vs, "_als": als,
    }


def selfcheck_points(tables, n=40, seed=20260914):
    """파이썬 기준값. 페이지가 JS 로 같은 값을 내는지 대조한다."""
    rnd = random.Random(seed)
    pts = []
    for name, t in tables.items():
        Vs, als, grid = t["_Vs"], t["_als"], t["_grid"]
        for _ in range(n // len(tables)):
            # 격자 안쪽·가장자리·범위 밖을 섞는다. 클램프까지 검사해야 한다.
            V = rnd.uniform(-5.0, Vs[-1] + 20.0)
            a = rnd.uniform(-0.3, als[-1] + 0.3)
            c = A.lookup(Vs, als, grid, V, a)
            pts.append({"t": name, "V": V, "a": a,
                        "e": {k: c[k] for k in ("C_A", "C_N", "x_cp")}})
    return pts


def load_speedrun(path):
    if not pathlib.Path(path).is_file():
        return None
    meta, rows = {}, []
    cols = None
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        if line.startswith("#"):
            if ":" in line:
                k, v = line[1:].split(":", 1)
                meta[k.strip()] = v.strip()
            continue
        q = [x.strip() for x in line.split(",")]
        if cols is None:
            cols = q
            continue
        if q and q[0] != "":
            rows.append([float(x) for x in q])
    ix = {c: i for i, c in enumerate(cols)}
    return {"meta": meta,
            "t": [r[ix["t_s"]] for r in rows],
            "cmd": [r[ix["cmd_mps"]] for r in rows],
            "meas": [r[ix["meas_mps"]] for r in rows],
            "alt": [r[ix["alt_m"]] for r in rows]}


def trajectory(f):
    """월드 위치 궤적. 로그에 위치 열이 있으면 그걸 쓰고, 없으면 속도를 적분한다.

    위치 열(pWx..)은 나중에 추가돼서 그 전에 찍은 로그에는 없다. 다시 날리지
    않고도 재생할 수 있게 사다리꼴 적분으로 메운다. 적분은 드리프트가 쌓이지만
    **자세는 기록된 쿼터니언 그대로**라 보고 싶은 것(기울임·풍향계)은 정확하다.
    """
    ix = f.ix
    if "pWx" in ix:
        return ([r[ix["pWx"]] for r in f.rows], [r[ix["pWy"]] for r in f.rows],
                [r[ix["pWz"]] for r in f.rows], True)
    x = y = z = 0.0
    xs, ys, zs = [], [], []
    for i, r in enumerate(f.rows):
        if i:
            dt = f.t[i] - f.t[i - 1]
            if 0.0 < dt < 1.0:
                pr = f.rows[i - 1]
                x += 0.5 * dt * (r[ix["vLx"]] + pr[ix["vLx"]])
                y += 0.5 * dt * (r[ix["vLy"]] + pr[ix["vLy"]])
                z += 0.5 * dt * (r[ix["vLz"]] + pr[ix["vLz"]])
        xs.append(x); ys.append(y); zs.append(z)
    return xs, ys, zs, False


def load_flight(path, stride=4):
    """실측 공력 로그를 얇게 만들어 심는다. 4800 행을 다 심을 필요는 없다."""
    f, err = A.open_flight(path)
    if f is None:
        return None, err
    px, py, pz, pos_logged = trajectory(f)
    ix = f.ix
    k = range(0, len(f.rows), stride)
    ok, lines = A.verdict(f)
    i_peak = max(range(len(f.F)), key=lambda j: f.F[j])
    return {
        "t": [sig(f.t[i]) for i in k], "V": [sig(f.V[i]) for i in k],
        "alpha": [sig(f.alpha[i]) for i in k], "F": [sig(f.F[i]) for i in k],
        "M": [sig(f.M[i]) for i in k], "tilt": [sig(f.tilt[i]) for i in k],
        "C_A": [sig(f.C_A[i]) for i in k], "C_N": [sig(f.C_N[i]) for i in k],
        "vg": [sig(f.v_ground[i]) for i in k],
        "mass": f.mass, "W": f.W, "S": f.S, "rho": f.rho,
        "kind": f.kind, "verdict": lines, "ok": ok,
        "consistency": f.consistency,
        "peak": {"t": f.t[i_peak], "V": f.V[i_peak], "alpha": f.alpha[i_peak],
                 "F": f.F[i_peak], "tilt": f.tilt[i_peak]},
        "n_rows": len(f.rows), "stride": stride,
        # 재생용. 자세는 기록된 쿼터니언 그대로 (scalar-last, 링크 -> 월드).
        "qx": [sig(f.rows[i][ix["qx"]], 9) for i in k],
        "qy": [sig(f.rows[i][ix["qy"]], 9) for i in k],
        "qz": [sig(f.rows[i][ix["qz"]], 9) for i in k],
        "qw": [sig(f.rows[i][ix["qw"]], 9) for i in k],
        "px": [sig(px[i]) for i in k], "py": [sig(py[i]) for i in k],
        "pz": [sig(pz[i]) for i in k],
        "pos_logged": pos_logged,
        "fx": [sig(f.rows[i][ix["fWx"]]) for i in k],
        "fy": [sig(f.rows[i][ix["fWy"]]) for i in k],
        "fz": [sig(f.rows[i][ix["fWz"]]) for i in k],
    }, ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="results/aero_dashboard.html")
    ap.add_argument("--log", default="", help="실측 공력 로그 (선택)")
    ap.add_argument("--speedrun",
                    default=str(HERE / "data" / "speedrun_2026-09-14.csv"))
    a = ap.parse_args()

    tables = {}
    for name, _, _ in SOURCES:
        t = build_table(name)
        if t:
            tables[name] = t
    if not tables:
        print("공력표를 못 찾았습니다", file=sys.stderr)
        return 1

    checks = selfcheck_points(tables)
    for t in tables.values():
        for k in ("_grid", "_Vs", "_als"):
            t.pop(k)

    payload = {
        "tables": tables,
        "labels": {n: lbl for n, lbl, _ in SOURCES},
        "mass": {n: m for n, _, m in SOURCES},
        "checks": checks,
        "speedrun": load_speedrun(a.speedrun),
        "flight": None,
        "flight_error": "",
    }
    if a.log:
        fl, err = load_flight(a.log)
        payload["flight"] = fl
        payload["flight_error"] = err

    html = TEMPLATE.replace("%%DATA%%", json.dumps(payload, ensure_ascii=False,
                                                   separators=(",", ":")))
    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    # 두 벌을 낸다.
    #   .artifact.html — Artifact 발행용. 감싸는 쪽이 doctype·charset 을 붙이므로
    #                    여기에 넣으면 안 된다.
    #   .html          — 로컬에서 그냥 열어 보는 용. charset 이 없으면 한글이
    #                    깨진다 (파일로 열거나 http.server 로 띄우면 실제로 깨졌다).
    bare = out.with_suffix(".artifact.html")
    bare.write_text(html, encoding="utf-8")
    out.write_text('<!doctype html>\n<html lang="ko">\n<head>\n'
                   '<meta charset="utf-8">\n'
                   '<meta name="viewport" content="width=device-width,'
                   'initial-scale=1">\n</head>\n<body style="margin:0">\n'
                   + html + "\n</body>\n</html>\n", encoding="utf-8")
    kb = out.stat().st_size / 1024
    print(f"저장 {out}  ({kb:.0f} KB)  — 로컬에서 열기용")
    print(f"     {bare}  — Artifact 발행용")
    print(f"  표 {', '.join(tables)}")
    print(f"  자기검사 기준점 {len(checks)} 개")
    print(f"  속도시험 {'있음' if payload['speedrun'] else '없음'}")
    print(f"  공력로그 {'있음' if payload['flight'] else (payload['flight_error'] or '없음')}")
    return 0


TEMPLATE = r"""<title>축대칭 동체 공력 계수</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans+Condensed:wght@600;700&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<style>
:root{
  /* 계기판 세계로 확실히 잡은 단일 테마 (시뮬레이터와 같은 토큰). */
  --ground:#101820; --panel:#18212B; --panel-2:#202B37; --rule:#2C3A48;
  --ink:#EEF2F6; --ink-2:#AEBAC6; --ink-3:#7C8A98;
  --accent:#F2AA4C;
  --s1:#c17f2a; --s2:#2d94bd; --ok:#4fb894; --warn:#F2AA4C; --bad:#e0766a;
  --grid:#1E2A35;
  color-scheme:dark;
}
*{box-sizing:border-box}
body{
  margin:0; background:var(--ground); color:var(--ink);
  font:400 15px/1.6 "IBM Plex Sans","Helvetica Neue",Arial,sans-serif;
  -webkit-font-smoothing:antialiased;
}
.wrap{max-width:1120px; margin:0 auto; padding:0 24px 96px}
h1,h2,h3{font-family:"IBM Plex Sans Condensed","IBM Plex Sans",sans-serif;
  font-weight:700; text-wrap:balance; margin:0}
h1{font-size:clamp(30px,4.4vw,46px); letter-spacing:-.015em; line-height:1.08}
h2{font-size:26px; letter-spacing:-.01em}
h3{font-size:15px; font-weight:600}
p{margin:0 0 14px; max-width:66ch; color:var(--ink-2)}
p.lead{color:var(--ink); font-size:17px}
code,.num{font-family:"IBM Plex Mono",ui-monospace,monospace;
  font-variant-numeric:tabular-nums}
.eyebrow{font-family:"IBM Plex Mono",monospace; font-size:11px;
  letter-spacing:.13em; text-transform:uppercase; color:var(--ink-3)}

header.top{border-bottom:1px solid var(--rule); padding:38px 0 26px;
  display:flex; flex-wrap:wrap; gap:20px; align-items:flex-end;
  justify-content:space-between}
.badges{display:flex; flex-wrap:wrap; gap:8px}
.badge{display:inline-flex; align-items:center; gap:7px; padding:5px 11px;
  border:1px solid var(--rule); border-radius:999px; background:var(--panel);
  font-size:12.5px; color:var(--ink-2)}
.badge .dot{width:7px; height:7px; border-radius:50%; flex:none}
.badge.ok .dot{background:var(--ok)} .badge.bad .dot{background:var(--bad)}
.badge.warn .dot{background:var(--warn)}
.badge b{color:var(--ink); font-weight:600}

section{padding:44px 0; border-bottom:1px solid var(--rule)}
section:last-of-type{border-bottom:0}
.shead{display:flex; align-items:baseline; gap:14px; margin-bottom:6px}
.shead .q{color:var(--ink-3); font-size:13.5px}

.hero{display:grid; grid-template-columns:repeat(auto-fit,minmax(190px,1fr));
  gap:1px; background:var(--rule); border:1px solid var(--rule);
  border-radius:10px; overflow:hidden; margin:24px 0 8px}
.hero > div{background:var(--panel); padding:18px 20px}
.hero .k{font-size:11px; letter-spacing:.11em; text-transform:uppercase;
  color:var(--ink-3); font-family:"IBM Plex Mono",monospace}
.hero .v{font-family:"IBM Plex Sans Condensed",sans-serif; font-weight:700;
  font-size:31px; line-height:1.15; margin-top:5px; letter-spacing:-.01em}
.hero .s{font-size:12.5px; color:var(--ink-3); margin-top:3px}

.panel{background:var(--panel); border:1px solid var(--rule); border-radius:10px}
.explorer{display:grid; grid-template-columns:300px 1fr; gap:24px;
  margin-top:24px; align-items:start}
@media(max-width:820px){.explorer{grid-template-columns:1fr}}
.ctl{padding:20px}
.ctl label{display:block; font-size:12px; letter-spacing:.09em;
  text-transform:uppercase; color:var(--ink-3);
  font-family:"IBM Plex Mono",monospace; margin-bottom:5px}
.ctl .rowv{display:flex; justify-content:space-between; align-items:baseline;
  margin-bottom:6px}
.ctl .rowv output{font-family:"IBM Plex Mono",monospace; font-size:18px;
  font-weight:600; color:var(--ink)}
input[type=range]{width:100%; margin:0 0 20px; accent-color:var(--s1)}
input[type=range]:focus-visible{outline:2px solid var(--s1); outline-offset:3px}
.seg{display:flex; gap:0; border:1px solid var(--rule); border-radius:7px;
  overflow:hidden; margin-bottom:18px}
.seg button{flex:1; border:0; background:var(--panel); color:var(--ink-2);
  padding:8px 6px; font:600 12.5px/1 "IBM Plex Sans",sans-serif; cursor:pointer}
.seg button[aria-pressed=true]{background:var(--panel-2); color:var(--ink)}
.seg button:focus-visible{outline:2px solid var(--s1); outline-offset:-2px}

table{border-collapse:collapse; width:100%; font-size:14px}
th,td{text-align:right; padding:7px 10px; border-bottom:1px solid var(--rule)}
th:first-child,td:first-child{text-align:left}
th{font-size:11px; letter-spacing:.09em; text-transform:uppercase;
  color:var(--ink-3); font-family:"IBM Plex Mono",monospace; font-weight:400}
td.n{font-family:"IBM Plex Mono",monospace; font-variant-numeric:tabular-nums}
td.c1{color:var(--s1)} td.c2{color:var(--s2)}
.scroll{overflow-x:auto}

.charts{display:grid; grid-template-columns:repeat(auto-fit,minmax(330px,1fr));
  gap:18px; margin-top:24px}
figure{margin:0; background:var(--panel); border:1px solid var(--rule);
  border-radius:10px; padding:16px 16px 10px}
figcaption{font-size:12.5px; color:var(--ink-3); margin-top:8px}
canvas{width:100%; display:block}
.legend{display:flex; gap:16px; flex-wrap:wrap; font-size:12.5px;
  color:var(--ink-2); margin-bottom:10px}
.legend span{display:inline-flex; align-items:center; gap:6px}
.legend i{width:14px; height:3px; border-radius:2px; display:inline-block}

ul.notes{margin:14px 0 0; padding:0; list-style:none;
  display:grid; gap:11px; max-width:78ch}
ul.notes li{padding-left:18px; position:relative; color:var(--ink-2);
  font-size:14.5px}
ul.notes li::before{content:""; position:absolute; left:0; top:9px;
  width:6px; height:6px; border-radius:50%; background:var(--warn)}
ul.notes li b{color:var(--ink); font-weight:600}
.play{display:flex; align-items:center; gap:14px; padding:12px 16px;
  border-top:1px solid var(--rule)}
.play button{width:38px; height:38px; flex:none; border:1px solid var(--rule);
  border-radius:8px; background:var(--panel); color:var(--ink); cursor:pointer;
  font-size:14px}
.play button:focus-visible{outline:2px solid var(--s1); outline-offset:2px}
.play input{flex:1; margin:0; accent-color:var(--s1)}
.play .num{font-size:12.5px; color:var(--ink-2); white-space:nowrap}
footer{padding:30px 0 0; color:var(--ink-3); font-size:13px}
</style>

<div class="wrap">
<header class="top">
  <div>
    <div class="eyebrow">fast_drone · gz_aero</div>
    <h1>축대칭 동체 공력 계수</h1>
    <p style="margin:10px 0 0">고속 ISR 미사일형 동체의 공력표와, 그 표로 PX4·Gazebo에서
    실제로 날린 결과입니다. 우분투 없이 브라우저에서 계수를 만져볼 수 있습니다.</p>
  </div>
  <div class="badges" id="badges"></div>
</header>

<section>
  <div class="shead"><h2>결론</h2><span class="q">이거 믿어도 되나</span></div>
  <div class="hero" id="hero"></div>
  <p style="margin-top:18px">공력 코어는 파이썬·C++ 두 번 이식해 <span class="num">146/146</span>
  일치를 확인했고, Gazebo 안에서 좌표 변환(2400 상태점)과 힘 적용점을 실측으로 검증했습니다.
  그 뒤 실기체 모델에 붙여 측풍과 고속 비행을 돌렸습니다.</p>
</section>

<section>
  <div class="shead"><h2>계수 탐색기</h2><span class="q">V와 α를 주면 힘이 얼마인가</span></div>
  <div class="explorer">
    <div class="panel ctl">
      <div class="seg" id="seg" role="group" aria-label="표 선택"></div>
      <div class="rowv"><label for="sv">속도 V</label><output id="ov"></output></div>
      <input type="range" id="sv" min="0" max="120" step="0.1" value="83.3">
      <div class="rowv"><label for="sa">받음각 α</label><output id="oa"></output></div>
      <input type="range" id="sa" min="0" max="180" step="0.5" value="15">
      <p style="font-size:13px;margin:0;color:var(--ink-3)">α는 전 받음각입니다.
      0°는 기수가 유동을 정면으로 향한 순항, 90°는 옆에서 맞는 상태입니다.</p>
    </div>
    <div class="panel scroll"><table id="tbl"></table></div>
  </div>
</section>

<section>
  <div class="shead"><h2>계수 곡선</h2><span class="q">이 형상에 이 값이 나올 수 있나</span></div>
  <div class="legend" id="leg"></div>
  <div class="charts">
    <figure><canvas id="cCA" height="420"></canvas>
      <figcaption>축력 계수. 기수 방향으로 작용합니다.</figcaption></figure>
    <figure><canvas id="cCN" height="420"></canvas>
      <figcaption>법선력 계수. 기수와 직각으로 작용합니다.</figcaption></figure>
    <figure><canvas id="cCT" height="420"></canvas>
      <figcaption>합성 계수. 두 표의 차이가 가장 크게 드러나는 곳입니다.</figcaption></figure>
    <figure><canvas id="cFV" height="420"></canvas>
      <figcaption>현재 α에서 속도에 따른 힘. 무게선을 넘으면 기체가 못 버팁니다.</figcaption></figure>
  </div>
</section>

<section id="secRun">
  <div class="shead"><h2>속도 시험</h2><span class="q">300 km/h가 나오나</span></div>
  <div class="charts" style="grid-template-columns:1fr">
    <figure><canvas id="cRun" height="380"></canvas>
      <figcaption id="runCap"></figcaption></figure>
  </div>
</section>

<section id="sec3d" hidden>
  <div class="shead"><h2>비행 재생</h2><span class="q">기체가 실제로 어떻게 움직였나</span></div>
  <div class="panel" style="margin-top:22px;overflow:hidden">
    <div id="view3d" style="position:relative;background:var(--panel-2)"></div>
    <div class="play">
      <button type="button" id="pp" aria-label="재생">▶</button>
      <input type="range" id="scrub" min="0" max="100" step="1" value="0">
      <span class="num" id="tnow"></span>
      <span class="num" id="hud"></span>
    </div>
  </div>
  <p id="cap3d" style="margin-top:12px"></p>
</section>

<section id="secFlight" hidden>
  <div class="shead"><h2>비행 중 공력</h2><span class="q">실제로 그렇게 나왔나</span></div>
  <div class="charts">
    <figure><canvas id="cFt" height="380"></canvas>
      <figcaption>비행 중 실제로 걸린 공력과 받음각.</figcaption></figure>
    <figure><canvas id="cFa" height="380"></canvas>
      <figcaption>표 곡선 위에 비행이 지난 자리를 덮어 그린 것.</figcaption></figure>
  </div>
  <div id="flightVerdict"></div>
</section>

<section>
  <div class="shead"><h2>알려진 한계</h2><span class="q">무엇을 아직 못 믿나</span></div>
  <ul class="notes">
    <li><b>placeholder 표의 횡류 계수가 작습니다.</b> α=90°에서 <span class="num">C_N</span>이
      1.20인데 팀 사이징 표는 8.85입니다. 기준면적 차이를 반영해도 힘으로 2.65배 차이입니다.
      평면적 대 기준면적 비가 빠진 것으로 보이며, 형상 확정 뒤 다시 잡아야 합니다.</li>
    <li><b>α&gt;90° 역류 영역은 검증되지 않았습니다.</b> 표가 α=180°에서 축력을 α=0°와
      같은 값으로 돌려주는데, 뒤로 나는 동체의 항력이 코부터 나는 것과 같을 수는 없습니다.
      감속 구간이 이 영역을 지나므로 CFD가 필요합니다.</li>
    <li><b>로터 전진비가 Gazebo에 없습니다.</b> 기본 모터 모델은 유입류를 무시해
      고속에서도 추력이 안 떨어집니다. 실제로는 크게 떨어집니다. 이 시뮬 결과는
      <b>동체 공력 검증이지 추진기 검증이 아닙니다.</b></li>
    <li><b>압력중심이 받음각에 무관합니다.</b> 두 표 모두 무게중심 뒤 0.10 m 고정입니다.
      정적 안정 방향은 맞지만 받음각에 따라 움직이는 것은 안 담겨 있습니다.</li>
    <li><b>감쇠계수가 두 표에서 같습니다.</b> <span class="num">C_mq</span> −10,
      <span class="num">C_lp</span> −5로 동일한데, 사이징 표가 팀 값을 쓰지 않고
      임시값을 그대로 물려받은 것으로 보입니다.</li>
  </ul>
</section>

<footer>
  값은 플러그인이 읽는 CSV 그대로입니다. 표를 고치고
  <code>build_aero_dashboard.py</code>를 다시 돌리면 이 페이지가 갱신됩니다.
</footer>
</div>

<script>
const D = %%DATA%%;
const $ = s => document.querySelector(s);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt = (x, n = 3) => (Math.abs(x) >= 1000 ? x.toFixed(0) : x.toFixed(n));
const G = 9.80665;
let CUR = Object.keys(D.tables)[0];

/* ── 보간: aero_table.hpp:Lookup() 과 같은 규칙 ────────────────────────
   양선형 + 가장자리 클램프. 이 함수가 플러그인과 어긋나면 페이지 전체가
   거짓이 되므로, 아래 selfCheck() 가 파이썬 기준값과 대조한다. */
function locate(g, x){
  if (x <= g[0]) return [0, 0, 0];
  if (x >= g[g.length - 1]) return [g.length - 1, g.length - 1, 0];
  let lo = 0, hi = g.length - 1;
  while (hi - lo > 1){ const m = (lo + hi) >> 1; if (g[m] <= x) lo = m; else hi = m; }
  const span = g[hi] - g[lo];
  return [lo, hi, span === 0 ? 0 : (x - g[lo]) / span];
}
function cell(f, i, j){ return typeof f === "number" ? f : f[i][j]; }
function lookup(t, V, a){
  const [i0, i1, tv] = locate(t.V, V), [j0, j1, ta] = locate(t.alpha, a);
  const w = [(1 - tv) * (1 - ta), (1 - tv) * ta, tv * (1 - ta), tv * ta];
  const out = {};
  for (const k of ["C_A", "C_N", "x_cp", "C_lp", "C_mq"]){
    const f = t[k];
    out[k] = w[0] * cell(f, i0, j0) + w[1] * cell(f, i0, j1)
           + w[2] * cell(f, i1, j0) + w[3] * cell(f, i1, j1);
  }
  return out;
}
function selfCheck(){
  let worst = 0;
  for (const p of D.checks){
    const c = lookup(D.tables[p.t], p.V, p.a);
    for (const k in p.e) worst = Math.max(worst, Math.abs(c[k] - p.e[k]));
  }
  return worst;
}

/* ── 캔버스 ──────────────────────────────────────────────────────── */
function setup(cv){
  // ⚠ 의도한 높이를 **한 번만** 기억한다. cv.height 에 배율을 곱해 쓰는 순간
  //   그 속성 자체가 바뀌므로, 다시 그릴 때 getAttribute 로 읽으면 420 -> 840
  //   -> 1680 로 매번 두 배가 된다. 슬라이더를 움직일수록 캔버스가 커져
  //   페이지가 흰 기둥으로 덮이고 느려진다. 실제로 그렇게 터졌다.
  if (!cv._h) cv._h = +cv.getAttribute("height");
  const r = Math.min(window.devicePixelRatio || 1, 2);
  const w = cv.clientWidth, h = cv._h;
  cv.width = w * r; cv.height = h * r;
  cv.style.height = h + "px";
  const c = cv.getContext("2d");
  c.setTransform(r, 0, 0, r, 0, 0);
  c.clearRect(0, 0, w, h);
  return [c, w, h];
}
function axes(c, w, h, xd, yd, xl, yl, title){
  const P = {l: 56, r: 14, t: 26, b: 38};
  const X = v => P.l + (v - xd[0]) / (xd[1] - xd[0] || 1) * (w - P.l - P.r);
  const Y = v => h - P.b - (v - yd[0]) / (yd[1] - yd[0] || 1) * (h - P.t - P.b);
  c.font = '400 11px "IBM Plex Mono", monospace';
  c.strokeStyle = css("--grid"); c.lineWidth = 1;
  c.fillStyle = css("--ink-3");
  for (let i = 0; i <= 4; i++){
    const v = yd[0] + (yd[1] - yd[0]) * i / 4, y = Math.round(Y(v)) + .5;
    c.beginPath(); c.moveTo(P.l, y); c.lineTo(w - P.r, y); c.stroke();
    c.textAlign = "right"; c.textBaseline = "middle";
    c.fillText(fmt(v, Math.abs(yd[1]) >= 100 ? 0 : 1), P.l - 8, y);
  }
  for (let i = 0; i <= 4; i++){
    const v = xd[0] + (xd[1] - xd[0]) * i / 4, x = Math.round(X(v)) + .5;
    c.textAlign = "center"; c.textBaseline = "top";
    c.fillText(fmt(v, 0), x, h - P.b + 8);
  }
  c.strokeStyle = css("--rule");
  c.beginPath(); c.moveTo(P.l, h - P.b + .5); c.lineTo(w - P.r, h - P.b + .5); c.stroke();
  c.fillStyle = css("--ink-2"); c.textAlign = "center"; c.textBaseline = "bottom";
  c.fillText(xl, (P.l + w - P.r) / 2, h - 4);
  c.save(); c.translate(13, (P.t + h - P.b) / 2); c.rotate(-Math.PI / 2);
  c.fillText(yl, 0, 0); c.restore();
  if (title){
    c.fillStyle = css("--ink"); c.textAlign = "left"; c.textBaseline = "top";
    c.font = '600 13px "IBM Plex Sans", sans-serif';
    c.fillText(title, P.l - 42, 4);
  }
  return [X, Y];
}
function line(c, X, Y, xs, ys, color, width){
  c.strokeStyle = color; c.lineWidth = width || 2;
  c.lineJoin = "round"; c.beginPath();
  let started = false;
  for (let i = 0; i < xs.length; i++){
    if (!isFinite(ys[i])) { started = false; continue; }
    const x = X(xs[i]), y = Y(ys[i]);
    if (!started){ c.moveTo(x, y); started = true; } else c.lineTo(x, y);
  }
  c.stroke();
}
function mark(c, X, Y, x, y, color){
  c.fillStyle = color; c.strokeStyle = css("--panel"); c.lineWidth = 2;
  c.beginPath(); c.arc(X(x), Y(y), 5, 0, 7); c.fill(); c.stroke();
}

/* ── 그리기 ──────────────────────────────────────────────────────── */
const NAMES = Object.keys(D.tables);
const COL = {}; // 표 이름 -> 색
function colors(){ NAMES.forEach((n, i) => COL[n] = css(i ? "--s2" : "--s1")); }

function curveAt(t, V, key){
  const xs = [], ys = [];
  for (let d = 0; d <= 180; d += 1){
    xs.push(d);
    const c = lookup(t, V, d * Math.PI / 180);
    ys.push(key === "tot" ? Math.hypot(c.C_A, c.C_N) : c[key]);
  }
  return [xs, ys];
}
function drawCoef(id, key, title, ylab){
  const cv = $(id); const [c, w, h] = setup(cv);
  const V = +$("#sv").value, A = +$("#sa").value;
  let lo = 0, hi = 0;
  const series = NAMES.map(n => {
    const [xs, ys] = curveAt(D.tables[n], V, key);
    ys.forEach(v => { lo = Math.min(lo, v); hi = Math.max(hi, v); });
    return [n, xs, ys];
  });
  const pad = (hi - lo) * .08 || 1;
  const [X, Y] = axes(c, w, h, [0, 180], [lo - pad, hi + pad],
                      "받음각 α [deg]", ylab, title);
  series.forEach(([n, xs, ys]) => line(c, X, Y, xs, ys, COL[n], 2));
  series.forEach(([n, xs, ys]) => mark(c, X, Y, A, ys[Math.round(A)], COL[n]));
}
function drawFV(){
  const cv = $("#cFV"); const [c, w, h] = setup(cv);
  const A = +$("#sa").value * Math.PI / 180, V = +$("#sv").value;
  const Vs = []; for (let v = 0; v <= 120; v += 2) Vs.push(v);
  let hi = 0;
  const series = NAMES.map(n => {
    const t = D.tables[n];
    const ys = Vs.map(v => {
      const c2 = lookup(t, v, A);
      return .5 * t.rho * v * v * t.S_ref * Math.hypot(c2.C_A, c2.C_N);
    });
    hi = Math.max(hi, ...ys);
    return [n, ys];
  });
  const W = D.mass[CUR] * G;
  hi = Math.max(hi, W * 1.15);
  const [X, Y] = axes(c, w, h, [0, 120], [0, hi], "속도 V [m/s]", "|F| [N]",
                      "속도에 따른 공력");
  c.strokeStyle = css("--ink-3"); c.setLineDash([4, 4]); c.lineWidth = 1;
  c.beginPath(); c.moveTo(X(0), Y(W)); c.lineTo(X(120), Y(W)); c.stroke();
  c.setLineDash([]);
  c.fillStyle = css("--ink-3"); c.textAlign = "right"; c.textBaseline = "bottom";
  c.font = '400 11px "IBM Plex Mono", monospace';
  c.fillText("무게 " + W.toFixed(0) + " N", X(119), Y(W) - 3);
  series.forEach(([n, ys]) => line(c, X, Y, Vs, ys, COL[n], 2));
  series.forEach(([n, ys]) => {
    const t = D.tables[n];
    const c2 = lookup(t, V, A);
    mark(c, X, Y, V, .5 * t.rho * V * V * t.S_ref * Math.hypot(c2.C_A, c2.C_N), COL[n]);
  });
}
function drawRun(){
  const r = D.speedrun; if (!r) { $("#secRun").hidden = true; return; }
  const cv = $("#cRun"); const [c, w, h] = setup(cv);
  const hi = Math.max(...r.cmd, ...r.meas) * 1.12;
  const [X, Y] = axes(c, w, h, [0, Math.max(...r.t)], [0, hi],
                      "시간 [s]", "속도 [m/s]", "명령 대 실측");
  line(c, X, Y, r.t, r.cmd, css("--ink-3"), 2);
  line(c, X, Y, r.t, r.meas, css("--s1"), 2.5);
  const pk = Math.max(...r.meas), i = r.meas.indexOf(pk);
  mark(c, X, Y, r.t[i], pk, css("--s1"));
  c.fillStyle = css("--ink"); c.textAlign = "center"; c.textBaseline = "bottom";
  c.font = '600 12px "IBM Plex Mono", monospace';
  c.fillText((pk * 3.6).toFixed(0) + " km/h", X(r.t[i]), Y(pk) - 10);
  $("#runCap").textContent =
    "명령 " + (Math.max(...r.cmd) * 3.6).toFixed(0) + " km/h 에 대해 실측 최고 "
    + (pk * 3.6).toFixed(1) + " km/h. 유지 구간에서 20초 내내 평평합니다 — "
    + "속도 제한이 아니라 공력과 자세 권한이 만든 한계입니다.";
}
function drawFlight(){
  const f = D.flight; if (!f) return;
  $("#secFlight").hidden = false;
  let cv = $("#cFt"); let [c, w, h] = setup(cv);
  const hiF = Math.max(...f.F) * 1.12;
  let [X, Y] = axes(c, w, h, [f.t[0], f.t[f.t.length - 1]], [0, hiF],
                    "시간 [s]", "|F| [N]", "비행 중 공력");
  c.strokeStyle = css("--ink-3"); c.setLineDash([4, 4]); c.lineWidth = 1;
  c.beginPath(); c.moveTo(X(f.t[0]), Y(f.W)); c.lineTo(X(f.t[f.t.length - 1]), Y(f.W));
  c.stroke(); c.setLineDash([]);
  line(c, X, Y, f.t, f.F, css("--s1"), 2);
  mark(c, X, Y, f.peak.t, f.peak.F, css("--s2"));

  cv = $("#cFa"); [c, w, h] = setup(cv);
  const [xs, ysA] = curveAt(D.tables[CUR], Math.max(...f.V), "C_A");
  const [, ysN] = curveAt(D.tables[CUR], Math.max(...f.V), "C_N");
  let lo = Math.min(...ysA, ...ysN, ...f.C_A, ...f.C_N);
  let hi = Math.max(...ysA, ...ysN, ...f.C_A, ...f.C_N);
  [X, Y] = axes(c, w, h, [0, 180], [lo * 1.1, hi * 1.1],
                "받음각 α [deg]", "계수", "표 위의 비행 궤적");
  line(c, X, Y, xs, ysA, css("--s1"), 2);
  line(c, X, Y, xs, ysN, css("--s2"), 2);
  c.globalAlpha = .5;
  for (let i = 0; i < f.alpha.length; i++){
    c.fillStyle = css("--ink-2");
    c.beginPath(); c.arc(X(f.alpha[i]), Y(f.C_N[i]), 2, 0, 7); c.fill();
  }
  c.globalAlpha = 1;
  $("#flightVerdict").innerHTML = "<ul class='notes'>" +
    f.verdict.map(s => "<li>" + s.replace(/^[✅❌⚠ℹ]\s*/, "") + "</li>").join("") +
    "</ul>";
}

/* ── 3D 재생 ─────────────────────────────────────────────────────── */
let R3 = null;
function init3D(){
  const f = D.flight;
  if (!f || !f.px || !window.THREE) return;
  const host = $("#view3d");
  $("#sec3d").hidden = false;
  const H = 460;
  const ren = new THREE.WebGLRenderer({antialias:true, alpha:true});
  ren.setPixelRatio(Math.min(devicePixelRatio, 2));
  ren.setSize(host.clientWidth, H);
  host.style.height = H + "px";
  host.appendChild(ren.domElement);

  const sc = new THREE.Scene();
  // ★ 월드를 Gazebo 그대로 ENU 로 둔다. 카메라의 위쪽만 +Z 로 바꾸면
  //   쿼터니언을 변환할 필요가 없다. 이 프로젝트에서 좌표 변환은 가장
  //   사고가 잦은 자리라, 변환을 아예 안 하는 쪽을 골랐다.
  const cam = new THREE.PerspectiveCamera(52, host.clientWidth / H, 0.5, 40000);
  cam.up.set(0, 0, 1);
  sc.add(new THREE.HemisphereLight(0xdfe8f2, 0x141c26, 1.15));
  const sun = new THREE.DirectionalLight(0xffffff, .75);
  sun.position.set(-1, -2, 3); sc.add(sun);

  const grid = new THREE.GridHelper(4000, 80, 0x35495d, 0x1c2733);
  grid.rotation.x = Math.PI / 2;
  grid.material.opacity = .35; grid.material.transparent = true;
  sc.add(grid);

  // 기체. 장축 = 링크 +X (DESIGN.md 9 장에서 확정한 축).
  const skin = new THREE.MeshStandardMaterial({color:0x1b2530, roughness:.45, metalness:.35});
  const tip  = new THREE.MeshStandardMaterial({color:0xb8802f, roughness:.45});
  const veh = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(.075, .075, .78, 28), skin);
  body.rotation.z = -Math.PI / 2; veh.add(body);
  const cone = new THREE.Mesh(new THREE.ConeGeometry(.075, .24, 28), tip);
  cone.rotation.z = -Math.PI / 2; cone.position.x = .51; veh.add(cone);
  const arms = new THREE.Mesh(new THREE.BoxGeometry(.04, .5, .04), skin);
  veh.add(arms);
  const arms2 = new THREE.Mesh(new THREE.BoxGeometry(.04, .04, .5), skin);
  veh.add(arms2);
  veh.scale.setScalar(4);   // 수 km 궤적 옆에서 1 m 기체는 안 보인다
  sc.add(veh);

  const path = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(
      f.px.map((x, i) => new THREE.Vector3(x, f.py[i], f.pz[i]))),
    new THREE.LineBasicMaterial({color:0x2d94bd}));
  sc.add(path);

  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(), 1, 0xf2aa4c, .6, .35);
  sc.add(arrow);

  let i = 0, playing = false, last = 0;
  const n = f.t.length;
  $("#scrub").max = n - 1;
  const W = f.W;

  function frame(){
    const p = new THREE.Vector3(f.px[i], f.py[i], f.pz[i]);
    veh.position.copy(p);
    veh.quaternion.set(f.qx[i], f.qy[i], f.qz[i], f.qw[i]);
    const F = new THREE.Vector3(f.fx[i], f.fy[i], f.fz[i]);
    const m = F.length();
    if (m > 1e-6){
      arrow.visible = true;
      arrow.position.copy(p);
      arrow.setDirection(F.normalize());
      arrow.setLength(Math.max(2, 14 * m / W), Math.max(.7, 2.4 * m / W),
                      Math.max(.4, 1.4 * m / W));
    } else arrow.visible = false;

    // 카메라는 **비스듬히 뒤옆**에서 따라간다. 정면 뒤에 두면 직선 비행이
    // 점 하나로 보여 아무것도 안 읽힌다 (처음에 그렇게 만들었다가 고쳤다).
    const j = Math.min(i + 4, n - 1), k2 = Math.max(i - 4, 0);
    const dir = new THREE.Vector3(f.px[j] - f.px[k2], f.py[j] - f.py[k2],
                                  f.pz[j] - f.pz[k2]);
    if (dir.lengthSq() < 1e-9) dir.set(1, 0, 0);
    dir.normalize();
    const up = new THREE.Vector3(0, 0, 1);
    let side = new THREE.Vector3().crossVectors(dir, up);
    if (side.lengthSq() < 1e-6) side.set(0, 1, 0);   // 수직 비행일 때 퇴화 방지
    side.normalize();
    cam.position.copy(p)
       .addScaledVector(dir, -15)
       .addScaledVector(side, 12)
       .addScaledVector(up, 6);
    cam.lookAt(p);

    $("#scrub").value = i;
    $("#tnow").textContent = "t " + f.t[i].toFixed(1) + " s";
    $("#hud").textContent =
      "V " + f.V[i].toFixed(1) + " m/s · α " + f.alpha[i].toFixed(0)
      + "° · |F| " + f.F[i].toFixed(1) + " N (" + (100 * f.F[i] / W).toFixed(0) + "% W)";
    ren.render(sc, cam);
  }
  function loop(ts){
    if (playing){
      if (ts - last > 33){ last = ts; i = (i + 1) % n; frame(); }
    }
    requestAnimationFrame(loop);
  }
  $("#pp").addEventListener("click", () => {
    playing = !playing;
    $("#pp").textContent = playing ? "❚❚" : "▶";
    $("#pp").setAttribute("aria-label", playing ? "일시정지" : "재생");
  });
  $("#scrub").addEventListener("input", e => {
    playing = false; $("#pp").textContent = "▶";
    i = +e.target.value; frame();
  });
  $("#cap3d").textContent =
    "자세는 기록된 쿼터니언 그대로입니다. 위치는 "
    + (f.pos_logged ? "로그에 기록된 값입니다."
       : "속도를 적분해 메운 값이라 드리프트가 쌓입니다. 보려는 것(기울임·기수 방향)은 자세라 정확합니다.")
    + " 기체는 궤적 옆에서 보이도록 6배로 키웠습니다. 주황 화살표가 공력입니다.";
  frame();
  R3 = () => { ren.setSize(host.clientWidth, H);
               cam.aspect = host.clientWidth / H; cam.updateProjectionMatrix(); frame(); };
  requestAnimationFrame(loop);
}

/* ── 표 · 배지 ───────────────────────────────────────────────────── */
function readout(){
  const V = +$("#sv").value, Ad = +$("#sa").value, A = Ad * Math.PI / 180;
  $("#ov").textContent = V.toFixed(1) + " m/s  ·  " + (V * 3.6).toFixed(0) + " km/h";
  $("#oa").textContent = Ad.toFixed(1) + "°";
  const rows = [["q̄ 동압", "Pa", t => .5 * t.rho * V * V, 0],
                ["C_A 축력계수", "", (t, c) => c.C_A, 4],
                ["C_N 법선력계수", "", (t, c) => c.C_N, 4],
                ["|C| 합성", "", (t, c) => Math.hypot(c.C_A, c.C_N), 4],
                ["x_cp 압력중심", "m", (t, c) => c.x_cp, 4],
                ["C_mq 피치감쇠", "", (t, c) => c.C_mq, 2],
                ["|F| 공력", "N", (t, c) => .5 * t.rho * V * V * t.S_ref
                  * Math.hypot(c.C_A, c.C_N), 2]];
  let html = "<thead><tr><th>값</th>"
    + NAMES.map(n => "<th>" + n + "</th>").join("") + "<th>단위</th></tr></thead><tbody>";
  for (const [lbl, unit, fn, dp] of rows){
    html += "<tr><td>" + lbl + "</td>";
    NAMES.forEach((n, i) => {
      const t = D.tables[n], c = lookup(t, V, A);
      html += "<td class='n c" + (i + 1) + "'>" + fn(t, c).toFixed(dp) + "</td>";
    });
    html += "<td class='n' style='color:var(--ink-3)'>" + unit + "</td></tr>";
  }
  NAMES.forEach(() => {});
  const fr = NAMES.map(n => {
    const t = D.tables[n], c = lookup(t, V, A);
    const F = .5 * t.rho * V * V * t.S_ref * Math.hypot(c.C_A, c.C_N);
    return 100 * F / (D.mass[n] * G);
  });
  html += "<tr><td>무게 대비</td>"
    + fr.map((v, i) => "<td class='n c" + (i + 1) + "'>" + v.toFixed(0) + "</td>").join("")
    + "<td class='n' style='color:var(--ink-3)'>%</td></tr></tbody>";
  $("#tbl").innerHTML = html;
}
function draw(){
  colors();
  drawCoef("#cCA", "C_A", "축력 계수 C_A", "C_A");
  drawCoef("#cCN", "C_N", "법선력 계수 C_N", "C_N");
  drawCoef("#cCT", "tot", "합성 계수 |C|", "|C|");
  drawFV(); drawRun(); drawFlight();
  $("#leg").innerHTML = NAMES.map(n =>
    "<span><i style='background:" + COL[n] + "'></i>" + n
    + " — " + D.labels[n] + "</span>").join("");
}
function init(){
  const err = selfCheck();
  const good = err < 1e-12;
  const r = D.speedrun;
  const pk = r ? Math.max(...r.meas) : null;
  $("#badges").innerHTML =
    "<span class='badge " + (good ? "ok" : "bad") + "'><span class='dot'></span>"
    + "자기검사 <b>" + (good ? "일치" : "불일치") + "</b> "
    + err.toExponential(1) + "</span>"
    + "<span class='badge ok'><span class='dot'></span>공력 코어 <b>146/146</b></span>"
    + "<span class='badge ok'><span class='dot'></span>좌표 변환 <b>2400 상태점</b></span>";
  const cells = [
    ["최고 속도", pk ? (pk * 3.6).toFixed(1) + " km/h" : "—",
     pk ? pk.toFixed(2) + " m/s · 목표 300의 " + (pk * 3.6 / 300 * 100).toFixed(0) + "%" : ""],
    ["공력 코어", "146 / 146", "C++와 파이썬이 비트 단위로 일치"],
    ["좌표 변환", "1e-15", "월드↔링크↔동체 왕복 오차 (2400 점)"],
    ["힘 적용점", "1082배", "옳은 가설과 틀린 가설의 판별력"],
  ];
  $("#hero").innerHTML = cells.map(([k, v, s]) =>
    "<div><div class='k'>" + k + "</div><div class='v'>" + v
    + "</div><div class='s'>" + s + "</div></div>").join("");

  $("#seg").innerHTML = NAMES.map(n =>
    "<button type='button' data-n='" + n + "' aria-pressed='"
    + (n === CUR) + "'>" + n + "</button>").join("");
  $("#seg").addEventListener("click", e => {
    const b = e.target.closest("button"); if (!b) return;
    CUR = b.dataset.n;
    [...$("#seg").children].forEach(x =>
      x.setAttribute("aria-pressed", x.dataset.n === CUR));
    readout(); draw();
  });
  for (const id of ["#sv", "#sa"])
    $(id).addEventListener("input", () => { readout(); draw(); });
  readout(); draw(); init3D();
}
init();
addEventListener("resize", () => { draw(); if (R3) R3(); });
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);
</script>
"""

if __name__ == "__main__":
    raise SystemExit(main())
