#!/usr/bin/env python3
"""브라우저에서 직접 푸는 6자유도 비행 시뮬레이터를 만든다.

    python3 gz_aero/tools/gen_sim_reference.py     (먼저)
    python3 gz_aero/tools/build_sim.py

녹화 재생이 아니라 **브라우저가 실제로 방정식을 푼다.** 우분투도 Gazebo 도
PX4 도 필요 없다. 조건을 바꾸면 즉시 다시 난다.

★ 검증 사슬
    control/dynamics.py (CasADi)   원본
        ↕ gen_sim_reference.py 가 대조 — 현재 4.4e-15
    plain_xdot (순수 파이썬)        JS 가 따라 할 대상
        ↕ 이 페이지가 로드될 때 대조
    JS 포트                         브라우저

  같은 물리의 세 번째 구현이라 어긋날 자리가 하나 는다. 배지가 초록이 아니면
  페이지의 모든 숫자를 믿으면 안 된다.
"""
import argparse
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="results/flight_sim.html")
    ap.add_argument("--ref", default=str(HERE / "data" / "sim_reference.json"))
    a = ap.parse_args()

    ref_path = pathlib.Path(a.ref)
    if not ref_path.is_file():
        print(f"기준값이 없습니다: {ref_path}\n"
              "  먼저 python3 gz_aero/tools/gen_sim_reference.py 를 돌리세요.",
              file=sys.stderr)
        return 1
    ref = json.loads(ref_path.read_text(encoding="utf-8"))

    html = TEMPLATE.replace("%%DATA%%", json.dumps(ref, separators=(",", ":")))
    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    bare = out.with_suffix(".artifact.html")
    bare.write_text(html, encoding="utf-8")
    out.write_text('<!doctype html>\n<html lang="ko">\n<head>\n'
                   '<meta charset="utf-8">\n'
                   '<meta name="viewport" content="width=device-width,'
                   'initial-scale=1">\n</head>\n<body style="margin:0">\n'
                   + html + "\n</body>\n</html>\n", encoding="utf-8")
    print(f"저장 {out}  ({out.stat().st_size / 1024:.0f} KB)  — 로컬에서 열기용")
    print(f"     {bare}  — Artifact 발행용")
    print(f"  기준 궤적 {ref['steps']} 스텝 (dt {ref['dt']} s)")
    print(f"  CasADi 대조 {ref['casadi_max_diff']:.3e}")
    return 0


TEMPLATE = r"""<title>축대칭 동체 비행 시뮬레이터</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans+Condensed:wght@600;700&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<style>
:root{
  /* 계기판 세계로 확실히 잡은 단일 테마. 밝은 변형을 두지 않는 대신
     배경과 모든 색을 명시해 어느 환경에서도 그대로 선다. */
  --ground:#101820; --panel:#18212B; --panel-2:#202B37; --rule:#2C3A48;
  --stage:#0A0F15; --stage-2:#141C25;
  --ink:#EEF2F6; --ink-2:#AEBAC6; --ink-3:#7C8A98;
  --accent:#F2AA4C;
  --s1:#c17f2a; --s2:#2d94bd; --s3:#4fb894;
  --ok:#4fb894; --warn:#F2AA4C; --bad:#e0766a; --grid:#1E2A35;
  color-scheme:dark;
}
*{box-sizing:border-box}
body{margin:0; background:var(--ground); color:var(--ink);
  font:400 14px/1.55 "IBM Plex Sans","Helvetica Neue",Arial,sans-serif;
  -webkit-font-smoothing:antialiased}
.num,code{font-family:"IBM Plex Mono",ui-monospace,monospace;
  font-variant-numeric:tabular-nums}

.app{display:grid; grid-template-rows:auto 1fr;
  height:100vh; min-height:640px; max-height:1100px}
@media(max-width:900px){.app{height:auto; max-height:none}}

header{display:flex; align-items:center; gap:16px; flex-wrap:wrap;
  padding:12px 20px; border-bottom:1px solid var(--rule); background:var(--panel)}
header h1{margin:0; font:700 19px/1.1 "IBM Plex Sans Condensed",sans-serif;
  letter-spacing:-.01em}
header .sub{color:var(--ink-3); font-size:12.5px}
.badges{display:flex; gap:8px; margin-left:auto; flex-wrap:wrap}
.badge{display:inline-flex; align-items:center; gap:6px; padding:4px 10px;
  border:1px solid var(--rule); border-radius:999px; font-size:12px;
  color:var(--ink-2); background:var(--panel-2)}
.badge .dot{width:7px;height:7px;border-radius:50%;flex:none;background:var(--ok)}
details.vchk{position:relative}
details.vchk summary{list-style:none; cursor:pointer}
details.vchk summary::-webkit-details-marker{display:none}
details.vchk .vbody{position:absolute; right:0; top:calc(100% + 7px); z-index:5;
  width:262px; padding:12px 14px; border:1px solid var(--rule);
  border-radius:10px; background:var(--panel-2);
  box-shadow:0 10px 26px rgba(0,0,0,.45)}
details.vchk .vbody div{display:flex; justify-content:space-between; gap:10px;
  font:400 12px/1.9 "IBM Plex Mono",monospace; color:var(--ink-3)}
details.vchk .vbody div b{color:var(--ink)}
details.vchk .vbody p{margin:8px 0 0; font-size:11.5px; color:var(--ink-3);
  line-height:1.55}
.badge.bad .dot{background:var(--bad)} .badge b{color:var(--ink);font-weight:600}

/* 폭을 사용자가 끌어서 바꾼다. 글이 좁은 칸에서 지저분하게 접히던 문제라
   고정폭 대신 변수로 두고 손잡이로 민다. */
.body{display:grid; min-height:0;
  grid-template-columns:var(--colL,268px) 5px 1fr 5px var(--colR,262px)}
.grip{cursor:col-resize; background:var(--rule); position:relative}
.grip::after{content:""; position:absolute; inset:0 -4px}
.grip:hover, .grip.on{background:var(--accent)}
@media(max-width:900px){.body{grid-template-columns:1fr}
  .grip{display:none}
  .col{border:0; border-bottom:1px solid var(--rule)}}

.col{padding:14px 16px; overflow-y:auto; background:var(--panel); min-height:0}
.col h2{margin:0 0 12px; font:600 11px/1 "IBM Plex Mono",monospace;
  letter-spacing:.13em; text-transform:uppercase; color:var(--ink-3)}
.col h2+h2{margin-top:22px}

.fld{margin-bottom:14px}
.fld .row{display:flex; justify-content:space-between; align-items:baseline; gap:8px}
.fld label{font-size:12.5px; color:var(--ink-2)}
.fld output{font:600 14px/1 "IBM Plex Mono",monospace; color:var(--ink)}
input[type=range]{width:100%; margin:5px 0 0; accent-color:var(--accent)}
input[type=range]:focus-visible{outline:2px solid var(--accent); outline-offset:3px}

.btns{display:flex; gap:8px; margin:4px 0 16px}
.btns button{flex:1; padding:9px 8px; border:1px solid var(--rule);
  border-radius:8px; background:var(--panel-2); color:var(--ink); cursor:pointer;
  font:600 13px/1 "IBM Plex Sans",sans-serif}
.btns button.go{background:var(--accent); border-color:var(--accent); color:#101820}
.btns button:focus-visible{outline:2px solid var(--accent); outline-offset:2px}

.gauges{display:grid; grid-template-columns:1fr 1fr; gap:1px;
  background:var(--rule); border:1px solid var(--rule); border-radius:9px;
  overflow:hidden}
.gauges div{background:var(--panel-2); padding:8px 10px}
.gauges .k{font:400 10px/1.2 "IBM Plex Mono",monospace; letter-spacing:.08em;
  text-transform:uppercase; color:var(--ink-3)}
.gauges .v{font:600 16px/1.25 "IBM Plex Mono",monospace; margin-top:2px}
.gauges .v small{font-size:11px; font-weight:400; color:var(--ink-3)}
.gauges .wide{grid-column:1/-1}

.stage{display:grid; grid-template-rows:1fr 26px 172px; min-height:0;
  background:var(--stage)}
#view{position:relative; min-height:0}
#view canvas{display:block}
.hud{position:absolute; left:14px; top:12px; pointer-events:none;
  font:400 12px/1.65 "IBM Plex Mono",monospace; color:#D3DCE6;
  text-shadow:0 1px 3px rgba(0,0,0,.8)}
.hud b{color:#ffffff; font-weight:600}
.viewbtns{position:absolute; right:12px; top:12px; display:flex; gap:5px;
  flex-wrap:wrap; justify-content:flex-end; max-width:62%}
.viewbtns button{padding:5px 10px; border:1px solid #2C3A48; border-radius:7px;
  background:rgba(10,15,21,.9); color:#AEBAC6; cursor:pointer;
  font:500 12px/1 "IBM Plex Sans",sans-serif}
.viewbtns button[aria-pressed=true]{border-color:#F2AA4C; color:#ffffff}
.viewbtns button:focus-visible{outline:2px solid #F2AA4C; outline-offset:2px}
.legend3d{position:absolute; right:12px; bottom:12px; display:flex; gap:14px;
  align-items:center; font:400 11.5px/1 "IBM Plex Mono",monospace; color:#7C8A98}
.legend3d span{display:inline-flex; align-items:center; gap:5px}
.legend3d i{width:12px; height:3px; border-radius:2px; display:inline-block}
.warnbox{position:absolute; left:14px; bottom:34px; right:14px;
  pointer-events:none; font-size:12.5px; color:#F2AA4C}
.segbar{height:26px; border-top:1px solid var(--rule); background:var(--stage-2);
  display:flex; align-items:stretch; position:relative; overflow:hidden}
.segbar i{display:block; height:100%}
.segbar{cursor:col-resize}
.segbar .head{position:absolute; top:0; bottom:0; width:2px; background:#fff;
  box-shadow:0 0 6px rgba(255,255,255,.8); pointer-events:none; margin-left:-1px}
.segbar .leg{margin-left:16px; display:inline-flex; gap:9px; opacity:.8}
.segbar .leg span{margin-right:2px}
.segbar .lab{position:absolute; left:10px; top:0; bottom:0; display:flex;
  align-items:center; gap:12px; pointer-events:none;
  font:400 10.5px/1 "IBM Plex Mono",monospace; letter-spacing:.08em; color:#c8d2dd;
  text-shadow:0 1px 3px rgba(0,0,0,.9)}
.segbar .lab b{font-weight:600; color:#fff}
.plots{border-top:1px solid var(--rule); background:var(--stage-2);
  display:grid; grid-template-columns:1fr 1fr 1fr; gap:1px}
.plots figure{margin:0; background:var(--stage); padding:8px 10px 4px;
  min-width:0}
.plots figcaption{font:400 10.5px/1.3 "IBM Plex Mono",monospace;
  color:#7C8A98; letter-spacing:.06em; text-transform:uppercase}
.plots canvas{width:100%; display:block}
@media(max-width:760px){.plots{grid-template-columns:1fr}
  .stage{grid-template-rows:340px 26px auto}}

.note{font-size:12px; color:var(--ink-3); line-height:1.6; margin:10px 0 0;
  overflow-wrap:anywhere}
.note b{color:var(--ink-2)}
details.more{margin-top:16px; border-top:1px solid var(--rule); padding-top:10px}
details.more summary{cursor:pointer; font:600 11px/1 "IBM Plex Mono",monospace;
  letter-spacing:.12em; text-transform:uppercase; color:var(--ink-3);
  list-style:none; padding:3px 0}
details.more summary::-webkit-details-marker{display:none}
details.more summary::before{content:"+ "; color:var(--accent)}
details.more[open] summary::before{content:"\2212 "}
details.more summary:focus-visible{outline:2px solid var(--accent); outline-offset:2px}
</style>

<div class="app">
<header>
  <div>
    <h1>축대칭 동체 비행 시뮬레이터</h1>
    <div class="sub">브라우저가 6자유도를 직접 풉니다. Gazebo도 PX4도 없습니다.</div>
  </div>
  <div class="badges" id="badges"></div>
</header>

<div class="body">
  <div class="col left" id="colL">
    <h2>비행 명령</h2>
    <div class="btns">
      <button type="button" id="go" class="go">▶ 시작  (Space)</button>
      <button type="button" id="rst">↺ 초기화</button>
    </div>
    <div class="fld"><div class="row"><label for="spd">목표 속도</label>
      <output id="o_spd"></output></div>
      <input type="range" id="spd" min="0" max="120" step="1" value="83"></div>
    <div class="fld"><div class="row"><label for="alt">목표 고도</label>
      <output id="o_alt"></output></div>
      <input type="range" id="alt" min="10" max="400" step="10" value="200"></div>
    <div class="fld"><div class="row"><label for="wsp">측풍</label>
      <output id="o_wsp"></output></div>
      <input type="range" id="wsp" min="0" max="40" step="1" value="0"></div>
    <div class="fld"><div class="row"><label for="wdir">바람 방위</label>
      <output id="o_wdir"></output></div>
      <input type="range" id="wdir" min="0" max="350" step="10" value="90"></div>
    <div class="fld"><div class="row"><label for="rtf">시간 배속</label>
      <output id="o_rtf"></output></div>
      <input type="range" id="rtf" min="0.25" max="4" step="0.25" value="1"></div>

    <h2>계기</h2>
    <div class="gauges" id="gauges"></div>
    <details class="more">
      <summary>이 시뮬에 대해</summary>
      <p class="note"><b>지금 기체는 로켓형입니다.</b> 로터가 동체축에 수직인
        평면에 놓이고 추력이 기수 방향이라, 호버에서 기수가 위를 봅니다.
        <code>vehicle_params.py</code> 의 <code>Iyy = Izz</code> 가 바로 이
        4겹 대칭을 요구합니다.</p>
      <p class="note"><b>제어기가 한계입니다.</b> 여기 제어기는 추력만으로 뜨는
        멀티로터식이라, 수평 순항에 필요한 <b>동체 받음각 양력 트림</b>을 못 찾습니다.
        그래서 170 km/h 근처에서 멈춥니다. 기체가 못 가는 게 아니라 이 제어기가
        못 태우는 것입니다 — 프로젝트가 NMPC 를 쓰기로 한 이유가 정확히 이
        천이·트림 문제입니다.</p>
      <p class="note">목표 속도를 올렸다 내리면 기체가 따라갑니다.
        <b>명령과 실측이 벌어지면</b> 거기가 이 기체의 한계입니다.</p>
      <p class="note">기본 설정(83 m/s 목표, 무풍)에서 <b>280 km/h</b> 에 수렴합니다.
        같은 기체를 PX4 + Gazebo 로 날렸을 때는 <b>281 km/h</b> 였습니다.
        제어기도 적분기도 공력 경로도 다른 두 스택이 <b>0.4% 안에서 일치</b>합니다 —
        이 한계는 설정이 아니라 물리입니다.</p>
      <p class="note">고도가 조금 가라앉는 것은 제어기 탓입니다. 여기 제어기는
        PX4 가 아니라 단순한 종속 루프라, 크게 기울인 동안 고도 권한을 일부
        잃습니다. 검증한 것은 <b>플랜트이지 제어기가 아닙니다.</b></p>
    </details>
  </div>

  <div class="grip" id="gripL" role="separator" aria-label="왼쪽 폭 조절"></div>

  <div class="stage">
    <div id="view"><div class="hud" id="hud"></div>
      <div class="viewbtns">
        <button type="button" data-view="back">뒤에서</button>
        <button type="button" data-view="side">옆에서</button>
        <button type="button" data-view="top">위에서</button>
        <button type="button" data-view="nose">기수축</button>
        <button type="button" id="follow" aria-pressed="true">따라가기</button>
      </div>
      <div class="legend3d">
        <span><i style="background:#f2aa4c"></i>공력</span>
        <span><i style="background:#2d94bd"></i>속도</span>
        <span>끌기 = 회전 · 휠 = 확대 · Shift+끌기 = 이동 · Space = 정지 · R = 리셋 · F = 따라가기</span>
      </div>
      <div class="warnbox" id="warn"></div></div>
    <div class="segbar" id="segbar" title="미션 구간"></div>
    <div class="plots">
      <figure><figcaption>속도 m/s</figcaption><canvas id="p1" height="132"></canvas></figure>
      <figure><figcaption>받음각 deg</figcaption><canvas id="p2" height="132"></canvas></figure>
      <figure><figcaption>공력 N</figcaption><canvas id="p3" height="132"></canvas></figure>
    </div>
  </div>

  <div class="grip" id="gripR" role="separator" aria-label="오른쪽 폭 조절"></div>

  <div class="col right" id="colR">
    <h2>동체 공력 계수</h2>
    <div id="coefs"></div>
    <div class="btns"><button type="button" id="def">기본값으로</button></div>
    <details class="more">
      <summary>계수에 대해</summary>
      <p class="note">계수를 바꾸면 <b>다음 스텝부터 즉시</b> 반영됩니다.
        <code>C_A0</code>를 올리면 항력이 커져 최고 속도가 떨어지고,
        <code>x_cp</code>를 0에 가깝게 하면 정적 안정이 사라집니다.</p>
      <p class="note">로터 전진비는 담겨 있지만(<code>J_max</code>) Gazebo 기본
        모터 모델에는 없습니다. 이 시뮬이 그쪽보다 보수적입니다.
        <code>α&gt;90°</code> 역류 영역은 어느 쪽도 검증되지 않았습니다.</p>
    </details>
  </div>
</div>
</div>

<script>
const D = %%DATA%%;
const P0 = D.params;
const P = JSON.parse(JSON.stringify(P0));
const EPS = 1e-8, NX = 17;
const $ = s => document.querySelector(s);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

/* ══ 물리 — gen_sim_reference.py:plain_xdot 의 그대로 ══════════════════
   연산 순서를 바꾸면 기준값과 마지막 자리가 어긋난다. 배지가 그걸 잡는다. */
function xdot(x, u, w, p){
  const vel0 = x[3], vel1 = x[4], vel2 = x[5];
  const qx = x[6], qy = x[7], qz = x[8], qw = x[9];
  const om0 = x[10], om1 = x[11], om2 = x[12];

  const R00 = 1 - 2*(qy*qy + qz*qz), R01 = 2*(qx*qy - qz*qw), R02 = 2*(qx*qz + qy*qw);
  const R10 = 2*(qx*qy + qz*qw), R11 = 1 - 2*(qx*qx + qz*qz), R12 = 2*(qy*qz - qx*qw);
  const R20 = 2*(qx*qz - qy*qw), R21 = 2*(qy*qz + qx*qw), R22 = 1 - 2*(qx*qx + qy*qy);

  const d0 = vel0 - w[0], d1 = vel1 - w[1], d2 = vel2 - w[2];
  const ub = R00*d0 + R10*d1 + R20*d2;
  const vb = R01*d0 + R11*d1 + R21*d2;
  const wb = R02*d0 + R12*d1 + R22*d2;

  const V_sq = ub*ub + vb*vb + wb*wb + EPS;
  const V = Math.sqrt(V_sq);
  const V_cf = Math.sqrt(vb*vb + wb*wb + EPS);
  const q_bar = 0.5 * p.rho * V_sq;

  const F_N_fac = 0.5 * p.rho * p.S_ref * (p.C_Na*ub + p.C_dc*V_cf);
  const Fy = -F_N_fac * vb;
  const Fz = -F_N_fac * wb;
  const C_A = p.C_A0 + p.C_Aa2 * (vb*vb + wb*wb) / V_sq;
  const Fx = -q_bar * p.S_ref * C_A;

  const xcp = p.x_cp;
  let Mx = 0.0, My = -xcp*Fz, Mz = xcp*Fy;
  const df = 0.25 * p.rho * V * p.S_ref * p.d_ref * p.d_ref;
  Mx += df * p.C_lp * om0;
  My += df * p.C_mq * om1;
  Mz += df * p.C_mq * om2;

  // thrust_axis 'x' = 로터가 동체축에 수직인 평면에 놓이고 추력이 기수 방향
  //                   (로켓형, 추진까지 축대칭). 'z' 는 역호환용.
  const axIsX = p.thrust_axis === "x";
  const V_axial = axIsX ? (ub > 0.0 ? ub : 0.0) : (-wb > 0.0 ? -wb : 0.0);
  let h_net = 0.0, Frx = 0.0, Frz = 0.0;
  for (let i = 0; i < p.num_rotors; i++){
    const ni = x[13 + i], di = p.rotor_directions[i], ri = p.rotor_positions[i];
    const n_rps = ni / (2.0 * Math.PI);
    const J = V_axial / (n_rps * p.D_prop + EPS);
    let fac = 1.0 - J / p.J_max; if (fac < 0.0) fac = 0.0;
    const Ti = p.k_T * ni * ni * fac, Qi = p.k_Q * ni * ni * fac;
    if (axIsX){
      Frx += Ti;
      My += ri[2] * Ti;
      Mz += -ri[1] * Ti;
      Mx += di * Qi;
    } else {
      Frz += -Ti;
      Mx += ri[1] * (-Ti);
      My += -ri[0] * (-Ti);
      Mz += di * Qi;
    }
    h_net += p.I_rotor * ni * di;
  }
  if (axIsX){
    My += -om2 * h_net;
    Mz += om1 * h_net;
  } else {
    Mx += -om1 * h_net;
    My += om0 * h_net;
  }

  const Fbx = Fx + Frx, Fby = Fy, Fbz = Fz + Frz;
  const m = p.mass;
  const ax = (R00*Fbx + R01*Fby + R02*Fbz) / m;
  const ay = (R10*Fbx + R11*Fby + R12*Fbz) / m;
  const az = -p.g + (R20*Fbx + R21*Fby + R22*Fbz) / m;

  const pw = om0, qq = om1, rr = om2;
  let qd0 = 0.5*(qw*pw - qz*qq + qy*rr);
  let qd1 = 0.5*(qz*pw + qw*qq - qx*rr);
  let qd2 = 0.5*(-qy*pw + qx*qq + qw*rr);
  let qd3 = 0.5*(-qx*pw - qy*qq - qz*rr);
  const nrm = qx*qx + qy*qy + qz*qz + qw*qw - 1.0;
  qd0 -= nrm*qx; qd1 -= nrm*qy; qd2 -= nrm*qz; qd3 -= nrm*qw;

  const Jx = p.Ixx, Jy = p.Iyy, Jz = p.Izz;
  const wdx = (Mx - (qq*(Jz*rr) - rr*(Jy*qq))) / Jx;
  const wdy = (My - (rr*(Jx*pw) - pw*(Jz*rr))) / Jy;
  const wdz = (Mz - (pw*(Jy*qq) - qq*(Jx*pw))) / Jz;

  const tau = p.tau_m;
  return [vel0, vel1, vel2, ax, ay, az, qd0, qd1, qd2, qd3, wdx, wdy, wdz,
          (u[0]-x[13])/tau, (u[1]-x[14])/tau, (u[2]-x[15])/tau, (u[3]-x[16])/tau];
}
function rk4(x, u, w, p, dt, sub){
  sub = sub || 4;
  const h = dt / sub;
  for (let s = 0; s < sub; s++){
    const k1 = xdot(x, u, w, p);
    const x2 = new Array(NX); for (let i=0;i<NX;i++) x2[i] = x[i] + 0.5*h*k1[i];
    const k2 = xdot(x2, u, w, p);
    const x3 = new Array(NX); for (let i=0;i<NX;i++) x3[i] = x[i] + 0.5*h*k2[i];
    const k3 = xdot(x3, u, w, p);
    const x4 = new Array(NX); for (let i=0;i<NX;i++) x4[i] = x[i] + h*k3[i];
    const k4 = xdot(x4, u, w, p);
    const xn = new Array(NX);
    for (let i=0;i<NX;i++) xn[i] = x[i] + (h/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    x = xn;
  }
  const qn = Math.sqrt(x[6]*x[6] + x[7]*x[7] + x[8]*x[8] + x[9]*x[9]);
  if (qn > 1e-10) for (let i=6;i<10;i++) x[i] /= qn;
  for (let i=13;i<17;i++)
    x[i] = x[i] < p.n_min ? p.n_min : (x[i] > p.n_max ? p.n_max : x[i]);
  return x;
}
function selfCheck(){
  // 기준 궤적을 같은 입력으로 다시 풀어 한 스텝씩 대조한다.
  let x = D.x0.slice(), worst = 0;
  for (let k = 1; k <= D.steps; k++){
    x = rk4(x, D.u, D.w, P0, D.dt);
    const ref = D.traj[k];
    for (let i = 0; i < NX; i++) worst = Math.max(worst, Math.abs(x[i] - ref[i]));
  }
  return worst;
}

/* ══ 제어기 ═══════════════════════════════════════════════════════════
   PX4 를 옮긴 것이 아니다. 평범한 종속 루프(속도 -> 자세 -> 모멘트)다.
   여기서 보려는 것은 제어 성능이 아니라 **기체가 어디서 한계에 걸리나** 이므로
   제어기는 단순할수록 원인이 분명해진다. 검증도 플랜트만 한다. */
// 자세 루프를 세게 잡는다. 동체가 정적 안정(x_cp 가 무게중심 뒤)이라 공력이
// 기수를 바람 쪽으로 미는데, 비례 제어만으로는 그 모멘트를 상쇄하며 정상편차가
// 남는다. Jy*KW*KR 이 약하면 30~60도까지 벌어져 추력축이 누워 고도를 못 버틴다.
const KV = 1.2, KZ = 1.0, KR = 8.0, KW = 18.0;
// 가속도 상한. 실제 PX4 시험은 0 -> 83 m/s 를 30 초에 올렸다 (2.8 m/s^2).
// 이보다 크게 잡으면 기체가 즉시 70 도씩 누워 아무것도 안 읽힌다.
const A_MAX = 12.0, RAMP = 3.0;
// 고도 적분. 기체가 크게 기울면 추력의 수직성분이 줄고 동체가 반양력을 내
// 고도가 서서히 가라앉는다. P 만으로는 정상편차가 남아 결국 지면에 닿았다.
const KI = 0.35, I_LIM = 6.0;
// 추력축이 수직에서 눕는 한계. 스윕 결과 60도면 351 km/h 까지 가지만 |w| 가
// 10 rad/s 로 텀블한다 (메모리의 "RMSE 만으론 텀블 못 잡음" 그대로다).
// 55도가 안전선이고 거기선 170 km/h 다. 이 트레이드오프가 곧 NMPC 가 필요한 이유다.
const TILT_MAX = 55.0 * Math.PI / 180.0;
let altI = 0;
// 지금 상태에서 동체 공력을 월드 좌표로. 제어기 앞먹임과 계기 표시가 같은
// 값을 쓰도록 한 곳에서 계산한다.
function aeroWorld(x){
  const w = windNow();
  const qx=x[6], qy=x[7], qz=x[8], qw=x[9];
  const R = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
             [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
             [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]];
  const d0=x[3]-w[0], d1=x[4]-w[1], d2=x[5]-w[2];
  const ub = R[0][0]*d0+R[1][0]*d1+R[2][0]*d2;
  const vb = R[0][1]*d0+R[1][1]*d1+R[2][1]*d2;
  const wb = R[0][2]*d0+R[1][2]*d1+R[2][2]*d2;
  const V_sq = ub*ub + vb*vb + wb*wb + EPS;
  const V_cf = Math.sqrt(vb*vb + wb*wb + EPS);
  const fac = 0.5*P.rho*P.S_ref*(P.C_Na*ub + P.C_dc*V_cf);
  const Fy = -fac*vb, Fz = -fac*wb;
  const C_A = P.C_A0 + P.C_Aa2*(vb*vb + wb*wb)/V_sq;
  const Fx = -0.5*P.rho*V_sq*P.S_ref*C_A;
  return [R[0][0]*Fx + R[0][1]*Fy + R[0][2]*Fz,
          R[1][0]*Fx + R[1][1]*Fy + R[1][2]*Fz,
          R[2][0]*Fx + R[2][1]*Fy + R[2][2]*Fz];
}

function control(x, cmd){
  const m = P.mass, g = P.g;
  const psi = cmd.psi;
  const eAlt = cmd.alt - x[2];
  altI += eAlt * DT;
  if (altI > I_LIM) altI = I_LIM; else if (altI < -I_LIM) altI = -I_LIM;
  const vdes = [cmd.spd*Math.cos(psi), cmd.spd*Math.sin(psi),
                Math.max(-15, Math.min(15, KZ*eAlt + KI*altI))];
  let ax = KV*(vdes[0]-x[3]), ay = KV*(vdes[1]-x[4]), az = KV*(vdes[2]-x[5]);
  const an = Math.hypot(ax, ay);
  if (an > A_MAX){ ax *= A_MAX/an; ay *= A_MAX/an; }
  // ★ 공력 앞먹임. 이게 없으면 제어기는 "뜨는 힘은 전부 추력" 이라고 믿는다.
  //   로켓형은 순항에서 동체가 받음각으로 양력을 내는데, 그걸 모르면 그 트림을
  //   못 찾아 170 km/h 에서 멈춘다. 지금 동체가 실제로 내는 힘을 빼 주면
  //   제어기가 남은 몫만 추력으로 채운다.
  const Fa = aeroWorld(x);
  let tx = m*ax - Fa[0], ty = m*ay - Fa[1];
  const tz = m*(az + g) - Fa[2];

  // ★ 추력축이 수직에서 너무 눕지 않게 **수평 요구를 깎는다**. 고도가 우선이다.
  //   앞먹임만 넣으면 제어기가 동체 양력을 믿고 90도를 넘겨 눕는다. 그러면
  //   추력의 수직성분이 사라져 양력에만 매달리고, 받음각이 조금만 변해도
  //   떨어진다 — 실제로 383 km/h 로 날면서 고도를 계속 잃었다.
  const hz = Math.hypot(tx, ty);
  const lim = Math.max(tz, 1e-6) * Math.tan(TILT_MAX);
  if (hz > lim && hz > 1e-9){ const s2 = lim/hz; tx *= s2; ty *= s2; }
  const tn = Math.hypot(tx, ty, tz) || 1e-9;

  // 원하는 자세. 로켓형은 **동체 x(기수)** 가 추력 방향이다.
  const xb = [tx/tn, ty/tn, tz/tn];

  const qx0=x[6], qy0=x[7], qz0=x[8], qw0=x[9];
  const R0 = [[1-2*(qy0*qy0+qz0*qz0), 2*(qx0*qy0-qz0*qw0), 2*(qx0*qz0+qy0*qw0)],
              [2*(qx0*qy0+qz0*qw0), 1-2*(qx0*qx0+qz0*qz0), 2*(qy0*qz0-qx0*qw0)],
              [2*(qx0*qz0-qy0*qw0), 2*(qy0*qz0+qx0*qw0), 1-2*(qx0*qx0+qy0*qy0)]];

  // ★ 두 번째 축은 **지금 자세에서 최소로 바뀌도록** 잡는다.
  //   임의의 기준축(월드 위 / 기수 방위)으로 만들면 목표 프레임이 현재에서
  //   180도 떨어진 쪽으로 잡히는 일이 생긴다. 그 지점에서 축각 추출이 퇴화해
  //   오차가 0 으로 읽히고 모멘트 명령이 사라진다 — 실제로 네 로터가 전부
  //   같은 값으로 나와 기체가 손도 못 쓰고 떨어졌다.
  //   축대칭 기체라 기수축 둘레 롤은 공력상 의미가 없으므로, 현재 롤을 그대로
  //   유지하는 것이 옳기도 하다.
  let yb = [R0[0][1], R0[1][1], R0[2][1]];           // 지금 동체 y
  const d = yb[0]*xb[0] + yb[1]*xb[1] + yb[2]*xb[2];
  yb = [yb[0]-d*xb[0], yb[1]-d*xb[1], yb[2]-d*xb[2]];  // xb 에 수직으로 투영
  let yn = Math.hypot(yb[0], yb[1], yb[2]);
  if (yn < 1e-6){                                     // 지금 y 가 xb 와 나란하면
    const alt = [R0[0][2], R0[1][2], R0[2][2]];       // 동체 z 로 대신한다
    const d2 = alt[0]*xb[0] + alt[1]*xb[1] + alt[2]*xb[2];
    yb = [alt[0]-d2*xb[0], alt[1]-d2*xb[1], alt[2]-d2*xb[2]];
    yn = Math.hypot(yb[0], yb[1], yb[2]) || 1e-9;
  }
  yb = [yb[0]/yn, yb[1]/yn, yb[2]/yn];
  const zb = [xb[1]*yb[2] - xb[2]*yb[1],
              xb[2]*yb[0] - xb[0]*yb[2],
              xb[0]*yb[1] - xb[1]*yb[0]];

  const R = R0;
  const Rd = [[xb[0], yb[0], zb[0]], [xb[1], yb[1], zb[1]], [xb[2], yb[2], zb[2]]];
  const E = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i=0;i<3;i++) for (let j=0;j<3;j++){
    let s2 = 0; for (let k=0;k<3;k++) s2 += R[k][i]*Rd[k][j]; E[i][j] = s2;
  }
  const tr = E[0][0]+E[1][1]+E[2][2];
  const ang = Math.acos(Math.max(-1, Math.min(1, (tr-1)/2)));
  let ex = E[2][1]-E[1][2], ey = E[0][2]-E[2][0], ez = E[1][0]-E[0][1];
  const es = Math.hypot(ex, ey, ez);
  if (es > 1e-9){ const k2 = ang/es; ex*=k2; ey*=k2; ez*=k2; }

  const Jx=P.Ixx, Jy=P.Iyy, Jz=P.Izz;
  const p_=x[10], q_=x[11], r_=x[12];
  const Mx = Jx*KW*(KR*ex-p_) + (q_*(Jz*r_) - r_*(Jy*q_));
  const My = Jy*KW*(KR*ey-q_) + (r_*(Jx*p_) - p_*(Jz*r_));
  const Mz = Jz*KW*(KR*ez-r_) + (p_*(Jy*q_) - q_*(Jx*p_));

  // 총추력은 **현재** 추력축(동체 x 를 월드로)에 투영한다
  const axw = R[0][0], ayw = R[1][0], azw = R[2][0];
  let T = tx*axw + ty*ayw + tz*azw;
  if (T < 0) T = 0;

  const A = D.alloc_inv, n = [0,0,0,0];
  for (let i=0;i<4;i++){
    let Ti = A[i][0]*T + A[i][1]*Mx + A[i][2]*My + A[i][3]*Mz;
    if (Ti < 0) Ti = 0;
    const ni = Math.sqrt(Ti / P.k_T);
    n[i] = ni > P.n_max ? P.n_max : ni;
  }
  return n;
}

/* ══ 상태 ═════════════════════════════════════════════════════════════ */
const DT = 0.002;
let X = D.x0.slice(), T = 0, running = false, sat = 0;
// 슬라이더를 확 올려도 명령은 RAMP [m/s^2] 로 따라간다. 실제 비행이 그렇고,
// 안 그러면 기체가 즉시 드러누워 화면에서 아무것도 안 읽힌다.
let cmdSpd = 0;
// 기록. 상태까지 담아 **시간을 앞뒤로 오갈 수 있게** 한다.
// 되감아서 재생을 누르면 그 지점부터 다시 난다 (뒤 기록은 버린다).
const REC = [];
let viewIdx = null;        // null = 실시간, 숫자 = 그 프레임을 보는 중
const REC_EVERY = 0.04;    // [s] 기록 간격
let recAcc = 0;
function cmdNow(){
  return {spd:cmdSpd, alt:+$("#alt").value, psi:0, target:+$("#spd").value};
}
function windNow(){
  const s = +$("#wsp").value, d = +$("#wdir").value * Math.PI/180;
  return [s*Math.cos(d), s*Math.sin(d), 0];
}
function reset(){
  REC.length = 0; viewIdx = null; recAcc = 0;
  // 지상에서 시작한다. 예전엔 목표 고도에 바로 놓고 시작해 이륙 단계가 아예
  // 없었다 — 미션의 첫 구간이 통째로 빠져 있던 셈이다.
  X = D.x0.slice(); X[2] = 0; T = 0; sat = 0; cmdSpd = 0; altI = 0;
  trailN = 0;
  if (trail) trail.geometry.setDrawRange(0, 0);
}
// 지면. 아래로 뚫고 내려가지 않게 막고, 닿아 있으면 수직속도를 죽인다.
function ground(){
  if (X[2] < 0){
    X[2] = 0;
    if (X[5] < 0) X[5] = 0;
  }
}
// 지금 어떤 미션 구간인가. 화면 아래 띠에 색으로 깔린다.
const SEGS = {
  ground: ["지상",  "#7C8A98"], climb: ["상승",  "#2d94bd"],
  accel:  ["가속",  "#c17f2a"], cruise:["순항",  "#4fb894"],
  decel:  ["감속",  "#9085e9"], hover: ["호버",  "#5C6B7A"],
};
function segNow(d, cmd, prevCmd){
  if (X[2] < 1.0 && d.gs < 1.0) return "ground";
  if (Math.abs(cmd.alt - X[2]) > 5.0) return "climb";
  if (cmd.spd - prevCmd < -1e-9) return "decel";
  if (cmd.spd < 1.0) return "hover";
  if (cmd.spd - d.gs > 2.0) return "accel";
  return "cruise";
}
function diag(){
  // 지금 상태의 공력·받음각. 표시 전용이라 물리와 분리해 다시 센다.
  const w = windNow();
  const qx=X[6], qy=X[7], qz=X[8], qw=X[9];
  const R = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
             [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
             [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]];
  const d0=X[3]-w[0], d1=X[4]-w[1], d2=X[5]-w[2];
  const ub = R[0][0]*d0+R[1][0]*d1+R[2][0]*d2;
  const vb = R[0][1]*d0+R[1][1]*d1+R[2][1]*d2;
  const wb = R[0][2]*d0+R[1][2]*d1+R[2][2]*d2;
  const Vcf = Math.hypot(vb, wb), V = Math.hypot(ub, vb, wb);
  const alpha = Math.atan2(Vcf, ub) * 180/Math.PI;
  const q_bar = 0.5*P.rho*(V*V);
  const fac = 0.5*P.rho*P.S_ref*(P.C_Na*ub + P.C_dc*Vcf);
  const Fy = -fac*vb, Fz = -fac*wb;
  const C_A = P.C_A0 + P.C_Aa2*(vb*vb+wb*wb)/(V*V+EPS);
  const Fx = -q_bar*P.S_ref*C_A;
  // 기울임 = **추력축**(로켓형은 동체 x)이 수직에서 벗어난 각.
  const tAx = (P.thrust_axis === "x") ? R[2][0] : -R[2][2];
  const tilt = Math.acos(Math.max(-1,Math.min(1, tAx)))*180/Math.PI;
  // 동체 공력을 월드로 돌린다. 화살표가 **실제 힘 방향**을 가리켜야 한다.
  // 예전엔 기수 방향을 그대로 써서 항력이 앞을 가리켰다.
  const Fw = [R[0][0]*Fx + R[0][1]*Fy + R[0][2]*Fz,
              R[1][0]*Fx + R[1][1]*Fy + R[1][2]*Fz,
              R[2][0]*Fx + R[2][1]*Fy + R[2][2]*Fz];
  return {V, alpha, q_bar, F:Math.hypot(Fx,Fy,Fz), tilt, Fw,
          gs:Math.hypot(X[3],X[4]), alt:X[2]};
}

/* ══ 3D ═══════════════════════════════════════════════════════════════ */
let ren, scene, cam, veh, arrow, velArrow, trail, trailPos, trailN = 0;
const PROPS = [];   // 프로펠러를 실제 회전수만큼 돌린다
// RViz 처럼 마우스로 궤도·이동·확대. OrbitControls 는 three 핵심 번들에 없어서
// 직접 쓴다 (CDN 에서 따로 받으면 막힐 수 있다).
const ORB = {az: -2.3, el: 0.32, dist: 15, follow: true,
             tgt: new THREE.Vector3()};
function init3D(){
  const host = $("#view");
  ren = new THREE.WebGLRenderer({antialias:true});
  ren.setPixelRatio(Math.min(devicePixelRatio, 2));
  host.appendChild(ren.domElement);
  scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x0a0f15, 140, 1000);
  cam = new THREE.PerspectiveCamera(52, 1, .5, 5000);
  cam.up.set(0, 0, 1);      // 월드는 z-up. 쿼터니언을 변환하지 않는다.
  scene.add(new THREE.HemisphereLight(0xdfe8f2, 0x141c26, 1.15));
  const sun = new THREE.DirectionalLight(0xffffff, .8);
  sun.position.set(-1,-2,3); scene.add(sun);

  const g1 = new THREE.GridHelper(3000, 120, 0x35495d, 0x1c2733);
  g1.rotation.x = Math.PI/2; scene.add(g1);

  // 레드불 레이싱 드론 배치. 어뢰형 동체가 수평이고 주황 X암이 동체를 감싼다.
  // 로터 추력은 동체축과 수직 — dynamics.py 의 V_axial = -w_b 가 이 배치다.
  // 물리는 처음부터 이랬고, 예전엔 프로펠러를 반투명 통짜 원판으로 그려서
  // 동체를 덮는 덩어리처럼 보였을 뿐이다.
  // 추진까지 축대칭인 로켓 배치. 기수축(동체 +x)을 따라 내려다보면 동체 단면이
  // 원으로 보이고 그 둘레에 로터 넷이 90도 간격으로 놓인다. 호버에서는 기수가
  // 위를 본다 — vehicle_params 의 Iyy = Izz 가 바로 이 4겹 대칭을 요구한다.
  const skin = new THREE.MeshStandardMaterial({color:0x1b2530, roughness:.42, metalness:.4});
  const accent = new THREE.MeshStandardMaterial({color:0xb8802f, roughness:.45, metalness:.25});
  const tipM = new THREE.MeshStandardMaterial({color:0xd8402f, roughness:.4});
  const blade = new THREE.MeshStandardMaterial({color:0x18222c, roughness:.65,
                  transparent:true, opacity:.5, side:THREE.DoubleSide});

  veh = new THREE.Group();
  const fus = new THREE.Mesh(new THREE.CylinderGeometry(.075, .075, .70, 28), skin);
  fus.rotation.z = -Math.PI/2; veh.add(fus);
  const cone = new THREE.Mesh(new THREE.ConeGeometry(.075, .26, 28), tipM);
  cone.rotation.z = -Math.PI/2; cone.position.x = .48; veh.add(cone);
  const tail = new THREE.Mesh(new THREE.CylinderGeometry(.062, .05, .12, 24), skin);
  tail.rotation.z = -Math.PI/2; tail.position.x = -.41; veh.add(tail);
  const ring = new THREE.Mesh(new THREE.CylinderGeometry(.0785, .0785, .07, 28), accent);
  ring.rotation.z = -Math.PI/2; veh.add(ring);

  const ARM = 0.25 / Math.SQRT2;      // 기수축 둘레 반지름 (vehicle_params)
  const RPROP = 0.30 / 2;
  PROPS.length = 0;
  // 로터는 yz 평면(동체축에 수직)에 90도 간격. rocket_params 의 위치와 같다.
  for (const [py, pz] of [[ARM, ARM], [-ARM, ARM], [-ARM, -ARM], [ARM, -ARM]]){
    const r = Math.hypot(py, pz), ph = Math.atan2(pz, py);
    const arm = new THREE.Mesh(new THREE.BoxGeometry(.020, r, .020), accent);
    arm.position.set(0, py/2, pz/2);
    arm.rotation.x = -ph;            // 길이축(y)을 (py,pz) 방향으로 돌린다
    veh.add(arm);
    const pod = new THREE.Mesh(new THREE.CylinderGeometry(.026,.030,.075,16), accent);
    pod.rotation.z = -Math.PI/2; pod.position.set(.02, py, pz);
    veh.add(pod);
    const prop = new THREE.Group();
    for (let k = 0; k < 2; k++){
      const bl = new THREE.Mesh(new THREE.BoxGeometry(.004, RPROP*.96, .030), blade);
      bl.position.y = RPROP*.48;
      const holder = new THREE.Group();
      holder.add(bl); holder.rotation.x = k * Math.PI;
      prop.add(holder);
    }
    prop.position.set(.066, py, pz);
    veh.add(prop);
    PROPS.push(prop);
  }
  veh.scale.setScalar(5);
  scene.add(veh);

  arrow = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(),
                                1, 0xf2aa4c, .8, .35);
  scene.add(arrow);
  velArrow = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(),
                                   1, 0x2d94bd, .8, .35);
  scene.add(velArrow);

  trailPos = new Float32Array(3 * 4000);
  const tg = new THREE.BufferGeometry();
  tg.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  tg.setDrawRange(0, 0);
  trail = new THREE.Line(tg, new THREE.LineBasicMaterial({color:0x2d94bd}));
  scene.add(trail);
  bindMouse(ren.domElement);
  resize3D();
}

function bindMouse(el){
  let drag = null, px = 0, py = 0;
  el.style.cursor = "grab";
  el.addEventListener("contextmenu", e => e.preventDefault());
  el.addEventListener("pointerdown", e => {
    drag = (e.button === 2 || e.shiftKey) ? "pan" : "orbit";
    px = e.clientX; py = e.clientY;
    el.setPointerCapture(e.pointerId); el.style.cursor = "grabbing";
  });
  el.addEventListener("pointerup", e => {
    drag = null; el.releasePointerCapture(e.pointerId); el.style.cursor = "grab";
  });
  el.addEventListener("pointermove", e => {
    if (!drag) return;
    const dx = e.clientX - px, dy = e.clientY - py;
    px = e.clientX; py = e.clientY;
    if (drag === "orbit"){
      ORB.az -= dx * 0.006;
      ORB.el = Math.max(-1.45, Math.min(1.45, ORB.el + dy * 0.006));
    } else {
      // 화면 기준으로 민다. 미는 순간 따라가기를 끈다 — 안 그러면 튕겨 돌아온다.
      ORB.follow = false;
      const f = ORB.dist * 0.0016;
      const right = new THREE.Vector3(-Math.sin(ORB.az), Math.cos(ORB.az), 0);
      const upv = new THREE.Vector3(0, 0, 1);
      ORB.tgt.addScaledVector(right, -dx * f).addScaledVector(upv, dy * f);
      $("#follow").setAttribute("aria-pressed", "false");
    }
    if (!running) paint(diag());
  });
  el.addEventListener("wheel", e => {
    e.preventDefault();
    ORB.dist = Math.max(4, Math.min(600, ORB.dist * (1 + Math.sign(e.deltaY)*0.12)));
    if (!running) paint(diag());
  }, {passive:false});
}
function resize3D(){
  const host = $("#view");
  const w = host.clientWidth, h = host.clientHeight;
  if (!w || !h) return;
  ren.setSize(w, h); cam.aspect = w/h; cam.updateProjectionMatrix();
}
function render3D(d){
  const p = new THREE.Vector3(X[0], X[1], X[2]);
  veh.position.copy(p);
  veh.quaternion.set(X[6], X[7], X[8], X[9]);
  // 로터를 실제 회전수 n [rad/s] 로 돌린다. 방향은 rotor_directions 대로.
  for (let i = 0; i < PROPS.length; i++)
    PROPS[i].rotation.x += P.rotor_directions[i] * X[13+i] * 0.016;
  if (trailN < 4000){
    trailPos[trailN*3] = X[0]; trailPos[trailN*3+1] = X[1]; trailPos[trailN*3+2] = X[2];
    trailN++;
    trail.geometry.setDrawRange(0, trailN);
    trail.geometry.attributes.position.needsUpdate = true;
  }
  const W = P.mass * P.g;
  const fv = new THREE.Vector3(d.Fw[0], d.Fw[1], d.Fw[2]);
  if (fv.length() > 1e-3){
    arrow.visible = true; arrow.position.copy(p);
    arrow.setDirection(fv.clone().normalize());
    const L = Math.min(26, Math.max(2.5, 14*d.F/W));
    arrow.setLength(L, Math.min(3.2, L*.2), Math.min(1.6, L*.1));
  } else arrow.visible = false;

  const vv = new THREE.Vector3(X[3], X[4], X[5]);
  if (vv.length() > .5){
    velArrow.visible = true; velArrow.position.copy(p);
    velArrow.setDirection(vv.clone().normalize());
    const L = Math.min(26, Math.max(2.5, vv.length()*.22));
    velArrow.setLength(L, Math.min(3.2, L*.2), Math.min(1.6, L*.1));
  } else velArrow.visible = false;

  if (ORB.follow) ORB.tgt.copy(p);
  const ce = Math.cos(ORB.el), se = Math.sin(ORB.el);
  cam.position.set(ORB.tgt.x + ORB.dist*ce*Math.cos(ORB.az),
                   ORB.tgt.y + ORB.dist*ce*Math.sin(ORB.az),
                   ORB.tgt.z + ORB.dist*se);
  cam.lookAt(ORB.tgt);
  ren.render(scene, cam);
}

/* ══ 시계열 ═══════════════════════════════════════════════════════════ */
function plot(id, ys, ys2, color2){
  const cv = $(id);
  if (!cv._h) cv._h = +cv.getAttribute("height");
  const r = Math.min(devicePixelRatio, 2), w = cv.clientWidth, h = cv._h;
  cv.width = w*r; cv.height = h*r; cv.style.height = h + "px";
  const c = cv.getContext("2d"); c.setTransform(r,0,0,r,0,0);
  c.clearRect(0,0,w,h);
  if (ys.length < 2) return;
  let lo = Infinity, hi = -Infinity;
  for (const v of ys){ if (v<lo) lo=v; if (v>hi) hi=v; }
  if (ys2) for (const v of ys2){ if (v<lo) lo=v; if (v>hi) hi=v; }
  if (hi - lo < 1e-6){ hi = lo + 1; }
  const pad = (hi-lo)*.12; lo -= pad; hi += pad;
  const X0 = 30, Y0 = 8, Y1 = h - 16;
  const fx = i => X0 + i/(ys.length-1)*(w-X0-6);
  const fy = v => Y1 - (v-lo)/(hi-lo)*(Y1-Y0);
  c.strokeStyle = "#1e2a35"; c.lineWidth = 1;
  c.beginPath(); c.moveTo(X0, Y1+.5); c.lineTo(w-6, Y1+.5); c.stroke();
  c.font = '400 10px "IBM Plex Mono", monospace'; c.fillStyle = "#7C8A98";
  c.textAlign = "right"; c.textBaseline = "middle";
  c.fillText(hi.toFixed(0), X0-4, Y0+4);
  c.fillText(lo.toFixed(0), X0-4, Y1-2);
  const draw = (arr, col) => {
    c.strokeStyle = col; c.lineWidth = 1.8; c.beginPath();
    for (let i=0;i<arr.length;i++){ const x=fx(i), y=fy(arr[i]);
      i ? c.lineTo(x,y) : c.moveTo(x,y); }
    c.stroke();
  };
  if (ys2) draw(ys2, color2 || "#7C8A98");
  draw(ys, "#f2aa4c");
}

/* ══ 루프 ═════════════════════════════════════════════════════════════ */
let lastFrame = 0;
function tick(ts){
  requestAnimationFrame(tick);
  if (!lastFrame) lastFrame = ts;
  const dtReal = Math.min(0.05, (ts - lastFrame)/1000);
  lastFrame = ts;
  if (running){
    const rtf = +$("#rtf").value;
    let budget = dtReal * rtf;
    const w = windNow();
    const target = +$("#spd").value;
    let guard = 0;
    while (budget > 0 && guard++ < 600){
      const dv = target - cmdSpd, step = RAMP * DT;
      cmdSpd += Math.abs(dv) < step ? dv : (dv > 0 ? step : -step);
      const cmd = cmdNow();
      const u = control(X, cmd);
      const prevCmd = cmdSpd - (Math.abs(target - cmdSpd) < RAMP*DT ? 0 : 0);
      X = rk4(X, u, w, P, DT);
      ground();
      T += DT; budget -= DT;
      if (u.some(v => v >= P.n_max - 1e-6)) sat = Math.min(1, sat + .02);
      else sat = Math.max(0, sat - .01);
    }
    const d = diag();
    recAcc += dtReal * rtf;
    if (recAcc >= REC_EVERY || !REC.length){
      recAcc = 0;
      const prev = REC.length ? REC[REC.length-1].cmd : 0;
      REC.push({t:T, x:X.slice(), cmd:cmdSpd, V:d.gs, al:d.alpha, F:d.F,
                seg:segNow(d, cmd, prev)});
      if (REC.length > 6000) REC.shift();
    }
    paint(d);
  }
}
function toggleRun(){
  // 되감아 둔 상태에서 재생하면 그 지점부터 이어 난다. 뒤 기록은 버린다.
  if (!running && viewIdx !== null){
    REC.length = viewIdx + 1;
    X = REC[viewIdx].x.slice();
    T = REC[viewIdx].t;
    cmdSpd = REC[viewIdx].cmd;
    viewIdx = null;
  }
  running = !running;
  $("#go").textContent = running ? "❚❚ 정지  (Space)" : "▶ 시작  (Space)";
  lastFrame = 0;
}
function curIdx(){ return viewIdx === null ? REC.length - 1 : viewIdx; }
function drawSegs(){
  const bar = $("#segbar");
  const n = REC.length;
  if (!n){ bar.innerHTML = "<div class='lab'>기록 없음 — ▶ 를 누르세요</div>"; return; }
  let html = "", run = 1;
  for (let i = 1; i <= n; i++){
    if (i < n && REC[i].seg === REC[i-1].seg){ run++; continue; }
    const c = SEGS[REC[i-1].seg] || SEGS.ground;
    html += "<i style='flex:" + run + ";background:" + c[1] + "'></i>";
    run = 1;
  }
  const i = curIdx(), cur = SEGS[REC[i].seg] || SEGS.ground;
  const pct = n > 1 ? (i / (n - 1)) * 100 : 0;
  bar.innerHTML = html
    + "<div class='head' style='left:" + pct + "%'></div>"
    + "<div class='lab'><b>" + cur[0] + "</b> "
    + REC[i].t.toFixed(1) + " s"
    + (viewIdx !== null ? " · 되감기" : "")
    + "<span class='leg'>" + Object.values(SEGS).map(c =>
        "<span style='color:" + c[1] + "'>■</span>" + c[0]).join(" ") + "</span></div>";
}
function paint(d){
  render3D(d);
  drawSegs();
  const upto = curIdx() + 1;
  plot("#p1", REC.slice(0, upto).map(r => r.V), REC.slice(0, upto).map(r => r.cmd));
  plot("#p2", REC.slice(0, upto).map(r => r.al));
  plot("#p3", REC.slice(0, upto).map(r => r.F));
  const W = P.mass * P.g;
  const cmd = cmdNow();
  $("#hud").innerHTML =
    "t <b>" + T.toFixed(1) + "</b> s<br>" +
    "속도 <b>" + d.gs.toFixed(1) + "</b> m/s = <b>" + (d.gs*3.6).toFixed(0) + "</b> km/h<br>" +
    "대기속도 " + d.V.toFixed(1) + " m/s · α " + d.alpha.toFixed(1) + "°<br>" +
    "명령 " + cmd.spd.toFixed(1) + " m/s (목표 " + cmd.target.toFixed(0) + ")<br>" +
    "고도 " + d.alt.toFixed(1) + " m · 기울임 " + d.tilt.toFixed(1) + "°<br>" +
    "공력 " + d.F.toFixed(1) + " N (" + (100*d.F/W).toFixed(0) + "% 무게)";
  // 램프가 끝난 뒤에도 실측이 못 따라오면 그게 한계다. 램프 중의 차이는
  // 그냥 가속 중이라는 뜻이므로 경고하면 안 된다.
  const settled = Math.abs(cmd.target - cmd.spd) < 0.2;
  const gap = cmd.spd - d.gs;
  let warn = "";
  if (sat > .5) warn = "⚠ 로터가 최대 회전수에 닿았습니다 — 추력 포화";
  else if (settled && gap > 3 && T > 8)
    warn = "⚠ 명령 " + cmd.spd.toFixed(0) + " m/s 에 실측 " + d.gs.toFixed(0)
         + " m/s — 여기가 이 기체의 한계입니다";
  $("#warn").textContent = warn;
  $("#gauges").innerHTML = [
    ["속도", d.gs.toFixed(1), "m/s"], ["", (d.gs*3.6).toFixed(0), "km/h"],
    ["받음각", d.alpha.toFixed(1), "deg"], ["기울임", d.tilt.toFixed(1), "deg"],
    ["공력", d.F.toFixed(1), "N"], ["무게대비", (100*d.F/W).toFixed(0), "%"],
    ["동압", d.q_bar.toFixed(0), "Pa"], ["고도", d.alt.toFixed(0), "m"],
  ].map(([k,v,u]) => "<div><div class='k'>" + k + "</div><div class='v'>"
    + v + " <small>" + u + "</small></div></div>").join("");
}

/* ══ UI ═══════════════════════════════════════════════════════════════ */
const COEFS = [
  ["C_A0", "영받음각 축력", 0, 1.0, 0.01],
  ["C_Na", "수직력 기울기 [/rad]", 0, 15, 0.1],
  ["C_dc", "교차류 항력", 0, 12, 0.1],
  ["C_Aa2", "유도 축력", 0, 4, 0.05],
  ["x_cp", "압력중심 [m]", -0.4, 0.2, 0.01],
  ["C_mq", "피치 감쇠", -40, 0, 0.5],
  ["C_lp", "롤 감쇠", -20, 0, 0.5],
];
function buildCoefs(){
  $("#coefs").innerHTML = COEFS.map(([k, lbl, lo, hi, st]) =>
    "<div class='fld'><div class='row'><label for='c_" + k + "'>" + k
    + " <span style='color:var(--ink-3)'>" + lbl + "</span></label>"
    + "<output id='o_" + k + "'></output></div>"
    + "<input type='range' id='c_" + k + "' min='" + lo + "' max='" + hi
    + "' step='" + st + "' value='" + P[k] + "'></div>").join("");
  for (const [k] of COEFS){
    const el = $("#c_" + k);
    const upd = () => { P[k] = +el.value; $("#o_" + k).textContent = (+el.value).toFixed(2); };
    el.addEventListener("input", upd); upd();
  }
}
function initUI(){
  const err = selfCheck();
  const good = err < 1e-9;
  // 배지는 이 페이지의 숫자를 믿어도 되는지에 대한 답이라 없앨 수 없다.
  // 다만 늘 두 줄을 차지할 이유는 없어서 하나로 접고 자세한 값은 펼쳐서 본다.
  $("#badges").innerHTML =
    "<details class='vchk'><summary class='badge " + (good ? "" : "bad") + "'>"
    + "<span class='dot'></span>검증 <b>"
    + (good ? "통과" : "실패") + "</b></summary>"
    + "<div class='vbody'>"
    + "<div><span>JS ↔ 파이썬</span><b>" + err.toExponential(1) + "</b></div>"
    + "<div><span>파이썬 ↔ CasADi</span><b>"
    + D.casadi_max_diff.toExponential(1) + "</b></div>"
    + "<p>같은 물리를 세 번 구현했습니다. 이 값은 구현끼리의 최대 차이이고,"
    + " 기계정밀도(1e-15) 수준이면 셋이 같은 답을 낸다는 뜻입니다.</p>"
    + "</div></details>";
  for (const [id, fn] of [["spd", v => v + " m/s · " + (v*3.6).toFixed(0) + " km/h"],
                          ["alt", v => v + " m"],
                          ["wsp", v => v + " m/s"],
                          ["wdir", v => v + "°"],
                          ["rtf", v => "×" + v]]){
    const el = $("#" + id), out = $("#o_" + id);
    const upd = () => out.textContent = fn(+el.value);
    el.addEventListener("input", upd); upd();
  }
  $("#alt").addEventListener("input", () => { if (!running) { X[2] = +$("#alt").value; paint(diag()); } });
  buildCoefs();
  $("#def").addEventListener("click", () => {
    for (const [k] of COEFS){ P[k] = P0[k]; $("#c_"+k).value = P0[k];
      $("#o_"+k).textContent = P0[k].toFixed(2); }
  });
  $("#go").addEventListener("click", toggleRun);

  // 스페이스로 정지·재생. 슬라이더를 만지는 중에는 가로채지 않고, 버튼에
  // 포커스가 있을 때도 비워 둔다 — 버튼은 브라우저가 이미 스페이스로 누른다.
  // 둘 다 처리하면 두 번 토글되어 아무 일도 안 일어난다.
  addEventListener("keydown", e => {
    const t = e.target;
    const tag = t && t.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (e.code === "Space"){
      if (tag === "BUTTON" || tag === "SUMMARY") return;
      e.preventDefault(); toggleRun();
    } else if (e.key === "r" || e.key === "R"){
      $("#rst").click();
    } else if (e.key === "f" || e.key === "F"){
      $("#follow").click();
    }
  });
  $("#follow").addEventListener("click", e => {
    ORB.follow = !ORB.follow;
    e.target.setAttribute("aria-pressed", String(ORB.follow));
    if (!running) paint(diag());
  });
  // 시점 프리셋. 마우스로도 되지만 정해진 각도에서 보고 싶을 때가 있다.
  // '기수축'은 동체축을 따라 내려다보는 시점이라 **단면이 원으로 보이고**
  // 로터 넷이 그 둘레에 놓인 것을 확인할 수 있다 — 축대칭인지 보는 자리다.
  const VIEWS = {
    back: [-2.36, 0.30, 15], side: [-1.57, 0.12, 14],
    top:  [-2.36, 1.35, 16], nose: [0.0, 0.0, 9],
  };
  for (const b of document.querySelectorAll("[data-view]"))
    b.addEventListener("click", () => {
      const v = VIEWS[b.dataset.view];
      ORB.az = v[0]; ORB.el = v[1]; ORB.dist = v[2];
      ORB.follow = true; $("#follow").setAttribute("aria-pressed", "true");
      if (!running) paint(diag());
    });
  $("#rst").addEventListener("click", () => {
    reset(); trailN = 0; trail.geometry.setDrawRange(0,0); paint(diag());
  });
  addEventListener("resize", () => { resize3D(); paint(diag()); });
}
// 구간 띠를 타임라인으로 쓴다. 끌면 그 시점의 기록을 보여주고, 재생을 누르면
// **그 지점부터 다시 난다** (뒤 기록은 버린다). 조건을 바꿔 다시 돌려보는
// 흐름이 자연스러워진다.
function bindTimeline(){
  const bar = $("#segbar");
  let on = false;
  const pick = e => {
    if (!REC.length) return;
    const r = bar.getBoundingClientRect();
    const f = Math.max(0, Math.min(1, (e.clientX - r.left) / r.width));
    viewIdx = Math.round(f * (REC.length - 1));
    if (running){ running = false; $("#go").textContent = "▶ 시작  (Space)"; }
    X = REC[viewIdx].x.slice();
    T = REC[viewIdx].t;
    paint(diag());
  };
  bar.addEventListener("pointerdown", e => {
    on = true; bar.setPointerCapture(e.pointerId); pick(e);
  });
  bar.addEventListener("pointermove", e => { if (on) pick(e); });
  bar.addEventListener("pointerup", e => {
    on = false; bar.releasePointerCapture(e.pointerId);
  });
}

function bindGrips(){
  for (const [id, varName, side] of [["#gripL", "--colL", 1], ["#gripR", "--colR", -1]]){
    const el = $(id);
    let on = false, x0 = 0, w0 = 0;
    const col = side > 0 ? $("#colL") : $("#colR");
    el.addEventListener("pointerdown", e => {
      on = true; x0 = e.clientX; w0 = col.getBoundingClientRect().width;
      el.classList.add("on"); el.setPointerCapture(e.pointerId);
    });
    el.addEventListener("pointermove", e => {
      if (!on) return;
      const w = Math.max(190, Math.min(560, w0 + side * (e.clientX - x0)));
      document.documentElement.style.setProperty(varName, w + "px");
      try { localStorage.setItem("fd" + varName, String(w)); } catch (_) {}
      resize3D(); if (!running) paint(diag());
    });
    el.addEventListener("pointerup", e => {
      on = false; el.classList.remove("on"); el.releasePointerCapture(e.pointerId);
    });
    el.addEventListener("dblclick", () => {
      document.documentElement.style.removeProperty(varName);
      try { localStorage.removeItem("fd" + varName); } catch (_) {}
      resize3D(); if (!running) paint(diag());
    });
    // 지난번 폭을 기억한다. 못 읽어도(사생활 모드 등) 기본값으로 그냥 선다.
    try {
      const v = localStorage.getItem("fd" + varName);
      if (v) document.documentElement.style.setProperty(varName, v + "px");
    } catch (_) {}
  }
}

init3D();
bindGrips();
bindTimeline();
reset();
initUI();
paint(diag());
requestAnimationFrame(tick);
</script>
"""

if __name__ == "__main__":
    raise SystemExit(main())
