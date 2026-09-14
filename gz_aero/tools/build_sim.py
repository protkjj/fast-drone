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
  --ground:#f5f7fa; --panel:#ffffff; --panel-2:#eef1f6; --rule:#d5dbe4;
  --stage:#0d1117; --stage-2:#161c25;
  --ink:#0f141b; --ink-2:#4b5665; --ink-3:#7a8697;
  --s1:#2a78d6; --s2:#eb6834; --s3:#1baf7a;
  --ok:#0f8d61; --warn:#a87200; --bad:#c93a39; --grid:#e3e8ef;
  color-scheme:light;
}
@media (prefers-color-scheme:dark){
  :root:not([data-theme="light"]){
    --ground:#0f131a; --panel:#161b23; --panel-2:#1e242e; --rule:#2a313c;
    --stage:#090c11; --stage-2:#12171f;
    --ink:#e9eef5; --ink-2:#a6b2c2; --ink-3:#78859a;
    --s1:#3987e5; --s2:#d95926; --s3:#199e70;
    --ok:#199e70; --warn:#c98500; --bad:#e66767; --grid:#222932;
    color-scheme:dark;
  }
}
:root[data-theme="dark"]{
  --ground:#0f131a; --panel:#161b23; --panel-2:#1e242e; --rule:#2a313c;
  --stage:#090c11; --stage-2:#12171f;
  --ink:#e9eef5; --ink-2:#a6b2c2; --ink-3:#78859a;
  --s1:#3987e5; --s2:#d95926; --s3:#199e70;
  --ok:#199e70; --warn:#c98500; --bad:#e66767; --grid:#222932;
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
.badge.bad .dot{background:var(--bad)} .badge b{color:var(--ink);font-weight:600}

.body{display:grid; grid-template-columns:246px 1fr 246px; min-height:0}
@media(max-width:1100px){.body{grid-template-columns:220px 1fr}
  .col.right{grid-column:1/-1; border-left:0; border-top:1px solid var(--rule)}}
@media(max-width:760px){.body{grid-template-columns:1fr}
  .col.left{border-right:0; border-bottom:1px solid var(--rule)}}

.col{padding:14px 16px; overflow-y:auto; background:var(--panel); min-height:0}
.col.left{border-right:1px solid var(--rule)}
.col.right{border-left:1px solid var(--rule)}
.col h2{margin:0 0 12px; font:600 11px/1 "IBM Plex Mono",monospace;
  letter-spacing:.13em; text-transform:uppercase; color:var(--ink-3)}
.col h2+h2{margin-top:22px}

.fld{margin-bottom:14px}
.fld .row{display:flex; justify-content:space-between; align-items:baseline; gap:8px}
.fld label{font-size:12.5px; color:var(--ink-2)}
.fld output{font:600 14px/1 "IBM Plex Mono",monospace; color:var(--ink)}
input[type=range]{width:100%; margin:5px 0 0; accent-color:var(--s1)}
input[type=range]:focus-visible{outline:2px solid var(--s1); outline-offset:3px}

.btns{display:flex; gap:8px; margin:4px 0 16px}
.btns button{flex:1; padding:9px 8px; border:1px solid var(--rule);
  border-radius:8px; background:var(--panel-2); color:var(--ink); cursor:pointer;
  font:600 13px/1 "IBM Plex Sans",sans-serif}
.btns button.go{background:var(--s1); border-color:var(--s1); color:#fff}
.btns button:focus-visible{outline:2px solid var(--s1); outline-offset:2px}

.gauges{display:grid; grid-template-columns:1fr 1fr; gap:1px;
  background:var(--rule); border:1px solid var(--rule); border-radius:9px;
  overflow:hidden}
.gauges div{background:var(--panel-2); padding:8px 10px}
.gauges .k{font:400 10px/1.2 "IBM Plex Mono",monospace; letter-spacing:.08em;
  text-transform:uppercase; color:var(--ink-3)}
.gauges .v{font:600 16px/1.25 "IBM Plex Mono",monospace; margin-top:2px}
.gauges .v small{font-size:11px; font-weight:400; color:var(--ink-3)}
.gauges .wide{grid-column:1/-1}

.stage{display:grid; grid-template-rows:1fr 172px; min-height:0;
  background:var(--stage)}
#view{position:relative; min-height:0}
#view canvas{display:block}
.hud{position:absolute; left:14px; top:12px; pointer-events:none;
  font:400 12px/1.65 "IBM Plex Mono",monospace; color:#cfd8e6;
  text-shadow:0 1px 3px rgba(0,0,0,.8)}
.hud b{color:#fff; font-weight:600}
.viewbtns{position:absolute; right:12px; top:12px; display:flex; gap:6px}
.viewbtns button{padding:5px 10px; border:1px solid #2b3644; border-radius:7px;
  background:rgba(18,23,31,.85); color:#c6d0de; cursor:pointer;
  font:500 12px/1 "IBM Plex Sans",sans-serif}
.viewbtns button[aria-pressed=true]{border-color:#3987e5; color:#fff}
.viewbtns button:focus-visible{outline:2px solid #3987e5; outline-offset:2px}
.legend3d{position:absolute; right:12px; bottom:12px; display:flex; gap:14px;
  align-items:center; font:400 11.5px/1 "IBM Plex Mono",monospace; color:#8e9bad}
.legend3d span{display:inline-flex; align-items:center; gap:5px}
.legend3d i{width:12px; height:3px; border-radius:2px; display:inline-block}
.warnbox{position:absolute; left:14px; bottom:34px; right:14px;
  pointer-events:none; font-size:12.5px; color:#ffd9a8}
.plots{border-top:1px solid var(--rule); background:var(--stage-2);
  display:grid; grid-template-columns:1fr 1fr 1fr; gap:1px}
.plots figure{margin:0; background:var(--stage); padding:8px 10px 4px;
  min-width:0}
.plots figcaption{font:400 10.5px/1.3 "IBM Plex Mono",monospace;
  color:#8794a6; letter-spacing:.06em; text-transform:uppercase}
.plots canvas{width:100%; display:block}
@media(max-width:760px){.plots{grid-template-columns:1fr}
  .stage{grid-template-rows:340px auto}}

.note{font-size:11.5px; color:var(--ink-3); line-height:1.5; margin:10px 0 0}
.note b{color:var(--ink-2)}
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
  <div class="col left">
    <h2>비행 명령</h2>
    <div class="btns">
      <button type="button" id="go" class="go">▶ 시작</button>
      <button type="button" id="rst">↺ 초기화</button>
    </div>
    <div class="fld"><div class="row"><label for="spd">목표 속도</label>
      <output id="o_spd"></output></div>
      <input type="range" id="spd" min="0" max="120" step="1" value="83"></div>
    <div class="fld"><div class="row"><label for="alt">목표 고도</label>
      <output id="o_alt"></output></div>
      <input type="range" id="alt" min="10" max="200" step="5" value="60"></div>
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
    <p class="note">목표 속도를 올렸다 내리면 기체가 따라갑니다.
      <b>명령과 실측이 벌어지면</b> 거기가 이 기체의 한계입니다.</p>
    <h2>교차 확인</h2>
    <p class="note">기본 설정(83 m/s 목표, 무풍)에서 이 시뮬은
      <b>280 km/h</b> 에 수렴합니다. 같은 기체를 PX4 + Gazebo 로 실제로 날렸을 때는
      <b>281 km/h</b> 였습니다. 제어기도 적분기도 공력 경로도 다른 두 스택이
      <b>0.4% 안에서 일치</b>합니다 — 이 한계는 설정이 아니라 물리입니다.
      로터는 포화하지 않았으니 추력 부족도 아닙니다.</p>
    <p class="note">고도가 조금 가라앉는 것은 제어기 탓입니다.
      여기 제어기는 PX4 가 아니라 단순한 종속 루프라, 크게 기울인 동안
      고도 권한을 일부 잃습니다. 검증한 것은 <b>플랜트이지 제어기가 아닙니다.</b></p>
  </div>

  <div class="stage">
    <div id="view"><div class="hud" id="hud"></div>
      <div class="viewbtns">
        <button type="button" id="follow" aria-pressed="true">기체 따라가기</button>
        <button type="button" id="reset3d">시점 초기화</button>
      </div>
      <div class="legend3d">
        <span><i style="background:#eb6834"></i>공력</span>
        <span><i style="background:#3987e5"></i>속도</span>
        <span>끌기 = 회전 · 휠 = 확대 · Shift+끌기 = 이동</span>
      </div>
      <div class="warnbox" id="warn"></div></div>
    <div class="plots">
      <figure><figcaption>속도 m/s</figcaption><canvas id="p1" height="132"></canvas></figure>
      <figure><figcaption>받음각 deg</figcaption><canvas id="p2" height="132"></canvas></figure>
      <figure><figcaption>공력 N</figcaption><canvas id="p3" height="132"></canvas></figure>
    </div>
  </div>

  <div class="col right">
    <h2>동체 공력 계수</h2>
    <div id="coefs"></div>
    <div class="btns"><button type="button" id="def">기본값으로</button></div>
    <p class="note">계수를 바꾸면 <b>다음 스텝부터 즉시</b> 반영됩니다.
      <code>C_A0</code>를 올리면 항력이 커져 최고 속도가 떨어지고,
      <code>x_cp</code>를 0에 가깝게 하면 정적 안정이 사라집니다.</p>
    <h2>주의</h2>
    <p class="note">로터 전진비는 담겨 있지만(<code>J_max</code>) Gazebo 기본
      모터 모델에는 없습니다. 이 시뮬이 그쪽보다 보수적입니다.
      <code>α&gt;90°</code> 역류 영역은 어느 쪽도 검증되지 않았습니다.</p>
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

  const V_axial = (-wb > 0.0) ? -wb : 0.0;
  let h_net = 0.0, Frz = 0.0;
  for (let i = 0; i < p.num_rotors; i++){
    const ni = x[13 + i], di = p.rotor_directions[i], ri = p.rotor_positions[i];
    const n_rps = ni / (2.0 * Math.PI);
    const J = V_axial / (n_rps * p.D_prop + EPS);
    let fac = 1.0 - J / p.J_max; if (fac < 0.0) fac = 0.0;
    const Ti = p.k_T * ni * ni * fac, Qi = p.k_Q * ni * ni * fac;
    Frz += -Ti;
    Mx += ri[1] * (-Ti);
    My += -ri[0] * (-Ti);
    Mz += di * Qi;
    h_net += p.I_rotor * ni * di;
  }
  Mx += -om1 * h_net;
  My += om0 * h_net;

  const Fbx = Fx, Fby = Fy, Fbz = Fz + Frz;
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
const KV = 1.2, KZ = 1.0, KR = 9.0, KW = 3.2;
// 가속도 상한. 실제 PX4 시험은 0 -> 83 m/s 를 30 초에 올렸다 (2.8 m/s^2).
// 이보다 크게 잡으면 기체가 즉시 70 도씩 누워 아무것도 안 읽힌다.
const A_MAX = 12.0, RAMP = 3.0;
function control(x, cmd){
  const m = P.mass, g = P.g;
  const psi = cmd.psi;
  const vdes = [cmd.spd*Math.cos(psi), cmd.spd*Math.sin(psi),
                Math.max(-8, Math.min(8, KZ*(cmd.alt - x[2])))];
  let ax = KV*(vdes[0]-x[3]), ay = KV*(vdes[1]-x[4]), az = KV*(vdes[2]-x[5]);
  const an = Math.hypot(ax, ay);
  if (an > A_MAX){ ax *= A_MAX/an; ay *= A_MAX/an; }
  // 필요한 월드 추력 벡터
  const tx = m*ax, ty = m*ay, tz = m*(az + g);
  const tn = Math.hypot(tx, ty, tz) || 1e-9;
  // 원하는 자세: 동체 -z 가 추력 방향, 기수는 psi 쪽
  const zbx = -tx/tn, zby = -ty/tn, zbz = -tz/tn;      // 동체 z (월드)
  const cx = Math.cos(psi), cy = Math.sin(psi);
  let ybx = zby*0 - zbz*cy, yby = zbz*cx - zbx*0, ybz = zbx*cy - zby*cx;
  const yn = Math.hypot(ybx, yby, ybz) || 1e-9;
  ybx/=yn; yby/=yn; ybz/=yn;
  const xbx = yby*zbz - ybz*zby, xby = ybz*zbx - ybx*zbz, xbz = ybx*zby - yby*zbx;

  // 현재 자세
  const qx=x[6], qy=x[7], qz=x[8], qw=x[9];
  const R = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
             [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
             [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]];
  const Rd = [[xbx, ybx, zbx], [xby, yby, zby], [xbz, ybz, zbz]];
  // 오차 회전 Re = R^T Rd, 축각 추출
  const E = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i=0;i<3;i++) for (let j=0;j<3;j++){
    let s2 = 0; for (let k=0;k<3;k++) s2 += R[k][i]*Rd[k][j]; E[i][j] = s2;
  }
  const tr = E[0][0]+E[1][1]+E[2][2];
  const ang = Math.acos(Math.max(-1, Math.min(1, (tr-1)/2)));
  let ex = E[2][1]-E[1][2], ey = E[0][2]-E[2][0], ez = E[1][0]-E[0][1];
  const es = Math.hypot(ex, ey, ez);
  if (es > 1e-9){ const k2 = ang/es; ex*=k2; ey*=k2; ez*=k2; }

  const wcx = KR*ex, wcy = KR*ey, wcz = KR*ez;
  const Jx=P.Ixx, Jy=P.Iyy, Jz=P.Izz;
  const p_=x[10], q_=x[11], r_=x[12];
  const Mx = Jx*KW*(wcx-p_) + (q_*(Jz*r_) - r_*(Jy*q_));
  const My = Jy*KW*(wcy-q_) + (r_*(Jx*p_) - p_*(Jz*r_));
  const Mz = Jz*KW*(wcz-r_) + (p_*(Jy*q_) - q_*(Jx*p_));

  // 총추력: 현재 추력축에 투영
  const axw = -R[0][2], ayw = -R[1][2], azw = -R[2][2];
  let T = tx*axw + ty*ayw + tz*azw;
  if (T < 0) T = 0;

  const A = D.alloc_inv, n = [0,0,0,0];
  for (let i=0;i<4;i++){
    let Ti = A[i][0]*T + A[i][1]*Mx + A[i][2]*My + A[i][3]*Mz;
    if (Ti < 0) Ti = 0;
    let ni = Math.sqrt(Ti / P.k_T);
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
const HIST = {t:[], V:[], al:[], F:[], cmd:[]};
function cmdNow(){
  return {spd:cmdSpd, alt:+$("#alt").value, psi:0, target:+$("#spd").value};
}
function windNow(){
  const s = +$("#wsp").value, d = +$("#wdir").value * Math.PI/180;
  return [s*Math.cos(d), s*Math.sin(d), 0];
}
function reset(){
  X = D.x0.slice(); X[2] = +$("#alt").value; T = 0; sat = 0; cmdSpd = 0;
  for (const k in HIST) HIST[k].length = 0;
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
  const tilt = Math.acos(Math.max(-1,Math.min(1, -R[2][2])))*180/Math.PI;
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
// RViz 처럼 마우스로 궤도·이동·확대. OrbitControls 는 three 핵심 번들에 없어서
// 직접 쓴다 (CDN 에서 따로 받으면 막힐 수 있다).
const ORB = {az: -2.3, el: 0.38, dist: 26, follow: true,
             tgt: new THREE.Vector3()};
function init3D(){
  const host = $("#view");
  ren = new THREE.WebGLRenderer({antialias:true});
  ren.setPixelRatio(Math.min(devicePixelRatio, 2));
  host.appendChild(ren.domElement);
  scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x0d1117, 120, 900);
  cam = new THREE.PerspectiveCamera(52, 1, .5, 5000);
  cam.up.set(0, 0, 1);      // 월드는 z-up. 쿼터니언을 변환하지 않는다.
  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a2030, 1.2));
  const sun = new THREE.DirectionalLight(0xffffff, .8);
  sun.position.set(-1,-2,3); scene.add(sun);

  const g1 = new THREE.GridHelper(3000, 120, 0x39465a, 0x232d3c);
  g1.rotation.x = Math.PI/2; scene.add(g1);

  const skin = new THREE.MeshStandardMaterial({color:0x2f3945, roughness:.5, metalness:.35});
  const tip = new THREE.MeshStandardMaterial({color:0xd8402f, roughness:.4});
  const prop = new THREE.MeshStandardMaterial({color:0x18202b, roughness:.7,
                 transparent:true, opacity:.55, side:THREE.DoubleSide});

  // 동체는 **링크 +X 가 기수** (DESIGN.md 9 장에서 확정). 동체 z 는 아래,
  // 로터 추력은 동체 -z 이므로 디스크는 z 가 음수 쪽에 놓인다.
  veh = new THREE.Group();
  const fus = new THREE.Mesh(new THREE.CylinderGeometry(.075, .075, .82, 30), skin);
  fus.rotation.z = -Math.PI/2; veh.add(fus);
  const cone = new THREE.Mesh(new THREE.ConeGeometry(.075, .30, 30), tip);
  cone.rotation.z = -Math.PI/2; cone.position.x = .56; veh.add(cone);
  const tail = new THREE.Mesh(new THREE.CylinderGeometry(.062, .05, .10, 24), skin);
  tail.rotation.z = -Math.PI/2; tail.position.x = -.46; veh.add(tail);

  const ARM = 0.25 / Math.SQRT2;      // vehicle_params 의 arm/√2 = 0.1768
  for (const ph of [Math.PI/4, 3*Math.PI/4, -3*Math.PI/4, -Math.PI/4]){
    const cx = Math.cos(ph), cy = Math.sin(ph);
    const arm = new THREE.Mesh(new THREE.BoxGeometry(ARM, .022, .022), skin);
    arm.position.set(ARM/2*cx, ARM/2*cy, 0);
    arm.rotation.z = ph;
    veh.add(arm);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(.022,.022,.055,14), skin);
    hub.rotation.x = Math.PI/2; hub.position.set(ARM*cx, ARM*cy, -.028);
    veh.add(hub);
    const disk = new THREE.Mesh(new THREE.CylinderGeometry(.15,.15,.008,26), prop);
    disk.rotation.x = Math.PI/2; disk.position.set(ARM*cx, ARM*cy, -.056);
    veh.add(disk);
  }
  veh.scale.setScalar(5);
  scene.add(veh);

  arrow = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(),
                                1, 0xeb6834, .8, .35);
  scene.add(arrow);
  velArrow = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(),
                                   1, 0x3987e5, .8, .35);
  scene.add(velArrow);

  trailPos = new Float32Array(3 * 4000);
  const tg = new THREE.BufferGeometry();
  tg.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  tg.setDrawRange(0, 0);
  trail = new THREE.Line(tg, new THREE.LineBasicMaterial({color:0x3987e5}));
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
  c.strokeStyle = "#2a3444"; c.lineWidth = 1;
  c.beginPath(); c.moveTo(X0, Y1+.5); c.lineTo(w-6, Y1+.5); c.stroke();
  c.font = '400 10px "IBM Plex Mono", monospace'; c.fillStyle = "#7d8b9e";
  c.textAlign = "right"; c.textBaseline = "middle";
  c.fillText(hi.toFixed(0), X0-4, Y0+4);
  c.fillText(lo.toFixed(0), X0-4, Y1-2);
  const draw = (arr, col) => {
    c.strokeStyle = col; c.lineWidth = 1.8; c.beginPath();
    for (let i=0;i<arr.length;i++){ const x=fx(i), y=fy(arr[i]);
      i ? c.lineTo(x,y) : c.moveTo(x,y); }
    c.stroke();
  };
  if (ys2) draw(ys2, color2 || "#7d8b9e");
  draw(ys, "#3987e5");
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
      X = rk4(X, u, w, P, DT);
      T += DT; budget -= DT;
      if (u.some(v => v >= P.n_max - 1e-6)) sat = Math.min(1, sat + .02);
      else sat = Math.max(0, sat - .01);
    }
    const d = diag();
    HIST.t.push(T); HIST.V.push(d.gs); HIST.al.push(d.alpha);
    HIST.F.push(d.F); HIST.cmd.push(cmdSpd);
    if (HIST.t.length > 900) for (const k in HIST) HIST[k].shift();
    paint(d);
  }
}
function paint(d){
  render3D(d);
  plot("#p1", HIST.V, HIST.cmd);
  plot("#p2", HIST.al);
  plot("#p3", HIST.F);
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
  $("#badges").innerHTML =
    "<span class='badge " + (good ? "" : "bad") + "'><span class='dot'></span>"
    + "JS↔파이썬 <b>" + err.toExponential(1) + "</b></span>"
    + "<span class='badge'><span class='dot'></span>파이썬↔CasADi <b>"
    + D.casadi_max_diff.toExponential(1) + "</b></span>";
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
  $("#go").addEventListener("click", () => {
    running = !running;
    $("#go").textContent = running ? "❚❚ 정지" : "▶ 시작";
    lastFrame = 0;
  });
  $("#follow").addEventListener("click", e => {
    ORB.follow = !ORB.follow;
    e.target.setAttribute("aria-pressed", String(ORB.follow));
    if (!running) paint(diag());
  });
  $("#reset3d").addEventListener("click", () => {
    ORB.az = -2.3; ORB.el = 0.38; ORB.dist = 26; ORB.follow = true;
    $("#follow").setAttribute("aria-pressed", "true");
    if (!running) paint(diag());
  });
  $("#rst").addEventListener("click", () => {
    reset(); trailN = 0; trail.geometry.setDrawRange(0,0); paint(diag());
  });
  addEventListener("resize", () => { resize3D(); paint(diag()); });
}
init3D();
reset();
initUI();
paint(diag());
requestAnimationFrame(tick);
</script>
"""

if __name__ == "__main__":
    raise SystemExit(main())
