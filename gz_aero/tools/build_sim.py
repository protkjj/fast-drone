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
    ap.add_argument("--lqr", default=str(HERE / "data" / "lqr_rocket.json"))
    a = ap.parse_args()

    ref_path = pathlib.Path(a.ref)
    if not ref_path.is_file():
        print(f"기준값이 없습니다: {ref_path}\n"
              "  먼저 python3 gz_aero/tools/gen_sim_reference.py 를 돌리세요.",
              file=sys.stderr)
        return 1
    ref = json.loads(ref_path.read_text(encoding="utf-8"))

    lqr_path = pathlib.Path(a.lqr)
    if not lqr_path.is_file():
        print(f"LQR 게인표가 없습니다: {lqr_path}\n"
              "  먼저 python3 gz_aero/tools/gen_lqr_table.py 를 돌리세요.",
              file=sys.stderr)
        return 1
    lqr = json.loads(lqr_path.read_text(encoding="utf-8"))

    html = (TEMPLATE
            .replace("%%DATA%%", json.dumps(ref, separators=(",", ":")))
            .replace("%%LQR%%", json.dumps(lqr, separators=(",", ":"))))
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

/* ⚠ 열을 안 정하면 암묵 열이 auto(=내용 크기) 라 격자가 뷰포트를 넘어 자란다.
   안쪽 칸을 아무리 minmax(0,1fr) 로 해도 바깥이 늘어나면 소용이 없다 —
   오른쪽 칸을 넓히면 화면 밖으로 밀려났다. */
.app{display:grid; grid-template-rows:auto 1fr; grid-template-columns:minmax(0,1fr);
  width:100%; max-width:100vw; overflow-x:hidden;
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
/* ⚠ 가운데를 1fr 로 두면 안 된다. 1fr 은 minmax(auto, 1fr) 이고 auto 최소폭이
   내용(3D 캔버스 + 3단 그래프)의 최소폭이라 바닥이 생긴다. 그러면 옆 칸을
   넓힐 때 가운데가 줄지 못하고 **격자가 창 밖으로 넘친다** — 실제로 오른쪽
   칸이 창 밖으로 342 px 까지 밀려났다. minmax(0,1fr) + min-width:0 로 푼다. */
.body{display:grid; min-height:0; overflow:hidden;
  grid-template-columns:var(--colL,268px) 5px minmax(0,1fr) 5px var(--colR,262px)}
.grip{cursor:col-resize; background:var(--rule); position:relative}
.grip::after{content:""; position:absolute; inset:0 -4px}
.grip:hover, .grip.on{background:var(--accent)}
@media(max-width:900px){.body{grid-template-columns:1fr}
  .grip{display:none}
  .col{border:0; border-bottom:1px solid var(--rule)}}

.col{padding:14px 16px; overflow-y:auto; background:var(--panel);
  min-height:0; min-width:0}
.col h2{margin:0 0 12px; font:600 11px/1 "IBM Plex Mono",monospace;
  letter-spacing:.13em; text-transform:uppercase; color:var(--ink-3)}
.col h2+h2{margin-top:22px}

.fld{margin-bottom:14px}
.fld .row{display:flex; justify-content:space-between; align-items:baseline; gap:8px}
.fld label{font-size:12.5px; color:var(--ink-2)}
.fld output{font:600 14px/1 "IBM Plex Mono",monospace; color:var(--ink)}
input[type=range]{width:100%; margin:5px 0 0; accent-color:var(--accent)}
select{width:100%; margin-top:5px; padding:7px 8px; border:1px solid var(--rule);
  border-radius:7px; background:var(--panel-2); color:var(--ink);
  font:500 13px/1.2 "IBM Plex Sans",sans-serif}
select:focus-visible{outline:2px solid var(--accent); outline-offset:2px}
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
  min-width:0; overflow:hidden; background:var(--stage)}
#view{position:relative; min-height:0; min-width:0; overflow:hidden}
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
.score{position:absolute; left:14px; top:14px; width:320px; z-index:6;
  background:rgba(16,24,32,.96); border:1px solid var(--rule); border-radius:10px;
  padding:14px 16px 12px; box-shadow:0 14px 34px rgba(0,0,0,.55)}
.score .hd{display:flex; justify-content:space-between; align-items:center;
  margin-bottom:10px; font:600 13px/1 "IBM Plex Sans",sans-serif; color:var(--ink)}
.score .hd button{border:0; background:none; color:var(--ink-3); cursor:pointer;
  font-size:14px; padding:2px 4px}
.score table{width:100%; border-collapse:collapse; font-size:12.5px}
.score td{padding:4px 0; border-bottom:1px solid rgba(255,255,255,.06)}
.score td:first-child{color:var(--ink-3)}
.score td:last-child{text-align:right; font-family:"IBM Plex Mono",monospace;
  font-variant-numeric:tabular-nums; color:var(--ink)}
.score .ok{color:var(--ok)} .score .bad{color:var(--bad)} .score .warn{color:var(--warn)}
.score p{margin:9px 0 0; font-size:11.5px; color:var(--ink-3); line-height:1.55}
.score p.bad{color:var(--bad)} .score p.warn{color:var(--warn)}
/* 구간별 표는 값 열이 여럿이라 마지막 열만 우측정렬하는 규칙을 덮어쓴다 */
.score table.segtab{margin-top:10px}
.score table.segtab th{font-size:10px; letter-spacing:.04em; color:var(--ink-3);
  text-align:right; padding:3px 0 4px; font-weight:500; text-transform:uppercase}
.score table.segtab th:first-child{text-align:left}
.score table.segtab td{text-align:right; font-family:"IBM Plex Mono",monospace;
  font-variant-numeric:tabular-nums; color:var(--ink); padding:3px 0}
.score table.segtab td:first-child{text-align:left; color:var(--ink-3);
  font-family:inherit}
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
  display:grid; grid-template-columns:repeat(3, minmax(0,1fr)); gap:1px;
  min-width:0}
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
    <div class="fld"><label for="ctrl">제어기</label>
      <select id="ctrl"></select>
      <p class="note" id="ctrlNote" style="margin-top:6px"></p></div>
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
        <button type="button" id="scoreBtn">채점</button>
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
      <div class="warnbox" id="warn"></div>
      <div class="score" id="score" hidden>
        <div class="hd"><b>미션 채점</b>
          <button type="button" id="scoreClose" aria-label="닫기">✕</button></div>
        <canvas id="scoreCv" height="96"></canvas>
        <div id="scoreBody"></div>
        <div class="exp">
          <button type="button" id="expWin">별창으로 크게</button>
          <button type="button" id="expCsv">CSV 저장</button>
          <button type="button" id="expCopy">값 복사</button>
        </div>
      </div></div>
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
    // 반토크를 +d_i*Q_i 로 쓰므로 로터는 -d_i 로 돈다 -> h = -I_r*sum(d_i*n_i).
    // dynamics.py / gen_sim_reference.py 와 같은 부호·같은 연산 순서.
    h_net -= p.I_rotor * ni * di;
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
  // 기준 구간들을 같은 입력으로 다시 풀어 검문소마다 대조한다.
  //
  // 예전에는 구간이 하나였고, 그 궤적의 실측 범위가 대기속도 3.0~3.2 m/s,
  // fac 0.999~1.000 이었다 — **전진비 분기가 한 번도 실행되지 않았다.**
  // 그래서 할당이 전진비를 무시하던 버그가 이 배지를 그대로 통과했다.
  // 지금은 6 구간이 대기속도 0~76 m/s, 받음각 0~160도, |ω| 0~178 rad/s,
  // fac 0.000~1.000 을 훑는다.
  let worst = 0;
  for (const sg of D.segs){
    let x = sg.x0.slice();
    for (let k = 1; k <= D.steps; k++){
      x = rk4(x, sg.u, sg.w, P0, D.dt);
      if (k % D.every === 0){
        const ref = sg.checks[k / D.every - 1];
        for (let i = 0; i < NX; i++) worst = Math.max(worst, Math.abs(x[i] - ref[i]));
      }
    }
  }
  return worst;
}

/* ══ 오차상태 LQR (게인표) ═════════════════════════════════════════════
   브라우저에는 solver 가 없다. ARE 도 CasADi 야코비안도 런타임에 못 돈다.
   그런데 LQR 은 게인이 **설계 시점에 한 번** 정해지는 제어기라, 속도별로
   미리 풀어 표로 넘기면 런타임에는 보간과 행렬곱만 남는다. 그래서 이 저장소의
   최적제어 자산 중 브라우저로 옮길 수 있는 유일한 것이다.
   표는 gz_aero/tools/gen_lqr_table.py 가 굽는다 (2.5 m/s 간격, 21 점).

   오차상태 14D:  [δz, δv(3), δφ(3), δω(3), δn(4)]
   제어법:        u = u_trim − K_r·δx,  그리고 [n_min, n_max] 로 자른다
   controller.py 의 LQRController 와 같다. 고도만 트림의 z 가 아니라
   **명령 고도**를 기준으로 삼는다 (트림은 위치를 정하지 않는다).

   ★ 표는 0~50 m/s (180 km/h) 까지다. 로켓 배치는 52 m/s 위에 정상비행
     트림이 아예 없다 — 후방 로터가 음추력을 요구한다. 그 위에서는 마지막
     게인을 유지하는데, 이건 '빠르게 못 난다' 를 숨기는 게 아니라 기체에
     그 속도의 평형점이 없다는 사실을 그대로 드러내는 것이다.              */
const LQ = %%LQR%%;

function lqrPick(V){
  const R = LQ.rows, n = R.length;
  if (V <= R[0].V) return [R[0].x, R[0].u, R[0].K];
  if (V >= R[n-1].V) return [R[n-1].x, R[n-1].u, R[n-1].K];
  let i = 0;
  while (i + 1 < n && R[i+1].V < V) i++;
  const a = R[i], b = R[i+1], t = (V - a.V) / (b.V - a.V);
  const x = new Array(17);
  for (let k = 0; k < 17; k++) x[k] = a.x[k] + t*(b.x[k] - a.x[k]);
  // 쿼터니언은 선형 보간 후 정규화. 격자가 2.5 m/s 라 이웃 사이 자세차가
  // 몇 도뿐이라 slerp 와 사실상 같다. gen_lqr_table.py 의 interp() 와 동일.
  const qn = Math.hypot(x[6], x[7], x[8], x[9]);
  if (qn > 1e-12) for (let k = 6; k < 10; k++) x[k] /= qn;
  const u = new Array(4);
  for (let k = 0; k < 4; k++) u[k] = a.u[k] + t*(b.u[k] - a.u[k]);
  const K = new Array(4);
  for (let r = 0; r < 4; r++){
    K[r] = new Array(14);
    for (let c = 0; c < 14; c++) K[r][c] = a.K[r][c] + t*(b.K[r][c] - a.K[r][c]);
  }
  return [x, u, K];
}
function qmul(p, q){   // Hamilton, scalar-last [x,y,z,w]
  return [p[3]*q[0] + p[0]*q[3] + p[1]*q[2] - p[2]*q[1],
          p[3]*q[1] - p[0]*q[2] + p[1]*q[3] + p[2]*q[0],
          p[3]*q[2] + p[0]*q[1] - p[1]*q[0] + p[2]*q[3],
          p[3]*q[3] - p[0]*q[0] - p[1]*q[1] - p[2]*q[2]];
}
function lqrLaw(V, x, zref){
  const [xt, ut, K] = lqrPick(V);
  const qti = [-xt[6], -xt[7], -xt[8], xt[9]];
  let dq = qmul(qti, [x[6], x[7], x[8], x[9]]);
  // 최단경로 보정. 이 기체는 부호 경계가 정상 비행자세 근처라, 안 하면
  // δφ 가 반전되어 정피드백이 된다 (controller.py 의 같은 처리).
  if (dq[3] < 0) dq = [-dq[0], -dq[1], -dq[2], -dq[3]];
  const dx = [x[2] - zref,
              x[3]-xt[3], x[4]-xt[4], x[5]-xt[5],
              2*dq[0], 2*dq[1], 2*dq[2],
              x[10]-xt[10], x[11]-xt[11], x[12]-xt[12],
              x[13]-xt[13], x[14]-xt[14], x[15]-xt[15], x[16]-xt[16]];
  const out = new Array(4);
  for (let r = 0; r < 4; r++){
    let s = 0;
    for (let c = 0; c < 14; c++) s += K[r][c]*dx[c];
    let v = ut[r] - s;
    out[r] = v < LQ.n_min ? LQ.n_min : (v > LQ.n_max ? LQ.n_max : v);
  }
  return out;
}
// 표를 제대로 읽었는지, 보간까지 파이썬과 같은지 페이지에서 확인한다.
// 샘플 절반은 격자 사이에서 뽑혀 있다.
function lqrCheck(){
  let worst = 0;
  for (const sm of LQ.samples){
    const got = lqrLaw(sm.V, sm.x, sm.zref);
    for (let i = 0; i < 4; i++) worst = Math.max(worst, Math.abs(got[i] - sm.u[i]));
  }
  return worst;
}
// 고도 기준을 한 번에 얼마나 멀리 둘지. LQR 은 트림 **주위**의 국소
// 조정기라 선형영역을 벗어난 오차를 주면 안 된다. 지상 0 m 에서 목표
// 200 m 를 그대로 주면 δz = -200 이라 명령이 폭주해 포화율 99 %,
// |ω| 72 로 추락한다 (실측). NMPC 가 겪은 것과 같은 종류의 실수다.
let LQR_ZCAP = 10.0;
function controlLQR(x, cmd){
  // 스케줄 속도는 **명령 속도**(램프된 값)로 잡는다. 현재 속도로 잡으면
  // 트림이 곧 현재 상태라 속도 오차가 0 이 되어 가속 명령이 안 나온다.
  // cmdSpd 자체가 RAMP 로 천천히 오르므로 '아직 느린데 80도 자세를 요구'
  // 하는 문제도 안 생긴다 (종속 루프가 겪었던 그 함정).
  // 표가 85 m/s 까지 열려 명령 속도를 그대로 써도 되지만, 아직 느린데
  // 순항 자세를 요구하면 고도 권한을 잃는다. 현재 속도로 제한한다.
  const V = Math.min(Math.hypot(x[3], x[4]) + 5.0, cmd.spd, LQ.V_max_table);
  // 움직이는 고도 기준. 기체가 오르면 기준도 같이 오른다.
  const dz = cmd.alt - x[2];
  const zr = x[2] + (dz > LQR_ZCAP ? LQR_ZCAP : (dz < -LQR_ZCAP ? -LQR_ZCAP : dz));
  return lqrLaw(V, x, zr);
}

/* ══ 가상 NMPC ════════════════════════════════════════════════════════
   hybrid_comparison.py 의 VirtualNMPC 를 옮긴 것이다. 입력이 [T, ν_ω] 이고
   ω̇ = ν_ω 라 자세 동역학이 곧 입력이다. 그래서 상태를 앞으로 굴려 소거하면
   **상자제약만 남는 문제**가 되어 브라우저에서도 풀 수 있다.

   IPOPT 를 옮기지 않았다. 압축 + 투영경사 + 되추적 선탐색으로 푼다. 지평선을
   8 로 줄였고(파이썬은 20) 20 Hz 로만 다시 푼다. 같은 해를 준다고 주장하지
   않는다 — 구조가 같고 같은 비용을 줄일 뿐이다.                              */
const NV = 13, NUV = 4;
const NMPC = {N:20, dt:0.08, rate:0.05, iters:40,
              Qv:[5,5,10], Qz:20, Qw:1, R:[1e-5,1e-3,1e-3,1e-3],
              // Rdu[0] 를 1e-4 에서 1e-3 으로. 700 N 변화에 벌점이 49 뿐이라
              // 고도항(수만) 앞에서 없는 것과 같았다. 0.05 까지 올려 봤더니
              // 반대로 T=0 에 갇혔다 — 한 번 0 이 되면 올리는 데 0.05*T^2 이
              // 들어 스스로 덫이 된다. 실제로 그 설정에서 추락했다.
              Rdu:[1e-3,0.01,0.01,0.01], nuMax:100};
let nmpcU = null, nmpcLast = -1e9, nmpcOut = null, nmpcApplied = null;

function vXdot(x, u, p){
  const qx=x[6], qy=x[7], qz=x[8], qw=x[9];
  const R00=1-2*(qy*qy+qz*qz), R01=2*(qx*qy-qz*qw), R02=2*(qx*qz+qy*qw);
  const R10=2*(qx*qy+qz*qw), R11=1-2*(qx*qx+qz*qz), R12=2*(qy*qz-qx*qw);
  const R20=2*(qx*qz-qy*qw), R21=2*(qy*qz+qx*qw), R22=1-2*(qx*qx+qy*qy);
  const ub=R00*x[3]+R10*x[4]+R20*x[5];
  const vb=R01*x[3]+R11*x[4]+R21*x[5];
  const wb=R02*x[3]+R12*x[4]+R22*x[5];
  const V_sq = ub*ub+vb*vb+wb*wb+EPS, V_cf = Math.sqrt(vb*vb+wb*wb+EPS);
  const fac = 0.5*p.rho*p.S_ref*(p.C_Na*ub + p.C_dc*V_cf);
  let Fx = -0.5*p.rho*V_sq*p.S_ref*(p.C_A0 + p.C_Aa2*(vb*vb+wb*wb)/V_sq);
  let Fy = -fac*vb, Fz = -fac*wb;
  // 추력축. 로켓형은 동체 +x, 평면형은 동체 -z.
  if (p.thrust_axis === "x") Fx += u[0]; else Fz += -u[0];
  const m = p.mass;
  const pw=x[10], qq=x[11], rr=x[12];
  let qd0=0.5*(qw*pw - qz*qq + qy*rr), qd1=0.5*(qz*pw + qw*qq - qx*rr);
  let qd2=0.5*(-qy*pw + qx*qq + qw*rr), qd3=0.5*(-qx*pw - qy*qq - qz*rr);
  const nrm = qx*qx+qy*qy+qz*qz+qw*qw-1;
  qd0-=nrm*qx; qd1-=nrm*qy; qd2-=nrm*qz; qd3-=nrm*qw;
  return [x[3], x[4], x[5],
          (R00*Fx+R01*Fy+R02*Fz)/m, (R10*Fx+R11*Fy+R12*Fz)/m,
          -p.g + (R20*Fx+R21*Fy+R22*Fz)/m,
          qd0, qd1, qd2, qd3, u[1], u[2], u[3]];
}
// ★ 큰 스텝은 쪼갠다. 등급 격자의 끝은 0.30 s 인데 그 크기로 RK4 를 한 번에
//   밟으면 자세 동역학이 터진다 — |ω| 가 1008 rad/s 까지 갔다. 그건 제어가
//   아니라 **예측 적분기가 발산한 것**이고, 그러면 기울기가 쓰레기가 되어
//   솔버가 아무리 좋아도 소용이 없다.
//   지평선은 길게 두고 적분 보폭만 0.05 s 이하로 묶는다. RTI 가 4~7 ms 라
//   여유가 있다.
const VSTEP_MAX_H = 0.05;
function vStep(x, u, dt, p){
  const nsub = dt > VSTEP_MAX_H ? Math.ceil(dt / VSTEP_MAX_H) : 1;
  if (nsub > 1){
    const h = dt / nsub;
    let y = x;
    for (let i = 0; i < nsub; i++) y = vStep1(y, u, h, p);
    return y;
  }
  return vStep1(x, u, dt, p);
}
function vStep1(x, u, dt, p){
  const k1=vXdot(x,u,p);
  const a=x.map((v,i)=>v+0.5*dt*k1[i]), k2=vXdot(a,u,p);
  const b=x.map((v,i)=>v+0.5*dt*k2[i]), k3=vXdot(b,u,p);
  const c=x.map((v,i)=>v+dt*k3[i]),     k4=vXdot(c,u,p);
  const y=new Array(NV);
  for(let i=0;i<NV;i++) y[i]=x[i]+(dt/6)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
  const qn=Math.hypot(y[6],y[7],y[8],y[9]);
  if (qn>1e-10) for(let i=6;i<10;i++) y[i]/=qn;
  return y;
}
// uLast = 직전에 실제로 낸 입력. 이걸 안 주고 uref(호버)로 두면 풀이 사이에
// 연속성이 없어서 첫 수가 매번 0 과 상한을 오간다 — 실제로 추력이 0 <-> 767 로
// 떨었다. MPC 는 "지금 내고 있는 것" 에서 얼마나 움직이는지를 벌해야 한다.
function nmpcCost(x0, U, vref, zref, uref, uLast){
  const C = NMPC; let J = 0, x = x0, uPrev = uLast || uref;
  for (let k=0;k<C.N;k++){
    const u = U.slice(k*NUV, k*NUV+NUV);
    x = vStep(x, u, C.dt, P);
    const ev=[x[3]-vref[0], x[4]-vref[1], x[5]-vref[2]], ez=x[2]-zref;
    let st = C.Qv[0]*ev[0]*ev[0] + C.Qv[1]*ev[1]*ev[1] + C.Qv[2]*ev[2]*ev[2]
           + C.Qz*ez*ez
           + C.Qw*(x[10]*x[10]+x[11]*x[11]+x[12]*x[12]);
    for (let i=0;i<NUV;i++){
      const du=u[i]-uPrev[i], e=u[i]-uref[i];
      st += C.R[i]*e*e + C.Rdu[i]*du*du;
    }
    J += st; uPrev = u;
    if (k === C.N-1){
      J += 10*(C.Qv[0]*ev[0]*ev[0] + C.Qv[1]*ev[1]*ev[1] + C.Qv[2]*ev[2]*ev[2])
         + 10*C.Qz*ez*ez;
    }
  }
  return J;
}
function nmpcSolve(x0, vref, zref){
  const C = NMPC, n = C.N*NUV, uL = nmpcApplied;
  const Tmax = 4*P.k_T*P.n_max*P.n_max, Tref = P.mass*P.g;
  const uref = [Tref, 0, 0, 0];
  const lo = [0, -C.nuMax, -C.nuMax, -C.nuMax];
  const hi = [Tmax, C.nuMax, C.nuMax, C.nuMax];
  if (!nmpcU || nmpcU.length !== n){
    nmpcU = new Array(n);
    for (let k=0;k<C.N;k++) for (let i=0;i<NUV;i++) nmpcU[k*NUV+i]=uref[i];
  } else {                       // 워밍 스타트: 한 칸 밀고 마지막을 복제
    for (let k=0;k<C.N-1;k++) for (let i=0;i<NUV;i++)
      nmpcU[k*NUV+i] = nmpcU[(k+1)*NUV+i];
  }
  const clamp = U => { for (let k=0;k<C.N;k++) for (let i=0;i<NUV;i++){
      const j=k*NUV+i; U[j] = U[j]<lo[i]?lo[i]:(U[j]>hi[i]?hi[i]:U[j]); } };
  clamp(nmpcU);
  // 스케일. T 는 수십~수백 N, ν 는 수십 rad/s^2 라 한 걸음 크기가 다르다.
  const sc = []; for (let k=0;k<C.N;k++) sc.push(Tmax, C.nuMax, C.nuMax, C.nuMax);
  let J0 = nmpcCost(x0, nmpcU, vref, zref, uref, uL);
  let step = 0.08;
  for (let it=0; it<C.iters; it++){
    const g = new Array(n);
    for (let j=0;j<n;j++){
      const h = sc[j]*1e-4, save = nmpcU[j];
      nmpcU[j] = save + h;
      g[j] = (nmpcCost(x0, nmpcU, vref, zref, uref, uL) - J0) / h;
      nmpcU[j] = save;
    }
    let gn = 0; for (let j=0;j<n;j++) gn += (g[j]*sc[j])**2;
    gn = Math.sqrt(gn); if (!(gn > 1e-12)) break;
    let ok = false;
    for (let ls=0; ls<6; ls++){
      const U2 = nmpcU.slice();
      for (let j=0;j<n;j++) U2[j] -= step*sc[j]*sc[j]*g[j]/gn;
      clamp(U2);
      const J2 = nmpcCost(x0, U2, vref, zref, uref, uL);
      if (J2 < J0){ nmpcU = U2; J0 = J2; step *= 1.3; ok = true; break; }
      step *= 0.4;
    }
    if (!ok) break;
  }
  nmpcApplied = nmpcU.slice(0, NUV);
  return nmpcApplied.slice();
}

/* ══ SQP-RTI ═══════════════════════════════════════════════════════════
   실시간 반복(Real-Time Iteration). 매 제어 주기에 SQP 를 **딱 한 번** 돈다.
   수렴할 때까지 돌리지 않고, 직전 해 주변에서 선형화한 QP 를 한 번 풀어
   바로 적용한다. 다음 주기에 상태가 갱신되면 거기서 또 한 번 돈다.

   왜 이걸 쓰나. 투영경사는 한 번 푸는 데 150 ms 인데 20 Hz 예산이 50 ms 다.
   그 시간의 거의 전부가 유한차분 기울기다 — 비용평가 81 회 x 20 스텝 x
   RK4 4 단 = 259200 회의 미분방정식 평가. RTI 는 **한 스텝짜리** 야코비안만
   있으면 되어 (13 상태 + 4 입력) x 20 스텝 x 4 단 = 1360 회로 끝난다.

   구조:
     1) 공칭 궤적을 한 번 굴린다.
     2) 스텝마다 A_k = dx_{k+1}/dx_k, B_k = dx_{k+1}/du_k 를 유한차분으로.
     3) 압축 — S[k][j] = dx_{k+1}/du_j 를 재귀로 쌓아 상태를 소거한다.
        그러면 상자제약만 남은 QP 가 된다 (등식제약이 사라진다).
     4) 가우스-뉴턴 헤시안. 비용이 상태·입력에 대해 이차식이라 이게 곧 정확한
        헤시안이다 (선형화된 모델 기준).
     5) 투영 뉴턴으로 상자 QP 를 푼다. 경계에 붙은 변수를 빼고 자유변수만
        촐레스키로 푼 뒤, 투영 선탐색으로 받는다.
   ------------------------------------------------------------------ */
/* 비용함수를 다시 짰다. 솔버를 25 배 빠르게 만들고 나니 진짜 병목이
   드러났기 때문이다 — 제대로 풀린 해가 각가속도 한계에 계속 붙었고,
   가중치 12 조합 중 고도를 지킨 것이 하나뿐이었다. 원인이 셋이었다.

   (1) **자세 기준이 없었다.** 비용에 Qv(속도)·Qz(고도)·Qw(각속도)뿐이라
       자세를 붙잡는 것이 아무것도 없었다. nu_roll 이 어떤 반복수에서도
       0.01 이었던 이유다 — 솔버를 아무리 좋게 해도 롤은 안 잡힌다.
       고치는 법이 마침 싸다: **LQR 게인표에 속도별 트림 자세가 이미 들어
       있다.** 쿼터니언은 상태(6~9)라 지금의 대각 가중치 기계에 그대로
       들어간다. |q - q_ref|^2 은 작은 오차에서 (Δφ/2)^2 이라 자세 오차의
       제곱과 같다. 이 한 항이 자세와 롤을 같이 잡는다.

   (2) **지평선이 짧았다.** N*dt = 1.6 s 인데 가속은 30 s 짜리다. 균일
       격자로 늘리면 앞쪽이 거칠어져 적용할 첫 수가 나빠진다. 그래서
       **등급 격자** — 앞은 30 ms 로 촘촘하고 뒤로 갈수록 벌어져(1.18 배,
       상한 0.30 s) 같은 20 스텝으로 약 4 s 를 본다.

   (3) **nu 상한이 내부루프가 낼 수 있는 값보다 컸다.** INDI 증분 한계가
       J*25 이므로 25 rad/s^2 인데 상자는 100(뒤에 40)이었다. NMPC 가 지키지
       못할 약속을 하고 있었던 셈이다. 메모리의 "FC/CC 분리" 가 정한 인터페이스
       계약이 곧 이 값이다. 25 로 맞춘다.                                   */
const RTI = {N:20, dt0:0.03, dtGrow:1.18, dtMax:0.30, rate:0.05,
             Qv:[5,5,10], Qz:20, Qw:20, Qq:400, R:[1e-5,0.01,0.01,0.01],
             Rdu:[1e-3,0.01,0.01,0.01], nuMax:25,
             reg:1e-6,        // 촐레스키 정칙화
             asIters:4};      // 능동집합 갱신 횟수
// 등급 시간격자. 앞은 촘촘하고 뒤는 벌어진다.
RTI.dts = (() => {
  const a = []; let d = RTI.dt0;
  for (let k = 0; k < RTI.N; k++){ a.push(Math.min(d, RTI.dtMax)); d *= RTI.dtGrow; }
  return a;
})();
RTI.horizon = RTI.dts.reduce((x, y) => x + y, 0);
// 자세(6~9)가 들어와 7 -> 11 개가 됐다.
const RTI_WIDX = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
let rtiU = null, rtiLast = -1e9, rtiOut = null, rtiApplied = null, rtiStat = null;
let vTrimRef = 0;   // 자세 기준을 뽑을 속도 (controlHybrid 가 넣는다)
let ROLL_HOLD_OUTSIDE = true;   // A/B 로 재보려고 let
// INDI 증분 한계 [N·m]. 권한의 일정 비율. 아래에서 기체 값으로 채운다.
// 스윕으로 고른 값. 통과 구간이 0.15~0.18 이고 0.12·0.22 는 실패한다.
let INDI_FRAC = 0.16;
let INDI_LIM = [0.5, 17.5, 17.5];
// 권한을 기체 값에서 한 번 계산해 둔다. control() 과 controlHybrid() 가 같이 쓴다.
function setIndiLim(){
  const Tm = P.k_T * P.n_max * P.n_max;
  const rollAuth  = 4 * P.k_Q * P.n_max * P.n_max;              // 로터 항력토크
  const pitchAuth = 2 * Math.abs(P.rotor_positions[0][2]) * Tm * 2;  // 차동추력
  INDI_LIM = [INDI_FRAC*rollAuth, INDI_FRAC*pitchAuth, INDI_FRAC*pitchAuth];
}
setIndiLim();

// 자유변수만 뽑아 촐레스키로 푼다. H 는 대칭 양정. 실패하면 null.
function cholSolve(H, b, m){
  const L = new Float64Array(m*m);
  for (let i=0;i<m;i++){
    for (let j=0;j<=i;j++){
      let s = H[i*m+j];
      for (let k=0;k<j;k++) s -= L[i*m+k]*L[j*m+k];
      if (i === j){
        if (s <= 0) return null;
        L[i*m+i] = Math.sqrt(s);
      } else {
        L[i*m+j] = s / L[j*m+j];
      }
    }
  }
  const y = new Float64Array(m);
  for (let i=0;i<m;i++){
    let s = b[i];
    for (let k=0;k<i;k++) s -= L[i*m+k]*y[k];
    y[i] = s / L[i*m+i];
  }
  const z = new Float64Array(m);
  for (let i=m-1;i>=0;i--){
    let s = y[i];
    for (let k=i+1;k<m;k++) s -= L[k*m+i]*z[k];
    z[i] = s / L[i*m+i];
  }
  return z;
}

function rtiSolve(x0, vref, zref){
  const t_start = (typeof performance !== "undefined") ? performance.now() : 0;
  const C = RTI, N = C.N, m = NUV, n = N*m, NS = NV;
  const Tmax = 4*P.k_T*P.n_max*P.n_max;
  const uref = [P.mass*P.g, 0, 0, 0];
  const lo = [0, -C.nuMax, -C.nuMax, -C.nuMax];
  const hi = [Tmax, C.nuMax, C.nuMax, C.nuMax];
  const uL = rtiApplied || uref;

  // 워밍스타트. 한 칸 밀고 마지막은 그대로 둔다.
  if (!rtiU || rtiU.length !== n){
    rtiU = new Float64Array(n);
    for (let k=0;k<N;k++) for (let i=0;i<m;i++) rtiU[k*m+i] = uref[i];
  } else {
    for (let k=0;k<N-1;k++) for (let i=0;i<m;i++) rtiU[k*m+i] = rtiU[(k+1)*m+i];
  }
  const clampU = U => { for (let k=0;k<N;k++) for (let i=0;i<m;i++){
    const j=k*m+i; if (U[j]<lo[i]) U[j]=lo[i]; else if (U[j]>hi[i]) U[j]=hi[i]; } };
  clampU(rtiU);

  // 1) 공칭 궤적
  const xs = new Array(N+1); xs[0] = x0.slice();
  const uk = new Array(m);
  for (let k=0;k<N;k++){
    for (let i=0;i<m;i++) uk[i] = rtiU[k*m+i];
    xs[k+1] = vStep(xs[k], uk, C.dts[k], P);
  }
  // 자세 기준. 명령 속도의 트림 자세를 그대로 쓴다 — 롤까지 그 안에 있다.
  const qr0 = lqrPick(Math.min(vTrimRef, LQ.V_max_table))[0];
  const qref = [qr0[6], qr0[7], qr0[8], qr0[9]];

  // 2) 한 스텝 야코비안. 전진차분이면 충분하다 — RTI 는 어차피 한 번만 돈다.
  const A = new Array(N), B = new Array(N);
  for (let k=0;k<N;k++){
    for (let i=0;i<m;i++) uk[i] = rtiU[k*m+i];
    const base = xs[k+1];
    const Ak = new Float64Array(NS*NS), Bk = new Float64Array(NS*m);
    for (let c=0;c<NS;c++){
      const h = 1e-6*(1 + Math.abs(xs[k][c]));
      const xp = xs[k].slice(); xp[c] += h;
      const yp = vStep(xp, uk, C.dts[k], P);
      for (let r=0;r<NS;r++) Ak[r*NS+c] = (yp[r]-base[r])/h;
    }
    for (let c=0;c<m;c++){
      const sc = (c===0) ? Tmax : C.nuMax;
      const h = 1e-6*sc;
      const up = uk.slice(); up[c] += h;
      const yp = vStep(xs[k], up, C.dts[k], P);
      for (let r=0;r<NS;r++) Bk[r*m+c] = (yp[r]-base[r])/h;
    }
    A[k] = Ak; B[k] = Bk;
  }

  // 3) 압축. S[k][j] = dx_{k+1}/du_j  (j <= k). 상태가 사라지고 상자제약만 남는다.
  const S = new Array(N);
  for (let k=0;k<N;k++){
    S[k] = new Array(k+1);
    S[k][k] = B[k];
    for (let j=0;j<k;j++){
      const prev = S[k-1][j], cur = new Float64Array(NS*m), Ak = A[k];
      for (let r=0;r<NS;r++) for (let c=0;c<m;c++){
        let s = 0;
        for (let q=0;q<NS;q++) s += Ak[r*NS+q]*prev[q*m+c];
        cur[r*m+c] = s;
      }
      S[k][j] = cur;
    }
  }

  // 4) 가우스-뉴턴 헤시안과 기울기. 비용이 이차식이라 이게 정확한 헤시안이다.
  //
  //   ★ 가중치가 걸린 상태는 13 개 중 7 개뿐이다 (고도 1, 속도 3, 각속도 3).
  //     나머지 6 개는 0 을 곱한다. 그래서 sqrt(q) 를 미리 곱해 7 행짜리로
  //     줄인 G 를 만들고 H += G^T G 로 쌓는다. 처음엔 13 행을 다 돌았고
  //     헤시안이 21.6 ms 로 전체의 절반이었다.
  const WI = RTI_WIDX;                       // [2,3,4,5,10,11,12]
  const NW = WI.length;
  const H = new Float64Array(n*n), g = new Float64Array(n);
  const qs = new Float64Array(NW), eh = new Float64Array(NW);
  const Gbuf = new Float64Array(N*NW*m);     // G[j] 를 한 버퍼에
  for (let k=0;k<N;k++){
    const term = (k === N-1) ? 11 : 1;       // 종단에 10 배를 더한다
    const xn = xs[k+1];
    // 쿼터니언 이중덮개. q 와 -q 가 같은 자세라, 기준의 부호를 공칭 궤적
    // 쪽으로 맞추지 않으면 멀쩡한 자세가 '180도 틀렸다' 로 읽힌다.
    const sgn = (xn[6]*qref[0] + xn[7]*qref[1] + xn[8]*qref[2] + xn[9]*qref[3]) < 0 ? -1 : 1;
    qs[0]=Math.sqrt(C.Qz*term);    eh[0]=qs[0]*(xn[2]-zref);
    qs[1]=Math.sqrt(C.Qv[0]*term); eh[1]=qs[1]*(xn[3]-vref[0]);
    qs[2]=Math.sqrt(C.Qv[1]*term); eh[2]=qs[2]*(xn[4]-vref[1]);
    qs[3]=Math.sqrt(C.Qv[2]*term); eh[3]=qs[3]*(xn[5]-vref[2]);
    for (let j=0;j<4;j++){
      qs[4+j]=Math.sqrt(C.Qq*term); eh[4+j]=qs[4+j]*(xn[6+j] - sgn*qref[j]);
    }
    // 종단에도 각속도를 벌해 봤는데 되레 나빴다 (|ω| 47.5 -> 83.0). 안 쓴다.
    qs[8]=Math.sqrt(C.Qw);         eh[8]=qs[8]*xn[10];
    qs[9]=Math.sqrt(C.Qw);         eh[9]=qs[9]*xn[11];
    qs[10]=Math.sqrt(C.Qw);        eh[10]=qs[10]*xn[12];
    const Sk = S[k];
    for (let j=0;j<=k;j++){
      const Sj = Sk[j], off = j*NW*m;
      for (let w=0;w<NW;w++){
        const row = WI[w]*m, qw2 = qs[w];
        for (let c=0;c<m;c++) Gbuf[off + w*m + c] = qw2*Sj[row+c];
      }
      // g += G^T eh
      for (let c=0;c<m;c++){
        let s = 0;
        for (let w=0;w<NW;w++) s += Gbuf[off + w*m + c]*eh[w];
        g[j*m+c] += s;
      }
    }
    // H += G^T G  (아래 삼각만 쌓고 뒤에서 대칭 복사)
    for (let j=0;j<=k;j++){
      const oj = j*NW*m;
      for (let i2=0;i2<=j;i2++){
        const oi = i2*NW*m;
        for (let a=0;a<m;a++){
          const ra = j*m+a;
          for (let b=0;b<m;b++){
            const cb = i2*m+b;
            if (cb > ra) continue;
            let s = 0;
            for (let w=0;w<NW;w++) s += Gbuf[oj + w*m + a]*Gbuf[oi + w*m + b];
            H[ra*n + cb] += s;
          }
        }
      }
    }
  }
  for (let i=0;i<n;i++) for (let j2=0;j2<i;j2++) H[j2*n+i] = H[i*n+j2];
  // 입력 항: R(u-uref) 와 Rdu(u_k - u_{k-1})
  for (let k=0;k<N;k++) for (let i=0;i<m;i++){
    const j = k*m+i;
    g[j] += C.R[i]*(rtiU[j]-uref[i]);
    H[j*n+j] += C.R[i];
    const uPrev = (k===0) ? uL[i] : rtiU[(k-1)*m+i];
    const d = rtiU[j] - uPrev;
    g[j] += C.Rdu[i]*d;
    H[j*n+j] += C.Rdu[i];
    if (k>0){
      const jp = (k-1)*m+i;
      g[jp] -= C.Rdu[i]*d;
      H[jp*n+jp] += C.Rdu[i];
      H[j*n+jp] -= C.Rdu[i];
      H[jp*n+j] -= C.Rdu[i];
    }
  }

  // 5) 상자 QP 를 투영 뉴턴으로. min 0.5 d^T H d + g^T d,  dlo <= d <= dhi
  const dlo = new Float64Array(n), dhi = new Float64Array(n);
  for (let k=0;k<N;k++) for (let i=0;i<m;i++){
    const j=k*m+i; dlo[j] = lo[i]-rtiU[j]; dhi[j] = hi[i]-rtiU[j];
  }
  const d = new Float64Array(n);
  const grad = new Float64Array(n), Hd = new Float64Array(n);
  // H 가 대칭이라 아래 삼각만 돌고 비대각을 두 번 센다. 선탐색에서 여러 번
  // 부르는 자리라 이 절반이 그대로 시간이 된다.
  const qobj = v => {
    let s = 0;
    for (let i=0;i<n;i++){
      const vi = v[i];
      if (vi !== 0){
        let hv = 0;
        for (let j=0;j<i;j++) hv += H[i*n+j]*v[j];
        s += vi*hv + 0.5*vi*vi*H[i*n+i];
      }
      s += g[i]*vi;
    }
    return s;
  };
  let J0 = qobj(d), used = 0;
  for (let it=0; it<C.asIters; it++){
    for (let i=0;i<n;i++){
      let s = 0;
      for (let j=0;j<n;j++) s += H[i*n+j]*d[j];
      Hd[i] = s; grad[i] = s + g[i];
    }
    // 능동집합: 경계에 붙었는데 기울기가 바깥으로 밀면 고정한다.
    const free = [];
    for (let i=0;i<n;i++){
      const atLo = d[i] <= dlo[i] + 1e-12, atHi = d[i] >= dhi[i] - 1e-12;
      if ((atLo && grad[i] > 0) || (atHi && grad[i] < 0)) continue;
      free.push(i);
    }
    if (!free.length) break;
    const mf = free.length;
    const Hf = new Float64Array(mf*mf), bf = new Float64Array(mf);
    for (let a=0;a<mf;a++){
      bf[a] = -grad[free[a]];
      for (let b=0;b<mf;b++) Hf[a*mf+b] = H[free[a]*n+free[b]];
      Hf[a*mf+a] += C.reg;
    }
    const step = cholSolve(Hf, bf, mf);
    if (!step) break;
    // 투영 선탐색
    const cand = new Float64Array(n);
    let ok = false;
    for (let al=1.0, t=0; t<8; t++, al*=0.5){
      cand.set(d);
      for (let a=0;a<mf;a++){
        const i = free[a];
        let v = d[i] + al*step[a];
        cand[i] = v < dlo[i] ? dlo[i] : (v > dhi[i] ? dhi[i] : v);
      }
      const J2 = qobj(cand);
      if (J2 < J0 - 1e-12){ d.set(cand); J0 = J2; ok = true; used = it+1; break; }
    }
    if (!ok) break;
  }

  for (let i=0;i<n;i++) rtiU[i] += d[i];
  clampU(rtiU);
  rtiApplied = [rtiU[0], rtiU[1], rtiU[2], rtiU[3]];
  rtiStat = {ms: ((typeof performance!=="undefined")?performance.now():0) - t_start,
             qpDrop: J0, asUsed: used};
  return rtiApplied.slice();
}

/* ══ 제어기 ═══════════════════════════════════════════════════════════
   PX4 를 옮긴 것이 아니다. 평범한 종속 루프(속도 -> 자세 -> 모멘트)다.
   여기서 보려는 것은 제어 성능이 아니라 **기체가 어디서 한계에 걸리나** 이므로
   제어기는 단순할수록 원인이 분명해진다. 검증도 플랜트만 한다. */
// 자세 루프를 세게 잡는다. 동체가 정적 안정(x_cp 가 무게중심 뒤)이라 공력이
// 기수를 바람 쪽으로 미는데, 비례 제어만으로는 그 모멘트를 상쇄하며 정상편차가
// 남는다. Jy*KW*KR 이 약하면 30~60도까지 벌어져 추력축이 누워 고도를 못 버틴다.
const KV = 1.2, KZ = 1.0, KR = 8.0, KW = 18.0;
// 롤 전용 게인. 롤은 Ixx 가 Iyy 의 1/35 라 로터 항력토크만으로도
// 1620 rad/s^2 까지 난다 (피치는 98). 같은 게인을 쓰면 과잉하다.
// 순항 자세는 살려야 하니 롤은 느리게 수십 초가 아니라 몇 초에 돌아오면 된다.
// 롤 전용 게인. 자세루프(KR/KW)보다 훨씬 느리게 잡는다. 롤은 Ixx 가 Iyy 의
// 1/35 라 로터 항력토크만으로 1620 rad/s^2 까지 나는 빠른 축이고(피치는 98),
// 축대칭 동체라 공력상 급할 이유도 없다. 스윕으로 고른 값이다.
// 오차를 +-20 도로 잘라 천천히 되돌리는 안도 재봤는데 되레 나빴다 —
// 요구 회전율이 0.14 rad/s 로 줄어 trim 이 80 도에서 못 돌아왔다. 안 쓴다.
let KR_R = 0.4, KW_R = 2.0, W_R_MAX = 25.0;
// NMPC 에 주는 고도 기준의 상승률 상한 [m/s]. 종속 루프의 clamp 와 같은 뜻.
const VZ_REF_MAX = 12.0;
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

// ── 제어기 선택 ───────────────────────────────────────────────────────
// NMPC 계열은 solver 가 필요해 브라우저에 넣지 않았다. 흉내만 낸 것을 NMPC 라
// 부르면 거짓이 된다. 대신 프로젝트의 내부루프인 INDI 는 그대로 이식했다.
const CTRLS = {
  cascade: {name:"종속 PID (기본)", ff:true, tilt:55, indi:false,
            note:"속도→자세→모멘트 종속 루프. 공력 앞먹임 켬. "
                 + "⚠ 전이 구간에서 롤 |p| 가 29~41 rad/s 로 튄다 (trim 은 0.6)."},
  noff:    {name:"앞먹임 없음", ff:false, tilt:55, indi:false,
            note:"동체가 내는 힘을 모르는 상태. 트림을 못 찾아 더 느리다."},
  fast:    {name:"틸트 60° (빠름·위험)", ff:true, tilt:60, indi:false,
            note:"351 km/h 까지 가지만 |ω| 가 10 rad/s 로 텀블한다."},
  trim:    {name:"트림 기반 (권장)", ff:true, tilt:58, indi:false, trim:true,
            note:"정상비행 트림을 플랜트에서 직접 풀어 앞먹임한다. 자세 한계 58도. "
                 + "62도로 풀면 568 km/h 까지 가지만 고도를 잃고 떨어진다 — "
                 + "그 사이를 찾는 것이 NMPC 의 일이다. "
                 + "롤까지 유지되는 유일한 제어기다 (|p| 0.6, 롤 오차 0도)."},
  hybrid:  {name:"하이브리드 NMPC+INDI (실험)", ff:true, tilt:80, indi:true, nmpc:true,
            note:"⚠ 아직 불안정합니다. 13차 가상모델을 브라우저에서 직접 풉니다"
                 + "(압축+투영경사). 솔버는 수렴합니다 — 낸 해가 최선의 정속추력보다 "
                 + "비용이 2배 낮습니다. 남은 문제는 한 번 푸는 데 150 ms 라 "
                 + "20 Hz 예산 50 ms 를 3배 넘는 것과, 비용함수에 자세 기준이 없어 "
                 + "롤을 바깥에서 얹어야 하는 것입니다. 실시간은 SQP/acados 가 필요합니다."},
  sqprti:  {name:"하이브리드 SQP-RTI (실험)", ff:true, tilt:80, indi:true, nmpc:true, rti:true,
            note:"같은 13차 가상모델을 실시간 반복(RTI)으로 푼다. 매 주기 SQP 한 번 — "
                 + "직전 해에서 선형화하고, 압축으로 상태를 소거해 상자제약만 남긴 QP 를 "
                 + "투영 뉴턴으로 푼다. 솔버는 성공했다: 평균 7.4 ms (투영경사 184.7 ms, "
                 + "25배), 같은 비용 수준, 20 Hz 예산 50 ms 에 6.8배 여유. "
                 + "비용함수도 다시 짰다: 자세 기준(LQR 표의 트림 자세)·등급 시간격자"
                 + "(30 ms~0.30 s, 지평선 3.3 s)·ν 상한을 INDI 한계 25 로 맞춤·"
                 + "예측 적분 보폭 0.05 s 제한. 추락하던 것이 고도를 지키며 325 km/h "
                 + "까지 간다. ⚠ 그래도 |ω| 47.5 로 실기 실패선 35 를 넘는다. "
                 + "남은 원인은 가중치가 아니라 **가상모델이 ω̇ = ν 를 즉시 이룬다고 "
                 + "가정**하는 것이다 — 실제 플랜트에는 공력 감쇠·모터 지연·자이로 "
                 + "결합이 있어 그 약속이 지켜지지 않는다. 지금은 LQR 이 299 km/h 를 "
                 + "|ω| 3.8 로 깨끗이 낸다."},
  lqr:     {name:"오차상태 LQR (게인표)", ff:true, tilt:55, indi:false, lqr:true,
            note:"속도별로 파이썬에서 ARE 를 미리 풀어 게인표로 구워 왔다 "
                 + "(2.5 m/s 간격 21 점, 폐루프 최대 실수부 -0.18 ~ -0.61 로 전부 안정). "
                 + "런타임에는 보간과 4x14 행렬곱뿐이라 solver 가 필요 없다. "
                 + "표를 제대로 읽었는지는 파이썬이 계산한 기준 샘플 16 개와 "
                 + "페이지에서 대조한다 — 절반은 격자 사이라 보간까지 검사한다. "
                 + "⚠ 표가 180 km/h 에서 끝난다. 로켓 배치는 52 m/s 위에 정상비행 "
                 + "트림이 아예 없다 (후방 로터가 음추력을 요구). 제어기 한계가 "
                 + "아니라 기체에 그 평형점이 없는 것이다."},
  indi:    {name:"INDI 내부루프", ff:true, tilt:55, indi:true,
            note:"측정 각가속도를 되먹여 증분으로 모멘트를 낸다. 모델오차에 강하다. "
                 + "⚠ 롤 |p| 가 40 rad/s 로 튄다. 포화 인지 할당이 필요하다."},
};
let CTRL = "cascade";
// INDI 상태: 직전 각속도·모멘트와 걸러낸 각가속도
let omPrev = [0,0,0], omDotF = [0,0,0], Mprev = [0,0,0];

// ── 트림 풀이 ─────────────────────────────────────────────────────────
// 명령 속도 V 로 수평 정상비행하려면 어떤 자세·추력이어야 하는가를
// **플랜트에서 직접** 푼다. 따로 수식을 세우면 틀릴 자리가 하나 는다.
// 미지수 두 개 (기울임 th, 로터 회전수 n), 조건 두 개 (v̇x = 0, v̇z = 0).
let trimV = -1, trimTh = 0, trimN = 0, trimOK = false;
function trimResid(V, th, n){
  const x = new Array(NX).fill(0);
  // y 축 둘레 회전각 a 로 동체 x 를 (sin th, 0, cos th) 로 향하게 한다.
  //   a = -pi/2 이면 동체 x 가 월드 +z (호버), 거기서 th 만큼 앞으로 눕힌다.
  const a = -Math.PI / 2 + th;
  x[7] = Math.sin(a / 2); x[9] = Math.cos(a / 2);
  x[3] = V;
  for (let i = 13; i < 17; i++) x[i] = n;
  const d = xdot(x, [n, n, n, n], [0, 0, 0], P);
  return [d[3], d[5]];
}
function trimSolve(V, th, n){
  let ok = false;
  for (let it = 0; it < 60; it++){
    const f = trimResid(V, th, n);
    if (Math.hypot(f[0], f[1]) < 1e-8){ ok = true; break; }
    const h1 = 1e-6, h2 = 1e-3;                 // 수치 야코비안
    const fa = trimResid(V, th + h1, n), fb = trimResid(V, th, n + h2);
    const J = [[(fa[0]-f[0])/h1, (fb[0]-f[0])/h2],
               [(fa[1]-f[1])/h1, (fb[1]-f[1])/h2]];
    const det = J[0][0]*J[1][1] - J[0][1]*J[1][0];
    if (!isFinite(det) || Math.abs(det) < 1e-14) break;
    let dth = (-f[0]*J[1][1] + f[1]*J[0][1]) / det;
    let dn  = (-J[0][0]*f[1] + J[1][0]*f[0]) / det;
    dth = Math.max(-0.15, Math.min(0.15, dth));   // 한 걸음 제한
    dn  = Math.max(-60, Math.min(60, dn));
    th += dth; n += dn;
    th = Math.max(-0.05, Math.min(1.5, th));      // 기울임 0~86도
    n  = Math.max(1, Math.min(P.n_max, n));
  }
  return [th, n, ok];
}
// ★ 연속법. 속도를 한 번에 주면 뉴턴이 엉뚱한 가지로 빠진다 (60 m/s 에서
//   기울임이 음수로 튀며 발산했다). 호버에서 출발해 조금씩 올리며 직전 해를
//   다음 초기값으로 쓴다. 트림 곡선을 따라가는 표준 방법이다.
function getTrim(V){
  if (trimOK && Math.abs(V - trimV) < 0.25) return [trimTh, trimN];
  let th, n, ok;
  let v0 = 0;
  th = 0.0; n = Math.sqrt(P.mass * P.g / (4 * P.k_T));
  if (trimOK && V > trimV && V - trimV < 30){    // 이어서 올릴 수 있으면
    v0 = trimV; th = trimTh; n = trimN;
  }
  const STEP = 5.0;
  for (let v = v0; v < V - 1e-9; ){
    v = Math.min(V, v + STEP);
    [th, n, ok] = trimSolve(v, th, n);
    if (!ok) break;
  }
  if (Math.abs(v0 - V) < 1e-9) [th, n, ok] = trimSolve(V, th, n);
  trimV = V; trimTh = th; trimN = n; trimOK = ok;
  return [th, n];
}
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

// ★ 추력 -> 회전수. 그냥 sqrt(T/k_T) 로 풀면 안 된다.
// 플랜트는 T = k_T n^2 * fac 를 내는데 fac = 1 - J/J_max 이고 J = 2 pi V_ax/(n D)
// 라 회전수 자신에 달려 있다. 순항에서 fac 가 로터마다 0.003~0.51 로 170 배
// 벌어져, 제어기가 요구한 롤 모멘트 0.16 N.m 이 플랜트에서 0.0000 이 됐다.
// (메모리의 "INDI G: 전진비 반영 필수" 가 바로 이 얘기다. 파이썬은 반영했고
//  브라우저 이식에서 빠져 있었다.)
// 다행히 닫힌 형태로 풀린다:
//   T = k_T n^2 - b n,  b = k_T * 2 pi V_ax / (D J_max)
//   n = [b + sqrt(b^2 + 4 k_T T)] / (2 k_T)
// T=0 이면 n = b/k_T 로, 추력이 0 이 되는 풍차회전수와 정확히 맞는다.
function nFromThrust(Ti, V_ax){
  if (Ti < 0) Ti = 0;
  const b = P.k_T * 2*Math.PI * V_ax / (P.D_prop * P.J_max);
  const n = (b + Math.sqrt(b*b + 4*P.k_T*Ti)) / (2*P.k_T);
  return n > P.n_max ? P.n_max : n;
}
// 지금 기수방향 유입속도. 할당이 쓰는 값과 플랜트가 쓰는 값이 같아야 한다.
function axialInflow(x){
  const qx=x[6], qy=x[7], qz=x[8], qw=x[9];
  const w = windNow();
  const d0=x[3]-w[0], d1=x[4]-w[1], d2=x[5]-w[2];
  const ub = (1-2*(qy*qy+qz*qz))*d0 + 2*(qx*qy+qz*qw)*d1 + 2*(qx*qz-qy*qw)*d2;
  return ub > 0 ? ub : 0;
}
function control(x, cmd){
  if (CTRLS[CTRL].nmpc) return controlHybrid(x, cmd);
  if (CTRLS[CTRL].lqr)  return controlLQR(x, cmd);
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
  let tx, ty, tz, tiltCap = CTRLS[CTRL].tilt * Math.PI / 180;
  if (CTRLS[CTRL].trim){
    // 트림이 이미 중력과 공력을 함께 균형 잡고 있으므로 여기서 g 를 더하지
    // 않는다. 피드백은 속도 오차만 얹는다.
    // ★ **현재** 속도의 트림을 쓴다. 명령 속도로 쓰면 아직 느린 상태에서
    //   80 도짜리 순항 자세를 요구해 고도 권한을 잃고 떨어진다 (실제로
    //   522 km/h 로 가속하다 지면에 닿았다). 앞먹임은 지금 상태와 맞아야 한다.
    const vNow = Math.hypot(x[3], x[4]);
    const [thT, nT] = getTrim(Math.min(vNow, cmd.spd));
    const T_ff = 4.0 * P.k_T * nT * nT;   // 트림은 플랜트로 직접 풀었으므로 fac 가 이미 들어 있다
    const st = Math.sin(thT), ct = Math.cos(thT);
    tx = T_ff*st*Math.cos(psi) + m*ax;
    ty = T_ff*st*Math.sin(psi) + m*ay;
    tz = T_ff*ct + m*az;
    // ★ 자세를 트림에서 크게 벗어나지 못하게 묶는다. 트림 80 도면 추력의
    //   수직성분이 17% 뿐이라 고도 권한이 거의 없다. 피드백이 거기서 더
    //   눕히면 속도가 폭주하고 떨어진다 (153 m/s 까지 가서 지면에 닿았다).
    tiltCap = Math.min(tiltCap, thT + 8.0 * Math.PI / 180);
  } else {
    const Fa = CTRLS[CTRL].ff ? aeroWorld(x) : [0, 0, 0];
    tx = m*ax - Fa[0]; ty = m*ay - Fa[1]; tz = m*(az + g) - Fa[2];
  }

  // ★ 추력축이 수직에서 너무 눕지 않게 **수평 요구를 깎는다**. 고도가 우선이다.
  //   앞먹임만 넣으면 제어기가 동체 양력을 믿고 90도를 넘겨 눕는다. 그러면
  //   추력의 수직성분이 사라져 양력에만 매달리고, 받음각이 조금만 변해도
  //   떨어진다 — 실제로 383 km/h 로 날면서 고도를 계속 잃었다.
  const hz = Math.hypot(tx, ty);
  const lim = Math.max(tz, 1e-6) * Math.tan(tiltCap);
  if (hz > lim && hz > 1e-9){ const s2 = lim/hz; tx *= s2; ty *= s2; }
  const tn0 = Math.hypot(tx, ty, tz);
  const tn = tn0 || 1e-9;

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
  const J = [Jx, Jy, Jz], om = [p_, q_, r_];
  // 원하는 각가속도. 두 제어기가 여기까지는 같고, 이걸 모멘트로 바꾸는
  // 방식만 다르다.
  const wdes = [KW*(KR*ex - p_), KW*(KR*ey - q_), KW*(KR*ez - r_)];

  // ★ 롤 유지. 위 Rd 는 목표 y 를 현재 y 에서 투영해 만들므로
  //   기수축 둘레 오차 ex 가 **구조적으로 0** 이다. 그러면 롤에는
  //   -KW*p 감쇠만 남아 각속도는 죽지만 각도는 그 자리에 남는다 —
  //   측풍 12 m/s 한 번에 90 도가 돌아가 영영 안 돌아오는 것을 실측했다.
  //   (화면에서는 매끄한 축대칭 동체 대신 로터 팔만 돌아 보인다.)
  //
  //   목표 프레임을 건드리지 않고 **롤 오차를 스칼라 하나로** 뽑아
  //   wdes[0] 을 덮어쓴다. 이유가 있다 — 목표 y 를 수평 기준으로 바꿔
  //   넣었더니, 롤 오차가 90 도 근처일 때 부호 맞춤(yh·yb ≈ 0)이 매
  //   스텝 뒤집혀 기준이 180 도씩 튀고 |ω| 가 46 rad/s 로 텀블했다.
  //   atan2 는 부호가 정의되어 그 모호함이 없다.
  //
  //   기준은 "동체 y 를 수평로". 순항에서는 추력축이 기울어 있어
  //   xb x z_hat 이 충분히 크다. 호버에서는 xb 가 월드 z 와 나란해
  //   퇴화하는데, 그때는 롤이 실제로 무의미하므로 그냥 놓아둔다.
  const hxy = Math.hypot(xb[0], xb[1]);
  if (hxy > 0.25){
    const yh = [xb[1]/hxy, -xb[0]/hxy, 0];          // 수평이면서 xb 에 수직
    const ybc = [R0[0][1], R0[1][1], R0[2][1]];      // 지금 동체 y
    const cr = [ybc[1]*yh[2]-ybc[2]*yh[1],           // yb_cur x yh
                ybc[2]*yh[0]-ybc[0]*yh[2],
                ybc[0]*yh[1]-ybc[1]*yh[0]];
    const sn = cr[0]*xb[0] + cr[1]*xb[1] + cr[2]*xb[2];
    const cs = ybc[0]*yh[0] + ybc[1]*yh[1] + ybc[2]*yh[2];
    const phi = Math.atan2(sn, cs);                  // (-pi, pi], 부호 확정
    const t = Math.min(1, (hxy - 0.25) / 0.20);
    const wgt = t*t*(3 - 2*t);                       // smoothstep 으로 서서히 개입
    // 롤은 급할 것이 없다. 자세루프보다 훨씬 느리게 잡고 상한을 둔다.
    let wr = KW_R*(KR_R*phi - p_);
    if (wr >  W_R_MAX) wr =  W_R_MAX;
    else if (wr < -W_R_MAX) wr = -W_R_MAX;
    wdes[0] = wdes[0] + wgt*(wr - wdes[0]);
  }

  let Mx, My, Mz;
  if (CTRLS[CTRL].indi){
    // INDI: 모델로 역산하지 않고 **측정 각가속도**를 되먹여 증분만 더한다.
    //   M = M_prev + J (ω̇_des - ω̇_meas)
    // 관성·공력 모델이 틀려도 측정된 ω̇ 이 그 오차를 이미 담고 있어 강건하다.
    // 프로젝트가 FC 쪽 내부루프로 고른 방식이다.
    const a = 0.06;   // ω̇ 추정 저역통과. 미분은 잡음을 키운다.
    for (let i = 0; i < 3; i++){
      const raw = (om[i] - omPrev[i]) / DT;
      omDotF[i] += a * (raw - omDotF[i]);
    }
    // 증분에 한계를 둔다. 모터가 1 차 지연(tau_m)으로 따라오는데 증분을 무한정
    // 쌓으면 감기(windup)로 발산한다. 첫 시도에서 |ω| 가 45 rad/s 까지 갔다.
    // ★ 여기는 J[i]*25 를 그대로 둔다. 하이브리드 쪽은 권한 비례로 바꿔
    //   텀블이 사라졌는데(|ω| 47.5 -> 14.4), 같은 값을 이 제어기에 넣으니
    //   되레 추락했다 (천장 포화 66 %, 599 km/h). 이 제어기는 틸트 한계 55 도
    //   와 옛 증분 한계에 맞춰 조율돼 있어서, 권한만 늘리면 과구동이 된다.
    //   바꾸려면 여기 게인까지 같이 다시 잡아야 한다 — 지금 할 일이 아니다.
    const M = [0, 0, 0];
    for (let i = 0; i < 3; i++){
      const lim = J[i] * 25.0;
      let m2 = Mprev[i] + J[i] * (wdes[i] - omDotF[i]);
      if (m2 > lim) m2 = lim; else if (m2 < -lim) m2 = -lim;
      M[i] = m2;
    }
    Mx = M[0]; My = M[1]; Mz = M[2];
  } else {
    Mx = Jx*wdes[0] + (q_*(Jz*r_) - r_*(Jy*q_));
    My = Jy*wdes[1] + (r_*(Jx*p_) - p_*(Jz*r_));
    Mz = Jz*wdes[2] + (p_*(Jy*q_) - q_*(Jx*p_));
  }
  omPrev = om.slice();
  Mprev = [Mx, My, Mz];

  // 총추력은 **현재** 추력축(동체 x 를 월드로)에 투영한다
  const axw = R[0][0], ayw = R[1][0], azw = R[2][0];
  let T = tx*axw + ty*ayw + tz*azw;
  if (T < 0) T = 0;

  const A = D.alloc_inv, n = [0,0,0,0], Vax = axialInflow(x);
  for (let i=0;i<4;i++){
    const Ti = A[i][0]*T + A[i][1]*Mx + A[i][2]*My + A[i][3]*Mz;
    n[i] = nFromThrust(Ti, Vax);
  }
  return n;
}

// ProperHybrid: NMPC 가 [T, ω̇] 를 내고 INDI 가 모터로 실현한다.
// 인터페이스 분리 — NMPC 는 "얼마나 돌릴지" 만 정하고, 모터 할당·자이로·공력
// 모멘트는 INDI 가 측정으로 처리한다.
function controlHybrid(x, cmd){
  const psi = cmd.psi;
  const vref = [cmd.spd*Math.cos(psi), cmd.spd*Math.sin(psi), 0];
  // ★ 도달 가능한 고도 기준을 준다. 지평선이 N*dt = 1.6 s 뿐인데 200 m 짜리
  //   고도 오차를 그대로 주면 Qz*200^2 = 800000 이 비용을 통째로 지배해서
  //   최적해가 "최대 추력" 에 박힌다. 반복을 40 -> 200 -> 1000 으로 늘려도
  //   778 N(상한) 에서 안 움직였다 — 솔버가 덜 수렴한 게 아니라 비용함수의
  //   진짜 최적점이 거기였다. 83 m/s 정상비행 트림은 222 N 이면 된다.
  //   종속 루프가 고도 오차를 상승률로 바꿔 쓰는 것(clamp +-15)과 같은 처리다.
  const useRti = !!CTRLS[CTRL].rti;
  const SOL = useRti ? RTI : NMPC;
  // ★ **현재** 속도의 트림을 기준으로 삼는다. 명령 속도로 잡으면 아직 느린
  //   상태에서 순항 자세(64도)를 요구해 고도 권한을 잃는다 — 종속 루프가
  //   같은 함정에 빠졌던 자리이고, 여기서도 |ω| 67 로 추락했다.
  const vNowH = Math.hypot(x[3], x[4]);
  vTrimRef = Math.min(vNowH, cmd.spd, LQ.V_max_table);
  const hz = useRti ? RTI.horizon : (NMPC.N * NMPC.dt);
  const dzCap = VZ_REF_MAX * hz;
  const dz = cmd.alt - x[2];
  const zr = x[2] + (dz > dzCap ? dzCap : (dz < -dzCap ? -dzCap : dz));
  let out;
  if (useRti){
    if (rtiOut === null || T - rtiLast >= RTI.rate - 1e-9){
      rtiOut = rtiSolve(x.slice(0, 13), vref, zr);
      rtiLast = T;
    }
    out = rtiOut;
  } else {
    if (nmpcOut === null || T - nmpcLast >= NMPC.rate - 1e-9){
      nmpcOut = nmpcSolve(x.slice(0, 13), vref, zr);
      nmpcLast = T;
    }
    out = nmpcOut;
  }
  const Tc = out[0], nu = [out[1], out[2], out[3]];
  // ★ 롤은 인터페이스 층에서 잡는다. 두 솔버 모두.
  //
  //   투영경사(hybrid)는 비용에 자세 항이 아예 없어 nu_roll 이 어떤 반복수
  //   에서도 0.01 이었다 — 솔버를 아무리 좋게 해도 롤은 안 잡힌다.
  //   RTI 는 q_ref(트림 자세)에 롤이 들어 있는데도 여기서 덮어쓴다. 재보고
  //   내린 결정이다: 끄면 |ω| 가 47.5 -> 64.7 로 나빠지고 NMPC 가 낸
  //   |ν_roll| 이 한계 25 에 붙는다.
  //
  //   이유가 있다. 비용이 |q - q_ref|^2 라 세 축을 **같은 무게**로 잰다.
  //   그런데 Ixx 가 Iyy 의 1/35 라 같은 자세 오차를 롤로 지우는 비용이
  //   훨씬 싸다 — 최적해가 롤을 과하게 쓴다. 축별로 다르게 재려면 쿼터니언
  //   차가 아니라 동체 프레임 오차로 써야 하는데, 그러면 지금의 대각 가중치
  //   기계(상태에 바로 가중)를 못 쓴다. 그건 다음 일이다.
  //
  //   축대칭 동체라 기수축 둘레 롤은 공력상 결합이 없다. NMPC 가 추력축
  //   기동을 정하고 롤은 인터페이스 층이 맡는 분리가 물리적으로도 맞다.
  if (ROLL_HOLD_OUTSIDE){
    const qx=x[6], qy=x[7], qz=x[8], qw=x[9];
    const xb=[1-2*(qy*qy+qz*qz), 2*(qx*qy+qz*qw), 2*(qx*qz-qy*qw)];
    const yb=[2*(qx*qy-qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz+qx*qw)];
    const h = Math.hypot(xb[0], xb[1]);
    if (h > 0.25){
      const yh=[xb[1]/h, -xb[0]/h, 0];
      const cr=[yb[1]*yh[2]-yb[2]*yh[1], yb[2]*yh[0]-yb[0]*yh[2], yb[0]*yh[1]-yb[1]*yh[0]];
      const phi = Math.atan2(cr[0]*xb[0]+cr[1]*xb[1]+cr[2]*xb[2],
                             yb[0]*yh[0]+yb[1]*yh[1]+yb[2]*yh[2]);
      const t = Math.min(1, (h - 0.25) / 0.20);
      const wgt = t*t*(3 - 2*t);
      let wr = KW_R*(KR_R*phi - x[10]);
      if (wr > W_R_MAX) wr = W_R_MAX; else if (wr < -W_R_MAX) wr = -W_R_MAX;
      nu[0] = nu[0] + wgt*(wr - nu[0]);
    }
  }

  // INDI 내부루프: 측정 각가속도를 되먹여 증분으로 모멘트를 낸다.
  const J = [P.Ixx, P.Iyy, P.Izz], om = [x[10], x[11], x[12]];
  const a = 0.06;
  for (let i=0;i<3;i++){
    const raw = (om[i] - omPrev[i]) / DT;
    omDotF[i] += a * (raw - omDotF[i]);
  }
  // ★ 증분 한계를 **권한 비례**로. 예전에는 J[i]*25, 즉 모든 축에 똑같이
  //   25 rad/s^2 였다. 그런데 Ixx 가 Iyy 의 1/35 라 롤에서는 그게 0.5 N·m
  //   밖에 안 된다. 실측하면 롤 외란이 ω̇ 88 rad/s^2 RMS(최대 191)로 오는데
  //   INDI 가 낼 수 있는 것이 25 뿐이라 **구조적으로 못 따라잡는다.**
  //   (피치·요는 명령 8.2 에 실제 10.2 로 잘 따라간다 — 롤만 문제다.)
  //   한계를 실제 모멘트 권한의 일정 비율로 잡으면 축마다 알맞게 나온다.
  //     롤    로터 항력토크  4*k_Q*n_max^2      = 64.8 N·m
  //     피치/요 차동추력    2*arm*k_T*n_max^2   = 68.7 N·m
  const M = [0,0,0];
  for (let i=0;i<3;i++){
    const lim = INDI_LIM[i];
    let m2 = Mprev[i] + J[i]*(nu[i] - omDotF[i]);
    M[i] = m2 > lim ? lim : (m2 < -lim ? -lim : m2);
  }
  omPrev = om.slice(); Mprev = M.slice();

  const A = D.alloc_inv, nOut = [0,0,0,0], Vax = axialInflow(x);
  for (let i=0;i<4;i++){
    const Ti = A[i][0]*Tc + A[i][1]*M[0] + A[i][2]*M[1] + A[i][3]*M[2];
    nOut[i] = nFromThrust(Ti, Vax);
  }
  return nOut;
}

/* ══ 상태 ═════════════════════════════════════════════════════════════ */
const DT = 0.002;
let X = D.x0.slice(), T = 0, running = false, sat = 0;
let lastSatHi = false, lastSatLo = false;   // 천장/바닥 포화, 기록에 같이 담는다
// ★ 고도 판정의 기준점. 지상 0 m 에서 목표 200 m 를 그대로 기준으로 쓰면
//   **이륙 자체가 고도 오차 200 m** 로 찍혀 RMSE 가 이륙에 먹히고 발산
//   판정도 매번 터진다.
//   공통 램프를 기준으로 두는 안을 먼저 재봤는데 더 나빴다. 제어기마다
//   상승률이 크게 달라서다 — trim/cascade 는 15 m/s 로 올라 14 초에 닿고,
//   LQR 은 6.7 m/s 로 32 초에 닿는다. 어떤 램프값을 잡아도 한쪽은 앞질러서,
//   다른 쪽은 뒤처져서 억울하게 발산으로 찍힌다 (실측: 램프 3/5/8 m/s 에서
//   최대 기준지연이 각각 160/130/86 m).
//   그래서 램프를 버리고 **'도달한 뒤 지켰나'** 로 바꾼다. 목표 고도에 한 번
//   닿기 전은 이륙이지 발산이 아니다.
let zSettled = false;
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
/* ══ 되감기 결정론 ═════════════════════════════════════════════════════
   REC 에 기체 상태만 담고 되감으면 **제어기 내부가 그대로 남는다**. 같은
   지점에서 다시 날려도 다른 궤적이 나온다. 안 돌아오던 것:
     altI            고도 적분기 (I_LIM ±6)
     omPrev·omDotF·Mprev   INDI 증분의 적분 상태
     trimV·trimTh·trimN·trimOK   트림 연속법 캐시
     sat             포화 표시의 저역통과
     nmpcLast·rtiLast  솔버 재해 시점
   마지막 것이 특히 나빴다. 재해 여부를 `T - nmpcLast >= rate` 로 정하는데
   60 s 에서 10 s 로 되감으면 그 값이 **-50** 이 되어 50 초(시뮬) 동안
   솔버가 한 번도 안 불린다. 그래서 절대시각이 아니라 **경과시간**으로 담는다.

   웜스타트(nmpcU·rtiU, 각 N*NUV=80 실수)까지 담는다. 항목당 640 바이트라
   6000 개면 3.8 MB 로, 결정론을 완전히 얻는 값으로는 싸다. 실제로 이걸
   빼면 NMPC 계열만 되감기 재생이 갈린다.                                */
function ctrlSnap(){
  return {
    altI: altI, sat: sat, zS: zSettled,
    om: omPrev.slice(), od: omDotF.slice(), mp: Mprev.slice(),
    tV: trimV, tTh: trimTh, tN: trimN, tOK: trimOK,
    nAge: T - nmpcLast, nOut: nmpcOut ? nmpcOut.slice() : null,
    nU: nmpcU ? Float64Array.from(nmpcU) : null,
    nAp: nmpcApplied ? nmpcApplied.slice() : null,
    rAge: T - rtiLast, rOut: rtiOut ? rtiOut.slice() : null,
    rU: rtiU ? Float64Array.from(rtiU) : null,
    rAp: rtiApplied ? rtiApplied.slice() : null,
  };
}
function ctrlLoad(c){
  if (!c) return;
  altI = c.altI; sat = c.sat; zSettled = c.zS;
  omPrev = c.om.slice(); omDotF = c.od.slice(); Mprev = c.mp.slice();
  trimV = c.tV; trimTh = c.tTh; trimN = c.tN; trimOK = c.tOK;
  // 경과시간으로 되돌린다. 저장 시점에 한 번도 안 풀었으면 nAge 가 아주 커서
  // 복원 직후 바로 한 번 푼다 — 그게 맞는 동작이다.
  nmpcLast = T - c.nAge; nmpcOut = c.nOut ? c.nOut.slice() : null;
  nmpcU = c.nU ? Array.from(c.nU) : null;
  nmpcApplied = c.nAp ? c.nAp.slice() : null;
  rtiLast = T - c.rAge; rtiOut = c.rOut ? c.rOut.slice() : null;
  rtiU = c.rU ? Float64Array.from(c.rU) : null;
  rtiApplied = c.rAp ? c.rAp.slice() : null;
}
// 되감기 복원은 여기 한 곳에서만 한다. 예전엔 toggleRun 과 타임라인이
// 각자 X·T 만 되돌려 놓고 cmdSpd 는 한쪽만 되돌렸다.
function restoreTo(i){
  const r = REC[i];
  X = r.x.slice(); T = r.t; cmdSpd = r.cmd;
  ctrlLoad(r.cs);
  recAcc = 0;
}
function reset(){
  REC.length = 0; viewIdx = null; recAcc = 0;
  // 지상에서 시작한다. 예전엔 목표 고도에 바로 놓고 시작해 이륙 단계가 아예
  // 없었다 — 미션의 첫 구간이 통째로 빠져 있던 셈이다.
  X = D.x0.slice(); X[2] = 0; T = 0; sat = 0; cmdSpd = 0; altI = 0; zSettled = false;
  omPrev = [0,0,0]; omDotF = [0,0,0]; Mprev = [0,0,0]; trimOK = false; trimV = -1;
  nmpcU = null; nmpcLast = -1e9; nmpcOut = null; nmpcApplied = null;
  rtiU = null; rtiLast = -1e9; rtiOut = null; rtiApplied = null; rtiStat = null;
  trailN = 0; trailDrawn = 0;
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
// 궤적선 버퍼는 REC 상한과 같게 둔다. 그래야 '타임라인이 갈 수 있는 곳'
// 과 '선이 그려진 곳' 이 어긋나지 않는다.
const TRAIL_MAX = 6000;
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
  // 롤 기준선. 매끈한 축대칭 동체는 기수축 둘레로 돌아도 화면에서 안 보인다.
  // 그래서 로터 팔만 도는 것처럼 읽혔다 — 실제로는 강체로 같이 돌고 있었다.
  // 동체 +z 쪽에 줄을 하나 그어 롤이 눈에 보이게 한다.
  const stripe = new THREE.Mesh(new THREE.BoxGeometry(.62, .010, .004), accent);
  stripe.position.set(-.02, 0, .0755); veh.add(stripe);

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

  trailPos = new Float32Array(3 * TRAIL_MAX);
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
// 궤적선. **기록(REC)에서 직접 그린다.**
//
// 예전에는 프레임마다 지금 위치를 하나씩 쌓았고 4000 개에서 그냥 멈췄다.
// 그래서 (a) 오래 날리면 선이 도중에 끊기고, (b) 배속을 올리면 점 간격이
// 벌어지고, (c) 되감아도 선이 안 따라와 지나간 미래가 그대로 남았다.
//
// REC 에서 그리면 셋 다 사라지고, **타임라인으로 갈 수 있는 구간과 궤적선이
// 정확히 같아진다**. 되감으면 REC 가 잘리고 선도 같이 잘린다.
// 전부 다시 채우는 건 비싸니 길이만 보고 늘어난 만큼만 이어 붙인다.
let trailDrawn = 0;
function syncTrail(){
  if (!trail) return;
  const n = Math.min(REC.length, TRAIL_MAX);
  const upTo = (viewIdx === null) ? n : Math.min(viewIdx + 1, n);
  if (upTo < trailDrawn || REC.length < trailDrawn){   // 되감겨 잘렸다 -> 다시
    trailDrawn = 0;
  }
  for (let i = trailDrawn; i < upTo; i++){
    const q = REC[i].x;
    trailPos[i*3] = q[0]; trailPos[i*3+1] = q[1]; trailPos[i*3+2] = q[2];
  }
  if (upTo !== trailDrawn){
    trailDrawn = upTo;
    trail.geometry.setDrawRange(0, trailDrawn);
    trail.geometry.attributes.position.needsUpdate = true;
  }
}
function render3D(d){
  const p = new THREE.Vector3(X[0], X[1], X[2]);
  veh.position.copy(p);
  veh.quaternion.set(X[6], X[7], X[8], X[9]);
  // 로터를 실제 회전수 n [rad/s] 로 돌린다. 방향은 rotor_directions 대로.
  for (let i = 0; i < PROPS.length; i++)
    PROPS[i].rotation.x += P.rotor_directions[i] * X[13+i] * 0.016;
  syncTrail();
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
    // cmd 를 루프 밖에 둔다. 안에서 const 로 잡으면 아래 기록 단계에서
    // 스코프를 벗어나 매 프레임 예외가 나고 시간이 아예 안 간다 — 실제로 그랬다.
    let cmd = cmdNow();
    while (budget > 0 && guard++ < 600){
      const dv = target - cmdSpd, step = RAMP * DT;
      cmdSpd += Math.abs(dv) < step ? dv : (dv > 0 ? step : -step);
      cmd = cmdNow();
      const u = control(X, cmd);
      X = rk4(X, u, w, P, DT);
      ground();
      T += DT; budget -= DT;
      // 포화는 천장만 보면 안 된다. 할당이 Ti<0 을 0 으로 눕히면 로터가
      // 완전히 멈추는데(n_min=0) 그건 권한을 잃은 것이지 여유가 아니다.
      lastSatHi = u.some(v => v >= P.n_max - 1e-6);
      lastSatLo = u.some(v => v <= P.n_min + 1e-6);
      if (!zSettled && Math.abs(X[2] - cmd.alt) < 5.0) zSettled = true;
      if (lastSatHi || lastSatLo) sat = Math.min(1, sat + .02);
      else sat = Math.max(0, sat - .01);
    }
    const d = diag();
    recAcc += dtReal * rtf;
    if (recAcc >= REC_EVERY || !REC.length){
      recAcc = 0;
      const prev = REC.length ? REC[REC.length-1].cmd : 0;
      // zr(그때의 목표 고도)를 같이 담는다. 예전엔 채점이 슬라이더의 **지금**
      // 값을 읽어서, 같은 기록 한 판에 고도 슬라이더만 움직이면 RMSEz 가
      // 1.23(통과) 에서 198.80(실패) 으로 뒤집혔다. 소급 채점이었다.
      // satHi/satLo 도 담는다. 바닥 포화(Ti<0 -> n=0)는 지금 화면에 안 뜬다.
      REC.push({t:T, x:X.slice(), cmd:cmdSpd, V:d.gs, al:d.alpha, F:d.F,
                zr:cmd.alt, st:zSettled, satHi:lastSatHi, satLo:lastSatLo,
                seg:segNow(d, cmd, prev), cs:ctrlSnap()});
      if (REC.length > 6000) REC.shift();
    }
    paint(d);
  }
}
function toggleRun(){
  // 되감아 둔 상태에서 재생하면 그 지점부터 이어 난다. 뒤 기록은 버린다.
  if (!running && viewIdx !== null){
    REC.length = viewIdx + 1;
    restoreTo(viewIdx);
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
// ── 채점 ─────────────────────────────────────────────────────────────
// 옛 미션 평가와 같은 기준이다. 속도·고도 추종 RMSE 에 **|ω| 를 반드시 함께**
// 본다 — RMSE 만으로는 텀블을 못 잡는다 (|ω| 79 인데 z 오차 1.46 인 사례가 있었다).
// 트림 기울임. 기울임 한계는 상수로 재면 안 된다 — 로켓 트림 자체가
// V=50 에서 63.9도, V=85 에서 80.7도라 상수 70도면 정상 순항이 전부 실패로
// 찍힌다. 트림 대비 **초과분**으로 재야 뜻이 있다.
function trimTiltDeg(V){
  const [xt] = lqrPick(Math.min(V, LQ.V_max_table));
  const qx=xt[6], qy=xt[7], qz=xt[8], qw=xt[9];
  return Math.acos(Math.max(-1, Math.min(1, 2*(qx*qz - qy*qw)))) * 180/Math.PI;
}
function tiltOf(r){
  const qx=r.x[6], qy=r.x[7], qz=r.x[8], qw=r.x[9];
  return Math.acos(Math.max(-1, Math.min(1, 2*(qx*qz - qy*qw)))) * 180/Math.PI;
}
function score(){
  const n = REC.length;
  if (n < 20) return null;

  // ★ 발산 판정을 **제일 먼저**. 상태가 NaN 이면 아래 비교가 전부 false 라
  //   텀블도 추락도 안 잡히고, 엉뚱하게 rmseZ 때문에 실패로 찍힌다.
  //   파이썬 기준과 같게 |z - z_ref| > 50 도 발산으로 본다
  //   (control/mission_sim.py:201).
  let diverged = false, divWhy = "";
  for (const r of REC){
    for (let i = 0; i < NX; i++)
      if (!isFinite(r.x[i])) { diverged = true; divWhy = "상태가 NaN/Inf"; break; }
    if (diverged) break;
    // ★ 목표 고도에 한 번 닿은 **뒤**에만 본다. 닿기 전은 이륙이다.
    if (r.st && Math.abs(r.x[2] - r.zr) > 50){
      diverged = true; divWhy = "고도에 닿은 뒤 50 m 이탈";
    }
    if (diverged) break;
  }
  const settled = REC.filter(r => r.st);

  // ★ |ω| 판정은 **최장 연속** 구간으로. 파이썬이 그렇게 센다
  //   (control/acados_fallback_mc.py:76-90 의 run25). 예전엔 총합이었고,
  //   게다가 `> 0.2` 가 5 샘플에서 IEEE754 상 거짓이라 실효 기준이 240 ms
  //   였다. 총합과 최장연속은 서로 다른 방향으로 둘 다 틀린다.
  let maxOm = 0, run = 0, run25 = 0, ms25 = 0;
  let maxTiltEx = -1e9, maxTilt = 0, satHiN = 0, satLoN = 0;
  for (const r of REC){
    const om = Math.hypot(r.x[10], r.x[11], r.x[12]);
    maxOm = Math.max(maxOm, om);
    if (om > 25){ run += REC_EVERY; ms25 += REC_EVERY; run25 = Math.max(run25, run); }
    else run = 0;
    const tl = tiltOf(r);
    maxTilt = Math.max(maxTilt, tl);
    maxTiltEx = Math.max(maxTiltEx, tl - trimTiltDeg(r.V));
    if (r.satHi) satHiN++;
    if (r.satLo) satLoN++;
  }

  // ★ 구간별로 잰다. 예전엔 순항만 집계했는데, segNow 가 |alt-z|>5 를
  //   'climb' 으로 걸러낸 **뒤** 순항만 보므로 RMSEz <= 5 가 구조적으로
  //   보장됐다. 제일 어려운 구간(감속·호버 복귀)이 통째로 빠져 있었다.
  const segs = {};
  for (const r of REC){
    const g = segs[r.seg] || (segs[r.seg] = {n:0, sv:0, sz:0, mz:0, mo:0});
    g.n++;
    g.sv += (r.V - r.cmd) ** 2;
    g.sz += (r.x[2] - r.zr) ** 2;
    g.mz = Math.max(g.mz, Math.abs(r.x[2] - r.zr));
    g.mo = Math.max(g.mo, Math.hypot(r.x[10], r.x[11], r.x[12]));
  }
  const bySeg = [];
  for (const k of ["ground","climb","accel","cruise","decel","hover"]){
    const g = segs[k];
    if (!g) continue;
    bySeg.push({seg:k, name:SEGS[k][0], n:g.n,
                rmseV:Math.sqrt(g.sv/g.n), rmseZ:Math.sqrt(g.sz/g.n),
                maxZ:g.mz, maxOm:g.mo});
  }
  // 대표값은 **정착 이후** 표본으로 낸다. 이륙 구간을 섞으면 RMSE 가
  // 이륙에 통째로 먹혀 제어기 차이가 안 보인다. 구간별 표에는 이륙도 있다.
  const use = settled.length > 20 ? settled : REC;
  let SV = 0, SZ = 0;
  for (const r of use){ SV += (r.V - r.cmd)**2; SZ += (r.x[2] - r.zr)**2; }
  const rmseV = Math.sqrt(SV/use.length), rmseZ = Math.sqrt(SZ/use.length);
  const cruise = segs.cruise;
  const rmseVc = cruise ? Math.sqrt(cruise.sv/cruise.n) : null;
  const rmseZc = cruise ? Math.sqrt(cruise.sz/cruise.n) : null;

  const tumble = maxOm > 35 || run25 >= 0.2;
  const crashed = REC.some(r => r.x[2] < 1 && r.t > 20);
  // ★ 속도 추종을 판정에 넣는다. 예전엔 없어서, '권장' 제어기가 명령
  //   83 m/s 에 실측 48.2 m/s(35 m/s 미달) 인데도 통과로 찍혔다.
  const last = REC[n-1];
  const vShort = last.cmd - last.V;
  const vMiss = vShort > Math.max(3.0, 0.1 * Math.max(last.cmd, 1e-9));

  // 판정을 둘로 나눈다.
  //   비행 건전성 — 기체가 제어 아래 있었나 (제어기 비교의 축)
  //   미션 달성   — 시킨 일을 해냈나 (300 km/h 목표의 축)
  // 하나로 묶으면 "로켓이 83 m/s 를 못 낸다" 는 사실이 모든 제어기를 똑같이
  // 실패로 만들어 제어기 사이의 차이를 덮어 버린다. 둘 다 봐야 한다.
  const sound = [], miss = [];
  if (diverged) sound.push("발산 — " + divWhy);
  if (tumble) sound.push("텀블 — |ω|max " + maxOm.toFixed(1)
                       + ", 25 초과 최장연속 " + (run25*1000).toFixed(0) + " ms");
  if (crashed) sound.push("추락 — 고도 1 m 아래");
  if (satHiN/n > 0.2) sound.push("모터 천장 포화 " + (100*satHiN/n).toFixed(0) + "%");
  if (satLoN/n > 0.2) sound.push("모터 바닥 포화 " + (100*satLoN/n).toFixed(0) + "%");
  if (vMiss) miss.push("속도 미달 — 명령 " + last.cmd.toFixed(1)
                      + " 대비 " + vShort.toFixed(1) + " m/s 부족");
  if (rmseZ >= 20) miss.push("고도 RMSE " + rmseZ.toFixed(1) + " m");

  return {rmseV, rmseZ, rmseVc, rmseZc, bySeg,
          maxOm, run25, ms25, maxTilt, maxTiltEx,
          satHi: satHiN/n, satLo: satLoN/n,
          tumble, crashed, diverged, divWhy, vMiss, vShort,
          vmax: Math.max(...REC.map(r => r.V)), n,
          nCruise: cruise ? cruise.n : 0,
          nSettled: settled.length, tSettle: settled.length ? settled[0].t : null,
          sound, miss, why: sound.concat(miss),
          soundOk: sound.length === 0, pass: sound.length === 0 && miss.length === 0};
}
// RMSE 를 시간에 따라. 한 숫자로만 보면 어느 구간이 나빴는지 안 보인다.
function rmseSeries(win){
  win = win || 25;
  const out = {t:[], v:[], z:[]};
  for (let i = win; i < REC.length; i++){
    let sv = 0, sz = 0;
    for (let k = i - win; k < i; k++){
      sv += (REC[k].V - REC[k].cmd) ** 2;
      sz += (REC[k].x[2] - REC[k].zr) ** 2;   // 그때의 기준값. 슬라이더 지금 값이 아니다
    }
    out.t.push(REC[i].t);
    out.v.push(Math.sqrt(sv / win));
    out.z.push(Math.sqrt(sz / win));
  }
  return out;
}
function drawScoreCv(){
  const cv = $("#scoreCv");
  if (!cv._h) cv._h = +cv.getAttribute("height");
  const r = Math.min(devicePixelRatio, 2), w = cv.clientWidth, h = cv._h;
  cv.width = w*r; cv.height = h*r; cv.style.height = h + "px";
  const c = cv.getContext("2d"); c.setTransform(r,0,0,r,0,0); c.clearRect(0,0,w,h);
  const S = rmseSeries();
  if (S.t.length < 2) return;
  const hi = Math.max(1e-6, Math.max(...S.v), Math.max(...S.z));
  const X = i => 34 + i/(S.t.length-1)*(w-40);
  const Y = v => h-14 - v/hi*(h-22);
  c.strokeStyle = "#1e2a35"; c.lineWidth = 1;
  c.beginPath(); c.moveTo(34, h-13.5); c.lineTo(w-6, h-13.5); c.stroke();
  c.font = '400 9.5px "IBM Plex Mono", monospace'; c.fillStyle = "#7C8A98";
  c.textAlign = "right"; c.textBaseline = "middle";
  c.fillText(hi.toFixed(0), 30, 10); c.fillText("0", 30, h-14);
  const line = (arr, col) => { c.strokeStyle = col; c.lineWidth = 1.6;
    c.beginPath(); arr.forEach((v,i)=>{ const x=X(i), y=Y(v); i?c.lineTo(x,y):c.moveTo(x,y); });
    c.stroke(); };
  line(S.z, "#2d94bd"); line(S.v, "#f2aa4c");
  c.textAlign = "left"; c.fillStyle = "#f2aa4c"; c.fillText("RMSE v", 38, 9);
  c.fillStyle = "#2d94bd"; c.fillText("RMSE z", 88, 9);
}
function scoreCsv(){
  const S = rmseSeries();
  const alt = +$("#alt").value;
  const head = "# fast_drone 비행 시뮬 채점\n"
    + "# 제어기: " + CTRLS[CTRL].name + "\n"
    + "# 목표속도: " + $("#spd").value + " m/s, 목표고도: " + alt + " m\n"
    + "# 측풍: " + $("#wsp").value + " m/s, 방위 " + $("#wdir").value + " deg\n"
    + "# 계수: " + COEFS.map(([k]) => k + "=" + P[k]).join(", ") + "\n"
    + "t_s,seg,cmd_mps,V_mps,alt_m,z_ref_m,alpha_deg,F_N,omega_rad_s,"
    + "tilt_deg,sat_hi,sat_lo,rmse_v,rmse_z\n";
  const off = REC.length - S.t.length;
  return head + REC.map((r, i) => {
    const om = Math.hypot(r.x[10], r.x[11], r.x[12]);
    const j = i - off;
    return [r.t.toFixed(3), r.seg, r.cmd.toFixed(3), r.V.toFixed(4),
            r.x[2].toFixed(3), r.zr.toFixed(2), r.al.toFixed(3), r.F.toFixed(4),
            om.toFixed(5), tiltOf(r).toFixed(2),
            r.satHi ? 1 : 0, r.satLo ? 1 : 0,
            j >= 0 ? S.v[j].toFixed(4) : "", j >= 0 ? S.z[j].toFixed(4) : ""].join(",");
  }).join("\n") + "\n";
}

/* ══ 채점 별창 ════════════════════════════════════════════════════════
   좁은 옆패널에서는 지표를 몇 개밖에 못 본다. 별창을 띄워 넓게 그리고,
   시간축을 직접 확대·축소·이동할 수 있게 한다.

   ★ 지표마다 단위가 다르다 (m, m/s, rad/s, 도, %). 한 축에 겹쳐 그리면
     스케일이 큰 놈이 나머지를 눌러 아무것도 안 읽힌다. 그래서 **작은 배수**
     로 간다 — 지표마다 제 띠와 제 y 축을 갖고, **시간축만 공유**한다.
     확대·이동은 그 공유 시간축에 건다.                                   */
let scoreWin = null;
const MET = [
  {k:"rz",    name:"고도 오차 RMSE",   unit:"m",     col:"#F2AA4C"},
  {k:"rv",    name:"속도 오차 RMSE",   unit:"m/s",   col:"#4fb894"},
  {k:"dz",    name:"고도 편차 z−z_ref", unit:"m",    col:"#2d94bd"},
  {k:"om",    name:"각속도 |ω|",       unit:"rad/s", col:"#e0664f"},
  {k:"tiltEx",name:"기울임 (트림 대비)", unit:"°",   col:"#9085e9"},
  {k:"alpha", name:"받음각 α",          unit:"°",    col:"#c9a227"},
];
function metricSeries(){
  const S = rmseSeries();
  const off = REC.length - S.t.length;
  const out = {t:[], rv:[], rz:[], om:[], tiltEx:[], alpha:[], dz:[], satHi:[], satLo:[]};
  for (let i = 0; i < REC.length; i++){
    const r = REC[i], j = i - off;
    out.t.push(r.t);
    out.rv.push(j >= 0 ? S.v[j] : null);
    out.rz.push(j >= 0 ? S.z[j] : null);
    out.om.push(Math.hypot(r.x[10], r.x[11], r.x[12]));
    out.tiltEx.push(tiltOf(r) - trimTiltDeg(r.V));
    out.alpha.push(r.al);
    out.dz.push(r.x[2] - r.zr);
    out.satHi.push(r.satHi ? 1 : 0);
    out.satLo.push(r.satLo ? 1 : 0);
  }
  return out;
}
function scoreWinDoc(){
  return "<!doctype html><html lang='ko'><head><meta charset='utf-8'>"
   + "<title>미션 채점</title><style>"
   + ":root{--bg:#101820;--fg:#e8edf2;--dim:#7C8A98;--line:#1e2a35;--gold:#F2AA4C;"
   + "--ok:#4fb894;--bad:#e0664f;--warn:#F2AA4C}"
   + "*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);"
   + "font:400 13px/1.55 -apple-system,'Segoe UI',system-ui,sans-serif}"
   + ".wrap{padding:14px 16px 28px;max-width:1100px;margin:0 auto}"
   + "h1{font:600 15px/1.3 inherit;margin:0 0 3px}"
   + ".sub{color:var(--dim);font-size:12px;margin:0 0 12px}"
   + ".chips{display:flex;gap:7px;flex-wrap:wrap;margin:0 0 12px}"
   + ".chip{font:600 11px/1 'IBM Plex Mono',monospace;padding:5px 9px;border-radius:3px;"
   + "background:#18232e;color:var(--dim);border:1px solid var(--line)}"
   + ".chip.ok{color:var(--ok);border-color:#24463c}.chip.bad{color:var(--bad);border-color:#4a2a26}"
   + ".chip.warn{color:var(--warn);border-color:#4a3a1e}"
   + ".toggles{display:flex;gap:12px;flex-wrap:wrap;margin:0 0 10px;font-size:12px}"
   + ".toggles label{display:inline-flex;align-items:center;gap:5px;cursor:pointer;color:var(--dim)}"
   + ".toggles input{accent-color:var(--gold)}"
   + "#plots{border:1px solid var(--line);border-radius:4px;background:#0c141b;cursor:crosshair}"
   + ".hint{color:var(--dim);font-size:11.5px;margin:7px 0 16px}"
   + "table{border-collapse:collapse;width:100%;font-size:12px;margin:0 0 14px}"
   + "th,td{padding:5px 8px;border-bottom:1px solid var(--line);text-align:right;"
   + "font-family:'IBM Plex Mono',monospace;font-variant-numeric:tabular-nums}"
   + "th{color:var(--dim);font-weight:500;font-size:10.5px;letter-spacing:.05em;"
   + "text-transform:uppercase;font-family:inherit}"
   + "th:first-child,td:first-child{text-align:left;font-family:inherit}"
   + ".why{color:var(--bad);font-size:12px;margin:0 0 10px}"
   + ".why.warn{color:var(--warn)}"
   + "button{font:600 11.5px/1 inherit;padding:7px 11px;border-radius:3px;cursor:pointer;"
   + "background:#18232e;color:var(--fg);border:1px solid var(--line)}"
   + "button:hover{border-color:var(--gold)}"
   + "</style></head><body><div class='wrap'>"
   + "<h1>미션 채점</h1><p class='sub' id='sub'></p>"
   + "<div class='chips' id='chips'></div><div id='why'></div>"
   + "<div class='toggles' id='tg'></div>"
   + "<canvas id='plots'></canvas>"
   + "<p class='hint'>휠 = 시간축 확대·축소 · 끌기 = 이동 · 더블클릭 = 처음으로</p>"
   + "<div id='segtab'></div>"
   + "<button type='button' id='csv'>CSV 저장</button>"
   // 닫는 태그 셋(script·body·html)을 모두 역슬래시로 끊어 쓴다. 이 문자열은
   // 그냥 데이터인데, 파일을 뒤져 태그를 끼워 넣는 쪽(build_site.py 가 목록
   // 링크를 넣는다)이 여기를 **진짜 문서의 끝**으로 오해해 문자열 한가운데를
   // 잘랐다. 실제로 GitHub Pages 판이 SyntaxError 로 통째로 죽었다.
   // 주의: 이 주석에도 닫는 태그를 그대로 쓰면 안 된다 — HTML 파서는 주석
   // 안이라도 그 자리에서 스크립트를 끝낸다. 고치다가 실제로 또 깨뜨렸다.
   + "</div><script>" + scoreWinScript() + "<\/script><\/body><\/html>";
}
function scoreWinScript(){
  return `
let DATA = null, MET = [], on = {}, t0 = null, t1 = null, full = null;
const $ = s => document.querySelector(s);
addEventListener("message", e => {
  if (!e.data || e.data.kind !== "score") return;
  const first = DATA === null;
  DATA = e.data;
  MET = DATA.met;
  if (first){
    for (const m of MET) on[m.k] = (m.k === "rz" || m.k === "rv" || m.k === "om");
    $("#tg").innerHTML = MET.map(m =>
      "<label><input type='checkbox' data-k='" + m.k + "'" + (on[m.k] ? " checked" : "")
      + "><span style='color:" + m.col + "'>\\u25A0</span>" + m.name + " [" + m.unit + "]</label>"
    ).join("");
    $("#tg").addEventListener("change", ev => {
      if (!ev.target.dataset.k) return;
      on[ev.target.dataset.k] = ev.target.checked; draw();
    });
    $("#csv").addEventListener("click", () => {
      try { opener.__saveCsvFromWin(); } catch (_) {}
    });
  }
  const T = DATA.s.t;
  const nf = [T[0], T[T.length-1]];
  if (!full || Math.abs(full[1]-nf[1]) > 1e-9){
    const stick = (t1 === null) || (full && Math.abs(t1 - full[1]) < 1e-9);
    full = nf;
    if (t0 === null || stick){ t0 = full[0]; t1 = full[1]; }
  }
  render();
});
function render(){
  const d = DATA, v = d.v;
  $("#sub").textContent = d.meta;
  const chip = (t, c) => "<span class='chip " + c + "'>" + t + "</span>";
  $("#chips").innerHTML =
      chip("비행 건전성 " + (v.soundOk ? "통과" : "실패"), v.soundOk ? "ok" : "bad")
    + chip("미션 " + (v.pass ? "통과" : "실패"), v.pass ? "ok" : "warn")
    + chip("최고 " + v.vmax.toFixed(1) + " m/s · " + (v.vmax*3.6).toFixed(0) + " km/h", "")
    + chip("RMSE z " + v.rmseZ.toFixed(2) + " m", "")
    + chip("RMSE v " + v.rmseV.toFixed(2) + " m/s", "")
    + chip("|\\u03C9|max " + v.maxOm.toFixed(1), v.maxOm > 35 ? "bad" : "ok")
    + chip("run25 " + (v.run25*1000).toFixed(0) + " ms", v.run25 >= 0.2 ? "bad" : "ok")
    + chip("포화 " + (100*v.satHi).toFixed(0) + "/" + (100*v.satLo).toFixed(0) + "%",
           (v.satHi+v.satLo) > 0.2 ? "bad" : "ok");
  let w = "";
  if (v.sound.length) w += "<p class='why'><b>건전성 실패</b> \\u00B7 " + v.sound.join(" \\u00B7 ") + "</p>";
  if (v.miss.length)  w += "<p class='why warn'><b>미션 미달</b> \\u00B7 " + v.miss.join(" \\u00B7 ") + "</p>";
  $("#why").innerHTML = w;
  let st = "<table><tr><th>구간</th><th>표본</th><th>RMSE v</th><th>RMSE z</th>"
         + "<th>최대 \\u0394z</th><th>|\\u03C9|max</th></tr>";
  for (const g of v.bySeg)
    st += "<tr><td>" + g.name + "</td><td>" + g.n + "</td><td>" + g.rmseV.toFixed(2)
        + "</td><td>" + g.rmseZ.toFixed(2) + "</td><td>" + g.maxZ.toFixed(1)
        + "</td><td>" + g.maxOm.toFixed(1) + "</td></tr>";
  $("#segtab").innerHTML = st + "</table>";
  draw();
}
function draw(){
  const cv = $("#plots"), s = DATA.s;
  const sel = MET.filter(m => on[m.k]);
  const rowH = 104, W = Math.max(360, document.querySelector(".wrap").clientWidth);
  const H = Math.max(rowH, sel.length * rowH) + 24;
  const r = Math.min(devicePixelRatio, 2);
  cv.width = W*r; cv.height = H*r; cv.style.width = W + "px"; cv.style.height = H + "px";
  const c = cv.getContext("2d"); c.setTransform(r,0,0,r,0,0);
  c.fillStyle = "#0c141b"; c.fillRect(0,0,W,H);
  if (!sel.length){
    c.fillStyle = "#7C8A98"; c.font = "13px system-ui"; c.textAlign = "center";
    c.fillText("지표를 하나 이상 고르세요", W/2, H/2); return;
  }
  const L = 56, R = 12;
  const X = t => L + (t - t0)/(t1 - t0 || 1)*(W - L - R);
  // 보이는 구간만 인덱스로 추린다
  let i0 = 0, i1 = s.t.length - 1;
  while (i0 < i1 && s.t[i0] < t0) i0++;
  while (i1 > i0 && s.t[i1] > t1) i1--;
  i0 = Math.max(0, i0 - 1); i1 = Math.min(s.t.length - 1, i1 + 1);
  sel.forEach((m, row) => {
    const yTop = row*rowH + 16, yBot = yTop + rowH - 34;
    const a = s[m.k];
    let lo = Infinity, hi = -Infinity;
    for (let i = i0; i <= i1; i++){
      const v2 = a[i]; if (v2 === null || !isFinite(v2)) continue;
      if (v2 < lo) lo = v2; if (v2 > hi) hi = v2;
    }
    if (!isFinite(lo)){ lo = 0; hi = 1; }
    if (hi - lo < 1e-9){ hi = lo + 1; }
    const pad = (hi - lo)*0.12; lo -= pad; hi += pad;
    const Y = v2 => yBot - (v2 - lo)/(hi - lo)*(yBot - yTop);
    c.strokeStyle = "#1a2530"; c.lineWidth = 1;
    c.beginPath(); c.moveTo(L, yBot + .5); c.lineTo(W - R, yBot + .5); c.stroke();
    if (lo < 0 && hi > 0){
      c.strokeStyle = "#243140"; c.beginPath();
      c.moveTo(L, Y(0) + .5); c.lineTo(W - R, Y(0) + .5); c.stroke();
    }
    c.font = "500 10px 'IBM Plex Mono',monospace"; c.textAlign = "right";
    c.textBaseline = "middle"; c.fillStyle = "#7C8A98";
    c.fillText(hi.toFixed(Math.abs(hi) < 10 ? 2 : 0), L - 6, yTop);
    c.fillText(lo.toFixed(Math.abs(lo) < 10 ? 2 : 0), L - 6, yBot);
    c.textAlign = "left"; c.fillStyle = m.col;
    c.font = "600 11px system-ui";
    c.fillText(m.name + "  [" + m.unit + "]", L, yTop - 7);
    c.strokeStyle = m.col; c.lineWidth = 1.6; c.beginPath();
    let started = false;
    for (let i = i0; i <= i1; i++){
      const v2 = a[i];
      if (v2 === null || !isFinite(v2)){ started = false; continue; }
      const x = X(s.t[i]), y = Y(v2);
      if (!started){ c.moveTo(x, y); started = true; } else c.lineTo(x, y);
    }
    c.stroke();
  });
  // 시간 눈금
  c.strokeStyle = "#1a2530"; c.fillStyle = "#7C8A98";
  c.font = "500 10px 'IBM Plex Mono',monospace"; c.textAlign = "center";
  c.textBaseline = "top";
  const span = t1 - t0, stepRaw = span/6;
  const mag = Math.pow(10, Math.floor(Math.log10(Math.max(stepRaw, 1e-9))));
  const step = [1,2,5,10].map(k => k*mag).find(v2 => v2 >= stepRaw) || mag*10;
  for (let t = Math.ceil(t0/step)*step; t <= t1 + 1e-9; t += step)
    c.fillText(t.toFixed(step < 1 ? 1 : 0) + " s", X(t), H - 18);
}
const cv = $("#plots");
cv.addEventListener("wheel", e => {
  if (!DATA) return;
  e.preventDefault();
  const r2 = cv.getBoundingClientRect(), L = 56, R = 12;
  const f = Math.max(0, Math.min(1, (e.clientX - r2.left - L)/(r2.width - L - R)));
  const at = t0 + f*(t1 - t0);
  const k = e.deltaY > 0 ? 1.18 : 1/1.18;
  let a2 = at - (at - t0)*k, b2 = at + (t1 - at)*k;
  const minSpan = (full[1] - full[0])/500;
  if (b2 - a2 < minSpan){ a2 = at - minSpan/2; b2 = at + minSpan/2; }
  t0 = Math.max(full[0], a2); t1 = Math.min(full[1], b2);
  if (t1 - t0 < minSpan) t1 = Math.min(full[1], t0 + minSpan);
  draw();
}, {passive:false});
let drag = null;
cv.addEventListener("pointerdown", e => {
  if (!DATA) return;
  drag = {x:e.clientX, a:t0, b:t1}; cv.setPointerCapture(e.pointerId);
});
cv.addEventListener("pointermove", e => {
  if (!drag) return;
  const r2 = cv.getBoundingClientRect(), L = 56, R = 12;
  const per = (drag.b - drag.a)/(r2.width - L - R);
  let dt = -(e.clientX - drag.x)*per;
  dt = Math.max(full[0] - drag.a, Math.min(full[1] - drag.b, dt));
  t0 = drag.a + dt; t1 = drag.b + dt; draw();
});
cv.addEventListener("pointerup", e => {
  drag = null; try { cv.releasePointerCapture(e.pointerId); } catch(_){}
});
cv.addEventListener("dblclick", () => { if (full){ t0 = full[0]; t1 = full[1]; draw(); } });
addEventListener("resize", () => { if (DATA) draw(); });
`;
}
function pushScore(){
  if (!scoreWin || scoreWin.closed) return;
  const v = score();
  if (!v) return;
  scoreWin.postMessage({kind:"score", v: v, s: metricSeries(), met: MET,
    meta: CTRLS[CTRL].name + " · 목표 " + $("#spd").value + " m/s · 고도 "
          + $("#alt").value + " m · 측풍 " + $("#wsp").value + " m/s "
          + $("#wdir").value + "°"}, "*");
}
function openScoreWin(){
  // 이미 떠 있으면 앞으로 가져온다
  if (scoreWin && !scoreWin.closed){ scoreWin.focus(); pushScore(); return true; }
  scoreWin = window.open("", "fastdrone_score", "width=1080,height=860");
  if (!scoreWin) return false;       // 팝업 차단 (Artifact 뷰어에서 흔하다)
  scoreWin.document.open();
  scoreWin.document.write(scoreWinDoc());
  scoreWin.document.close();
  setTimeout(pushScore, 60);
  return true;
}
function drawScore(){
  const el = $("#score");
  if (el.hidden) return;
  drawScoreCv();
  const s2 = score();
  if (!s2){ $("#scoreBody").innerHTML =
    "<p>기록이 모자랍니다. ▶ 로 좀 더 날려보세요.</p>"; return; }
  const row = (k, v, cls) => "<tr><td>" + k + "</td><td"
    + (cls ? " class='" + cls + "'" : "") + ">" + v + "</td></tr>";
  const pct = v => (100*v).toFixed(0) + "%";
  let html = "<table>"
    + row("최고 속도", s2.vmax.toFixed(1) + " m/s · " + (s2.vmax*3.6).toFixed(0) + " km/h")
    + row("속도 RMSE (전 구간)", s2.rmseV.toFixed(2) + " m/s", s2.rmseV < 3 ? "ok" : "warn")
    + row("고도 RMSE (전 구간)", s2.rmseZ.toFixed(2) + " m",
          s2.rmseZ < 5 ? "ok" : (s2.rmseZ < 20 ? "warn" : "bad"))
    + row("최대 |ω|", s2.maxOm.toFixed(2) + " rad/s", s2.maxOm > 35 ? "bad" : "ok")
    + row("|ω|&gt;25 최장연속", (s2.run25*1000).toFixed(0) + " ms"
          + (s2.ms25 > s2.run25 + 1e-9 ? " (총합 " + (s2.ms25*1000).toFixed(0) + ")" : ""),
          s2.run25 >= 0.2 ? "bad" : "ok")
    + row("기울임 (트림 대비)", s2.maxTilt.toFixed(0) + "° · 초과 "
          + (s2.maxTiltEx >= 0 ? "+" : "") + s2.maxTiltEx.toFixed(0) + "°",
          s2.maxTiltEx > 15 ? "bad" : (s2.maxTiltEx > 8 ? "warn" : "ok"))
    + row("모터 포화 (천장/바닥)", pct(s2.satHi) + " / " + pct(s2.satLo),
          (s2.satHi + s2.satLo) > 0.2 ? "bad" : ((s2.satHi + s2.satLo) > 0.02 ? "warn" : "ok"))
    + row("지면 접촉", s2.crashed ? "있음" : "없음", s2.crashed ? "bad" : "ok")
    + row("발산", s2.diverged ? s2.divWhy : "없음", s2.diverged ? "bad" : "ok")
    + row("비행 건전성", s2.soundOk ? "통과" : "실패", s2.soundOk ? "ok" : "bad")
    + row("미션 달성", s2.pass ? "통과" : "실패", s2.pass ? "ok" : "warn")
    + "</table>";
  if (s2.sound.length)
    html += "<p class='bad'><b>건전성 실패</b><br>· " + s2.sound.join("<br>· ") + "</p>";
  if (s2.miss.length)
    html += "<p class='warn'><b>미션 미달</b><br>· " + s2.miss.join("<br>· ") + "</p>";
  if (s2.bySeg.length > 1){
    html += "<table class='segtab'><tr><th>구간</th><th>표본</th>"
          + "<th>RMSE v</th><th>RMSE z</th><th>최대 Δz</th><th>|ω|max</th></tr>";
    for (const g of s2.bySeg)
      html += "<tr><td>" + g.name + "</td><td>" + g.n + "</td><td>"
            + g.rmseV.toFixed(2) + "</td><td>" + g.rmseZ.toFixed(2) + "</td><td>"
            + g.maxZ.toFixed(1) + "</td><td>" + g.maxOm.toFixed(1) + "</td></tr>";
    html += "</table>";
  }
  html += "<p>전 구간 " + s2.n + " 샘플 · 대표값은 <b>목표 고도 도달 이후</b> "
    + s2.nSettled + " 샘플로 냅니다"
    + (s2.tSettle !== null ? " (도달 t=" + s2.tSettle.toFixed(0) + " s)" : " — 아직 도달 못함")
    + ".</p>"
    + "<p><b>왜 구간별로 보나.</b> 예전에는 순항 구간만 집계했는데, 구간 판정이 "
    + "고도 오차 5 m 초과를 '상승' 으로 걸러낸 <b>뒤</b> 순항만 보므로 "
    + "고도 RMSE 가 5 이하로 <b>구조적으로 보장</b>됐습니다. 제일 어려운 "
    + "구간(감속·호버 복귀)이 통째로 빠져 있었습니다.</p>"
    + "<p><b>RMSE 만 보면 안 됩니다</b> — 텀블은 |ω| 로만 잡힙니다. 실기 실패 "
    + "기준은 |ω| &gt; 35 rad/s 또는 25 초과가 <b>연속</b> 200 ms 이상입니다 "
    + "(총합이 아니라 최장연속 — 파이썬 판정과 같은 정의).</p>";
  $("#scoreBody").innerHTML = html;
  pushScore();
}
function paint(d){
  render3D(d);
  drawSegs();
  drawScore();
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
  if (sat > .5) warn = "⚠ 로터가 최대 회전수에 도달했습니다 — 추력 포화";
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
    + "<div><span>LQR 표 ↔ 파이썬</span><b>"
    + lqrCheck().toExponential(1) + "</b></div>"
    + "<p>같은 물리를 세 번 구현했습니다. 이 값은 구현끼리의 최대 차이이고,"
    + " 기계정밀도 수준이면 셋이 같은 답을 낸다는 뜻입니다.</p>"
    + "<p><b>무엇을 보증하나.</b> 검증 구간 " + D.segs.length + " 개가"
    + " 대기속도 0~76 m/s, 받음각 0~160°, |ω| 0~178 rad/s, 전진비 계수"
    + " fac 0.000~1.000 을 훑습니다. 예전에는 구간이 하나뿐이라 fac 가"
    + " 0.999~1.000 이었고, 그래서 할당이 전진비를 무시하던 버그가 이 배지를"
    + " 그대로 통과했습니다.</p>"
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
  const sel = $("#ctrl");
  sel.innerHTML = Object.entries(CTRLS).map(([k, c]) =>
    "<option value='" + k + "'>" + c.name + "</option>").join("");
  const updCtrl = () => {
    CTRL = sel.value;
    $("#ctrlNote").textContent = CTRLS[CTRL].note;
    omPrev = [0,0,0]; omDotF = [0,0,0]; Mprev = [0,0,0]; trimOK = false; trimV = -1;
  nmpcU = null; nmpcLast = -1e9; nmpcOut = null; nmpcApplied = null;   // 전환 시 INDI 초기화
  rtiU = null; rtiLast = -1e9; rtiOut = null; rtiApplied = null; rtiStat = null;
  };
  sel.addEventListener("change", updCtrl); updCtrl();
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
  $("#scoreBtn").addEventListener("click", () => {
    const el = $("#score"); el.hidden = !el.hidden; drawScore();
  });
  $("#scoreClose").addEventListener("click", () => { $("#score").hidden = true; });
  // 내려받기는 두 곳에서 다르게 동작한다.
  //   GitHub Pages — 평범한 페이지라 <a download> 가 그냥 된다.
  //   Artifact 뷰어 — 샌드박스가 <a download> 와 blob 저장을 막는다.
  //                   대신 downloads 능력을 선언하고 claude.use 로 저장한다.
  // 둘 다 안 되면 버튼이 그렇게 말한다. 조용히 아무 일도 안 하는 것이 최악이다.
  $("#expWin").addEventListener("click", e => {
    const ok = openScoreWin();
    if (!ok){
      e.target.textContent = "팝업 차단됨";
      setTimeout(() => { e.target.textContent = "별창으로 크게"; }, 2200);
    }
  });
  // 별창의 CSV 버튼이 부모의 저장 경로를 그대로 쓰게 한다. 저장은 뷰어마다
  // 되고 안 되고가 달라(아래 expCsv 주석) 한 곳에 모아 둔다.
  window.__saveCsvFromWin = () => $("#expCsv").click();
  $("#expCsv").addEventListener("click", async e => {
    const name = "flight_score_" + CTRL + "_" + Math.round(T) + "s.csv";
    const text = scoreCsv();
    let dl = null;
    try { dl = window.claude && claude.use ? await claude.use("downloads") : null; }
    catch (_) { dl = null; }
    if (dl){
      try {
        await dl.save({filename:name, data:new Blob([text], {type:"text/csv"})});
        e.target.textContent = "저장됨";
      } catch (err) {
        e.target.textContent = (err && err.code === "declined") ? "취소됨" : "저장 실패";
      }
    } else {
      const b = new Blob([text], {type:"text/csv;charset=utf-8"});
      const a = document.createElement("a");
      a.href = URL.createObjectURL(b); a.download = name;
      document.body.appendChild(a); a.click(); a.remove();
      setTimeout(() => URL.revokeObjectURL(a.href), 4000);
      e.target.textContent = "저장됨";
    }
    setTimeout(() => { e.target.textContent = "CSV 저장"; }, 1800);
  });
  $("#expCopy").addEventListener("click", async e => {
    // ⚠ 내려받기는 Artifact 보기창에서 막힌다 (샌드박스). 복사는 된다.
    //   GitHub Pages 로 열면 둘 다 된다.
    try { await navigator.clipboard.writeText(scoreCsv());
          e.target.textContent = "복사됨"; }
    catch (_) { e.target.textContent = "복사 실패"; }
    setTimeout(() => { e.target.textContent = "값 복사"; }, 1600);
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
    reset(); trailN = 0; trailDrawn = 0; trail.geometry.setDrawRange(0,0); paint(diag());
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
    restoreTo(viewIdx);
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
