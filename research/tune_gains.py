"""CPID · GINDI · GSLQR 게인 튜닝 — 논문 §5.3 프로토콜의 코드화.

논문이 요구하는 것은 세 가지이고, 이 파일은 그 셋을 **구조로** 강제한다.

  1. "모든 제어기는 **독립적인 튜닝 시나리오**에서" (§5.3)
     -> TUNING_SCENARIOS 는 표8 본시험(sweep.cjs CONDITIONS)과 속도·바람·난수가
        모두 겹치지 않는다. 겹침 여부는 test_tune_gains.py 가 검사한다.
  2. "**같은 튜닝 예산**을 부여받고" (§5.3)
     -> BUDGET 은 제어기당 동일한 '폐루프 평가 횟수'다. 탐색 알고리즘도 셋이 같다
        (아래 곱셈 나침반 탐색). 제어기마다 다른 탐색을 쓰면 예산이 같아도
        공정하지 않다. 단 한 가지 비대칭은 명시해 둔다 — GSLQR 은 ARE 고윳값으로
        폐루프 불안정을 **시뮬레이션 없이** 걸러낼 수 있어서, 그 후보는 예산을
        쓰지 않는다. 연산 예산은 같고 탐색 시도 횟수는 GSLQR 이 많아질 수 있다.
        (실측: 개정 1 에서 기각 0건이라 이 비대칭은 실제로 발생하지 않았다.)
  3. "**본시험 결과를 보고 가중치를 다시 선택하지 않는다**" (§5.3),
     "학습·튜닝·본시험 집합 분리" (§5.2)
     -> 이 드라이버는 표8 조건을 아예 읽지 않는다. 산출물(tuned_gains.json)에
        탐색 이력을 그대로 남겨 나중에 "본시험 보고 골랐는지"를 검증할 수 있게 한다.

목적함수는 sweep.cjs 의 evaluateRun 을 그대로 쓴다(tune_eval.cjs 가 import).
튜닝이 본시험과 다른 것을 최적화하면 결과 해석이 불가능해지기 때문이다.

한계 — 이 파일이 주장하지 않는 것:
  · 전역 최적이 아니다. 예산 안에서의 무작위+국소 탐색 결과다.
  · 게인이 '검증됐다'는 뜻이 아니다. 안정성 증명도 강건성 보장도 아니다.
  · 튜닝 시나리오에서 좋은 값이 본시험에서 좋다는 보장은 없다. 그 차이가 곧
    일반화 성능이고, 프로토콜이 그것을 정직하게 재려고 집합을 나눈 것이다.

실행:  python3 -m research.tune_gains --budget 64
       python3 -m research.tune_gains --controllers gslqr --budget 16   (부분 실행)
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np

from research.gain_schedule import design, linearize, reduce, trim_point
from research.model import build, profile
from research.trim_envelope import LevelTrimAudit

RESEARCH = Path(__file__).parent

# ── 튜닝 시나리오 — 표8(V 8/14, 측풍 5·7, 수직풍 ±5, seed 1000+)과 겹치지 않는다 ──
#
# 개정 2 (2026-09-22). 개정 1 은 "겹치지 않을 것"만 지키고 **강도를 맞추지 않아서**
# 일반화에 실패했다. 개정 1 의 최강 외란은 측풍 3 @ V=13(권한 소모 52.6%)인데
# 본시험 최강은 측풍 7 @ V=14(84.8%)였다. 결과: gindi 가 튜닝 세트에서 13.8배
# 좋아지고 본시험에서는 4개 조건이 악화(측풍 조건에서 RMSE_v 1.445 -> 10.964,
# 측풍 7 은 실패)했다 — 교과서적 과적합이고, 훈련/시험 분리가 그걸 잡아냈다.
#
# ⚠ 이 개정은 **프로토콜 결함을 고친 것**이지 가중치를 다시 고른 것이 아니다
#    (§5.3 이 금지하는 것은 후자다). 다만 본시험 세트를 두 번 쓰게 되므로
#    그 사실을 GAIN_TUNING.md 에 명시한다. 개정 1 의 결과도 지우지 않고 남긴다.
#
# 이제 권한 소모율이 8.6% ~ 92.0% 로 본시험 범위(11.8% ~ 84.8%)를 **양쪽에서
# 감싼다**. 속도도 7~15 로 본시험 8·14 를 감싸므로 본시험이 외삽이 아니라 내삽이다.
TUNING_SCENARIOS = [
    {"id": "T1", "speed": 7, "seed": 2000, "opts": {}},                                    # 8.6%
    {"id": "T2", "speed": 12, "seed": 2001, "opts": {}},                                   # 33.3%
    {"id": "T3", "speed": 15, "seed": 2002, "opts": {}},                                   # 57.9%
    {"id": "T4", "speed": 13, "seed": 2003, "opts": {"wind_speed": 8, "wind_angle": 90}},  # 77.5%
    {"id": "T5", "speed": 15, "seed": 2004, "opts": {"wind_speed": 6, "wind_angle": 90}},  # 92.0%
    # 파라미터 오차 일반화용 — 본시험의 질량 1.2배·모터 tau 2배와 값이 겹치지 않게.
    {"id": "T6", "speed": 12, "seed": 2005,
     "opts": {"wind_vertical_mps": 4, "scales": [1.1, 1, 1, 1, 1, 1.5]}},                  # 30.0%
]
RUN_SETTINGS = {"altitude": 20, "ramp_t0": 1, "ramp_duration_s": 3, "seconds": 8}
SCHEDULE_SPEEDS = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18]   # build_bundle.py 와 동일 격자

# ── 탐색: 곱셈 나침반 탐색(compass search) ──
# 처음엔 무작위 로그균등 표본(밴드 x8)을 썼는데 10개 게인을 동시에 흔드니 후보가
# 전부 발산했다(실측: 표본 2개 모두 4런 전부 실패). 기준선 자체가 안정 한계 근처라
# (kR 2배면 발산) 넓은 동시 섭동이 성립하지 않는다. 그래서 기준선에서 출발해
# 한 번에 한 게인만 x STEP / ÷ STEP 해보고 좋아지면 채택하는 방식으로 바꿨다.
# 한 바퀴 돌아 개선이 없으면 보폭을 줄인다(제곱근). 결정론적이라 재현된다.
INITIAL_STEP = 2.0
MIN_STEP = 1.05
SEARCH_SEED = 20260922   # 현재 탐색은 결정론적이라 미사용. 기록용으로 남긴다.

CPID_BASE = {"kpV_h": 1.4, "kpV_z": 4.0, "kiV_h": .15, "kiV_z": .6,
             "kdV_h": .1, "kdV_z": .2, "kz": 1.8, "kR": 8.0, "kpW": 45.0, "kiW": 5.0}
GINDI_BASE = {**CPID_BASE, "kR": 3.0, "kpW": 15.0, "kiW": 2.0}
# GSLQR 은 게인이 아니라 LQR 가중치가 튜닝 대상이다(식40-42).
GSLQR_BASE = {"q_z": 100., "q_v_h": 10., "q_v_z": 20., "q_phi": 50.,
              "q_omega": 5., "q_n": .01, "r": .05}


def expand_pid(g):
    """탐색 파라미터 -> runtime.js 가 받는 게인 dict (x/y 축은 묶어서 다룬다)."""
    return {"kpV": [g["kpV_h"], g["kpV_h"], g["kpV_z"]],
            "kiV": [g["kiV_h"], g["kiV_h"], g["kiV_z"]],
            "kdV": [g["kdV_h"], g["kdV_h"], g["kdV_z"]],
            "kz": g["kz"], "kR": g["kR"], "kpW": g["kpW"], "kiW": g["kiW"]}


def gslqr_weights(g):
    Q = np.diag([g["q_z"], g["q_v_h"], g["q_v_h"], g["q_v_z"],
                 g["q_phi"], g["q_phi"], g["q_phi"],
                 g["q_omega"], g["q_omega"], g["q_omega"], *([g["q_n"]]*4)])
    return Q, np.eye(4)*g["r"]


class ScheduleFactory:
    """트림·선형화는 Q/R 과 무관하므로 한 번만 한다. 후보마다 ARE 만 다시 푼다."""

    def __init__(self, speeds=None):
        p = profile("selected")
        audit, f = LevelTrimAudit(p), build(p)
        self.cache = []
        for v in (SCHEDULE_SPEEDS if speeds is None else speeds):
            x18, n, voltage = trim_point(audit, v)          # 트림 없으면 ValueError
            A17, B17 = linearize(f, x18, n, voltage)
            A_r, B_r, _ = reduce(A17, B17, x18[6:10])
            self.cache.append({"speed": float(v), "A": A_r, "B": B_r,
                               "x_trim": x18[:17].tolist(), "u_trim": n.tolist()})

    def build(self, g):
        """반환: 번들의 data.gslqr 자리에 그대로 넣을 dict. 불안정하면 None."""
        Q, R = gslqr_weights(g)
        rows = []
        for entry in self.cache:
            try:
                K_r, max_real = design(entry["A"], entry["B"], Q, R)
            except Exception:
                return None
            if max_real >= -1e-6:       # 선형 폐루프가 불안정한 가중치는 버린다
                return None
            rows.append((entry, K_r.tolist()))
        return {"nr": 14, "speeds_mps": [e["speed"] for e, _ in rows],
                "K_r": [k for _, k in rows],
                "x_trim": [e["x_trim"] for e, _ in rows],
                "u_trim": [e["u_trim"] for e, _ in rows], "skipped": []}


def evaluate(candidates, tag, out_dir):
    """후보 묶음을 node 평가기에 넘긴다. 반환: id -> 결과 dict."""
    if not candidates:
        return {}
    job = {"profile": "selected", "scenarios": TUNING_SCENARIOS,
           "candidates": candidates, **RUN_SETTINGS}
    job_path, out_path = out_dir/f"job_{tag}.json", out_dir/f"out_{tag}.json"
    job_path.write_text(json.dumps(job))
    subprocess.run(["node", "tune_eval.cjs", str(job_path), str(out_path)],
                   cwd=RESEARCH, check=True)
    return {r["id"]: r for r in json.loads(out_path.read_text())["results"]}


def tune(controller, budget, out_dir, factory=None, start=None):
    """곱셈 나침반 탐색(완전 폴링). 세 제어기 모두 같은 절차·같은 예산.

    한 바퀴(poll)는 각 게인을 x step / ÷ step 한 후보 전부다. 한 바퀴를 한 번의
    평가 호출로 묶어 돌리고(무작위성 없음), 개선이 있으면 그 점으로 옮기고 없으면
    보폭을 줄인다. 예산은 '폐루프 평가 횟수'로 센다 — ARE 가 불안정해 폐루프를
    돌리지도 못한 후보는 예산을 쓰지 않고 rejected 로만 기록한다.
    """
    base = {"cpid": CPID_BASE, "gindi": GINDI_BASE, "gslqr": GSLQR_BASE}[controller]
    keys = list(base)

    def make(params, cid):
        """탐색 파라미터 -> 평가기 후보. 설계가 불가능하면 None."""
        if controller == "gslqr":
            schedule = factory.build(params)
            return None if schedule is None else {"id": cid, "controller": controller,
                                                  "gslqr_schedule": schedule}
        return {"id": cid, "controller": controller, "gains": expand_pid(params)}

    # 기준선(placeholder)은 예산 밖이다 — 탐색 단계가 아니라 비교 기준이라서.
    base_row = evaluate([c for c in [make(base, "baseline")] if c],
                        f"{controller}_base", out_dir).get("baseline")
    # start 를 주면 **출발점만** 옮긴다. 탐색 절차·예산·기준선 보고는 그대로다.
    # 나침반 탐색은 한 번에 한 좌표만 움직이므로 출발점에 따라 다른 국소점에
    # 멈춘다 — gslqr 이 개정1·개정2 에서 각각 q_v_h 와 q_z 한 축만 찾고 그 조합을
    # 한 번도 방문하지 않은 것이 실례다(GAIN_TUNING.md).
    current = dict(start or base)
    start_row = None
    if start:
        start_row = evaluate([c for c in [make(current, "start")] if c],
                             f"{controller}_start", out_dir).get("start")
    current_score = (start_row or base_row or {}).get("score", float("inf"))

    history, spent, rejected, step, poll_index = [], 0, 0, INITIAL_STEP, 0
    while spent < budget and step > MIN_STEP:
        batch, params_by_id = [], {}
        for key in keys:
            for factor in (step, 1/step):
                if spent + len(batch) >= budget:
                    break
                trial = dict(current)
                trial[key] = current[key]*factor
                cid = f"{controller}_p{poll_index}_{len(batch)}"
                cand = make(trial, cid)
                if cand is None:      # 선형 폐루프 불안정 — 폐루프를 돌리지 않으므로 예산 미소모
                    rejected += 1
                    history.append({"id": cid, "params": trial, "score": None,
                                    "rejected": "linear closed loop unstable",
                                    "step": step, "poll": poll_index})
                    continue
                params_by_id[cid] = trial
                batch.append(cand)
        if not batch:
            step = step**.5
            continue
        scored = evaluate(batch, f"{controller}_p{poll_index}", out_dir)
        poll_rows = []
        for cand in batch:
            r = scored[cand["id"]]
            row = {"id": cand["id"], "params": params_by_id[cand["id"]],
                   "score": r["score"], "failures": r["failures"],
                   "mean_rmse_v": r["mean_rmse_v"], "mean_rmse_z": r["mean_rmse_z"],
                   "step": step, "poll": poll_index}
            history.append(row)
            poll_rows.append(row)
        spent += len(batch)
        poll_index += 1
        # 보폭 축소 판정은 **이번 바퀴**만 본다. 과거 최적까지 넣으면 개선이 없어도
        # 계속 개선한 것처럼 보여 보폭이 영원히 안 줄어든다.
        winner = min(poll_rows, key=lambda h: h["score"])
        if winner["score"] < current_score - 1e-12:
            current, current_score = winner["params"], winner["score"]
        else:
            step = step**.5   # 이번 바퀴에서 개선 없음 -> 보폭 축소

    scored = [h for h in history if h["score"] is not None]
    best = min(scored, key=lambda h: h["score"]) if scored else None
    return {"controller": controller, "budget": budget, "evaluations_spent": spent,
            "start": {"params": start or base,
                      "source": "placeholder" if start is None else "given",
                      "score": (start_row or base_row or {}).get("score")},
            "search": {"method": "multiplicative compass search, complete polling",
                       "initial_step": INITIAL_STEP, "min_step": MIN_STEP,
                       "polls": poll_index, "final_step": step,
                       "rejected_unstable_no_budget": rejected,
                       "parameters": len(keys)},
            "baseline": {"params": base, "score": base_row["score"] if base_row else None,
                         "failures": base_row["failures"] if base_row else None,
                         "mean_rmse_v": base_row["mean_rmse_v"] if base_row else None,
                         "mean_rmse_z": base_row["mean_rmse_z"] if base_row else None},
            "best": best, "history": history}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--budget", type=int, default=64, help="제어기당 폐루프 평가 횟수")
    parser.add_argument("--controllers", default="cpid,gindi,gslqr")
    parser.add_argument("--start-from", choices=["placeholder", "selected"], default="placeholder",
                        help="탐색 출발점. selected 면 results/tuned_gains.json 의 "
                             "selection.chosen 값에서 출발한다(절차·예산은 동일).")
    parser.add_argument("--out", type=Path,
                        # results/ 로 둔다 — research/generated/ 는 43MB 번들 때문에
                        # gitignore 대상이라, 거기 두면 논문 재현의 근거인 이 기록이
                        # git 에 안 남는다.
                        default=RESEARCH.parent/"results"/"tuned_gains.json")
    parser.add_argument("--work", type=Path, default=None, help="중간 파일 디렉터리")
    args = parser.parse_args()

    out_dir = args.work or (RESEARCH/"generated"/"tuning_work")
    out_dir.mkdir(parents=True, exist_ok=True)
    controllers = args.controllers.split(",")
    factory = ScheduleFactory() if "gslqr" in controllers else None

    protocol = {
        "source": "논문 §5.3 '독립적인 튜닝 시나리오 + 같은 튜닝 예산 + 본시험 비참조'",
        "tuning_scenarios": TUNING_SCENARIOS, "run_settings": RUN_SETTINGS,
        "budget_per_controller": args.budget,
        "objective": "sweep.cjs evaluateRun 의 (RMSE_v + RMSE_z) 평균 + 실패 벌점",
        "test_set_not_consulted": True}
    # 제어기 하나가 끝날 때마다 즉시 저장하고 기존 파일과 합친다. 전체가 끝난 뒤에
    # 한 번만 쓰면, 중간에 프로세스가 죽을 때 이미 끝난 제어기의 결과까지 함께
    # 날아간다(2026-09-22 개정2 실행에서 실제로 gslqr 도중 중단돼 cpid·gindi
    # 결과를 산출물에서 복원해야 했다). --controllers 로 나눠 돌리는 것도 이 덕에
    # 가능해진다.
    report = (json.loads(args.out.read_text()) if args.out.exists()
              else {"protocol": protocol, "results": {}})
    report["protocol"] = protocol
    chosen = {}
    if args.start_from == "selected":
        chosen = json.loads(args.out.read_text()).get("selection", {}).get("chosen", {})
    for name in controllers:
        started = time.time()
        result = tune(name, args.budget, out_dir, factory,
                      start=chosen.get(name, {}).get("params"))
        result["wall_seconds"] = round(time.time()-started, 1)
        report["results"][name] = result
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(f"[{name}] 완료 {result['wall_seconds']}s "
              f"기준선={result['baseline']['score']} "
              f"최적={result['best']['score'] if result['best'] else None} "
              f"-> {args.out}", flush=True)
    print(f"saved: {args.out} ({', '.join(report['results'])})")


if __name__ == "__main__":
    main()
