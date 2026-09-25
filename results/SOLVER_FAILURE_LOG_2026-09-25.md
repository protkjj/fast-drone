# 작업 로그 — NMPC 계열 실패 원인 규명 (2026-09-25 저녁~)

지시서: kj, 2026-09-25 18:xx. 최종 산출물은 `results/SOLVER_FAILURE_REPORT_2026-09-25.md` 하나.
이 로그는 과정 기록이며 최종 판단은 보고서에만 담는다(0단계 원칙: V13을 이기게 만들지 않는다).

## 타임라인

### 18:01 시작
- 직전 체크포인트 커밋(5ff211f): `control/kh_adapter.py`, `control/kh_repro.py`,
  `control/test_kh_convention_match.py` — 팀원(규현) 기체 `KYUHYUNKANG03/fastdrone_kh`
  기준 재현 하네스. 전체 테스트 118 passed, 1 error(사전 존재, `test_fallback.py`
  `test_recovery_resets` fixture 누락 — 8/18 커밋부터 있던 문제, 이번 작업과 무관 확인).
- 새 지시서: 팀원 저장소가 `kms301111/fast-drone-control`로 바뀜(규현 기체 모델 +
  민석 시뮬레이션 코드, control 브랜치 기반). 기존 `KYUHYUNKANG03/fastdrone_kh` 작업과의
  관계 확인 필요 — 같은 기체를 다른 사람이 이어받은 것인지, 별개인지부터 본다.

### 18:10 kms301111/fast-drone-control 정체 확인 — 중요 발견 2건

**발견 1 (좋은 소식)**: `kms301111/fast-drone-control`은 별개 저장소가 아니라
**우리 `protkjj/fast-drone`의 control 브랜치 자체를 민석이 포크**한 것이다
(공통조상 593b017 = 오늘 새벽 내가 만든 커밋, 로컬 HEAD가 그 후손임을
`git merge-base --is-ancestor`로 확인). 그 안의 `models/team_light/` 는
규현의 `fastdrone_kh`(커밋 34981c5, SHA256 `13a1dcc6...`)를 **내가 이미 벤더링한
것과 완전히 동일한 소스·동일한 해시**로 다시 벤더링한 것(경로만
`kh_control`↔`models.team_light.control`로 다름). → **`control/kh_adapter.py`의
최소제곱 공력 적합·좌표계 일치 테스트는 그대로 재사용 가능, 단계1(최소 이식)
재작업 불필요.**

**발견 2 (주의 필요 — 결정 필요 후보)**: 민석의 `docs/TIMING_OPTIMIZATION.md`
(가감속 완화 시험)는 제목에 "팀원 제공 light_rocket_v2_pack_forward 명목 모델"
이라 적었지만, 실제로 그 실험이 돌린 `control/mission_sim.py`·`control/gust_comparison.py`는
`from control.vehicle_params import vehicle_params as P`로 **모듈 최상위의
옛 8kg/30cm-프로펠러 플레이스홀더 딕셔너리**를 그대로 쓴다(규현 기체 아님 —
직접 확인: `vehicle_params['mass']==8.0`, `D_prop==0.3`, 양쪽 저장소 동일).
규현의 실제 기체(1.174kg)를 쓴 코드는 `control/validation_suite.py`·
`control/timing_study.py` 뿐인데, 이 둘은 `models.team_light.control.run_baseline_comparison.Factory`를
불러 **규현 자신의 Split/직접NMPC/GS-LQR 구현**을 돌린다 — 우리 V13
(`hybrid_comparison.VirtualNMPC`+`ProperHybrid`)은 이 경로 어디에도 연결되어
있지 않다. 즉 **가감속 완화(8s/25s) 대 원래(15s/15s) 타이밍 자체는 실재하는
비교이지만, 검증은 8kg 레거시 기체 기준이었고 규현 기체·우리 V13 조합에는
아직 적용된 적이 없다** — 이번 재현이 처음이 된다.
→ 결론 아님, 재현/보고서에서 "완화 설정" 시간값(가속8s/감속25s)만 가져다
규현 기체+우리 컨트롤러 조합에 새로 적용한다(수치 자체를 재검증하지 않고
차용 — kj 결정 필요 항목으로 남김).

### 19:40 핵심 발견 — V13(VirtualNMPC) "1스텝 열화" 재현·근본원인

> **🔴 20:XX 정정 있음 — 아래 "지지되는 가설"(입력비용 가중치가 원인)은
> 부분적으로만 맞다. kj가 직접 재확인해 더 근본적인 원인(콜드스타트
> 웜스타트)을 짚었고, 그건 실제 수치 실패였다 — 아래 "19:20 kj 정정"
> 절 참고. 이 절의 관찰(k=0,1만 비정상, tol 강화해도 안 없어짐)은
> 사실로 남아있지만 "원인"에 대한 결론은 이 절 대신 정정 절을 따를 것.**

80 m/s 짧은 시험(kh_repro.py)에서 우리 V13이 계속 T≈81N(=T_max)·wdot≈±100
(둘 다 박스 경계)로 포화되는 걸 추적했다(스크립트: 스크래치패드 diag2~9,
재현 명령은 아래 "재현 명령" 참고).

1. **U_box는 정지추력 기준이지만 문제의 원인이 아니다.** T_max=81.13N
   (=4·k_T·n_max², J=0 정지 가정)이 맞고, 속도별 가용추력 감소는 반영 안
   한다 — kj가 지목한 부분은 사실이다. 하지만 실제 필요 트림추력은 80m/s
   에서 11.32N, 85m/s에서 12.44N으로 mg(11.52N)에서 0.98~1.08배에 불과하다
   (축추력식 몸체라 순항해도 mg 근처 — 다행히 U_box 정적/동적 불일치가
   지금 실패의 직접 원인은 아니다. 다만 실제 형상팀 기체가 mg 대비 훨씬
   큰 순항추력이 필요한 형상이라면 이 문제가 다시 표면화할 수 있다).
2. **의심했던 것(웜스타트, 트림 자체의 모델 불일치)은 기각.**
   `nmpc.F(x_trim, u_trim)` 한 스텝은 dv≈1e-3 m/s로 트림을 정확히 보존한다
   (diag7). 트림 상태를 웜스타트로 직접 넣어도 동일 증상 재현(diag6) —
   웜스타트 문제 아님.
3. **실제 원인(지지되는 가설): 논문 비용함수(식17)의 입력비용 가중치가
   너무 작아 첫 스텝(U_0) 방향의 민감도가 사실상 0에 가깝다.** 전체
   20스텝 해를 뜯어보면(diag8) k=2~19는 트림값(~11.4N)에 정확히 붙어
   있고, **오직 k=0(0.376N)·k=1(8.95N)만 비정상**이다 — 0.05초짜리 저추력이
   상태비용에 주는 영향이 질량·관성 대비 무시할 만큼 작아서, 입력편차
   비용(0.02/0.1 가중치)이 그 정도를 못 이긴다. `ipopt.tol`을 1e-4→1e-7,
   `max_iter`30→300으로 강화해도 U_0는 그대로(diag9) — **느슨한 수렴
   허용오차 문제가 아니라, 이 비용/이산화 조합에서 실제로 존재하는
   해(계산상 정확)의 성질**이다. MPC는 매 사이클 U_0만 실제로 적용하므로,
   매 0.02초(dt_ctrl)마다 "이번엔 문제없어 보이는" 저추력이 실제로
   연속 적용되어 실제 폐루프에서는 진짜 발산(피치 -70°~+65° 진동,
   |ω| 15rad/s+)으로 나타난다(diag2 궤적).
4. **M17도 정도는 약하지만 같은 방향 증상**: 80m/s 트림 로터
   [2798.6,2798.6,2968.1,2968.1] 대비 1차 출력 [1728,1728,2758.7,2758.7]
   (앞쌍 -38%, diag_m17) — 완전붕괴는 아니지만 트림에서 벗어난 1차 출력.
   F13/GSLQR은 아직 미확인(다음 단계).
5. kj의 3분류(정식화불능/수치실패/제어실패) 중 **어디에도 깔끔히
   맞지 않는다** — "해는 정확하게 수렴하지만 제어에 실제 적용되는
   조각(u_0)이 나머지 호라이즌과 다르게 거의 무페널티인 구간에 있다"는
   네 번째 성격에 가깝다. 결론 내리지 않고 가설로만 남긴다.

**재현 명령**:
```
cd /Users/kj/Desktop/dynamic/fast_drone-control-paper
python3 -c "
import sys; sys.path.insert(0,'external/fastdrone_kh')
from kh_control.trim import find_trim as kh_find_trim
from control.kh_adapter import kh_native_params, build_controller_params
from control.hybrid_comparison import VirtualNMPC
native = kh_native_params(); cp = build_controller_params(native)
tr = kh_find_trim(native, 80.0)
x13 = __import__('numpy').concatenate([tr['state'][0:10], tr['state'][10:13]])
x13[2] = 20.0
x0 = x13.copy(); x0[2]+=0.2; x0[3]+=0.5
nmpc = VirtualNMPC(cp, v_ref=[80.,0,0], z_ref=20.0, dt_ctrl=0.02, cost_spec='paper')
print(nmpc._solve(x0), nmpc.last_status)
"
```
(트림추력 11.32N 대비 출력 T_cmd≈0.38N 재현 확인용 — 전체 20스텝 분해는
diag8.py 패턴 참고, 스크래치패드 파일은 세션 종료 시 삭제되므로 로직은
위 커밋에 재기술 필요.)

### 19:20 kj 정정 — 19:40 진단의 분류가 틀렸다, 진짜 원인은 콜드스타트 웜스타트

kj가 직접 재확인해 지적: "정확한 트림 상태에서 VirtualNMPC(cost_spec=
'paper')를 풀면 입력이 [81.1,-95.4,62.0,-100.0]으로 한계에 붙고
Maximum_Iterations_Exceeded가 난다... '정확한 해의 성질'이 아니라
수치 실패다." 재현해서 확인했다 — 정확히 일치했다
(`nmpc._solve(x_trim13)`, 섭동 전혀 없이, 80m/s: 입력=[81.1,-95.4,62.0,
-99.97], status=Maximum_Iterations_Exceeded, NaN 경고 동반. 85m/s는
오히려 Solve_Succeeded로 트림에 붙어 — 속도마다 증상이 다르다).

**격리 실험(warm×eps 2×2)으로 원인을 분리했다**:

| 웜스타트 | eps(쿼터니언 정칙화) | 정확한 트림 | 섭동 있음(+0.2m/+0.5m/s) |
|---|---|---|---|
| hover(기존) | 0 | 실패(박스포화, MaxIter) | 실패(박스포화, MaxIter) |
| hover(기존) | 1e-9 | 실패(거의 동일) | 실패(거의 동일) |
| **트림** | 0 | **성공, 정확히 트림(11.36≈11.32)** | 성공하지만 T=0.381(19:40 발견한 그 증상) |
| 트림 | 1e-9 | 성공(hover eps=0과 동일) | 성공(hover eps=0과 동일) |

결론: **kj가 제안한 `ca.norm_2` 나눗셈 특이점(eps 정칙화) 자체는 최종
결과에 아무 영향이 없었다** — eps 유무가 4가지 조합 모두에서 결과를
전혀 안 바꿨다. NaN 경고는 실재하지만(아마 나쁜 웜스타트가 만드는 극단적
중간 반복해에서 발생하는 부수 증상), 그게 최종 오답의 원인은 아니었다.
**진짜 원인은 콜드스타트 웜스타트다**: `_build_nlp_paper`가 X_1..X_N의
초기 추측값을 실측과 무관하게 고정된 "호버형"([0,0,0,...,호버쿼터니언,
...,0,0,0])으로 채워서, 순항 트림(80m/s, 피치 -8°)처럼 호버에서 먼
조건에서 다중사격 등식제약이 전체 호라이즌에서 동시에 크게 위반된 채
IPOPT가 시작한다. 이건 19:40에 "정확한 해의 성질"이라 잘못 분류한
것과 다른, **진짜 수치 실패**다 — kj 지적이 맞았다.

**수정**: `_solve()`에 `self._cold_start` 플래그를 추가해, 인스턴스의
**첫 호출에서만** `self.w0`의 X_k(상태) 블록을 그 호출의 실측 상태로
덮어쓴다(U_k 가상입력/로터속도 추측값은 이미 합리적이라 안 건드림).
kj 규칙대로 **V13(`hybrid_comparison.py`)·M17(`nmpc.py`)·
F13(`nmpc_f13.py`) 전체에 동일하게 적용**했다. 수정 후:
- 정확한 트림(80/85m/s) 전부 `Solve_Succeeded`로 정확히 트림에 수렴
  (V13: 11.36/12.41 vs 실제 11.32/12.44, F13: 11.37/12.33 vs 동일,
  M17은 로터속도 기준 ~10-15% 이내로 트림 근접 — M17은 17상태 전체를
  예측해 자체 잔차가 있어 V13/F13만큼 정확하진 않지만 붕괴는 없음)
- 짧은 시험(kh_repro.py) 재실행: 85m/s가 "5회 연속 최적화 실패"
  (60% 실패율, Max_Iter_Exceeded 반복)에서 80m/s와 같은 "velocity error
  limit"(2/51 실패, 정상 수렴하지만 폐루프가 발산)로 바뀜 — **85m/s의
  솔버 자체 수렴 문제는 해소됐다.**
- 회귀 테스트: `control/test_nmpc_family_cold_start.py`(신규, 9개) +
  전체 스위트 168 passed(0 fail).

**남은 문제(증상2, 미해결)**: 콜드스타트 수정 후에도 **작은 섭동**
(+0.2m/+0.5m/s)이 있으면 U_0가 여전히 트림보다 크게 낮게 나온다(80m/s:
T=0.381N vs 트림 11.32N) — 이건 19:40에 발견한 원래 현상이고, 웜스타트를
수동으로 트림값으로 줘도, 수렴 허용오차를 강화해도(1e-4→1e-7,
max_iter 30→300) 없어지지 않는다(재확인, `test_v13_degenerate_first_step.py`
갱신판). 즉 **정확한 트림 자체를 유지하는 문제는 콜드스타트 수정으로
풀렸지만, 트림 근처의 작은 섭동에 대한 첫 스텝 반응성 문제는 별개로
남아 있다** — 논문 식(17) 입력비용 가중치가 U_0 민감도를 너무 낮춘다는
가설은 유지(지지도 기각도 아님, 판정 불가), 다음 세션 후보.

**재현 명령**:
```
cd /Users/kj/Desktop/dynamic/fast_drone-control-paper
python3 -m pytest control/test_nmpc_family_cold_start.py control/test_v13_degenerate_first_step.py -v
```

**"65초 미션"의 실체**: 규현 자신의 `docs/BASELINE_V2_RESULTS.md`(이미 벤더링됨,
`external/fastdrone_kh/docs/`)에 있다 — `kh_control.mission_sim.MissionProfile`
(이륙10s+안정화3s+가속15s+순항15s+감속15s+호버링7s=65s, 순항 중 35s에
10m/s 수직돌풍) + `run_baseline_comparison.run_case(mission=True)`. 여기서
분리형 46.400s/직접NMPC·나이브 46.120s에 감속 중 연속 최적화 실패,
GS-LQR·INDI는 65.000s 완주(단 포화·RMSE 기준 미달)가 **이미 문서화된 결과**다.
민석의 "이전 지속시간"(3/15/15/15/7=55s, 이륙 제외)과 규현의 15/15/15(가속/순항/감속)이
정확히 일치 — "원래 설정"은 두 출처가 서로 검증해 준다.
