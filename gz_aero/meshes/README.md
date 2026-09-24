# 형상팀 CAD 메시 (Gazebo용, 연결 보류)

`HSD_drone_assembly.step`(형상팀, Fusion 360, 2026-09-21)에서 FreeCAD로 뽑은
컴포넌트 메시. **아직 어떤 SDF에도 연결하지 않았다** — 이유는 아래 "왜 보류했는가" 참고.

## 파일

| 파일 | 내용 | 로컬 원점 |
|---|---|---|
| `body.stl` | 동체(라디얼 100mm 이내 솔리드 전부) | 무게중심(CG). +x = 기수 방향 |
| `motor_mount.stl` | 로터 1개분 정적 모터 마운트(스테이터 하우징) | 그 로터의 허브 축 |
| `motor_bell.stl` | 로터 1개분 회전 모터벨 + 인접 소형 하드웨어 | 같은 허브 축 |

좌표계는 `research/stl.js`의 `parse()`와 동일한 변환을 그대로 씀: CAD는 mm,
기수 X=0, +X가 뒤쪽. 이를 `[cgFromNose - x, y, -z]` (미터)로 바꿔 동체 좌표계
(+x 기수, thrust_axis='x' 관례)로 만든다.

`motor_mount.stl`/`motor_bell.stl`은 로터 1개(사분면 하나)만 뽑았다 — 4개 로터가
기수축 둘레 90도 대칭 배치라, 나머지 3개는 이 메시를 그대로 두고 허브 위치로
평행이동 + 기수축(x) 둘레 90도씩 돌리면 된다:

- 허브 위치(4개, 기수축 둘레 반경 `radial_arm_m/√2` = 0.119388 m):
  `(x, ±0.119388, ±0.119388)`, `x = cg_from_nose_m - 0.611`
  (여기서 `cg_from_nose_m`, `radial_arm_m`은 `research/profiles/selected.json`)
- 프로펠러 블레이드는 이 STEP에 없다(형상팀이 구조 어셈블리만 보낸 것으로
  보임) — 기존 `1345_prop_ccw.stl`/`1345_prop_cw.stl` 재사용 필요.

재질/밀도는 51개 솔리드 중 1개에만 있어(강철 7850kg/m³, Fusion 기본값으로
보임) 질량·관성은 이 메시에서 계산하지 않는다 — `research/profiles/selected.json`의
CSV 사이징 값이 여전히 기준이다.

## 왜 아직 SDF에 연결 안 했는가

`gz_aero/tools/fix_model_rocket.py`를 읽어보면, 현재 Ubuntu 쪽
`fast_missile_base/model.sdf`는 메시가 아니라 **원기둥/구 프리미티브**로
동체를 그리고, 그 치수가 `control/vehicle_params.py`의 옛 플레이스홀더
(질량 8kg, 동체 지름 0.15m, 암 0.25/√2≈0.177m)를 따른다. 반면 이 CAD는
`research/`의 실제 사이징 결과(질량 1.7117kg, 동체 지름 0.0871m, 암 0.169m)다 —
**두 배 가까이 다른 스케일**이다.

이 메시를 지금 SDF에 그대로 연결하면 로터 조인트 위치(옛 스케일)와 메시
치수(새 스케일)가 안 맞아 로터가 팔 밖에 떠 있는 것처럼 보이는 등 오히려
더 틀린 그림이 된다. `control`(제어기 실험용, 8kg)과 `research`(논문 기준구현,
1.7117kg) 두 기체 규격을 어떻게 합칠지 먼저 정해야 한다 — 이건 2026-09-22
세션에서 의도적으로 다음으로 미룬 결정이다.

## 다음에 할 일 (SDF 연결 시)

1. 위 기체 규격 통합 결정 (control vs research 스케일)
2. Ubuntu에서 실제 `fast_missile_base/model.sdf`를 읽고, `fix_model_rocket.py`
   패턴(`--model`/`--restore`, 정규식 기반 `<visual>` 블록 치환, `<inertial>`/
   공력 플러그인은 안 건드림)을 따르는 새 스크립트 작성
3. 이 메시들을 `Tools/simulation/gz/models/fast_missile_base/meshes/`로 복사,
   위 허브 위치 공식으로 `<visual>`의 `<uri>`+`<pose>` 갱신
4. Gazebo에서 실제 스폰해 로터-암 정합, 충돌체 크기 확인
