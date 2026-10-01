# DISTRIBUTED_RUN A절 교체 초안 — 센서 설정 튜닝 `tune-final-7` (동욱님 저장소 기준)

> **초안(2026-10-01).** 태그 `tune-final-7`이 동욱님 저장소에 생기면 `<태그 커밋>`·`<재현 기준 해시>` 자리를 채워 `DISTRIBUTED_RUN.md` A절에 옮긴다.
> 그 전에는 학교에서 이 절차로 튜닝을 **시작하지 않는다**.

## 바뀐 점 (옛 A절 대비)
| 항목 | 옛 A절 | 새 A절 |
|---|---|---|
| 저장소 | `protkjj/fast-drone`(비공개, 토큰 필요) | **`leo11dk/fast-drone-sensor-fusion`(공개, 토큰 불필요)** — D0 결정 |
| 태그 | `tune-final-7`(자리표시) | `tune-final-7` = `87039e9`(태그 객체 `074fd616`) |
| 설정 | `<센서 설정>` | **`configs/arena_tune7.json`** |
| 설정 해시 | `<설정 해시>` | **`e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801`** |
| 재현 점검 | V13 참값 기록(`env_check`) | 참값 `env_check` **+ 센서 포함 `env_check_tune7` + 센서 재현 기준 `*_tune7.json`** |
| 폴더 | `fast-drone-shard` | 학교 점검 때 받은 **`fds`를 그대로 씀**(설치·가상환경 재사용) |

설정 내용: v9 센서(가정치, joint_baro, telemetry_predictor) + 시나리오별 튜닝 시드 2001~2018(D4) + 항법 사전 수렴 30 s(D5) + 기본 seed 3. V13·F13 옵션은 기본값(D3).

## A.1 받기 (학교 점검을 한 컴퓨터는 `fds` 폴더 재사용)
```powershell
cd $HOME\fds
```
```powershell
git fetch origin --tags
```
```powershell
git checkout tune-final-7
```
```powershell
git describe --tags --exact-match
```
마지막 줄이 `tune-final-7`이어야 한다. `fds`가 없는 컴퓨터는 학교 점검 절차(`results/SCHOOL_CHECK_SENSOR_2026-10-01.md`) 1~3번(받기·사양·가상환경)을 먼저 하고, 1번의 `git checkout`만 `tune-final-7`로 바꾼다.

## A.2 환경변수 (새 PowerShell 창마다)
```powershell
cd $HOME\fds; Set-ExecutionPolicy -Scope Process Bypass; .\.venv\Scripts\Activate.ps1
```
```powershell
$env:OMP_NUM_THREADS="1"; $env:OPENBLAS_NUM_THREADS="1"; $env:VECLIB_MAXIMUM_THREADS="1"; $env:MKL_NUM_THREADS="1"; $env:PYTHONUTF8="1"
```

## A.3 설정·모델 점검 (FAIL이면 멈춤)
```powershell
python scripts\setup_env.py --commit $(git rev-parse HEAD) --config configs\arena_tune7.json --expect-config-sha256 e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801 2>&1 | Tee-Object -FilePath setup_env_A3.txt
```
- 마지막 줄 `실행해도 좋음(PASS)`. 패키지 WARN은 괜찮다. `model …` 계수 줄 FAIL이면 멈춘다.

## A.4 재현 점검 (참여 조건 — 하나라도 FAIL이면 이 컴퓨터는 튜닝하지 않는다)
작업자 수 N은 `python -m control.arena_suggest_workers 0.6 --physical-cores <코어 수>`의 값, 3 이하.

**A.4a 참값 튜닝 경로**(학교 점검 8번과 같음, 맥 순차 약 13분):
```powershell
python -m control.arena_tune_repro --controller V13 --run-dir results/arena/tuning/env_check --index 0 --scenario-workers N 2>&1 | Tee-Object -FilePath env_check_V13.txt
```
**A.4b 센서 포함 튜닝 경로**(새 — 시나리오별 시드·사전 수렴 경로, 맥 순차 약 18분):
```powershell
python -m control.arena_tune_repro --config configs/arena_tune7.json --controller V13 --run-dir results/arena/tuning/env_check_tune7 --index 0 --scenario-workers N 2>&1 | Tee-Object -FilePath env_check_tune7_V13.txt
```
**A.4c 센서 재현 기준 두 개**(각 약 3분):
```powershell
python scripts\sensor_reproduce.py --output results\tune7_repro_projected *> tune7_repro_projected.txt
```
```powershell
python scripts\sensor_reproduce.py --reference scripts\data\sensor_legacy_reproduction_tune7.json --output results\tune7_repro_legacy *> tune7_repro_legacy.txt
```
- A.4a·b 합격: 마지막 줄 `PASS (rtol 0.001 + identical verdicts)`. `bit-identical` False여도 합격(다른 OS·칩).
- A.4c 합격: 판정 일치 + rtol 1e-3, **해시 검사 포함 통과**(태그 코드로 기록한 기준이라 해시가 맞아야 한다). legacy는 원래 '모델 범위 실패' 사례 — 실패까지 똑같이 재현되면 합격.
- 시간을 재 두면 일정 추정에 쓴다. ⚠️ 센서 설정은 시행마다 사전 구간(약 12 s)이 붙는데 기록의 `wall_seconds`에는 안 잡힌다 — **벽시계 시간**으로 적는다.

## A.5 고아 작업자 점검
학교 점검 절차 9번을 이미 통과한 컴퓨터는 생략. 아니면 그 9번(이 시험의 PID 기준)을 한다.

## A.6 시작 (컴퓨터마다 맡은 제어기)
배분(kj 결정, 학교 점검 속도로 재확인): 컴퓨터 1 = M17, 2 = F13, 3 = V13, 4 = GSLQR + CPID. F13·M17은 다른 컴퓨터. 한 제어기는 한 컴퓨터에서 끝까지.
작업자 수: `python -m control.arena_suggest_workers <작업자당 GiB> --physical-cores <코어 수>`(M17 2.0, F13 1.2, V13·GSLQR·CPID 0.6). 컴퓨터 4는 0.6 결과를 반씩.
```powershell
$c = '<제어기>'; $stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers',$c,'--budget','120','--config','configs/arena_tune7.json','--run-dir','results/arena/tuning/tune7','--scenario-workers','N' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune7_${c}_$stamp.log" -RedirectStandardError "tune7_${c}_$stamp.err"; $p.Id | Out-File -Encoding ascii "tune_$c.pid"; $p.Id
```
- 재개도 같은 명령(같은 컴퓨터). 첫 평가가 끝나면(`spent` 1) **벽시계 시간**을 kj에게.
- 튜닝이 도는 동안 `fds` 폴더의 `control/`·`models/team_light/control/`·`configs/`에 **아무것도 추가·수정하지 않는다**(코드 해시가 바뀌면 재개·연장·I-4가 거부된다). 결과·로그 저장은 괜찮다.

## A.7~A.11
옛 A절 A.7(살아 있는지·메모리), A.8(멈추기, `taskkill /T /F`), A.9(결과 회수 — 폴더 이름만 `fds`), A.10(맥에서 합치기·연장 판정·I-4, `--config configs/arena_tune7.json`), A.11(마무리 — 토큰 폐기 단계는 없음)을 그대로 쓴다.

## 채울 자리 (태그 뒤)
- `<태그 커밋>`: 동욱님 저장소 `tune-final-7`이 가리키는 커밋
- `<재현 기준 해시>`: `scripts/data/sensor_reproduction_tune7.json`, `sensor_legacy_reproduction_tune7.json`의 sha256(기록용)
- 학교 점검 결과로 확인한 컴퓨터별 작업자 수·평가 1회 벽시계 시간
