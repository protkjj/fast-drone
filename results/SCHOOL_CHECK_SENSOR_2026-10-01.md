# 학교 컴퓨터 점검 — 센서 코드(동욱님 저장소) 기준, 2026-10-01

**오늘의 목표 = 튜닝 전 점검.** 다섯 제어기 120회 튜닝(`tune-final-7`)은 오늘 **시작하지 않는다.**
최종 센서 설정·성공 기준이 아직 정해지지 않았기 때문이다(동욱님 5단계의 2·3번).
오늘 결과(사양·재현·중단/재개·고아 작업자·속도)는 어느 저장소로 최종 튜닝을 하든 그대로 쓴다.

- 대상 코드: `https://github.com/leo11dk/fast-drone-sensor-fusion` 커밋 `a44c7181c3a6d4b791cfd1678cf1d95ba51a3eb7`
  (우리 `protkjj/arena-completion` `6cad076` + 센서 작업. 실행 코드는 GitHub Windows CI를 통과한 `f2dc38be`와 같다 —
  그 뒤 바뀐 파일은 `scripts/sensor_preflight.py` 3줄뿐)
- ZIP 패키지는 쓰지 않는다: 결과 폴더를 빼서 8번(A.4)에 필요한 `results/arena/tuning/env_check/`가 없다. **git clone**으로 받는다(공개 저장소, 토큰 불필요).
- 컴퓨터마다 1~11을 위에서 아래로. 명령은 **PowerShell**에서 한 줄씩. 오류가 나면 멈추고 화면·파일을 그대로 kj에게.
- 받은 폴더의 `control/`, `models/team_light/control/`, `configs/`에는 **파일을 추가·수정하지 않는다**(튜닝 기록은 두 코드 폴더의 `.py`, 분산 실행은 여기에 테스트와 `configs/` JSON까지 해시로 고정한다 — 바뀌면 재개·합치기가 거부된다). 결과·로그 저장은 괜찮다. 분석·개발은 별도 복사본에서.

---

## 1. 받기
```powershell
cd $HOME
```
```powershell
git clone https://github.com/leo11dk/fast-drone-sensor-fusion.git fds
```
```powershell
cd fds
```
```powershell
git checkout a44c7181c3a6d4b791cfd1678cf1d95ba51a3eb7
```
```powershell
git rev-parse HEAD
```
마지막 줄이 `a44c7181c3a6d4b791cfd1678cf1d95ba51a3eb7`이어야 한다. `detached HEAD` 안내는 정상이다.

## 2. 사양 기록
```powershell
& { "computer: $env:COMPUTERNAME"; (Get-CimInstance Win32_OperatingSystem).Caption; Get-CimInstance Win32_Processor | Format-List Name,NumberOfCores,NumberOfLogicalProcessors | Out-String; "total_mem_GiB: " + [math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory/1GB,1); "free_mem_GiB: " + [math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1MB,1) } | Tee-Object -FilePath check_spec.txt
```
- `NumberOfCores`가 **물리 코어**다. 7번의 `--physical-cores`에 이 값을 쓴다. GPU는 쓰지 않는다(CPU 전용 코드).

## 3. 파이썬 환경
```powershell
py -3.13 --version
```
```powershell
py -3.13 -m venv .venv
```
```powershell
Set-ExecutionPolicy -Scope Process Bypass
```
```powershell
.\.venv\Scripts\Activate.ps1
```
```powershell
python -m pip install -r requirements-lock.txt
```
```powershell
$env:OMP_NUM_THREADS="1"; $env:OPENBLAS_NUM_THREADS="1"; $env:VECLIB_MAXIMUM_THREADS="1"; $env:MKL_NUM_THREADS="1"; $env:PYTHONUTF8="1"
```
새 PowerShell 창을 열면 `cd $HOME\fds`, 위 `Set-ExecutionPolicy`, `Activate.ps1`, 환경변수 줄을 다시 한다.

## 4. 환경 점검 (FAIL이면 멈춤)
```powershell
python scripts\setup_env.py --commit a44c7181c3a6d4b791cfd1678cf1d95ba51a3eb7 --config configs\arena_rotor_projected_development_v9.json 2>&1 | Tee-Object -FilePath check_setup_env.txt
```
- 마지막 줄 `실행해도 좋음(PASS)`이면 통과. 패키지 버전 WARN은 괜찮다(기준이 맥이라).
- 스레드 FAIL = 3번 환경변수 줄을 빠뜨림. git FAIL = 커밋이 다르거나 파일이 수정됨. `model …` 계수 줄 FAIL이면 멈춘다.

## 5. 센서 기준 사례 재현 ① — 85 m/s V13, 새 로터 관측기 (약 3분)
```powershell
$t = Measure-Command { python scripts\sensor_reproduce.py --output results\school_repro_projected *> check_repro_projected.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_repro_projected_time.txt
```
```powershell
Get-Content check_repro_projected.txt -Tail 5
```
- 통과 판정은 기록된 결과와 **허용오차(rtol 1e-3) 안 일치 + 판정 일치**다. 궤적 비트 동일은 기록만 한다(다른 OS라 False여도 됨).
- 참고: GitHub Windows 서버에서 약 2분 34초 걸렸다(`results/windows_sensor_v13/attempt_3.json`, 설치 제외).

## 6. 센서 기준 사례 재현 ② — 20 m/s V13, 옛 로터 관측기 (약 3분)
```powershell
$t = Measure-Command { python scripts\sensor_reproduce.py --reference scripts\data\sensor_legacy_reproduction_v13.json --output results\school_repro_legacy *> check_repro_legacy.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_repro_legacy_time.txt
```
```powershell
Get-Content check_repro_legacy.txt -Tail 5
```
- 이 사례는 **원래 '모델 범위 실패'로 기록된 사례**다. 실패가 똑같이 재현되는 것이 합격이다.

## 7. 작업자 수 계산
```powershell
python -m control.arena_suggest_workers 0.6 --physical-cores <2번의 NumberOfCores>
```
```powershell
python -m control.arena_suggest_workers 2.0 --physical-cores <2번의 NumberOfCores>
```
두 결과를 적어 둔다(0.6 = V13·GSLQR·CPID, 2.0 = M17 기준). 8번의 N은 0.6 결과로, 3을 넘기지 않는다.

## 8. 튜닝 경로 재현 — 우리 A.4 (참값 경로, 맥 순차 약 13분, 작업자 3개 약 4~7분)
```powershell
$t = Measure-Command { python -m control.arena_tune_repro --controller V13 --run-dir results/arena/tuning/env_check --index 0 --scenario-workers N *> check_env_V13.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_env_V13_time.txt
```
```powershell
Get-Content check_env_V13.txt -Tail 3
```
- 마지막 줄 `PASS (rtol 0.001 + identical verdicts)`면 합격. `bit-identical` False여도 합격(기록만).
- `FAIL`이면 이 컴퓨터는 튜닝 후보에서 뺀다 — 파일을 kj에게.
- 속도 비교를 위해 **한 대는 `--scenario-workers 1`(순차)로도 한 번** 돌려 시간을 적으면 좋다(맥 순차 약 13분과 비교 → 속도비).

## 9. 고아 작업자 점검 — 우리 A.5 (1~2분, 처음 한 번)
```powershell
$p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers','GSLQR','--budget','3','--run-dir','orphan_check','--scenario-workers','2' -PassThru -WindowStyle Hidden
```
40초 기다린 뒤, **이 시험의 프로세스만** 기록한다: 튜닝 본체와 그 아래 모든 자손(작업자, venv 중계 포함). 다른 프로그램의 파이썬은 판정에 넣지 않는다.
```powershell
Start-Sleep 40; $tune = @(Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -like '*orphan_check*' }); $inner = @($tune | Where-Object { $tune.ProcessId -contains $_.ParentProcessId }); if (-not $inner) { $inner = $tune }; $all = Get-CimInstance Win32_Process; $ids = @(); $frontier = @($inner.ProcessId); while ($frontier) { $next = @($all | Where-Object { $frontier -contains $_.ParentProcessId } | ForEach-Object ProcessId); $ids += $next; $frontier = $next }; "tune: $($tune.ProcessId -join ',')  inner: $($inner.ProcessId -join ',')  descendants: $($ids -join ',')" | Tee-Object -FilePath check_orphan_before.txt
```
- `descendants`가 비어 있으면 작업자가 아직 안 떴다. 20초 더 기다렸다가 같은 줄을 다시 실행한다.

튜닝 본체만 강제 종료한다.
```powershell
Stop-Process -Id $inner.ProcessId -Force
```
15초 뒤, 기록해 둔 프로세스 중 **아직 살아 있는 것**을 본다.
```powershell
Start-Sleep 15; $left = @(Get-Process -Id ($ids + $tune.ProcessId) -ErrorAction SilentlyContinue); "left: $($left.Id -join ',')" | Tee-Object -FilePath check_orphan.txt
```
- **`left:` 뒤가 비어 있어야 합격.** 남은 번호가 있으면 파일을 kj에게 보내고 `taskkill /PID <번호> /T /F`로 정리한다.
- 같은 PowerShell 창에서 이어서 실행해야 한다(`$tune`, `$inner`, `$ids` 변수를 쓴다).
```powershell
Remove-Item -Recurse -Force orphan_check
```

## 10. 중단 후 이어 돌리기 점검 (새 항목, 약 5분)
짧은 튜닝(GSLQR 예산 3)을 중간에 끊고 **같은 명령으로 재개**했을 때, 끝난 평가를 다시 계산하지 않고 이어서 끝나는지 본다.
```powershell
$p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers','GSLQR','--budget','3','--run-dir','resume_check' -PassThru -WindowStyle Hidden -RedirectStandardOutput resume_1.log -RedirectStandardError resume_1.err; $p.Id
```
1~2분마다 아래로 `"spent"`를 본다. **1 또는 2가 되면** 바로 다음 명령으로 끊는다(3이면 너무 늦었으니 폴더를 지우고 처음부터).
```powershell
Select-String '"spent"' resume_check\GSLQR.record.json
```
```powershell
taskkill /PID $p.Id /T /F
```
```powershell
Select-String '"spent"' resume_check\GSLQR.record.json | Tee-Object -FilePath check_resume_before.txt
```
끊기 전에 끝난 평가 기록(한 줄 = 평가 1회, `control/arena_tune.py:359` 덧붙이기 모드)을 복사해 둔다.
```powershell
Copy-Item resume_check\GSLQR.jsonl check_resume_before.jsonl
```
같은 명령으로 재개(이번엔 창에서 바로 돌려 끝날 때까지 기다린다):
```powershell
python -m control.arena_tune --controllers GSLQR --budget 3 --run-dir resume_check 2>&1 | Tee-Object -FilePath check_resume_after.log
```
```powershell
Select-String '"spent"|"status"' resume_check\GSLQR.record.json | Tee-Object -FilePath check_resume_after.txt
```
끊기 전 기록이 재개 뒤에도 **앞부분에 한 글자도 안 바뀌고** 남아 있는지 비교한다.
```powershell
$b = @(Get-Content check_resume_before.jsonl); $a = @(Get-Content resume_check\GSLQR.jsonl); $same = -not (Compare-Object $b @($a | Select-Object -First $b.Count) -SyncWindow 0); "before=$($b.Count) after=$($a.Count) prefix_same=$same" | Tee-Object -FilePath check_resume_compare.txt
```
- 합격: `"status": "complete"`, `"spent": 3`, 재개 로그에 오류 없음, **`prefix_same=True`**, `after`가 `before`보다 크거나 같음.
- `prefix_same=False`면 끊기 전 기록이 바뀐 것이다 — 멈추고 두 `.jsonl`을 kj에게.
- 재개가 `JSONDecodeError`로 실패하면 강제 종료가 줄을 쓰는 도중에 걸려 마지막 줄이 잘린 것일 수 있다(재개 때 기존 줄을 전부 읽는다, `arena_tune.py:316`) — 그대로 kj에게 보낸다(실제 튜닝에서도 생길 수 있는 문제라 기록 가치가 있다).
- 끊기 전 평가를 처음부터 다시 계산했는지는 재개 로그와 걸린 시간으로도 함께 본다.
- 확인 뒤 `Remove-Item -Recurse -Force resume_check`

## 11. 결과 묶기 → kj에게
```powershell
Compress-Archive -Path check_*.txt, check_*.log, check_*.jsonl, results\school_repro_projected, results\school_repro_legacy -DestinationPath "school_check_$env:COMPUTERNAME.zip"
```
`school_check_<컴퓨터이름>.zip`을 보낸다. `fds` 폴더는 지우지 않고 둔다(최종 코드가 정해지면 새로 받는다).

## 12. 결정 상태 — 최종본은 `results/SENSOR_DECISIONS_2026-10-01.md` 맨 아래 "결정 기록" 표
**확정(2026-09-30 밤, 다시 정하지 않는다)**: D0 동욱님 저장소, D2 돌풍 전 안정화 기준·실패 판정 유지, D3 V13·F13 옵션 기본값 고정(50 Hz·가드 off·V13 S1·F13 S0)·제거실험 분리,
D4 시나리오별 고정 시드, D5 v9 가정치·항법 초기값 정확 가정 명시, D7 d3 제외(표 7은 17행), D8 ν_act 참 회전수.
**아직 열린 것**: D1·D6(동욱님 측 원인 격리 결과를 보고), 본 실험 시드 수(제안: 비교 20개, 사다리 5~10개 — 오늘 속도 측정 뒤, 결과 보기 전 확정).

## 13. 태그(`tune-final-7`)가 나온 뒤 짧은 재확인 — 오늘 점검이 대신하지 못하는 것
오늘 점검은 `a44c718`(D4 없음)과 **참값** V13 재현 기준이다. 최종 태그는 D4(시나리오별 시드)가 들어간 새 코드라, 튜닝 전에 컴퓨터마다 다시 확인한다(동욱님 측 지적, 2026-09-30 밤).
- 새 태그로 받기 + 4번 환경 점검(`--config <최종 설정> --expect-config-sha256 <해시>`)
- **센서 포함 튜닝 경로 재현**: 맥에서 최종 설정으로 만든 센서 포함 기준 기록(평가 0)을 이 컴퓨터에서 재현 — 시드 배정·병렬 작업자에서 판정 일치 + rtol 1e-3. (기준 기록은 태그 전에 맥에서 만들어 태그에 넣어야 한다 — 동욱님 측·우리 준비 항목)
- **센서 포함 설정으로 10번(중단 후 재개)** 다시 — 시드 매핑이 재개 뒤에도 같은지 기록으로 확인
- 설치·사양(1~3번)과 9번(고아 작업자)은 다시 하지 않아도 된다.

## 판정 요약표 (컴퓨터마다 채움)
| 항목 | 합격 기준 | 결과 | 시간 |
|---|---|---|---|
| 4 환경 점검 | PASS | | |
| 5 재현 85 m/s | 판정 일치 + rtol 1e-3 | | |
| 6 재현 20 m/s | 실패까지 똑같이 재현 | | |
| 8 튜닝 경로 재현 | `PASS (rtol 0.001 …)` | | |
| 9 고아 작업자 | `left:` 비어 있음(이 시험의 프로세스 기준) | | — |
| 10 이어 돌리기 | complete, spent 3, `prefix_same=True` | | |
| 7 작업자 수 | (0.6: ___ / 2.0: ___) | | — |
