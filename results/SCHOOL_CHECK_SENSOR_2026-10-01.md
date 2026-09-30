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
```powershell
Start-Sleep 40; Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Select-Object ProcessId,ParentProcessId,@{n='cmd';e={$_.CommandLine.Substring(0,[math]::Min(80,$_.CommandLine.Length))}}
```
튜닝 본체만 강제 종료한다.
```powershell
$tune = Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -like '*orphan_check*' }; $inner = $tune | Where-Object { $tune.ProcessId -contains $_.ParentProcessId }; if (-not $inner) { $inner = $tune }; Stop-Process -Id $inner.ProcessId -Force; $inner.ProcessId
```
```powershell
Start-Sleep 15; Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Select-Object ProcessId,ParentProcessId | Tee-Object -FilePath check_orphan.txt
```
- **비어 있어야 합격.** 남아 있으면 목록을 kj에게 보내고 `taskkill /PID <번호> /T /F`로 정리한다.
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
같은 명령으로 재개(이번엔 창에서 바로 돌려 끝날 때까지 기다린다):
```powershell
python -m control.arena_tune --controllers GSLQR --budget 3 --run-dir resume_check 2>&1 | Tee-Object -FilePath check_resume_after.log
```
```powershell
Select-String '"spent"|"status"' resume_check\GSLQR.record.json | Tee-Object -FilePath check_resume_after.txt
```
- 합격: `"status": "complete"`, `"spent": 3`, 재개 로그에 오류 없음.
- 끊기 전 평가를 처음부터 다시 계산했는지는 재개 로그와 걸린 시간으로 kj가 판단한다.
- 확인 뒤 `Remove-Item -Recurse -Force resume_check`

## 11. 결과 묶기 → kj에게
```powershell
Compress-Archive -Path check_*.txt, check_*.log, results\school_repro_projected, results\school_repro_legacy -DestinationPath "school_check_$env:COMPUTERNAME.zip"
```
`school_check_<컴퓨터이름>.zip`을 보낸다. `fds` 폴더는 지우지 않고 둔다(최종 코드가 정해지면 새로 받는다).

## 12. 동욱님과 정할 것 (튜닝 결과를 보기 전에, 다섯 제어기에 같게)
별도 결정표(작성 예정)를 보고 정한다. 요지:
1. 저속 프로펠러 모델 범위 이탈(10,000 RPM 하한)을 튜닝에서 실패로 칠지 — 지금은 벌점 없음
2. 고속 돌풍 전 안정화(2.5–3.0 s, 0.5 m/s)를 판정에 넣을지
3. 센서 맞춤 비교의 범위 — **동욱님 동의(2026-09-30 밤)**: V13·F13 전용 옵션(INDI 필터 주파수·시작 가드·시간 정렬)은 INDI 센서 민감도 분석용이며 추가 튜닝 기회가 아니다. INDI 고유 기능은 제거실험으로 효과만 확인하고, 맞춤 비교에서는 나머지 제어기에도 구조에 맞는 조정 기회와 비교 가능한 탐색 예산을 준다. 최종 튜닝 설정에서는 기본값(50 Hz, 시작 가드 off)을 유지하는 안이 유력 — 확정 필요
4. 튜닝 센서 시드 — 여러 시드 묶음(예: 2001–2003)을 다섯 제어기에 같게(동욱님 측 권고, 코드 변경 필요), 시드별 실패 처리 규칙, 본 실험 시드 수
5. 최종 센서 설정(v9 개발안을 그대로 쓸지)과 초기 상태 오차 포함 여부

## 판정 요약표 (컴퓨터마다 채움)
| 항목 | 합격 기준 | 결과 | 시간 |
|---|---|---|---|
| 4 환경 점검 | PASS | | |
| 5 재현 85 m/s | 판정 일치 + rtol 1e-3 | | |
| 6 재현 20 m/s | 실패까지 똑같이 재현 | | |
| 8 튜닝 경로 재현 | `PASS (rtol 0.001 …)` | | |
| 9 고아 작업자 | 목록 비어 있음 | | — |
| 10 이어 돌리기 | complete, spent 3 | | |
| 7 작업자 수 | (0.6: ___ / 2.0: ___) | | — |
