# 학교 전산실 — 센서 설정 튜닝 `tune-final-7` 점검 + 시작 (2026-10-01 저녁)

**계획 A**: 태그 `tune-final-7`이 있으면 이 문서대로 점검 후 튜닝 시작.
**계획 B**: 태그가 없으면 `results/SCHOOL_CHECK_SENSOR_2026-10-01.md`(1~11, `a44c718`)로 점검만 하고 튜닝은 시작하지 않는다.

- 저장소: `https://github.com/leo11dk/fast-drone-sensor-fusion` (공개, 토큰 불필요)
- 태그: `tune-final-7` = `<태그 커밋>` (출발 전에 채움)
- 설정: `configs/arena_tune7.json`, config_sha256 = `e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801`
- 배분(kj): 컴퓨터 1 = M17, 2 = F13, 3 = V13, 4 = GSLQR + CPID. F13·M17은 다른 컴퓨터. 한 제어기는 한 컴퓨터에서 끝까지.
- 명령은 PowerShell에서 한 줄씩. FAIL이면 그 컴퓨터는 멈추고 화면·파일을 kj에게. 다른 컴퓨터는 계속해도 된다.
- 한 컴퓨터 안에서는 무거운 단계를 동시에 돌리지 않는다(시간 측정이 틀어진다).

## 1. 받기
```powershell
cd $HOME
```
```powershell
git clone https://github.com/leo11dk/fast-drone-sensor-fusion.git fds
```
(이미 `fds`가 있으면 위 대신 `cd $HOME\fds` → `git fetch origin --tags`)
```powershell
cd $HOME\fds
```
```powershell
git checkout tune-final-7
```
```powershell
git describe --tags --exact-match
```
→ `tune-final-7`이 나와야 한다.

## 2. 사양 기록
```powershell
& { "computer: $env:COMPUTERNAME"; (Get-CimInstance Win32_OperatingSystem).Caption; Get-CimInstance Win32_Processor | Format-List Name,NumberOfCores,NumberOfLogicalProcessors | Out-String; "total_mem_GiB: " + [math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory/1GB,1); "free_mem_GiB: " + [math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1MB,1) } | Tee-Object -FilePath check_spec.txt
```
`NumberOfCores`(물리 코어)를 아래 `<코어>`에 쓴다.

## 3. 파이썬 환경 (이미 `.venv`가 있으면 활성화·환경변수 줄만)
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

## 4. 설정·모델 점검
```powershell
python scripts\setup_env.py --commit $(git rev-parse HEAD) --config configs\arena_tune7.json --expect-config-sha256 e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801 2>&1 | Tee-Object -FilePath check_setup_env.txt
```
→ 마지막 줄 `실행해도 좋음(PASS)`. 패키지 WARN은 괜찮다.

## 5. 작업자 수
```powershell
python -m control.arena_suggest_workers 0.6 --physical-cores <코어>
```
```powershell
python -m control.arena_suggest_workers 2.0 --physical-cores <코어>
```
6번의 N은 0.6 결과(3 이하).

## 6. 재현 점검 (하나라도 FAIL이면 이 컴퓨터는 튜닝하지 않는다)
센서 재현 기준 두 개(각 약 3분):
```powershell
$t = Measure-Command { python scripts\sensor_reproduce.py --output results\tune7_repro_projected *> check_repro_projected.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_repro_projected_time.txt
```
```powershell
$t = Measure-Command { python scripts\sensor_reproduce.py --reference scripts\data\sensor_legacy_reproduction_tune7.json --output results\tune7_repro_legacy *> check_repro_legacy.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_repro_legacy_time.txt
```
센서 포함 튜닝 경로(맥 순차 약 18분, 작업자 N이면 더 짧음):
```powershell
$t = Measure-Command { python -m control.arena_tune_repro --controller V13 --run-dir results/arena/tuning/env_check_tune7 --index 0 --scenario-workers N *> check_env_tune7.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_env_tune7_time.txt
```
```powershell
Get-Content check_env_tune7.txt -Tail 2
```
- 합격: 재현 기준은 판정 일치 + rtol 1e-3(해시 검사 포함 통과, legacy는 원래 '모델 범위 실패'가 똑같이 나와야 함). 튜닝 경로는 마지막 줄 `PASS (rtol 0.001 + identical verdicts)`. `bit-identical` False여도 합격.
- 시간은 **벽시계**로 적는다(센서 설정은 시행마다 사전 구간 약 12 s가 붙는데 기록의 `wall_seconds`에는 안 잡힘). 맥 순차: 튜닝 경로 V13 1,064 s.

## 7. 고아 작업자 점검 (컴퓨터마다 한 번, 1~2분)
`results/SCHOOL_CHECK_SENSOR_2026-10-01.md` 9번 그대로(이 시험의 PID 기준, `left:` 비어 있어야 합격).

## 8. 튜닝 시작 (맡은 제어기마다)
작업자 수: M17 → 5번의 2.0 결과, F13 → `python -m control.arena_suggest_workers 1.2 --physical-cores <코어>`, V13 → 0.6 결과, 컴퓨터 4 → 0.6 결과를 GSLQR·CPID 반씩.
```powershell
$c = '<제어기>'; $stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers',$c,'--budget','120','--config','configs/arena_tune7.json','--run-dir','results/arena/tuning/tune7','--scenario-workers','N' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune7_${c}_$stamp.log" -RedirectStandardError "tune7_${c}_$stamp.err"; $p.Id | Out-File -Encoding ascii "tune_$c.pid"; $p.Id
```
- 컴퓨터 4는 `$c`를 GSLQR, CPID로 두 번 실행.
- 첫 평가가 끝나면(`spent` 1) 벽시계 시간을 kj에게:
```powershell
Select-String '"spent"|"status"' results\arena\tuning\tune7\<제어기>.record.json; (Get-Item results\arena\tuning\tune7\<제어기>.jsonl).LastWriteTime
```
- 튜닝 중 `fds`의 `control/`·`models/team_light/control/`·`configs/`에 아무것도 추가·수정하지 않는다. 결과·로그 저장은 괜찮다.
- 멈추기: `taskkill /PID (Get-Content tune_<제어기>.pid) /T /F`. 재개: 8번 명령을 같은 컴퓨터에서 그대로.
- 재시작·로그오프 뒤: 새 PowerShell → `cd $HOME\fds; Set-ExecutionPolicy -Scope Process Bypass; .\.venv\Scripts\Activate.ps1` → 3번 환경변수 줄 → 8번 명령.

## 9. 점검 결과 묶기 (튜닝 시작 뒤, 컴퓨터마다)
```powershell
Compress-Archive -Path check_*.txt, results\tune7_repro_projected, results\tune7_repro_legacy -DestinationPath "tune7_check_$env:COMPUTERNAME.zip"
```

## 판정표 (컴퓨터마다)
| 컴퓨터 | 맡은 제어기 | 4 점검 | 6 재현 기준 2개 | 6 튜닝 경로 | 7 고아 | 작업자 N | 6 튜닝 경로 벽시계 | 첫 평가 벽시계 |
|---|---|---|---|---|---|---|---|---|
| 1 | M17 | | | | | | | |
| 2 | F13 | | | | | | | |
| 3 | V13 | | | | | | | |
| 4 | GSLQR+CPID | | | | | | | |
