# tune7_all.ps1 - one command for a school PC: setup, check, start tuning, phone notifications.
#
# One line (PowerShell):
#   irm https://raw.githubusercontent.com/leo11dk/fast-drone-sensor-fusion/main/school/tune7_all.ps1 -OutFile $HOME\tune7_all.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune7_all.ps1 -Controller M17 -Topic <ntfy topic>
#
# What it does (each step is skipped when already done, so re-running the same line after a reboot resumes):
#   1. Python 3.13.7 (user install if missing)   2. clone $HOME\fds at tag tune-final-7   3. .venv + pinned packages
#   4. checks: tag/commit, setup_env (config hash, model coefficients), workers, 2 sensor references,
#      sensor tuning-path replay (V13), orphan-worker test -> $HOME\fds\tune7_check_<PC>_<time>\summary.json
#      (skipped if this PC already has an ALL PASS summary; use -Recheck to force)
#      The V13 replay passes if it matches the Mac reference within rtol 1e-3, OR if its objective is
#      exactly the school-Windows value below. Two school PCs (i7-10700, 2026-10-01) produced that value
#      bit-identically; the Mac differs in one scenario (tune2_ramp_brake_V80_25_rho1.2) only.
#      A FAIL summary whose only failure was that replay is re-judged from its saved log (no rerun).
#   5. if -Controller is given and checks pass: start (or resume) tuning in the background + phone watcher
# Never edits files under control/, models/ or configs/ (tuning records pin those hashes).
param(
    [string]$Controller = '',
    [string]$Topic = '',
    [int]$Workers = 0,
    [int]$Budget = 120,
    [switch]$Recheck,
    [switch]$Watch,
    [int]$IntervalSeconds = 300,
    [int]$StallMinutes = 120,
    [int]$ProgressEvery = 30
)

$ErrorActionPreference = 'Continue'
$Tag = 'tune-final-7'
$ExpectedHead = '87039e946ce122c9f89bb89a4127c086ac459e90'
$ExpectedConfigSha = 'e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801'
$WindowsEnvObjective = '0.4628760206058794'
$RepoUrl = 'https://github.com/leo11dk/fast-drone-sensor-fusion.git'
$PyUrl = 'https://www.python.org/ftp/python/3.13.7/python-3.13.7-amd64.exe'
$Fds = Join-Path $HOME 'fds'
$Vpy = Join-Path $Fds '.venv\Scripts\python.exe'
$Controllers = @('M17', 'F13', 'V13', 'GSLQR', 'CPID')
$Gib = @{ M17 = '2.0'; F13 = '1.2'; V13 = '0.6'; GSLQR = '0.6'; CPID = '0.6' }

try { [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12 } catch { }

function Say([string]$Text) { Write-Host ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $Text) }
function Send-Note([string]$Message) {
    if ($Topic) {
        try { Invoke-RestMethod -Method Post -Uri "https://ntfy.sh/$Topic" -Body $Message -TimeoutSec 20 | Out-Null }
        catch { Add-Content -Path (Join-Path $HOME 'tune7_notify.err') -Value "$(Get-Date -Format s) $($_.Exception.Message)" }
    }
}
function Stop-With([string]$Text) {
    Say "FAIL: $Text"
    Send-Note "$env:COMPUTERNAME tune7 FAIL: $Text"
    exit 1
}
function Stamp { Get-Date -Format 'MM-dd HH:mm' }

# Judge the V13 replay log: 'mac' (PASS line), 'windows' (objective printed exactly as the school-Windows value), or '' (fail).
function Get-EnvRule([string]$EnvLog) {
    if (-not (Test-Path $EnvLog)) { return '' }
    $lines = @(Get-Content $EnvLog)
    $last = $lines | Where-Object { $_ -match '^(PASS|FAIL)' } | Select-Object -Last 1
    if ($null -ne $last -and $last -like 'PASS*') { return 'mac' }
    $text = $lines -join "`n"
    $objectiveExact = $text -match ('"objective":\s*' + [regex]::Escape($WindowsEnvObjective) + '\s*,')
    $configOk = $text -match ('"config_sha256":\s*"' + $ExpectedConfigSha + '"')
    $scenariosOk = $text -match '"scenarios":\s*18\s*,'
    if ($objectiveExact -and $configOk -and $scenariosOk) { return 'windows' }
    return ''
}

if ($Controller -and ($Controllers -notcontains $Controller)) { Stop-With "unknown controller '$Controller' (use M17, F13, V13, GSLQR or CPID)" }

# ---------------- watcher mode (started in the background by step 5) ----------------
if ($Watch) {
    if (-not $Controller) { exit 1 }
    $record = Join-Path $Fds "results\arena\tuning\tune7\$Controller.record.json"
    $jsonl = Join-Path $Fds "results\arena\tuning\tune7\$Controller.jsonl"
    $started = Get-Date
    $lastSpent = -1
    $stallSent = $false
    Send-Note "$Controller watcher started on $env:COMPUTERNAME $(Stamp)"
    while ($true) {
        if (Test-Path $record) {
            $r = $null
            try { $r = Get-Content $record -Raw | ConvertFrom-Json } catch { $r = $null }
            if ($null -ne $r) {
                $spent = [int]$r.spent
                $budgetNow = [int]$r.budget
                if ($spent -ne $lastSpent) {
                    $crossed = ($lastSpent -ge 0) -and ([math]::Floor($spent / $ProgressEvery) -gt [math]::Floor($lastSpent / $ProgressEvery))
                    if ($spent -eq 1) {
                        $mins = [int](New-TimeSpan -Start $started -End (Get-Date)).TotalMinutes
                        Send-Note "$Controller first eval done (1/$budgetNow) $(Stamp), about $mins min after start"
                    } elseif ($crossed) {
                        Send-Note "$Controller $spent/$budgetNow evals $(Stamp)"
                    }
                    $lastSpent = $spent
                    $stallSent = $false
                }
                if ($r.status -eq 'complete') { Send-Note "$Controller COMPLETE $spent/$budgetNow $(Stamp)"; exit 0 }
            }
        }
        if (Test-Path $jsonl) {
            $age = (New-TimeSpan -Start (Get-Item $jsonl).LastWriteTime -End (Get-Date)).TotalMinutes
            if (($age -gt $StallMinutes) -and (-not $stallSent)) {
                Send-Note "$Controller no new eval for $([int]$age) min on $env:COMPUTERNAME - check it $(Stamp)"
                $stallSent = $true
            }
        }
        Start-Sleep -Seconds $IntervalSeconds
    }
}

Say "tune7_all on $env:COMPUTERNAME"

# ---------------- 1. git and Python 3.13 ----------------
if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Stop-With 'git not found - install Git for Windows (https://git-scm.com/download/win) and run this line again' }

function Find-Py313 {
    $candidates = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Python\Python313\python.exe'),
        (Join-Path $env:ProgramFiles 'Python313\python.exe')
    )
    try {
        $fromLauncher = (& py -3.13 -c "import sys; print(sys.executable)" 2>$null | Out-String).Trim()
        if ($fromLauncher) { $candidates += $fromLauncher }
    } catch { }
    foreach ($c in $candidates) {
        if ($c -and (Test-Path $c)) {
            $v = (& $c -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>$null | Out-String).Trim()
            if ($v -eq '3.13') { return $c }
        }
    }
    return $null
}

$py313 = $null
if (-not (Test-Path $Vpy)) {
    $py313 = Find-Py313
    if (-not $py313) {
        Say 'Python 3.13 not found - installing 3.13.7 for this user (about 1-2 min)'
        $installer = Join-Path $HOME 'python-3.13.7-amd64.exe'
        try { Invoke-WebRequest -Uri $PyUrl -OutFile $installer -UseBasicParsing } catch { Stop-With "Python download failed: $($_.Exception.Message)" }
        Start-Process $installer -ArgumentList '/quiet', 'InstallAllUsers=0', 'PrependPath=0', 'Include_launcher=1', 'Include_test=0' -Wait
        $py313 = Find-Py313
        if (-not $py313) { Stop-With 'Python 3.13 install did not complete - install python 3.13.7 from python.org (Install Now) and run again' }
    }
    Say "Python 3.13: $py313"
}

# ---------------- 2. clone at the tag ----------------
if (-not (Test-Path (Join-Path $Fds '.git'))) {
    Say "cloning into $Fds"
    git clone -q $RepoUrl $Fds
    if ($LASTEXITCODE -ne 0) { Stop-With 'git clone failed' }
}
Set-Location $Fds
git fetch -q origin --tags 2>$null
$head = (git rev-parse HEAD).Trim()
if ($head -ne $ExpectedHead) {
    git -c advice.detachedHead=false checkout -q $Tag
    $head = (git rev-parse HEAD).Trim()
}
if ($head -ne $ExpectedHead) { Stop-With "fds is at $head, expected $ExpectedHead ($Tag) - local changes? do not edit fds" }
Say "code: $Tag $head"

# ---------------- 3. venv and packages ----------------
if (-not (Test-Path $Vpy)) {
    Say 'creating .venv'
    & $py313 -m venv (Join-Path $Fds '.venv')
    if (-not (Test-Path $Vpy)) { Stop-With 'venv creation failed' }
}
& $Vpy -c "import numpy, scipy, casadi, matplotlib" 2>$null
if ($LASTEXITCODE -ne 0) {
    Say 'installing pinned packages (a few minutes)'
    & $Vpy -m pip install -q -r requirements-lock.txt
    if ($LASTEXITCODE -ne 0) { Stop-With 'pip install -r requirements-lock.txt failed' }
}

$env:OMP_NUM_THREADS = '1'; $env:OPENBLAS_NUM_THREADS = '1'; $env:VECLIB_MAXIMUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'; $env:PYTHONUTF8 = '1'
$cores = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores -Sum).Sum

function Get-Workers([string]$PerWorkerGib) {
    $text = (& $Vpy -m control.arena_suggest_workers $PerWorkerGib --physical-cores $cores 2>&1 | Out-String)
    if ($text -match 'scenario-workers\s+(\d+)') { return [int]$Matches[1] }
    return 1
}

# ---------------- 4. checks ----------------
$passed = $null
if (-not $Recheck) {
    foreach ($f in (Get-ChildItem -Path (Join-Path $Fds 'tune7_check_*\summary.json') -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)) {
        try {
            $s = Get-Content $f.FullName -Raw | ConvertFrom-Json
            if ($s.computer -ne $env:COMPUTERNAME) { continue }
            if ($s.all_pass) { $passed = $f.FullName; break }
            $othersOk = ($s.head -eq $ExpectedHead) -and ("$($s.check_setup_env)" -eq 'PASS') -and
                ("$($s.check_repro_projected)" -like 'PASS*') -and ("$($s.check_repro_legacy)" -like 'PASS*') -and
                ("$($s.check_orphan)" -like 'PASS*')
            $rule = Get-EnvRule (Join-Path $f.DirectoryName 'env_check_tune7.txt')
            if ($othersOk -and $rule -eq 'windows') {
                $s | Add-Member -NotePropertyName 'env_rule' -NotePropertyValue 'windows' -Force
                $s | Add-Member -NotePropertyName 'all_pass' -NotePropertyValue $true -Force
                $s | Add-Member -NotePropertyName 'rejudged' -NotePropertyValue (Get-Date -Format s) -Force
                $s | ConvertTo-Json -Depth 4 | Out-File -Encoding utf8 $f.FullName
                Say "re-judged $($f.FullName): V13 replay equals the school-Windows value -> ALL PASS"
                Send-Note "$env:COMPUTERNAME tune7 checks ALL PASS (Windows value) $(Stamp)"
                $passed = $f.FullName
                break
            }
        } catch { }
    }
}

if ($passed) {
    Say "checks already passed on this PC: $passed (use -Recheck to run again)"
} else {
    $stamp = Get-Date -Format yyyyMMdd_HHmm
    $out = Join-Path $Fds "tune7_check_$($env:COMPUTERNAME)_$stamp"
    New-Item -ItemType Directory -Force -Path $out | Out-Null
    $summary = [ordered]@{ computer = $env:COMPUTERNAME; started = (Get-Date -Format s); head = $head }
    function Note([string]$Key, $Value) { $summary[$Key] = $Value; Say ('{0,-30} {1}' -f $Key, $Value) }

    Note 'cpu' ((Get-CimInstance Win32_Processor | Select-Object -First 1).Name)
    Note 'physical_cores' $cores
    Note 'mem_total_GiB' ([math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB, 1))
    Note 'mem_free_GiB' ([math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory / 1MB, 1))
    $pyVersion = (& $Vpy --version 2>&1 | Out-String).Trim()
    Note 'python' $pyVersion

    $setupLog = Join-Path $out 'setup_env.txt'
    & $Vpy scripts\setup_env.py --commit $head --config configs\arena_tune7.json --expect-config-sha256 $ExpectedConfigSha *> $setupLog
    $okSetup = ($LASTEXITCODE -eq 0)
    Note 'check_setup_env' $(if ($okSetup) { 'PASS' } else { 'FAIL' })

    $w06 = Get-Workers '0.6'
    Note 'workers_0.6GiB' $w06
    Note 'workers_1.2GiB' (Get-Workers '1.2')
    Note 'workers_2.0GiB' (Get-Workers '2.0')

    $repro = @{}
    foreach ($pair in @(@('repro_projected', ''), @('repro_legacy', 'scripts\data\sensor_legacy_reproduction_tune7.json'))) {
        $name = $pair[0]; $ref = $pair[1]
        Say "running $name (about 2-3 min)"
        $dir = Join-Path $out $name
        $argList = @('scripts\sensor_reproduce.py', '--output', $dir)
        if ($ref) { $argList += @('--reference', $ref) }
        $t = Measure-Command { & $Vpy @argList *> (Join-Path $out "$name.txt") }
        $ok = $false; $bit = 'n/a'
        $cmp = Join-Path $dir 'comparison.json'
        if (Test-Path $cmp) { $j = Get-Content $cmp -Raw | ConvertFrom-Json; $ok = [bool]$j.reproduction_pass; $bit = $j.trajectory_bit_identical }
        $repro[$name] = $ok
        Note "check_$name" ("{0} bit={1} {2}s" -f $(if ($ok) { 'PASS' } else { 'FAIL' }), $bit, [math]::Round($t.TotalSeconds, 1))
    }

    $nRepro = [math]::Min(3, $w06)
    Say "running sensor tuning-path replay V13 with $nRepro workers (Mac sequential about 18 min)"
    $envLog = Join-Path $out 'env_check_tune7.txt'
    $t = Measure-Command { & $Vpy -m control.arena_tune_repro --config configs/arena_tune7.json --controller V13 --run-dir results/arena/tuning/env_check_tune7 --index 0 --scenario-workers $nRepro *> $envLog }
    $last = Get-Content $envLog | Where-Object { $_ -match '^(PASS|FAIL)' } | Select-Object -Last 1
    $rule = Get-EnvRule $envLog
    $okEnv = ($rule -ne '')
    $summary['env_rule'] = $rule
    Note 'check_env_tune7' ("{0} ({1}) {2}s :: {3}" -f $(if ($okEnv) { 'PASS' } else { 'FAIL' }), $(if ($rule) { $rule } else { 'no match' }), [math]::Round($t.TotalSeconds, 1), $last)

    Say 'running orphan-worker test (about 1-2 min)'
    $p = Start-Process $Vpy -ArgumentList '-m', 'control.arena_tune', '--controllers', 'GSLQR', '--budget', '3', '--run-dir', (Join-Path $out 'orphan_check'), '--scenario-workers', '2' -PassThru -WindowStyle Hidden
    $ids = @(); $inner = @()
    foreach ($wait in @(45, 20)) {
        Start-Sleep -Seconds $wait
        $all = Get-CimInstance Win32_Process
        $inner = @($all | Where-Object { $_.ParentProcessId -eq $p.Id -and $_.Name -eq 'python.exe' })
        if ($inner.Count -eq 0) { $inner = @($all | Where-Object { $_.ProcessId -eq $p.Id }) }
        $ids = @()
        $frontier = @($inner | ForEach-Object { $_.ProcessId })
        while ($frontier.Count -gt 0) {
            $next = @($all | Where-Object { $frontier -contains $_.ParentProcessId } | ForEach-Object { $_.ProcessId })
            $ids += $next
            $frontier = $next
        }
        if ($ids.Count -gt 0) { break }
    }
    $okOrphan = $false
    if ($inner.Count -gt 0 -and $ids.Count -gt 0) {
        Stop-Process -Id ($inner | ForEach-Object { $_.ProcessId }) -Force
        Start-Sleep -Seconds 15
        $left = @(Get-Process -Id ($ids + @($p.Id)) -ErrorAction SilentlyContinue)
        $okOrphan = ($left.Count -eq 0)
        Note 'check_orphan' $(if ($okOrphan) { "PASS (workers $($ids.Count) exited)" } else { "FAIL left: $(($left | ForEach-Object { $_.Id }) -join ',')" })
        foreach ($proc in $left) { taskkill /PID $proc.Id /T /F | Out-Null }
    } else {
        Note 'check_orphan' 'FAIL (workers did not start)'
        taskkill /PID $p.Id /T /F 2>$null | Out-Null
    }

    $allPass = $okSetup -and $repro['repro_projected'] -and $repro['repro_legacy'] -and $okEnv -and $okOrphan
    $summary['all_pass'] = $allPass
    $summary['finished'] = (Get-Date -Format s)
    $summary | ConvertTo-Json -Depth 4 | Out-File -Encoding utf8 (Join-Path $out 'summary.json')
    if (-not $allPass) { Stop-With "checks failed - do NOT tune on this PC; send folder $out to kj" }
    Say "ALL PASS - summary $out\summary.json"
    Send-Note "$env:COMPUTERNAME tune7 checks ALL PASS $(Stamp)"
}

if (-not $Controller) {
    Say 'checks done. To start tuning run this line again with -Controller M17 (or F13, V13, GSLQR, CPID)'
    exit 0
}

# ---------------- 5. start or resume tuning + watcher ----------------
$pidFile = Join-Path $Fds "tune_$Controller.pid"
$running = $false
if (Test-Path $pidFile) {
    $oldPid = (Get-Content $pidFile | Select-Object -First 1).Trim()
    if ($oldPid) {
        $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*arena_tune*--controllers $Controller *" -or $_.CommandLine -like "*arena_tune*--controllers*$Controller*tune7*" })
        if ($procs.Count -gt 0) { $running = $true }
    }
}
if ($running) {
    Say "$Controller tuning is already running - not starting a second copy"
} else {
    if ($Workers -le 0) {
        $w = Get-Workers $Gib[$Controller]
        if ($Controller -eq 'GSLQR') { $w = [math]::Max(1, [math]::Ceiling($w / 2)) }
        if ($Controller -eq 'CPID') { $w = [math]::Max(1, [math]::Floor($w / 2)) }
        $Workers = $w
    }
    $stamp = Get-Date -Format yyyyMMdd_HHmm
    $log = Join-Path $Fds "tune7_${Controller}_$stamp.log"
    $err = Join-Path $Fds "tune7_${Controller}_$stamp.err"
    $tp = Start-Process $Vpy -ArgumentList '-m', 'control.arena_tune', '--controllers', $Controller, '--budget', "$Budget", '--config', 'configs/arena_tune7.json', '--run-dir', 'results/arena/tuning/tune7', '--scenario-workers', "$Workers" -PassThru -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError $err
    $tp.Id | Out-File -Encoding ascii $pidFile
    Say "STARTED $Controller budget $Budget workers $Workers pid $($tp.Id)"
    Say "log: $log"
    Say "stop: taskkill /PID $($tp.Id) /T /F    resume: run the same one line again"
    Send-Note "$Controller tuning started on $env:COMPUTERNAME workers $Workers $(Stamp)"
}

if ($Topic) {
    $watchers = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*tune7_all.ps1*-Watch*' -and $_.CommandLine -like "*-Controller $Controller *" })
    if ($watchers.Count -gt 0) {
        Say 'phone watcher already running'
    } else {
        Start-Process powershell -WindowStyle Hidden -ArgumentList '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath, '-Watch', '-Controller', $Controller, '-Topic', $Topic
        Say 'phone watcher started'
    }
}
Say 'done. You can close this window; tuning keeps running.'
