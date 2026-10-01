# tune7 school check: runs every pre-tuning check on this PC and writes one summary.
# Usage (from any folder):  powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune7_school_check.ps1 [-Topic <ntfy topic>]
# Needs: $HOME\fds checked out at tune-final-7 and $HOME\fds\.venv with requirements installed.
# Writes only under $HOME\fds\tune7_check_<PC>_<time>\ (outside control/, models/, configs/).
param([string]$Topic = '')

$ErrorActionPreference = 'Continue'
$ExpectedHead = '87039e946ce122c9f89bb89a4127c086ac459e90'
$ExpectedConfigSha = 'e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801'

$fds = Join-Path $HOME 'fds'
$py = Join-Path $fds '.venv\Scripts\python.exe'
if (-not (Test-Path $fds)) { Write-Host 'FAIL: $HOME\fds not found (clone first)'; exit 1 }
if (-not (Test-Path $py)) { Write-Host 'FAIL: .venv not found in fds (create venv and pip install first)'; exit 1 }
Set-Location $fds

$env:OMP_NUM_THREADS = '1'; $env:OPENBLAS_NUM_THREADS = '1'; $env:VECLIB_MAXIMUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'; $env:PYTHONUTF8 = '1'

$pc = $env:COMPUTERNAME
$stamp = Get-Date -Format yyyyMMdd_HHmm
$out = Join-Path $fds "tune7_check_${pc}_$stamp"
New-Item -ItemType Directory -Force -Path $out | Out-Null
$summary = [ordered]@{ computer = $pc; started = (Get-Date -Format s) }

function Note([string]$Key, $Value) {
    $summary[$Key] = $Value
    Write-Host ('{0,-34} {1}' -f $Key, $Value)
}
function Send-Note([string]$Message) {
    if ($Topic) {
        try { Invoke-RestMethod -Method Post -Uri "https://ntfy.sh/$Topic" -Body $Message -TimeoutSec 20 | Out-Null } catch { }
    }
}

Write-Host "== tune7 school check on $pc, results in $out"

$tag = (git describe --tags --exact-match 2>$null)
$head = (git rev-parse HEAD)
$okTag = ($tag -eq 'tune-final-7') -and ($head -eq $ExpectedHead)
Note 'tag' "$tag $head"
Note 'check_tag' $(if ($okTag) { 'PASS' } else { 'FAIL' })

$cores = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores -Sum).Sum
$cpuName = (Get-CimInstance Win32_Processor | Select-Object -First 1).Name
Note 'cpu' $cpuName
Note 'physical_cores' $cores
Note 'mem_total_GiB' ([math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB, 1))
Note 'mem_free_GiB' ([math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory / 1MB, 1))
$pyVersion = (& $py --version 2>&1 | Out-String).Trim()
Note 'python' $pyVersion

$setupLog = Join-Path $out 'setup_env.txt'
& $py scripts\setup_env.py --commit $head --config configs\arena_tune7.json --expect-config-sha256 $ExpectedConfigSha *> $setupLog
$okSetup = ($LASTEXITCODE -eq 0)
Note 'check_setup_env' $(if ($okSetup) { 'PASS' } else { "FAIL (see $setupLog)" })

function Get-Workers([string]$Gib) {
    $text = (& $py -m control.arena_suggest_workers $Gib --physical-cores $cores 2>&1 | Out-String)
    if ($text -match 'scenario-workers\s+(\d+)') { return [int]$Matches[1] }
    return 1
}
$w06 = Get-Workers '0.6'
$w12 = Get-Workers '1.2'
$w20 = Get-Workers '2.0'
Note 'workers_0.6GiB(V13,GSLQR,CPID)' $w06
Note 'workers_1.2GiB(F13)' $w12
Note 'workers_2.0GiB(M17)' $w20

function Invoke-Repro([string]$Name, [string]$Reference) {
    $dir = Join-Path $out $Name
    $log = Join-Path $out "$Name.txt"
    $argList = @('scripts\sensor_reproduce.py', '--output', $dir)
    if ($Reference) { $argList += @('--reference', $Reference) }
    $time = Measure-Command { & $py @argList *> $log }
    $cmp = Join-Path $dir 'comparison.json'
    $pass = $false
    $bit = 'n/a'
    if (Test-Path $cmp) {
        $j = Get-Content $cmp -Raw | ConvertFrom-Json
        $pass = [bool]$j.reproduction_pass
        $bit = $j.trajectory_bit_identical
    }
    return [ordered]@{ pass = $pass; bit_identical = $bit; seconds = [math]::Round($time.TotalSeconds, 1); log = $log }
}

Write-Host '== sensor reproduction references (about 2-3 min each)'
$rp = Invoke-Repro 'repro_projected' ''
Note 'check_repro_projected' ("{0} bit={1} {2}s" -f $(if ($rp.pass) { 'PASS' } else { 'FAIL' }), $rp.bit_identical, $rp.seconds)
$rl = Invoke-Repro 'repro_legacy' 'scripts\data\sensor_legacy_reproduction_tune7.json'
Note 'check_repro_legacy' ("{0} bit={1} {2}s" -f $(if ($rl.pass) { 'PASS' } else { 'FAIL' }), $rl.bit_identical, $rl.seconds)

$nRepro = [math]::Min(3, $w06)
Write-Host "== sensor tuning-path reproduction V13 (workers $nRepro, Mac sequential about 18 min)"
$envLog = Join-Path $out 'env_check_tune7.txt'
$time = Measure-Command { & $py -m control.arena_tune_repro --config configs/arena_tune7.json --controller V13 --run-dir results/arena/tuning/env_check_tune7 --index 0 --scenario-workers $nRepro *> $envLog }
$last = Get-Content $envLog | Where-Object { $_ -match '^(PASS|FAIL)' } | Select-Object -Last 1
$okEnv = ($null -ne $last) -and ($last -like 'PASS*')
Note 'check_env_tune7' ("{0} {1}s :: {2}" -f $(if ($okEnv) { 'PASS' } else { 'FAIL' }), [math]::Round($time.TotalSeconds, 1), $last)

Write-Host '== orphan worker check (about 1-2 min)'
$orphanDir = Join-Path $out 'orphan_check'
$p = Start-Process $py -ArgumentList '-m', 'control.arena_tune', '--controllers', 'GSLQR', '--budget', '3', '--run-dir', $orphanDir, '--scenario-workers', '2' -PassThru -WindowStyle Hidden
$ids = @()
$inner = @()
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
if ($inner.Count -eq 0 -or $ids.Count -eq 0) {
    $okOrphan = $false
    Note 'check_orphan' 'FAIL (workers did not start; nothing to test)'
} else {
    Stop-Process -Id ($inner | ForEach-Object { $_.ProcessId }) -Force
    Start-Sleep -Seconds 15
    $left = @(Get-Process -Id ($ids + @($p.Id)) -ErrorAction SilentlyContinue)
    $okOrphan = ($left.Count -eq 0)
    Note 'check_orphan' $(if ($okOrphan) { "PASS (workers $($ids.Count) exited)" } else { "FAIL left: $(($left | ForEach-Object { $_.Id }) -join ',')" })
    foreach ($proc in $left) { taskkill /PID $proc.Id /T /F | Out-Null }
}

$allPass = $okTag -and $okSetup -and $rp.pass -and $rl.pass -and $okEnv -and $okOrphan
$summary['all_pass'] = $allPass
$summary['finished'] = (Get-Date -Format s)
$gslqr = [math]::Max(1, [math]::Ceiling($w06 / 2))
$cpid = [math]::Max(1, [math]::Floor($w06 / 2))
$summary['start_workers'] = [ordered]@{ M17 = $w20; F13 = $w12; V13 = $w06; GSLQR = $gslqr; CPID = $cpid }
$summary | ConvertTo-Json -Depth 4 | Out-File -Encoding utf8 (Join-Path $out 'summary.json')

Write-Host ''
Write-Host '============================================================'
if ($allPass) { Write-Host "ALL PASS on $pc - this PC may start tuning" } else { Write-Host "FAIL on $pc - do NOT start tuning here; send $out to kj" }
Write-Host "Start command (pick this PC's controller):"
Write-Host "  powershell -NoProfile -ExecutionPolicy Bypass -File `$HOME\tune7_start.ps1 -Controller M17  (workers $w20)"
Write-Host "  ... F13 (workers $w12), V13 (workers $w06), GSLQR (workers $gslqr) + CPID (workers $cpid) on computer 4"
Write-Host "Summary: $out\summary.json"
Send-Note ("{0} tune7 check {1} {2}" -f $pc, $(if ($allPass) { 'ALL PASS' } else { 'FAIL' }), (Get-Date -Format 'MM-dd HH:mm'))
if ($allPass) { exit 0 } else { exit 1 }
