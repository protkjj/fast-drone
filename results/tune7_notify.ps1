# tune7 tuning watcher: phone push via ntfy.sh. Read-only on the tuning folder.
# Run OUTSIDE the fds code folders (save this file in $HOME). It never touches control/, models/, configs/.
# Messages carry only the controller name, counts and times (ntfy.sh is a public server).
param(
    [Parameter(Mandatory = $true)][string]$Controller,
    [Parameter(Mandatory = $true)][string]$Topic,
    [string]$Dir = "$HOME\fds\results\arena\tuning\tune7",
    [int]$IntervalSeconds = 300,
    [int]$StallMinutes = 120,
    [int]$ProgressEvery = 30,
    [switch]$Test
)

$errLog = Join-Path $HOME "tune7_notify_$Controller.err"

function Send-Note([string]$Message) {
    try {
        Invoke-RestMethod -Method Post -Uri "https://ntfy.sh/$Topic" -Body $Message -TimeoutSec 20 | Out-Null
    } catch {
        Add-Content -Path $errLog -Value "$(Get-Date -Format s) send failed: $($_.Exception.Message)"
    }
}

function Stamp { Get-Date -Format 'MM-dd HH:mm' }

if ($Test) {
    Send-Note "$Controller notify test OK $(Stamp)"
    exit 0
}

$record = Join-Path $Dir "$Controller.record.json"
$jsonl = Join-Path $Dir "$Controller.jsonl"
$started = Get-Date
$lastSpent = -1
$stallSent = $false

Send-Note "$Controller watcher started $(Stamp)"

while ($true) {
    if (Test-Path $record) {
        $r = $null
        try { $r = Get-Content $record -Raw | ConvertFrom-Json } catch { $r = $null }
        if ($null -ne $r) {
            $spent = [int]$r.spent
            $budget = [int]$r.budget
            if ($spent -ne $lastSpent) {
                $crossed = ($lastSpent -ge 0) -and ([math]::Floor($spent / $ProgressEvery) -gt [math]::Floor($lastSpent / $ProgressEvery))
                if ($spent -eq 1) {
                    $mins = [int](New-TimeSpan -Start $started -End (Get-Date)).TotalMinutes
                    Send-Note "$Controller first eval done (1/$budget) $(Stamp), about $mins min after watcher start"
                } elseif ($crossed) {
                    Send-Note "$Controller $spent/$budget evals $(Stamp)"
                }
                $lastSpent = $spent
                $stallSent = $false
            }
            if ($r.status -eq 'complete') {
                Send-Note "$Controller COMPLETE $spent/$budget $(Stamp)"
                exit 0
            }
        }
    }
    if (Test-Path $jsonl) {
        $age = (New-TimeSpan -Start (Get-Item $jsonl).LastWriteTime -End (Get-Date)).TotalMinutes
        if (($age -gt $StallMinutes) -and (-not $stallSent)) {
            Send-Note "$Controller no new eval for $([int]$age) min - check this PC $(Stamp)"
            $stallSent = $true
        }
    }
    Start-Sleep -Seconds $IntervalSeconds
}
