<#
.SYNOPSIS
  Wheel100Hz M3: pilots (Gate B) or full matrix (Gate C).
  Never overwrites existing phase4r_w100_* captures. Stops on first failure.
#>
param(
  [ValidateSet("pilots", "full")][string]$Phase = "pilots"
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Cap = Join-Path $PSScriptRoot "capture_wheel100_attest.ps1"
$Report = Join-Path $Root "thesis\03_rt_experiment\analysis\wheel100_m3\CAMPAIGN_PROGRESS.txt"
New-Item -ItemType Directory -Force -Path (Split-Path $Report) | Out-Null

$modes = @("original", "prng", "fpe")
if ($Phase -eq "pilots") {
  $Bs = @(1024, 2048, 4096)
  $reps = @(1)  # one run per config; tag ..._r1
} else {
  $Bs = @(64, 128, 256, 512, 1024, 2048, 4096)
  $reps = @(1, 2, 3)
}

$total = $modes.Count * $Bs.Count * $reps.Count
$done = 0
"START phase=$Phase utc=$((Get-Date).ToUniversalTime().ToString('o')) total=$total" | Set-Content $Report

foreach ($m in $modes) {
  foreach ($B in $Bs) {
    foreach ($r in $reps) {
      $tag = "phase4r_w100_${m}_B${B}_r$r"
      $uart = Join-Path $Root "thesis\03_rt_experiment\logs\${tag}_uart_capture.txt"
      if (Test-Path $uart) {
        "SKIP existing $tag" | Add-Content $Report
        Write-Host "SKIP $tag"
        $done++
        continue
      }
      Write-Host "==== $tag ($($done+1)/$total) ===="
      & powershell -NoProfile -ExecutionPolicy Bypass -File $Cap -ShuffleMode $m -B $B -Run $r
      if ($LASTEXITCODE -ne 0) {
        "FAIL $tag exit=$LASTEXITCODE" | Add-Content $Report
        throw "Capture failed: $tag"
      }
      $done++
      "OK $tag" | Add-Content $Report
    }
  }
}
"DONE utc=$((Get-Date).ToUniversalTime().ToString('o')) ok=$done/$total" | Add-Content $Report
Write-Host "CAMPAIGN $Phase COMPLETE $done/$total"
