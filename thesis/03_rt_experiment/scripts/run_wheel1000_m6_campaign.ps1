<#
.SYNOPSIS
  Wheel1000Hz M6 full matrix: 7 B x 3 modes x 3 reps.
  Skips existing phase4r_w1000_* captures (pilots count as r1).
  Builds once per (mode,B) then flashes/captures with -SkipBuild.
#>
param(
  [ValidateSet("full")][string]$Phase = "full"
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Cap = Join-Path $PSScriptRoot "capture_wheel1000_attest.ps1"
$Build = Join-Path $PSScriptRoot "build_rt_profile.ps1"
$Report = Join-Path $Root "thesis\03_rt_experiment\analysis\wheel1000_m6\CAMPAIGN_PROGRESS.txt"
New-Item -ItemType Directory -Force -Path (Split-Path $Report) | Out-Null

$modes = @("original", "prng", "fpe")
$Bs = @(64, 128, 256, 512, 1024, 2048, 4096)
$reps = @(1, 2, 3)

$total = $modes.Count * $Bs.Count * $reps.Count
$done = 0
"START phase=$Phase utc=$((Get-Date).ToUniversalTime().ToString('o')) total=$total" | Set-Content $Report

foreach ($m in $modes) {
  foreach ($B in $Bs) {
    $needAny = $false
    foreach ($r in $reps) {
      $tag = "phase4r_w1000_${m}_B${B}_r$r"
      $uart = Join-Path $Root "thesis\03_rt_experiment\logs\${tag}_uart_capture.txt"
      if (!(Test-Path $uart)) { $needAny = $true; break }
    }
    if ($needAny) {
      Write-Host "BUILD once $m B=$B"
      & powershell -NoProfile -ExecutionPolicy Bypass -File $Build `
        -Profile Wheel1000Hz -RunLen wall3 -ShuffleMode $m -B $B
      if ($LASTEXITCODE -ne 0) {
        "FAIL build ${m}_B$B" | Add-Content $Report
        throw "Build failed ${m}_B$B"
      }
    }
    foreach ($r in $reps) {
      $tag = "phase4r_w1000_${m}_B${B}_r$r"
      $uart = Join-Path $Root "thesis\03_rt_experiment\logs\${tag}_uart_capture.txt"
      if (Test-Path $uart) {
        "SKIP existing $tag" | Add-Content $Report
        Write-Host "SKIP $tag"
        $done++
        continue
      }
      Write-Host "==== $tag ($($done+1)/$total) ===="
      & powershell -NoProfile -ExecutionPolicy Bypass -File $Cap `
        -ShuffleMode $m -B $B -Run $r -SkipBuild
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
