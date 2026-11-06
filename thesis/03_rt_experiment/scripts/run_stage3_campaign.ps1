<#
.SYNOPSIS
  Stage-3 Gate B: 42 wall30 captures (prng+fpe x 7 B x 3 runs).
  Requires prior Gate A PASS. Stops on first capture failure.
#>
param(
  [ValidateSet("prng", "fpe", "both")][string]$Modes = "both",
  [int[]]$BlockSizes = @(64, 128, 256, 512, 1024, 2048, 4096)
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Cap = Join-Path $PSScriptRoot "capture_stage3_bsweep.ps1"
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$Report = Join-Path $Root "thesis\03_rt_experiment\analysis\stage3_bsweep_readiness\CAMPAIGN_PROGRESS.txt"
$modeList = if ($Modes -eq "both") { @("prng", "fpe") } else { @($Modes) }

# Refuse if Gate A failed
$ga = Join-Path $Root "thesis\03_rt_experiment\analysis\stage3_bsweep_readiness\digest_probe"
foreach ($m in @("prng", "fpe")) {
  $c = Join-Path $ga "${m}_B1024_COMPARE.txt"
  if (!(Test-Path $c)) { throw "Gate A missing for ${m}: $c" }
  $t = Get-Content -Raw $c
  if ($t -notmatch "match=True") { throw "Gate A did not PASS for ${m}" }
}

$done = 0
$total = $modeList.Count * $BlockSizes.Count * 3
"START utc=$((Get-Date).ToUniversalTime().ToString('o')) total=$total" | Set-Content $Report

foreach ($m in $modeList) {
  foreach ($B in $BlockSizes) {
    for ($r = 1; $r -le 3; $r++) {
      $tag = "phase3r_${m}_B${B}_r$r"
      $uart = Join-Path $LogDir "${tag}_uart_capture.txt"
      if (Test-Path $uart) {
        "SKIP existing $tag" | Add-Content $Report
        Write-Host "SKIP existing $tag"
        $done++
        continue
      }
      Write-Host "==== CAPTURE $tag ($($done+1)/$total) ===="
      powershell -NoProfile -ExecutionPolicy Bypass -File $Cap -ShuffleMode $m -Mode "B$B" -Run $r
      if ($LASTEXITCODE -ne 0) {
        "FAIL $tag exit=$LASTEXITCODE" | Add-Content $Report
        throw "Capture failed: $tag"
      }
      # Validate completeness
      $text = Get-Content -Raw $uart
      if ($text -notmatch "PHASE3 done") { throw "Incomplete UART: $tag" }
      if ($text -match "SecureFault") { throw "SecureFault: $tag" }
      if ($text -notmatch "incomplete=0") { throw "incomplete!=0: $tag" }
      if ($text -notmatch "shuffle_mode=$m") { throw "shuffle_mode mismatch: $tag" }
      if ($text -notmatch "B=$B") { throw "B mismatch: $tag" }
      if ($text -notmatch "IRQ_HMAC=1") { throw "IRQ_HMAC mismatch: $tag" }
      $done++
      "OK $tag" | Add-Content $Report
    }
  }
}
"DONE utc=$((Get-Date).ToUniversalTime().ToString('o')) ok=$done/$total" | Add-Content $Report
Write-Host "CAMPAIGN COMPLETE $done/$total"
