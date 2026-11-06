<#
.SYNOPSIS
  Build all wall30 configs for the Original SMARM B-sweep (+ No Attestation).
.DESCRIPTION
  Does NOT flash or open COM3. Archives under thesis/03_rt_experiment/builds/.
  Order: noattest, then B=64,128,256,512,1024,2048,4096.
#>
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
$Bs = @(64, 128, 256, 512, 1024, 2048, 4096)

Write-Host "=== wall30 B-sweep offline builds (no flash) ==="
& (Join-Path $here "build_rt_profile.ps1") -Profile 10Hz10Hz -RunLen wall30
if ($LASTEXITCODE -ne 0) { throw "noattest build failed" }

foreach ($B in $Bs) {
  Write-Host "=== building Original B=$B ==="
  & (Join-Path $here "build_rt_profile.ps1") -Profile 10Hz10Hz -RunLen wall30 -WithOriginalSmarm -B $B
  if ($LASTEXITCODE -ne 0) { throw "B$B build failed" }
}

$root = (Resolve-Path (Join-Path $here "..\builds")).Path
$verify = @()
$verify += [pscustomobject]@{ Config = "noattest"; Dir = "10Hz10Hz_wall30_noattest"; NeedSecure = $false; B = 0 }
foreach ($B in $Bs) {
  $verify += [pscustomobject]@{ Config = "B$B"; Dir = "10Hz10Hz_wall30_original_B$B"; NeedSecure = $true; B = $B }
}

Write-Host "`n=== VERIFY ARCHIVES ==="
$fail = 0
foreach ($v in $verify) {
  $d = Join-Path $root $v.Dir
  $ns = Join-Path $d "NonSecure.elf"
  $meta = Join-Path $d "run_meta.txt"
  $cfg = Join-Path $d "attest_overhead_cfg.h"
  $ok = (Test-Path $ns) -and (Test-Path $meta)
  if ($v.NeedSecure) { $ok = $ok -and (Test-Path (Join-Path $d "Secure.elf")) }
  $metaT = if (Test-Path $meta) { Get-Content $meta -Raw } else { "" }
  $cfgT = if (Test-Path $cfg) { Get-Content $cfg -Raw } else { "" }
  $p3 = if ($metaT -match "PHASE3_WITH_ORIGINAL_SMARM=(\d+)") { $Matches[1] } else { "?" }
  $bMeta = if ($metaT -match "ATTEST_BLOCK_SIZE=(\d+)") { $Matches[1] } else { "?" }
  $sau = if ($cfgT -match "USE_SAU_APPROACH\s+(\d+)") { $Matches[1] } else { "?" }
  $snap = if ($cfgT -match "USE_SNAP\s+(\d+)") { $Matches[1] } else { "?" }
  $bCfg = if ($cfgT -match "ATTEST_BLOCK_SIZE\s+(\d+)") { $Matches[1] } else { "?" }
  $expectP3 = if ($v.NeedSecure) { "1" } else { "0" }
  $expectB = if ($v.NeedSecure) { "$($v.B)" } else { $bMeta }
  $pass = $ok -and ($p3 -eq $expectP3) -and ($sau -eq "0") -and ($snap -eq "0")
  if ($v.NeedSecure) { $pass = $pass -and ($bCfg -eq "$($v.B)") -and ($bMeta -eq "$($v.B)") }
  $status = if ($pass) { "OK" } else { "FAIL"; $fail++ }
  Write-Host ("{0,-12} {1} P3={2} Bmeta={3} Bcfg={4} SAU={5} SNAP={6} Secure={7}" -f `
    $v.Config, $status, $p3, $bMeta, $bCfg, $sau, $snap, $v.NeedSecure)
}
if ($fail -gt 0) { throw "$fail config verification failures" }
Write-Host "ALL_BUILDS_VERIFIED"
Write-Host "Flash/COM3 intentionally NOT performed."
