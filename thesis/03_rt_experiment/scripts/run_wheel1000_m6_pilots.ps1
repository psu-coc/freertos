<# Gate3: one run each Original/PRNG/FPE x B1024/2048/4096 (r1). #>
$ErrorActionPreference = "Stop"
$Cap = Join-Path $PSScriptRoot "capture_wheel1000_attest.ps1"
$modes = @("original", "prng", "fpe")
$Bs = @(1024, 2048, 4096)
$fail = @()
foreach ($m in $modes) {
  foreach ($B in $Bs) {
    Write-Host "`n==== PILOT $m B=$B r1 ===="
    try {
      & powershell -NoProfile -ExecutionPolicy Bypass -File $Cap -ShuffleMode $m -B $B -Run 1
      if ($LASTEXITCODE -ne 0) { throw "exit $LASTEXITCODE" }
    } catch {
      Write-Host "FAIL $m B=$B : $_"
      $fail += "${m}_B${B}"
    }
  }
}
if ($fail.Count -gt 0) {
  Write-Host "PILOTS FAILED: $($fail -join ', ')"
  exit 1
}
Write-Host "ALL PILOTS OK"
