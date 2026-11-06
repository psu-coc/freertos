<#
.SYNOPSIS
  Run remaining Table I cells (skips configs that already have UART capture).
#>
param(
  [string[]]$Modes = @("original", "prng", "fpe"),
  [int[]]$Bs = @(4096, 2048, 512, 256, 128, 64),
  [switch]$IncludePilots  # also run B=1024 if missing
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Cap = Join-Path $PSScriptRoot "capture_table_i.ps1"
$LogDir = Join-Path $Root "thesis\03_rt_experiment\table_i_runtime\logs"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

$allB = $Bs
if ($IncludePilots) { $allB = @(1024) + $Bs }

$failures = @()
foreach ($B in $allB) {
  foreach ($mode in $Modes) {
    $tag = "table_i_${mode}_B$B"
    $uart = Join-Path $LogDir "${tag}_uart_capture.txt"
    if (Test-Path $uart) {
      Write-Host "SKIP existing $tag"
      continue
    }
    Write-Host "==== RUN $tag ===="
    try {
      & powershell -NoProfile -ExecutionPolicy Bypass -File $Cap -ShuffleMode $mode -B $B
      if ($LASTEXITCODE -ne 0) { throw "capture exit $LASTEXITCODE" }
    } catch {
      $failures += "$tag : $_"
      Write-Host "FAIL $tag : $_"
      # Continue other cells; report at end
    }
  }
}

python (Join-Path $PSScriptRoot "analyze_table_i.py")
if ($failures.Count -gt 0) {
  Write-Host "FAILURES:"
  $failures | ForEach-Object { Write-Host $_ }
  exit 1
}
Write-Host "CAMPAIGN COMPLETE"
