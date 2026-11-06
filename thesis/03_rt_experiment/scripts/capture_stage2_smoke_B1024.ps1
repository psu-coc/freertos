<#
.SYNOPSIS
  Stage-2 B1024 hardware smoke: flash + capture ONE run for original|prng|fpe.
.DESCRIPTION
  Unique tags: phase3r_smoke_{mode}_B1024_r{N}
  NEVER overwrites historical phase3r_B1024_r* campaign logs.
  Requires explicit user authorization for flash/COM3 (caller responsibility).
#>
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet("original", "prng", "fpe")][string]$ShuffleMode,
  [ValidateRange(1, 9)][int]$Run = 1
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

$Build = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_wall30_${ShuffleMode}_B1024"
$tag = "phase3r_smoke_${ShuffleMode}_B1024_r$Run"

# Hard refuse historical campaign tag collision
if ($tag -match '^phase3r_B\d+_r') {
  throw "Refusing campaign-style tag $tag"
}
foreach ($legacy in @(
    "phase3r_B1024_r1", "phase3r_B1024_r2", "phase3r_B1024_r3",
    "phase2r_noattest_r1", "phase2r_noattest_r2", "phase2r_noattest_r3"
  )) {
  if ($tag -eq $legacy) { throw "Refusing to clobber historical tag $legacy" }
}

$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
if (!(Test-Path $nsElf)) { throw "Missing NS ELF in $Build" }
if (!(Test-Path $sElf)) { throw "Missing Secure ELF in $Build" }

# Refuse overwrite of existing smoke capture (force unique Run)
$uartOut = Join-Path $LogDir "${tag}_uart_capture.txt"
if (Test-Path $uartOut) {
  throw "Smoke log already exists: $uartOut - bump -Run or rename; never overwrite"
}

Write-Host "STAGE2 SMOKE: mode=$ShuffleMode tag=$tag build=$Build"
Write-Host "Secure SHA16=$((Get-FileHash $sElf -Algorithm SHA256).Hash.Substring(0,16))"
Write-Host "NS     SHA16=$((Get-FileHash $nsElf -Algorithm SHA256).Hash.Substring(0,16))"

& $prog -c port=SWD mode=UR -d $sElf
if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item (Join-Path $Build "run_meta.txt") (Join-Path $LogDir "${tag}_run_meta.txt") -Force
# Annotate flash performed
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "flash=PERFORMED_stage2_smoke"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "tag=$tag"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "secure_sha16=$((Get-FileHash $sElf -Algorithm SHA256).Hash.Substring(0,16))"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "ns_sha16=$((Get-FileHash $nsElf -Algorithm SHA256).Hash.Substring(0,16))"

$timeoutSec = 90
python -c @"
import serial, time, subprocess
from pathlib import Path
tag = r'$tag'
base = Path(r'$LogDir')
out = base / f'{tag}_uart_capture.txt'
prog = r'$prog'
timeout = $timeoutSec
port = serial.Serial('COM3', 115200, timeout=0.2)
buf, t0, done = [], time.time(), False
print(f'listening {tag} timeout={timeout}s ...', flush=True)
subprocess.run([prog, '-c', 'port=SWD', 'mode=UR', '-rst', '-hardRst'], capture_output=True)
time.sleep(0.3)
while time.time() - t0 < timeout:
    n = port.in_waiting
    if n:
        chunk = port.read(n).decode('utf-8', errors='replace')
        buf.append(chunk)
        text = ''.join(buf)
        if 'END obs_per_second.csv' in text and 'END wheel' in text:
            if any(m in text for m in ('PHASE3 done', 'baseline done', 'Idle.')):
                time.sleep(0.5)
                if port.in_waiting:
                    buf.append(port.read(port.in_waiting).decode('utf-8', errors='replace'))
                done = True
                break
    else:
        time.sleep(0.05)
port.close()
text = ''.join(buf)
out.write_text(text, encoding='utf-8')
print(f'saved {out.name} bytes={len(text)} done={done}')

def extract(begin, end, dest):
    i, j = text.find(begin), text.find(end)
    if i < 0 or j <= i:
        print('MISSING', begin.strip()); return
    dest.write_text(text[i+len(begin):j].replace('\r\n','\n').strip()+'\n', encoding='utf-8')
extract('BEGIN obs_per_second.csv\r\n', 'END obs_per_second.csv', base/f'{tag}_obs_per_second.csv')
extract('BEGIN sonar_timing.csv\r\n', 'END sonar_timing.csv', base/f'{tag}_sonar_timing.csv')
extract('BEGIN wheel_timing.csv\r\n', 'END wheel_timing.csv', base/f'{tag}_wheel_timing.csv')
"@

Write-Host "Done $tag - parse with analyze_wall30x3.py when ready"
