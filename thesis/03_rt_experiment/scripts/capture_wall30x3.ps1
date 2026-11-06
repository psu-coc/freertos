<#
.SYNOPSIS
  Flash + capture one wall30 run (warm-up 2s + obs 30s). Call 3x per config.
.DESCRIPTION
  Does NOT run unless invoked explicitly. Never overwrites phase2w_/phase3w_ or
  phase2_10Hz10Hz_/phase3_10Hz10Hz_ legacy logs.
.PARAMETER Mode
  noattest | B64 | B128 | B256 | B512 | B1024 | B2048 | B4096
.PARAMETER Run
  Repetition index 1..3
#>
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet("noattest", "B64", "B128", "B256", "B512", "B1024", "B2048", "B4096")][string]$Mode,
  [Parameter(Mandatory = $true)]
  [ValidateRange(1, 3)][int]$Run
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

if ($Mode -eq "noattest") {
  $Build = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_wall30_noattest"
  $tag = "phase2r_noattest_r$Run"
  $needSecure = $false
  $B = 0
} else {
  $B = [int]($Mode.Substring(1))
  $Build = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_wall30_original_B$B"
  $tag = "phase3r_B${B}_r$Run"
  $needSecure = $true
}

$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
if (!(Test-Path $nsElf)) { throw "Missing NS ELF in $Build - build first" }
if ($needSecure -and !(Test-Path $sElf)) { throw "Missing Secure ELF in $Build - build first" }

# Refuse to clobber legacy names
foreach ($legacy in @("phase2w_", "phase3w_", "phase2_10Hz10Hz_", "phase3_10Hz10Hz_")) {
  if ($tag.StartsWith($legacy.TrimEnd('_'))) { throw "Refusing legacy tag $tag" }
}

Write-Host "REVIEW GATE: invoke only after metric approval. Mode=$Mode Run=$Run"
if ($needSecure) {
  & $prog -c port=SWD mode=UR -d $sElf
  if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
}
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item (Join-Path $Build "run_meta.txt") (Join-Path $LogDir "${tag}_run_meta.txt") -Force
# wall30: 2s warm + 30s obs + CSV dump; leave margin beyond 32s wall time
$timeoutSec = if ($B -ge 4096) { 120 } elseif ($B -ge 2048) { 90 } else { 90 }

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
