<#
.SYNOPSIS
  Flash Secure+NS (or NS-only for noattest) for wall60 TIM2 observation and capture COM3.
.DESCRIPTION
  Does NOT run unless invoked explicitly. Legacy phase3_10Hz10Hz_* logs are untouched.
  Saves under logs/phase3w_B<B>_* or logs/phase2w_noattest_*.
.PARAMETER Mode
  noattest | B1024 | B2048 | B4096
#>
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet("noattest", "B1024", "B2048", "B4096")][string]$Mode
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

if ($Mode -eq "noattest") {
  $Build = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_wall60_noattest"
  $tag = "phase2w_noattest"
  $needSecure = $false
  $B = 0
} else {
  $B = [int]($Mode.Substring(1))
  $Build = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_wall60_original_B$B"
  $tag = "phase3w_B$B"
  $needSecure = $true
}

$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
if (!(Test-Path $nsElf)) { throw "Missing NS ELF in $Build - build first" }
if ($needSecure -and !(Test-Path $sElf)) { throw "Missing Secure ELF in $Build - build first" }

if ($needSecure) {
  Write-Host "Flash Secure $Mode"
  & $prog -c port=SWD mode=UR -d $sElf
  if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
}
Write-Host "Flash NonSecure $Mode"
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item (Join-Path $Build "run_meta.txt") (Join-Path $LogDir "${tag}_run_meta.txt") -Force

# Wall obs ~60s + dump + optional attest drain. B=4096 may drain longer.
$timeoutSec = if ($B -ge 4096) { 300 } elseif ($B -ge 2048) { 180 } else { 120 }

python -c @"
import serial, time, subprocess
from pathlib import Path
tag = r'$tag'
B = $B
base = Path(r'$LogDir')
out = base / f'{tag}_uart_capture.txt'
prog = r'$prog'
timeout = $timeoutSec
port = serial.Serial('COM3', 115200, timeout=0.2)
buf, t0, done = [], time.time(), False
print(f'listening {tag} timeout={timeout}s ...', flush=True)
subprocess.run([prog, '-c', 'port=SWD', 'mode=UR', '-rst', '-hardRst'], capture_output=True)
time.sleep(0.3)
markers = ('PHASE3 done', 'No-attest baseline done', '=== PHASE3 done', 'Idle.')
while time.time() - t0 < timeout:
    n = port.in_waiting
    if n:
        chunk = port.read(n).decode('utf-8', errors='replace')
        buf.append(chunk)
        text = ''.join(buf)
        if 'END obs_per_second.csv' in text and ('END wheel_timing.csv' in text or 'END wheel' in text):
            if any(m in text for m in ('PHASE3 done', 'baseline done', 'Idle.')):
                time.sleep(0.8)
                if port.in_waiting:
                    buf.append(port.read(port.in_waiting).decode('utf-8', errors='replace'))
                done = True
                break
    else:
        time.sleep(0.05)
port.close()
text = ''.join(buf)
out.write_text(text, encoding='utf-8')
print(f'saved {out.name} bytes={len(text)} done={done} elapsed={time.time()-t0:.1f}s')

def extract(begin, end, dest):
    i, j = text.find(begin), text.find(end)
    if i < 0 or j <= i:
        t2 = text.replace('\r\n', '\n')
        b2, e2 = begin.replace('\r\n', '\n'), end.replace('\r\n', '\n')
        i, j = t2.find(b2), t2.find(e2)
        if i < 0 or j <= i:
            print('MISSING', begin.strip()); return
        body = t2[i+len(b2):j].strip() + '\n'
    else:
        body = text[i+len(begin):j].replace('\r\n', '\n').strip() + '\n'
    dest.write_text(body, encoding='utf-8')
    print('wrote', dest.name)

extract('BEGIN obs_per_second.csv\r\n', 'END obs_per_second.csv', base / f'{tag}_obs_per_second.csv')
extract('BEGIN sonar_timing.csv\r\n', 'END sonar_timing.csv', base / f'{tag}_sonar_timing.csv')
extract('BEGIN wheel_timing.csv\r\n', 'END wheel_timing.csv', base / f'{tag}_wheel_timing.csv')
"@

Write-Host "Done $Mode -> $LogDir\${tag}_*"
Write-Host "Parse OBS_WALL lines from ${tag}_uart_capture.txt for comparison table."
