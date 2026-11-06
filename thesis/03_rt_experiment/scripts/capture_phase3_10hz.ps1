<#
.SYNOPSIS
  Flash Secure+NS for one Original SMARM B on 10Hz10Hz 60s and capture COM3.
.DESCRIPTION
  Requires explicit user authorization before use.
  Saves logs under logs/phase3_10Hz10Hz_B<B>_*.
#>
param(
  [Parameter(Mandatory = $true)][ValidateSet(1024, 2048, 4096)][int]$B
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Build = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_60s_original_B$B"
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"
$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
if (!(Test-Path $sElf) -or !(Test-Path $nsElf)) { throw "Missing ELFs in $Build — build first" }

Write-Host "Flash Secure B=$B"
& $prog -c port=SWD mode=UR -d $sElf
if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
Write-Host "Flash NonSecure B=$B"
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item (Join-Path $Build "run_meta.txt") (Join-Path $LogDir "phase3_10Hz10Hz_B${B}_run_meta.txt") -Force

python -c @"
import serial, time, sys, subprocess
from pathlib import Path
B = $B
base = Path(r'$LogDir')
out = base / f'phase3_10Hz10Hz_B{B}_uart_capture.txt'
prog = r'$prog'
port = serial.Serial('COM3', 115200, timeout=0.2)
buf, t0, done = [], time.time(), False
print(f'listening B={B} ...', flush=True)
subprocess.run([prog, '-c', 'port=SWD', 'mode=UR', '-rst', '-hardRst'], capture_output=True)
time.sleep(0.3)
while time.time() - t0 < 120:
    n = port.in_waiting
    if n:
        chunk = port.read(n).decode('utf-8', errors='replace')
        buf.append(chunk)
        text = ''.join(buf)
        if 'PHASE3 done' in text or f'B={B}' in text and 'Idle.' in text and 'END wheel' in text:
            # wait for definitive done marker
            if 'PHASE3 done' in text or '=== PHASE3 done' in text:
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
        t2 = text.replace('\r\n','\n')
        i, j = t2.find(begin.replace('\r\n','\n')), t2.find(end.replace('\r\n','\n'))
        if i < 0 or j <= i:
            print('MISSING', begin); return
        body = t2[i+len(begin.replace('\r\n','\n')):j].strip()+'\n'
    else:
        body = text[i+len(begin):j].replace('\r\n','\n').strip()+'\n'
    dest.write_text(body, encoding='utf-8')

extract('BEGIN sonar_timing.csv\r\n', 'END sonar_timing.csv', base/f'phase3_10Hz10Hz_B{B}_sonar_timing.csv')
extract('BEGIN wheel_timing.csv\r\n', 'END wheel_timing.csv', base/f'phase3_10Hz10Hz_B{B}_wheel_timing.csv')
"@

python (Join-Path $PSScriptRoot "analyze_phase3_capture.py") `
  --uart (Join-Path $LogDir "phase3_10Hz10Hz_B${B}_uart_capture.txt") `
  --sonar-period-us 100000 --wheel-period-us 100000 `
  --out (Join-Path $LogDir "phase3_10Hz10Hz_B${B}_SUMMARY.md")

Write-Host "Done B=$B"
