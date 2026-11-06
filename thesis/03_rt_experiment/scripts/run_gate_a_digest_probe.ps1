<#
.SYNOPSIS
  Gate A: flash digest-probe build, dump |M|, compare on-target vs host digest.
  Does not write phase3r_* performance logs.
#>
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet("original", "prng", "fpe")][string]$ShuffleMode
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$OutDir = Join-Path $Root "thesis\03_rt_experiment\analysis\stage3_bsweep_readiness\digest_probe"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$Build = Join-Path $Root "thesis\03_rt_experiment\builds\digest_probe_${ShuffleMode}_B1024"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"
$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
if (!(Test-Path $sElf) -or !(Test-Path $nsElf)) { throw "Missing probe build $Build" }
$meta = Get-Content -Raw (Join-Path $Build "run_meta.txt")
if ($meta -notmatch "RT_DIGEST_PROBE=1") { throw "Build is not a digest probe" }
if ($meta -notmatch "ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode") { throw "Mode mismatch" }

$chal = "0123456789abcdeffedcba9876543210"
$memBin = Join-Path $OutDir "${ShuffleMode}_B1024_M.bin"
$uartLog = Join-Path $OutDir "${ShuffleMode}_B1024_uart.txt"
$hostExe = Join-Path $OutDir "host_digest_from_mem.exe"

$patterned = Join-Path $OutDir "patterned_M_128KiB.bin"
Write-Host "GATE A flash $ShuffleMode probe"
& $prog -c port=SWD mode=UR -d $sElf
if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

# Patterned |M| required: erased 0xFF collapses mode digests
if (!(Test-Path $patterned)) { throw "Missing patterned |M|: $patterned" }
Write-Host "Write patterned |M| @ 0x08060000"
& $prog -c port=SWD mode=UR -w $patterned 0x08060000 -v
if ($LASTEXITCODE -ne 0) { throw "Patterned |M| write failed" }

# Dump attested window AFTER patterned write (verified |M|)
if (Test-Path $memBin) { Remove-Item $memBin -Force }
& $prog -c port=SWD mode=UR -r 0x08060000 0x20000 $memBin
if ($LASTEXITCODE -ne 0 -or !(Test-Path $memBin)) { throw "Flash dump failed" }
$len = (Get-Item $memBin).Length
if ($len -ne 0x20000) { throw "Dump size $len != 131072" }
$patSha = (Get-FileHash $patterned -Algorithm SHA256).Hash
$dumpSha = (Get-FileHash $memBin -Algorithm SHA256).Hash
if ($patSha -ne $dumpSha) { throw "Dumped |M| != patterned_M_128KiB.bin" }

# Capture DIGEST_PROBE line
python -c @"
import serial, time, subprocess, re, sys
from pathlib import Path
prog = r'$prog'
out = Path(r'$uartLog')
port = serial.Serial('COM3', 115200, timeout=0.2)
buf, t0 = [], time.time()
subprocess.run([prog, '-c', 'port=SWD', 'mode=UR', '-rst', '-hardRst'], capture_output=True)
time.sleep(0.3)
text = ''
while time.time() - t0 < 25:
    n = port.in_waiting
    if n:
        buf.append(port.read(n).decode('utf-8', errors='replace'))
        text = ''.join(buf)
        if 'DIGEST_PROBE' in text and 'digest=' in text:
            time.sleep(0.2)
            if port.in_waiting:
                buf.append(port.read(port.in_waiting).decode('utf-8', errors='replace'))
            text = ''.join(buf)
            break
    else:
        time.sleep(0.05)
port.close()
out.write_text(text, encoding='utf-8')
m = re.search(r'DIGEST_PROBE mode=(\w+) B=(\d+) challenge=([0-9a-fA-F]{32}) digest=([0-9a-fA-F]{64})', text)
if not m:
    print('FAIL: DIGEST_PROBE line missing')
    print(text[-800:])
    sys.exit(2)
print('ON_TARGET', m.group(1), m.group(2), m.group(3), m.group(4))
Path(r'$OutDir').joinpath('${ShuffleMode}_B1024_ontarget.txt').write_text(
    f'mode={m.group(1)}\nB={m.group(2)}\nchallenge={m.group(3)}\ndigest={m.group(4)}\n', encoding='utf-8')
if m.group(1) != r'$ShuffleMode' or m.group(2) != '1024':
    print('FAIL mode/B mismatch'); sys.exit(3)
if m.group(3).lower() != r'$chal':
    print('FAIL challenge mismatch'); sys.exit(4)
print(m.group(4).lower())
"@
if ($LASTEXITCODE -ne 0) { throw "On-target digest capture failed" }

$onTarget = (Get-Content (Join-Path $OutDir "${ShuffleMode}_B1024_ontarget.txt") | Where-Object { $_ -match '^digest=' }) -replace 'digest=',''
$hostOut = & $hostExe $ShuffleMode $memBin $chal
if ($LASTEXITCODE -ne 0) { throw "Host digest failed" }
$hostOut = $hostOut.Trim().ToLower()
$onTarget = $onTarget.Trim().ToLower()
Write-Host "HOST     $hostOut"
Write-Host "ONTARGET $onTarget"
$memSha = (Get-FileHash $memBin -Algorithm SHA256).Hash.Substring(0, 16)
"mode=$ShuffleMode`nmem_sha16=$memSha`nhost=$hostOut`nontarget=$onTarget`nmatch=$($hostOut -eq $onTarget)" |
  Set-Content (Join-Path $OutDir "${ShuffleMode}_B1024_COMPARE.txt")
if ($hostOut -ne $onTarget) {
  Write-Host "GATE A FAIL: digest mismatch for $ShuffleMode"
  exit 1
}
Write-Host "GATE A PASS: $ShuffleMode B1024 digest match"
exit 0
