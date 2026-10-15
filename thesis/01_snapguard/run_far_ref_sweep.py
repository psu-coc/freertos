#!/usr/bin/env python3
"""Build/flash/capture FAR reference (FAR_REF_NO_IRQ_DISABLE) for B in {128..4096}."""
from __future__ import annotations

import re
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "Secure_nsclib" / "attest_overhead_cfg.h"
LOG_DIR = Path(__file__).resolve().parent / "logs"
SECURE_DBG = ROOT / "STM32CubeIDE" / "Secure" / "Debug"
NS_DBG = ROOT / "STM32CubeIDE" / "NonSecure" / "Debug"
SECURE_ELF = SECURE_DBG / "FreeRTOS_SecureIOToggle_TrustZone_Secure.elf"
NS_ELF = NS_DBG / "FreeRTOS_SecureIOToggle_TrustZone_NonSecure.elf"
PROG = Path(
    r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)
GCC_BIN = Path(
    r"C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin"
)
MAKE_BIN = Path(
    r"C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
)

BS = [128, 256, 512, 1024, 2048, 4096]
PORT = "COM3"
BAUD = 115200


def set_cfg(b: int) -> None:
    text = CFG.read_text(encoding="utf-8")
    text = re.sub(
        r"#define\s+ATTEST_BLOCK_SIZE\s+\d+.*",
        f"#define ATTEST_BLOCK_SIZE  {b}  /* FAR ref sweep */",
        text,
        count=1,
    )
    text = re.sub(
        r"#define\s+FAR_REF_NO_IRQ_DISABLE\s+\d+",
        "#define FAR_REF_NO_IRQ_DISABLE  1",
        text,
    )
    text = re.sub(r"#define\s+USE_SAU_APPROACH\s+\d+", "#define USE_SAU_APPROACH  0", text)
    text = re.sub(r"#define\s+USE_SNAP\s+\d+", "#define USE_SNAP          0", text)
    text = re.sub(r"#define\s+TDIS_RAW_DUMP\s+\d+", "#define TDIS_RAW_DUMP  0", text)
    CFG.write_text(text, encoding="utf-8")
    print(f"cfg: B={b} FAR_REF_NO_IRQ_DISABLE=1 Baseline path", flush=True)


def make_all(cwd: Path) -> None:
    import os

    env = os.environ.copy()
    env["Path"] = str(GCC_BIN) + ";" + str(MAKE_BIN) + ";" + env.get("Path", "")
    r = subprocess.run(
        ["make", "-j8", "all"],
        cwd=str(cwd),
        env=env,
        capture_output=True,
        text=True,
    )
    if r.returncode != 0:
        print(r.stdout)
        print(r.stderr)
        raise SystemExit(f"make failed in {cwd}")
    print(f"built {cwd.name}", flush=True)


def flash() -> None:
    r = subprocess.run(
        [
            str(PROG),
            "-c",
            "port=SWD",
            "mode=UR",
            "-d",
            str(SECURE_ELF),
            "-d",
            str(NS_ELF),
            "-rst",
        ],
        capture_output=True,
        text=True,
    )
    if r.returncode != 0:
        print(r.stdout)
        print(r.stderr)
        raise SystemExit("flash failed")
    print("flash OK", flush=True)


def capture(out: Path, stop_evt: threading.Event, timeout_s: float = 240.0) -> str:
    import serial

    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    ser.reset_input_buffer()
    buf = bytearray()
    deadline = time.time() + timeout_s
    last = time.time()
    print(f"capturing -> {out.name}", flush=True)
    while time.time() < deadline and not stop_evt.is_set():
        chunk = ser.read(8192)
        if chunk:
            buf.extend(chunk)
            last = time.time()
            text = buf.decode("utf-8", errors="replace")
            nrounds = len(re.findall(r"^\.?Round \d+:", text, flags=re.M))
            if ("Done. Press RESET" in text or "SUMMARY FAR-REF" in text) and nrounds >= 10:
                time.sleep(0.5)
                break
        elif time.time() - last > 90 and b"SUMMARY" in buf:
            break
    ser.close()
    out.write_bytes(bytes(buf))
    text = buf.decode("utf-8", errors="replace")
    nrounds = len(re.findall(r"^\.?Round \d+:", text, flags=re.M))
    print(f"  wrote {len(buf)} bytes, Round lines={nrounds}", flush=True)
    return text


def kill_putty() -> None:
    subprocess.run(
        ["powershell", "-Command", "Stop-Process -Name putty -Force -ErrorAction SilentlyContinue"],
        capture_output=True,
    )


def main() -> int:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    kill_putty()
    only = [int(x) for x in sys.argv[1:]] if len(sys.argv) > 1 else BS
    for b in only:
        set_cfg(b)
        make_all(SECURE_DBG)
        make_all(NS_DBG)
        out = LOG_DIR / f"far_ref_B{b}.txt"
        stop = threading.Event()
        holder: dict = {}

        def worker():
            holder["text"] = capture(out, stop)

        t = threading.Thread(target=worker, daemon=True)
        t.start()
        time.sleep(1.0)  # serial open first
        flash()
        t.join(timeout=300)
        stop.set()
        text = holder.get("text", "")
        if "EVAL-ONLY" not in text:
            print("WARNING: EVAL-ONLY banner not found in capture", flush=True)
        nrounds = len(re.findall(r"^\.?Round \d+:", text, flags=re.M))
        if nrounds < 10:
            raise SystemExit(f"incomplete capture for B={b} (rounds={nrounds})")
    print("sweep done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
