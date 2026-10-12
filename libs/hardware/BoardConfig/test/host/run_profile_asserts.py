#!/usr/bin/env python3
"""BoardConfig.h profile rules: compile the header with each good and each bad flag combination, and say which
expectation failed.

The Silkscreen profiles (the 4.26" board and the 3.68" variant, SILKSCREEN_PANEL_368) are guarded by static_asserts
and #errors in BoardConfig.h, not by anything a test executes. This runs the compiler's syntax check on the header,
against the XteinkDetect host stubs, for the combinations that must build and for the ones that must be refused,
and checks that a refusal is the assert meant for it (its message). A few cases edit a scratch copy of the header
(the profile's default device, a field appended to the probe policy): each proves one assert is live, and says what
it guards. Needs a C++20-capable compiler with GNU extensions: CXX, else the PlatformIO xtensa g++, else g++.

    python3 libs/hardware/BoardConfig/test/host/run_profile_asserts.py [-j N] [-v]
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
INCLUDE = HERE.parents[1] / "include"
STUBS = HERE.parents[2] / "XteinkDetect" / "test" / "host" / "stubs"
HEADER = INCLUDE / "BoardConfig.h"

SILK = ["-DFREEINK_DEVICE_SILKSCREEN=1"]
V368 = SILK + ["-DSILKSCREEN_PANEL_368=1"]
PROBE = "SILKSCREEN_DISPLAY_CONTROLLER"

TU = "#include <BoardConfig.h>\nint main() { return BoardConfig::ACTIVE.displayWidth; }\n"


@dataclass
class Case:
    label: str
    flags: list
    refused: str = ""  # empty: must compile cleanly; otherwise the text the refusal's diagnostics must contain
    edits: list = field(default_factory=list)  # (old, new) replacements, each once, on a scratch copy of the header
    tu: str = ""  # extra lines after the include (a static_assert on the edited header)


CASES = [
    # --- must compile -------------------------------------------------------------------------------------------
    Case("4.26\" board", SILK),
    Case("4.26\" board, SILKSCREEN_PANEL_368=0 written out", SILK + ["-DSILKSCREEN_PANEL_368=0"]),
    Case("4.26\" board, forced ssd1677", SILK + [f"-D{PROBE}=ssd1677"]),
    Case("4.26\" board, forced uc8279", SILK + [f"-D{PROBE}=uc8279"]),
    Case("4.26\" board, alternate touch jumpers", SILK + ["-DSILKSCREEN_TOUCH_ALT_JUMPERS=1"]),
    Case("3.68\" variant", V368),
    Case("3.68\" variant, FREEINK_CAP_TOUCH=0 written out", V368 + ["-DFREEINK_CAP_TOUCH=0"]),
    Case("3.68\" variant, forced uc8279 (a no-op: the controller is fixed in its profile)", V368 + [f"-D{PROBE}=uc8279"]),
    Case("3.68\" variant, forced UC8279 (the upper-case name)", V368 + [f"-D{PROBE}=UC8279"]),
    Case("3.68\" variant, diagnostics", V368 + ["-DFREEINK_DIAGNOSTICS=1"]),
    Case("X3 + X4 (C3)", ["-DFREEINK_DEVICE_X3=1", "-DFREEINK_DEVICE_X4=1"]),
    Case("X3 (C3)", ["-DFREEINK_DEVICE_X3=1"]),
    Case("X4 (C3)", ["-DFREEINK_DEVICE_X4=1"]),
    Case("X4 Pro", ["-DFREEINK_DEVICE_X4PRO=1"]),
    Case("X4 Classic", ["-DFREEINK_DEVICE_X4CLASSIC=1"]),
    Case("de-link", ["-DFREEINK_DEVICE_DELINK=1"]),
    Case("X3 + X4 with the probe policy on (not a Silkscreen build: no rule applies)",
         ["-DFREEINK_DEVICE_X3=1", "-DFREEINK_DEVICE_X4=1", "-DFREEINK_CAP_DISPLAY_PROBE_POLICY=1"]),
    # --- must be refused, by the assert meant for it -------------------------------------------------------------
    Case("3.68\" variant with touch", V368 + ["-DFREEINK_CAP_TOUCH=1"],
         refused="SILKSCREEN_PANEL_368 has no touch hardware"),
    Case("3.68\" variant, forced ssd1677", V368 + [f"-D{PROBE}=ssd1677"],
         refused="SILKSCREEN_DISPLAY_CONTROLLER=ssd1677 is refused for the 3.68\" variant"),
    Case("3.68\" variant, forced uc8253", V368 + [f"-D{PROBE}=uc8253"],
         refused="SILKSCREEN_DISPLAY_CONTROLLER=uc8253 is refused: no Silkscreen build links the UC8253 driver"),
    Case("3.68\" variant, forced uc8179 (not linked, not declared)", V368 + [f"-D{PROBE}=uc8179"],
         refused="uc8179"),
    Case("SILKSCREEN_PANEL_368=2", SILK + ["-DSILKSCREEN_PANEL_368=2"],
         refused="SILKSCREEN_PANEL_368 is 0 (the 4.26\" panel) or 1"),
    Case("SILKSCREEN_PANEL_368=1 without the Silkscreen device", ["-DFREEINK_DEVICE_X4=1", "-DSILKSCREEN_PANEL_368=1"],
         refused="needs -DFREEINK_DEVICE_SILKSCREEN=1"),
    Case("3.68\" variant with the alternate touch jumpers", V368 + ["-DSILKSCREEN_TOUCH_ALT_JUMPERS=1"],
         refused="SILKSCREEN_TOUCH_ALT_JUMPERS is for a J4 touch tail"),
    Case("4.26\" board, forced uc8253", SILK + [f"-D{PROBE}=uc8253"],
         refused="SILKSCREEN_DISPLAY_CONTROLLER=uc8253 is refused: no Silkscreen build links the UC8253 driver"),
    Case("3.68\" variant with another SSD1677 device (de-link)", V368 + ["-DFREEINK_DEVICE_DELINK=1"],
         refused="links the X3's UC8279d driver and no other panel driver"),
    Case("4.26\" board without the touch code", SILK + ["-DFREEINK_CAP_TOUCH=0"],
         refused="Silkscreen needs FREEINK_CAP_TOUCH"),
    Case("4.26\" board without the display probe policy", SILK + ["-DFREEINK_CAP_DISPLAY_PROBE_POLICY=0"],
         refused="Silkscreen needs FREEINK_CAP_DISPLAY_PROBE_POLICY"),
    Case("3.68\" variant without the display probe policy", V368 + ["-DFREEINK_CAP_DISPLAY_PROBE_POLICY=0"],
         refused="SILKSCREEN_PANEL_368 needs FREEINK_CAP_DISPLAY_PROBE_POLICY"),
    Case("Silkscreen without probe identity", SILK + ["-DFREEINK_CAP_PROBE_IDENTITY=0"],
         refused="Silkscreen needs FREEINK_CAP_PROBE_IDENTITY and FREEINK_CAP_POWER_POLICY"),
    Case("Silkscreen without the power policy", SILK + ["-DFREEINK_CAP_POWER_POLICY=0"],
         refused="Silkscreen needs FREEINK_CAP_PROBE_IDENTITY and FREEINK_CAP_POWER_POLICY"),
    Case("Silkscreen without the ADC ladders", SILK + ["-DFREEINK_CAP_ADC_LADDERS=0"],
         refused="Silkscreen needs FREEINK_CAP_ADC_LADDERS"),
    Case("Silkscreen with a C3 device (two MCU families)", SILK + ["-DFREEINK_DEVICE_X3=1"],
         refused="must share one MCU family"),
    # --- an assert is live: the header edited, the rule it guards broken -----------------------------------------
    Case("boot profile: the variant's DEFAULT_DEVICE is the 4.26\" one", V368,
         refused="the Silkscreen 3.68\" build must boot as SILKSCREEN_368",
         edits=[("constexpr BoardProfile DEFAULT_DEVICE = SILKSCREEN_368;",
                 "constexpr BoardProfile DEFAULT_DEVICE = SILKSCREEN;")]),
    Case("boot profile: the 4.26\" board's DEFAULT_DEVICE is the variant", SILK,
         refused="the Silkscreen 4.26\" build must boot as SILKSCREEN",
         edits=[("constexpr BoardProfile DEFAULT_DEVICE = SILKSCREEN;\n#endif\n#elif FREEINK_DEVICE_METALIO_EINK4",
                 "constexpr BoardProfile DEFAULT_DEVICE = SILKSCREEN_368;\n#endif\n#elif FREEINK_DEVICE_METALIO_EINK4")]),
    Case("controller: derive() makes the variant a UC8253", V368,
         refused="Silkscreen 3.68\" is a UC8279d panel and only that",
         edits=[("  p.displayController = DisplayController::UC8279;  // fixed in data",
                 "  p.displayController = DisplayController::UC8253;  // fixed in data")]),
    Case("probe policy: derive() leaves the variant probing (forced off)", V368,
         refused="Silkscreen 3.68\" probes nothing",
         edits=[("  p.displayProbe.forced = true;\n", "  p.displayProbe.forced = false;\n")]),
    Case("probe policy: derive() keeps a verdict cache", V368,
         refused="Silkscreen 3.68\" probes nothing",
         edits=[("  p.displayProbe.cacheAcrossDeepSleep = false;\n", "  p.displayProbe.cacheAcrossDeepSleep = true;\n")]),
    Case("probe policy: derive() changes the UltraChip answer SILKSCREEN takes", V368,
         refused="Silkscreen 3.68\" probe policy must be the 4.26\" board's but for forced and cacheAcrossDeepSleep",
         edits=[("  p.displayProbe.cacheAcrossDeepSleep = false;\n",
                 "  p.displayProbe.cacheAcrossDeepSleep = false;\n"
                 "  p.displayProbe.ultraChipController = DisplayController::UC8179;\n")]),
    Case("probe policy: x4ClassPanel false on SILKSCREEN", V368,
         refused="must keep displayProbe.x4ClassPanel true",
         edits=[("                     .x4ClassPanel = true}};\n\nstatic_assert(SILKSCREEN.displayWidth",
                 "                     .x4ClassPanel = false}};\n\nstatic_assert(SILKSCREEN.displayWidth")]),
    Case("probe policy: a field appended to DisplayProbeConfig and set on SILKSCREEN reaches the variant", V368,
         tu="static_assert(BoardConfig::SILKSCREEN.displayProbe.futureUpstreamPolicy, \"set on SILKSCREEN\");\n"
            "static_assert(BoardConfig::SILKSCREEN_368.displayProbe.futureUpstreamPolicy, \"inherited\");\n",
         edits=[("  bool x4ClassPanel = false;\n};", "  bool x4ClassPanel = false;\n  bool futureUpstreamPolicy = false;\n};"),
                ("                     .x4ClassPanel = true}};\n\nstatic_assert(SILKSCREEN.displayWidth",
                 "                     .x4ClassPanel = true,\n                     .futureUpstreamPolicy = true}};\n\n"
                 "static_assert(SILKSCREEN.displayWidth")]),
]


def find_compiler() -> str:
    if os.environ.get("CXX"):
        return os.environ["CXX"]
    home = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    for package in ("toolchain-xtensa-esp-elf", "toolchain-xtensa-esp32s3"):
        for exe in sorted((home / "packages" / package / "bin").glob("xtensa-esp32s3-elf-g++*")):
            return str(exe)
    return shutil.which("g++") or shutil.which("c++") or sys.exit("run_profile_asserts: no C++ compiler (set CXX)")


def run_case(index: int, case: Case, compiler: str, scratch: Path) -> tuple:
    work = scratch / f"case{index}"
    work.mkdir()
    include = [str(STUBS), str(INCLUDE)]
    if case.edits:
        text = HEADER.read_bytes().decode("utf-8").replace("\r\n", "\n")
        for old, new in case.edits:
            if text.count(old) != 1:
                return (index, False, f"the edit's text is not in BoardConfig.h exactly once ({text.count(old)}): {old[:60]!r}")
            text = text.replace(old, new)
        (work / "BoardConfig.h").write_text(text, encoding="utf-8", newline="\n")
        include.insert(0, str(work))
    source = work / "tu.cpp"
    source.write_text(TU.replace("int main", case.tu + "int main", 1))
    command = [compiler, "-std=gnu++2a", "-fsyntax-only", "-Wall", "-Wextra", *case.flags]
    for directory in include:
        command += ["-I", directory]
    result = subprocess.run(command + [str(source)], capture_output=True, text=True)
    output = result.stdout + result.stderr
    errors = [line for line in output.splitlines() if "error" in line or "static assertion" in line]
    warnings = [line for line in output.splitlines() if "warning:" in line]
    if case.refused:
        if result.returncode == 0:
            return (index, False, f"expected the build to be refused with {case.refused!r}, but it compiled")
        if case.refused not in output:
            return (index, False, f"refused, but not by the assert meant for it (expected {case.refused!r}): "
                    + (errors[0].strip() if errors else output.strip()[:300]))
        return (index, True, "")
    if result.returncode != 0:
        return (index, False, "expected it to compile, but: " + (errors[0].strip() if errors else output.strip()[:300]))
    if warnings:
        return (index, False, "compiled, but with a warning: " + warnings[0].strip())
    return (index, True, "")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("-j", type=int, default=min(4, os.cpu_count() or 1), help="parallel compiles")
    parser.add_argument("-v", action="store_true", help="also list the cases that held")
    args = parser.parse_args()
    compiler = find_compiler()
    print(f"compiler: {compiler}", flush=True)
    failed = 0
    with tempfile.TemporaryDirectory(prefix="boardconfig-asserts-") as directory:
        scratch = Path(directory)
        with ThreadPoolExecutor(max_workers=args.j) as pool:
            futures = [pool.submit(run_case, i, case, compiler, scratch) for i, case in enumerate(CASES)]
            for future in futures:
                index, ok, why = future.result()
                case = CASES[index]
                kind = "refused" if case.refused else "builds"
                if not ok:
                    failed += 1
                    print(f"FAIL  [{kind}] {case.label}\n      {why}")
                elif args.v:
                    print(f"ok    [{kind}] {case.label}")
    print(f"{len(CASES)} cases, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
