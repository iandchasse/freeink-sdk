"""Run the display policy against the production notch curve, including its asserts."""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
LIB = HERE.parents[1]
source = (LIB / "src/BatteryMonitor.cpp").read_text(encoding="utf-8")
start = source.index("constexpr uint16_t LIION_NOTCH_MV[11]")
end = source.index("uint16_t BatteryMonitor::percentageFromMillivolts", start)
with tempfile.TemporaryDirectory(prefix="battery-display-") as temporary:
    build = Path(temporary)
    (build / "ProductionCurve.h").write_text("#include <cstdint>\n" + source[start:end], encoding="utf-8")
    compiler = os.environ.get("CXX", "c++")
    command = [compiler] if Path(compiler).is_file() else shlex.split(compiler)
    for name in ["test_display_filter", "test_charge_estimator"]:
        executable = build / (name + ".exe")
        subprocess.run([*command, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(LIB / "include"), "-I" + str(LIB.parent / "BoardConfig/include"), "-I" + str(build),
                        str(HERE / (name + ".cpp")), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
