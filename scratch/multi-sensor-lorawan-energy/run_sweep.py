"""
Run multi-sensor-lorawan-energy with parameter sweeps over runNum.

Each runNum produces an independent RNG seed, giving different random start
times under the same configuration.  Output lands in:

    multi-sensor-lorawan-energy-output/<timestamp>/<runNum>/

Edit the variables below to change simulation parameters.
"""

import subprocess
import sys
from datetime import datetime
from pathlib import Path

NS3_DIR = Path(__file__).resolve().parent.parent.parent
SCRIPT_NAME = "multi-sensor-lorawan-energy"

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
numNodes = 100
# period = 180.0
period = 600.0
stopTime = 57000000
radius = 1000.0

spreadingFactor = 12
initialEnergy = 10
packetSize = 15

# BME280
# measurementDuration = 0.01498 # 14.98 ms
# measurementCurrent = 0.02149 #  21.49 mA
# sensorOverhead = 0.0044376 # 4437.6 uC
# numSample = 5

# SEN55
measurementDuration = 35.57 # s
measurementCurrent = 0.1332 # 133.2 mA
sensorOverhead = 0
numSample = 1 # SEN55 is still using T_avg * I_avg model

# Sweep range
runStart = 1
runEnd = 10


def main():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")

    for runNum in range(runStart, runEnd + 1):
        subfolder = f"{timestamp}/{runNum}"
        cmd = [
            str(NS3_DIR / "ns3"),
            "run",
            SCRIPT_NAME,
            "--",
            f"--runNum={runNum}",
            f"--numNodes={numNodes}",
            f"--period={period}",
            f"--stopTime={stopTime}",
            f"--radius={radius}",
            f"--measurementDuration={measurementDuration}",
            f"--measurementCurrent={measurementCurrent}",
            f"--sensorOverhead={sensorOverhead}",
            f"--numSample={numSample}",
            f"--spreadingFactor={spreadingFactor}",
            f"--initialEnergy={initialEnergy}",
            f"--packetSize={packetSize}",
            f"--subfolder={subfolder}",
        ]
        print(f"[runNum={runNum}] {subfolder}")
        result = subprocess.run(cmd, cwd=str(NS3_DIR))
        if result.returncode != 0:
            print(f"  FAILED (exit code {result.returncode})", file=sys.stderr)


if __name__ == "__main__":
    main()
