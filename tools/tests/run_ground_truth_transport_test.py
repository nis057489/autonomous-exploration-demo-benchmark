"""Run in a Gazebo development environment: python3 this_file.py recorder_path."""
import argparse
import csv
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recorder", type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="truth_transport_") as tmp:
        tmp = Path(tmp)
        source = Path(__file__).with_name("ground_truth_transport")
        subprocess.run(["cmake", "-S", str(source), "-B", str(tmp / "build")], check=True)
        subprocess.run(["cmake", "--build", str(tmp / "build"), "-j2"], check=True)
        env = dict(os.environ, GZ_PARTITION="coverage_test_" + uuid.uuid4().hex, GZ_IP="127.0.0.1")
        output = tmp / "coverage.csv"
        process = subprocess.Popen([str(args.recorder.resolve()), str(output), "1",
                                    "robot1=/test/robot1/scan", "robot2=/test/robot2/scan"], env=env)
        try:
            subprocess.run([str(tmp / "build" / "gt_publisher")], env=env, check=True, timeout=20)
        finally:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise
        assert process.returncode == 0
        assert output.with_suffix(".csv.complete").exists()
        with output.open() as stream:
            rows = list(csv.DictReader(stream))
        assert [int(r["union_cells"]) for r in rows] == [3, 3, 3, 5, 7], rows
        assert [int(r["overlap_cells"]) for r in rows] == [0, 0, 3, 3, 3], rows
        print("PASS: native Gazebo transport, world translation/rotation, repeated observations, "
              "cross-robot deduplication, and clean shutdown")


if __name__ == "__main__":
    main()
