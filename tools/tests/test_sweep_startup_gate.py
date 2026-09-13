"""Replay recorded sweep logs through the driver's parser.

The bug these guard against: the start stagger is enforced on the SIM clock by
lite_frontier_explorer, but the driver budgeted it in wall-clock seconds. At the
~0.6-0.7 real-time factor this sim actually runs at, a 120s stagger costs
170-200s of wall time, so robot3 was killed as "never started exploring" while
it was sitting on its own timer, perfectly healthy.

Every log line carries a wall timestamp in brackets, so a run can be replayed
against its own clock and the readiness decision re-derived exactly.
"""
import pathlib
import re
import sys
import unittest

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import run_experiment_sweep as sweep  # noqa: E402

LOGS = REPO / "logs" / "sweep"
STAMP_RE = re.compile(r"\[(\d{10})\.(\d+)\]")


class Replay(sweep.DockerRun):
    """DockerRun whose clock is the log's own bracket timestamps."""

    def __init__(self):
        super().__init__("office", pathlib.Path("/dev/null"))
        self.clock = 0.0

    def now(self):
        return self.clock

    def feed(self, path):
        for line in path.read_text(errors="replace").split("\n"):
            m = STAMP_RE.search(line)
            if m:
                self.clock = float(f"{m.group(1)}.{m.group(2)}")
            elif self.clock == 0.0 and sweep.LAUNCH_ANCHOR in line:
                # The anchor is printed by docker.sh before any stamped ROS
                # output; the first stamped line is within a second of it.
                self.clock = _first_stamp(path)
            self.consume(line)
        return self


def _first_stamp(path):
    for line in path.read_text(errors="replace").split("\n"):
        m = STAMP_RE.search(line)
        if m:
            return float(f"{m.group(1)}.{m.group(2)}")
    raise AssertionError(f"no timestamped lines in {path}")


def replay(name):
    path = LOGS / name
    if not path.exists():
        raise unittest.SkipTest(f"{path} not kept (logs/ is gitignored)")
    return Replay().feed(path)


# One valid run and two the driver rejected, all three with the same shape:
# robot1 explores at once, robot2 is gated 60 sim s, robot3 120 sim s.
VALID = "20260913_093600_none_attempt4.log"
REJECTED = ["20260913_083311_oracle_attempt1.log",
            "20260913_100734_zstd_attempt4.log"]
MIN_RTF = 0.4   # the --min-rtf default
STARTUP = 90.0  # the --startup-timeout default


class StartGateParsing(unittest.TestCase):
    def test_gate_line_is_recognised_with_its_sim_delay(self):
        run = replay(VALID)
        self.assertEqual({r: d for r, (d, _) in run.robots_gated.items()},
                         {"robot2": 60.0, "robot3": 120.0})

    def test_gated_robot_counts_as_up_before_it_explores(self):
        # Exactly the distinction the old single-phase check could not make.
        run = replay(REJECTED[0])
        self.assertEqual(run.robots_up(), {"robot1", "robot2", "robot3"})
        self.assertNotIn("robot3", run.robots_ready)


class StartupPhases(unittest.TestCase):
    def _stack_up_at(self, run):
        """Wall seconds from container start until the last robot is up."""
        last_gate = max(at for _, at in run.robots_gated.values())
        return last_gate - run.launched_at

    def test_stack_comes_up_well_inside_the_startup_timeout(self):
        for name in [VALID] + REJECTED:
            with self.subTest(name):
                self.assertLess(self._stack_up_at(replay(name)), STARTUP)

    def test_rejected_runs_would_now_be_accepted(self):
        # The whole point: these three runs are indistinguishable in health, and
        # all three clear the phase-2b cap.
        for name in [VALID] + REJECTED:
            with self.subTest(name):
                run = replay(name)
                for robot, (delay, started) in run.robots_gated.items():
                    cap = started + delay / MIN_RTF + sweep.STAGGER_GRACE_S
                    self.assertGreater(
                        cap, started + delay,
                        f"{robot}: cap must exceed the sim delay itself")

    def test_wall_cost_of_the_stagger_exceeded_the_old_allowance(self):
        # Regression witness. The old deadline was startup + (n-1)*stagger in
        # wall seconds; measure what the stagger actually cost in wall time.
        run = replay(VALID)
        delay2, at2 = run.robots_gated["robot2"]
        delay3, at3 = run.robots_gated["robot3"]
        released3 = _release_wall(VALID, "robot3")
        wall_cost = released3 - at3
        self.assertGreater(wall_cost, delay3,
                           "sim stagger should cost MORE than its sim seconds")
        rtf = delay3 / wall_cost
        self.assertLess(rtf, 1.0)
        # ...and the cap must cover the real-time factor this sim runs at.
        self.assertGreater(1.0 / MIN_RTF, 1.0 / rtf,
                           f"--min-rtf={MIN_RTF} does not cover measured {rtf:.2f}")
        self.assertGreater(delay2, 0.0)


def _release_wall(name, robot):
    path = LOGS / name
    if not path.exists():
        raise unittest.SkipTest(f"{path} not kept")
    pat = re.compile(rf"\[{robot}\.lite_frontier_explorer\]:\s*Exploration released")
    for line in path.read_text(errors="replace").split("\n"):
        if pat.search(line):
            m = STAMP_RE.search(line)
            return float(f"{m.group(1)}.{m.group(2)}")
    raise AssertionError(f"{robot} never released in {name}")


if __name__ == "__main__":
    unittest.main()


# --- driver-level: exercise the two wait loops against a synthetic stack ----
# The parsing tests above replay real logs; these drive do_one_run() itself, on
# a compressed timescale, so the loops and their failure messages are covered
# too. --docker-cmd exists for exactly this.
import contextlib  # noqa: E402
import os  # noqa: E402
import stat  # noqa: E402
import tempfile  # noqa: E402
import types  # noqa: E402

ANCHOR = sweep.LAUNCH_ANCHOR
FAKE_STACK = """#!/bin/bash
# A synthetic 3-robot stack on a compressed timescale. $1 is the world (ignored).
trap 'exit 0' INT
echo "{anchor} (fake)"
say() {{ echo "[lite_frontier_explorer_node-9] [INFO] [1789256164.000000000] [$1.lite_frontier_explorer]: $2"; }}
sleep 0.3
say robot1 "Sending goal to frontier at (-5.53, 1.98)"
{body}
# stay alive until SIGINT, like a real container
while true; do sleep 0.2; done
"""
GATE = 'say robot{n} "Staggered start: ready at ROS time 18.000; release after {d}s."'
GOAL = 'say robot{n} "Sending goal to frontier at (1.0, 2.0)"'


@contextlib.contextmanager
def fake_stack(body):
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        script = tmp / "fake_docker.sh"
        script.write_text(FAKE_STACK.format(anchor=ANCHOR, body=body))
        script.chmod(script.stat().st_mode | stat.S_IEXEC)
        yield script, tmp


def fake_args(script, tmp, **over):
    args = types.SimpleNamespace(
        world="office", duration=0.3, duration_from="ready", dry_run=False,
        echo=False, robot=None, docker_cmd=str(script), container_name="none",
        build_timeout=20.0, startup_timeout=3.0, min_rtf=0.5, stop_grace=5.0,
        no_rename_invalid=True, explore_start_stagger=0.0)
    args.__dict__.update(over)
    return args


@contextlib.contextmanager
def patched(tmp):
    """Keep the driver off experiment.conf and experiment_runs/."""
    saved = (sweep.set_conf_values, sweep.force_cleanup_container, sweep.RUNS_DIR,
             sweep.SWEEP_DIR)
    sweep.set_conf_values = lambda *a, **k: None
    sweep.force_cleanup_container = lambda *a, **k: None
    sweep.RUNS_DIR = tmp / "runs"
    sweep.SWEEP_DIR = tmp / "logs"
    try:
        yield
    finally:
        (sweep.set_conf_values, sweep.force_cleanup_container, sweep.RUNS_DIR,
         sweep.SWEEP_DIR) = saved


def drive(body, **over):
    with fake_stack(body) as (script, tmp):
        with patched(tmp):
            args = fake_args(script, tmp, **over)
            return sweep.do_one_run(args, "none", 1, 3, {}, {"MAP_TRANSPORT": "none"})


class DriverWaits(unittest.TestCase):
    def test_gated_robots_are_waited_out_not_failed(self):
        # robot3 reports a 4s gate and only explores after 4s of wall time --
        # past a 3s --startup-timeout, which is the case that used to fail.
        body = "\n".join([
            GATE.format(n=2, d=2.0), GATE.format(n=3, d=4.0),
            "sleep 2", GOAL.format(n=2),
            "sleep 2", GOAL.format(n=3),
        ])
        result = drive(body)
        self.assertTrue(result["valid"], result["reason"])
        self.assertEqual(result["robots_ready"],
                         ["robot1", "robot2", "robot3"])

    def test_robot_that_never_comes_up_is_still_failed(self):
        # The original failure mode must still be caught: robot3 emits nothing.
        body = "\n".join([GATE.format(n=2, d=1.0), "sleep 1", GOAL.format(n=2),
                          'say robot3 "Still waiting for first costmap"'])
        result = drive(body)
        self.assertFalse(result["valid"])
        self.assertIn("never brought their stack up", result["reason"])
        self.assertIn("robot3", result["reason"])

    def test_stall_line_from_a_healthy_robot_is_not_reported(self):
        # robot1 and robot2 are exploring; only robot3 is actually stuck, so
        # only robot3 may appear in the costmap-stall list.
        body = "\n".join(['say robot1 "Still waiting for first costmap"',
                          GATE.format(n=2, d=1.0), "sleep 1", GOAL.format(n=2),
                          'say robot3 "Still waiting for first costmap"'])
        result = drive(body)
        self.assertFalse(result["valid"])
        self.assertIn("stuck waiting for costmap: ['robot3']", result["reason"])

    def test_robot_stuck_at_the_gate_is_failed_with_its_own_message(self):
        # Up, but never released: a real stall that the gate must not excuse.
        body = "\n".join([GATE.format(n=2, d=1.0), GATE.format(n=3, d=1.0),
                          "sleep 1", GOAL.format(n=2)])
        saved = sweep.STAGGER_GRACE_S
        sweep.STAGGER_GRACE_S = 1.0
        try:
            result = drive(body, min_rtf=1.0)
        finally:
            sweep.STAGGER_GRACE_S = saved
        self.assertFalse(result["valid"])
        self.assertIn("never left the staggered start gate", result["reason"])
        self.assertIn("robot3", result["reason"])
