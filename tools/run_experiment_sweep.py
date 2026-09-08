#!/usr/bin/env python3
"""Drive repeated ./docker.sh runs across map-transport methods.

Automates the manual loop: set MAP_TRANSPORT in experiment.conf, launch
./docker.sh <world>, let it run, Ctrl+C, wait for docker to clean up, repeat.
Everything else in experiment.conf is left exactly as you set it -- the method
is the only thing this script changes, and the original value is restored on
exit.

What it adds over doing it by hand:

  * Detects the "a robot never started up" failure and repeats that run instead
    of silently banking a broken one. A run counts as valid only once EVERY
    robot has reported at least one "Sending goal to frontier" -- that single
    line proves the whole chain is alive for that robot (gz bridge -> SLAM ->
    nav2 global_costmap -> explorer). The known failure mode this catches is a
    nav2 lifecycle bringup that leaves global_costmap configured but never
    activated, where the explorer just logs "Still waiting for first costmap".

  * Round-robins the methods instead of doing all of one then all of the next,
    so if you stop the sweep early (or it hits the deadline) you still have a
    balanced comparison rather than 10 baseline runs and no vxch. It also keeps
    slow machine drift from landing entirely on one method.

  * Reaches --min-runs for every method first, then tops up toward
    --target-runs. So you bank the 5 you need before spending time on the
    10 you would like.

  * Writes a resumable state file, per-run logs, and a manifest with a
    ready-to-paste generate_comparison_figure.py command listing only the
    valid runs.

Typical use -- 5 good runs of each method, then top up to 10 if time allows:

    tools/run_experiment_sweep.py --world office

Resume after an interruption:

    tools/run_experiment_sweep.py --world office --resume

Check what it would do without launching anything:

    tools/run_experiment_sweep.py --world office --dry-run
"""

import argparse
import errno
import json
import os
import pty
import re
import select
import signal
import subprocess
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CONF = REPO / "experiment.conf"
RUNS_DIR = REPO / "experiment_runs"
SWEEP_DIR = REPO / "logs" / "sweep"

# docker.sh prints this once the image is built and the container is actually
# starting. Everything before it is build time, which must not count against
# the startup timeout or the run duration -- the first run of a session can
# spend many minutes building the image.
LAUNCH_ANCHOR = "Starting benchmark in Docker"

# Proof that one robot's full stack came up. Namespace is absent in
# single-robot runs, hence the optional group.
GOAL_RE = re.compile(r"\[(?:(robot\d+)\.)?lite_frontier_explorer\]:\s*Sending goal to frontier")
# Explicit symptom of the bringup failure we are trying to catch. Not used to
# fail a run on its own (it is throttled and can appear transiently during a
# healthy startup) -- it is surfaced in the log summary to explain a timeout.
COSTMAP_STALL_RE = re.compile(
    r"\[(?:(robot\d+)\.)?lite_frontier_explorer\]:\s*Still waiting for first costmap")


def rel(path):
    """Repo-relative display path, falling back to absolute for paths outside it
    (--state and --docker-cmd may legitimately point anywhere)."""
    try:
        return str(Path(path).relative_to(REPO))
    except ValueError:
        return str(path)


def parse_conf(path):
    """Read KEY=VALUE lines from experiment.conf, ignoring trailing comments."""
    values = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, rest = line.partition("=")
        values[key.strip()] = rest.split("#", 1)[0].strip()
    return values


def set_conf_method(path, method):
    """Rewrite only MAP_TRANSPORT's value, preserving its trailing comment."""
    lines = path.read_text().splitlines(keepends=True)
    found = False
    for i, line in enumerate(lines):
        if not line.lstrip().startswith("MAP_TRANSPORT="):
            continue
        prefix, _, rest = line.partition("=")
        comment = ""
        if "#" in rest:
            value_part, _, comment_part = rest.partition("#")
            # Keep the original column of the comment so the file stays tidy.
            pad = len(value_part) - len(value_part.rstrip())
            comment = " " * max(pad, 1) + "#" + comment_part.rstrip("\n")
        newline = "\n" if line.endswith("\n") else ""
        lines[i] = f"{prefix}={method}{comment}{newline}"
        found = True
        break
    if not found:
        raise SystemExit(f"no MAP_TRANSPORT= line found in {path}")
    path.write_text("".join(lines))


class RunAborted(Exception):
    """Raised when the operator interrupts the driver itself."""


class DockerRun:
    """One ./docker.sh invocation, driven through a pty.

    docker.sh hard-codes `docker run -it`, which needs a tty on stdin -- so the
    child gets one. It also means signals are NOT proxied by the docker CLI
    (--sig-proxy is documented as non-TTY only); the way to interrupt the
    container is to write the interrupt character into the tty, exactly as a
    real Ctrl+C would. killpg is kept as a fallback for docker.sh itself.
    """

    def __init__(self, world, log_path, echo=False, cmd="./docker.sh", robot=None):
        self.world = world
        self.cmd = cmd
        self.robot = robot
        self.log_path = log_path
        self.echo = echo
        self.proc = None
        self.master = None
        self._buf = b""
        self._log = None
        self.launched_at = None
        self.robots_ready = set()
        self.saw_costmap_stall = set()

    def start(self):
        self._log = self.log_path.open("wb")
        env = os.environ.copy()
        if self.robot:
            # docker.sh forwards ROBOT into the container; launch.sh then
            # requires <ROBOT>.urdf to exist and exits immediately if it does
            # not, so a typo here fails the run before anything starts.
            env["ROBOT"] = self.robot
        master, slave = pty.openpty()
        self.master = master
        self.proc = subprocess.Popen(
            [self.cmd, self.world],
            cwd=str(REPO),
            stdin=slave,
            stdout=slave,
            stderr=slave,
            start_new_session=True,   # own process group, so killpg is precise
            close_fds=True,
            env=env,
        )
        os.close(slave)

    def pump(self, timeout=0.5):
        """Read whatever is available; returns False once the pty is closed."""
        try:
            ready, _, _ = select.select([self.master], [], [], timeout)
        except (OSError, ValueError):
            return False
        if not ready:
            return True
        try:
            chunk = os.read(self.master, 65536)
        except OSError as exc:
            if exc.errno == errno.EIO:   # normal: child closed the pty
                return False
            raise
        if not chunk:
            return False

        self._log.write(chunk)
        self._log.flush()
        if self.echo:
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()

        self._buf += chunk
        *lines, self._buf = self._buf.split(b"\n")
        for raw in lines:
            text = raw.decode("utf-8", "replace")
            if self.launched_at is None and LAUNCH_ANCHOR in text:
                self.launched_at = time.monotonic()
            m = GOAL_RE.search(text)
            if m:
                self.robots_ready.add(m.group(1) or "robot1")
            m = COSTMAP_STALL_RE.search(text)
            if m:
                self.saw_costmap_stall.add(m.group(1) or "robot1")
        return True

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self, grace=90):
        """Ctrl+C, then escalate. Returns a short description of what it took."""
        if self.proc is None:
            return "not started"
        how = []

        if self.alive():
            how.append("ctrl-c")
            try:
                os.write(self.master, b"\x03")
            except OSError:
                pass
            if self._wait_quiet(grace):
                return "+".join(how)

        if self.alive():
            how.append("SIGINT")
            self._signal_group(signal.SIGINT)
            if self._wait_quiet(grace):
                return "+".join(how)

        if self.alive():
            how.append("SIGTERM")
            self._signal_group(signal.SIGTERM)
            if self._wait_quiet(30):
                return "+".join(how)

        if self.alive():
            how.append("SIGKILL")
            self._signal_group(signal.SIGKILL)
            self._wait_quiet(15)

        return "+".join(how) or "already exited"

    def _signal_group(self, sig):
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _wait_quiet(self, timeout):
        """Keep draining output (so the container can flush its shutdown) while
        waiting for exit. Not draining risks blocking the child on a full pty."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.alive():
                # Drain anything still buffered.
                for _ in range(50):
                    if not self.pump(timeout=0.05):
                        break
                return True
            self.pump(timeout=0.25)
        return False

    def close(self):
        if self.master is not None:
            try:
                os.close(self.master)
            except OSError:
                pass
            self.master = None
        if self._log is not None:
            self._log.close()
            self._log = None


def force_cleanup_container(name, dry_run=False):
    """Belt-and-braces: docker.sh removes a stale container on its next start,
    but leaving a live gzserver holding the GPU between runs is worth avoiding."""
    if dry_run:
        return
    subprocess.run(["docker", "rm", "-f", name],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def snapshot_runs():
    return {p.name for p in RUNS_DIR.iterdir()} if RUNS_DIR.is_dir() else set()


def new_run_dirs(before):
    return sorted(snapshot_runs() - before)


def do_one_run(args, method, attempt, num_robots, state):
    """Execute a single run. Returns a result dict."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    SWEEP_DIR.mkdir(parents=True, exist_ok=True)
    log_path = SWEEP_DIR / f"{stamp}_{method}_attempt{attempt}.log"

    print(f"\n=== {method}  attempt {attempt}  ({stamp}) ===", flush=True)
    print(f"    log: {rel(log_path)}", flush=True)

    if args.dry_run:
        print("    [dry-run] would set MAP_TRANSPORT and launch ./docker.sh "
              f"{args.world} for {args.duration}s", flush=True)
        return {"method": method, "attempt": attempt, "valid": True,
                "dry_run": True, "log": str(log_path), "run_dirs": []}

    set_conf_method(CONF, method)
    before = snapshot_runs()

    run = DockerRun(args.world, log_path, echo=args.echo, cmd=args.docker_cmd,
                    robot=args.robot)
    result = {"method": method, "attempt": attempt, "started": stamp,
              "log": rel(log_path), "valid": False,
              "reason": None, "run_dirs": [], "robots_ready": []}
    try:
        run.start()

        # --- phase 1: wait for the container to actually start -------------
        build_deadline = time.monotonic() + args.build_timeout
        while run.launched_at is None:
            if not run.alive() and not run.pump(0.2):
                result["reason"] = "docker.sh exited before the container started"
                return _finish(run, result, args, before)
            if time.monotonic() > build_deadline:
                result["reason"] = f"no '{LAUNCH_ANCHOR}' within {args.build_timeout}s"
                return _finish(run, result, args, before)
            run.pump(0.5)
        print("    container started; waiting for all robots to begin exploring...",
              flush=True)

        # --- phase 2: readiness -- every robot must send a frontier goal ----
        ready_deadline = run.launched_at + args.startup_timeout
        while len(run.robots_ready) < num_robots:
            if not run.alive() and not run.pump(0.2):
                result["reason"] = "stack exited during startup"
                return _finish(run, result, args, before)
            if time.monotonic() > ready_deadline:
                missing = num_robots - len(run.robots_ready)
                stalled = sorted(run.saw_costmap_stall)
                result["reason"] = (
                    f"{missing} robot(s) never started exploring within "
                    f"{args.startup_timeout}s (ready: {sorted(run.robots_ready) or 'none'}"
                    + (f"; stuck waiting for costmap: {stalled}" if stalled else "")
                    + ")")
                return _finish(run, result, args, before)
            run.pump(0.5)

        ready_at = time.monotonic()
        print(f"    all {num_robots} robots exploring after "
              f"{ready_at - run.launched_at:.0f}s -- running for {args.duration}s",
              flush=True)

        # --- phase 3: hold for the run duration ----------------------------
        origin = ready_at if args.duration_from == "ready" else run.launched_at
        end = origin + args.duration
        next_note = time.monotonic() + 120
        while time.monotonic() < end:
            if not run.alive() and not run.pump(0.2):
                elapsed = time.monotonic() - origin
                result["reason"] = f"stack exited early after {elapsed:.0f}s"
                return _finish(run, result, args, before)
            run.pump(0.5)
            if time.monotonic() >= next_note:
                print(f"    ... {end - time.monotonic():.0f}s remaining", flush=True)
                next_note += 120

        result["valid"] = True
        result["robots_ready"] = sorted(run.robots_ready)
        print(f"    duration reached -- stopping", flush=True)
        return _finish(run, result, args, before)

    except KeyboardInterrupt:
        print("\n    interrupted by operator -- stopping this run", flush=True)
        result["reason"] = "operator interrupt"
        _finish(run, result, args, before)
        raise RunAborted()


def _finish(run, result, args, before):
    result["stop"] = run.stop(grace=args.stop_grace)
    run.close()
    force_cleanup_container(args.container_name, args.dry_run)
    result["run_dirs"] = new_run_dirs(before)

    # Mark a failed run's bag directory so it is obvious later which output
    # came from a run that should not be analysed. The directory is left in
    # place and NOT renamed -- generate_comparison_figure.py is given bag
    # paths explicitly, so a stray marker file cannot confuse it, whereas
    # renaming could break anything that parses the timestamp_condition_world
    # naming convention.
    if not result["valid"]:
        for name in result["run_dirs"]:
            marker = RUNS_DIR / name / "INVALID_RUN.txt"
            try:
                marker.write_text(
                    "This run was discarded by tools/run_experiment_sweep.py.\n"
                    f"Reason: {result.get('reason')}\n"
                    f"Log: {result.get('log')}\n")
            except OSError:
                pass
    return result


def summarize(state, args):
    good = {m: [r for r in state["results"] if r["method"] == m and r["valid"]]
            for m in args.methods}
    print("\n" + "=" * 68)
    print("sweep summary")
    print("=" * 68)
    for m in args.methods:
        attempts = [r for r in state["results"] if r["method"] == m]
        print(f"  {m:9s} {len(good[m])} valid / {len(attempts)} attempts")
        for r in attempts:
            if not r["valid"]:
                print(f"      discarded: {r.get('reason')}  ({r.get('log')})")
    invalid_dirs = [d for r in state["results"] if not r["valid"] for d in r["run_dirs"]]
    if invalid_dirs:
        print("\n  bag dirs from discarded runs (marked with INVALID_RUN.txt):")
        for d in invalid_dirs:
            print(f"      experiment_runs/{d}")

    manifest = SWEEP_DIR / "valid_runs.json"
    manifest.write_text(json.dumps(
        {m: [d for r in good[m] for d in r["run_dirs"]] for m in args.methods}, indent=2))
    print(f"\n  valid-run manifest: {rel(manifest)}")

    # generate_comparison_figure.py wants robot=bag_dir pairs per condition.
    num_robots = int(parse_conf(CONF).get("NUM_ROBOTS", "1"))
    parts = []
    for m in args.methods:
        for r in good[m]:
            for d in r["run_dirs"]:
                pairs = " ".join(
                    f"robot{i}={RUNS_DIR.name}/{d}/bag" for i in range(1, num_robots + 1))
                parts.append(f"  --{m} {pairs}")
    if parts:
        print("\n  compare with:\n")
        print("    python3 generate_comparison_figure.py \\")
        print(" \\\n".join(parts) + " \\")
        print("      --out comparison.png --table markdown")
    print()


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", default="office", help="world passed to ./docker.sh")
    p.add_argument("--methods", default="baseline,vxch,zstd",
                   help="comma-separated MAP_TRANSPORT values to sweep")
    p.add_argument("--min-runs", type=int, default=5,
                   help="valid runs per method to secure first (default 5)")
    p.add_argument("--target-runs", type=int, default=10,
                   help="valid runs per method to aim for (default 10)")
    p.add_argument("--duration", type=float, default=730.0,
                   help="seconds to let each run go (default 730)")
    p.add_argument("--duration-from", choices=("ready", "launch"), default="ready",
                   help="measure duration from when all robots are exploring "
                        "(default, keeps runs comparable) or from container start")
    p.add_argument("--cooldown", type=float, default=60.0,
                   help="seconds to wait after each run for docker cleanup (default 60)")
    p.add_argument("--startup-timeout", type=float, default=90.0,
                   help="seconds after container start for every robot to begin "
                        "exploring before the run is called invalid (default 90)")
    p.add_argument("--build-timeout", type=float, default=3600.0,
                   help="seconds to allow for image build + container start "
                        "(default 3600; the first run may build the image)")
    p.add_argument("--stop-grace", type=float, default=90.0,
                   help="seconds to wait after Ctrl+C before escalating (default 90)")
    p.add_argument("--max-attempts", type=int, default=3,
                   help="consecutive failed attempts per method before giving up "
                        "on it (default 3)")
    p.add_argument("--max-hours", type=float, default=None,
                   help="stop starting new runs after this many hours")
    p.add_argument("--container-name", default=os.environ.get(
        "BENCHMARK_DOCKER_CONTAINER_NAME", "autonomous-exploration-benchmark"))
    p.add_argument("--state", type=Path, default=SWEEP_DIR / "sweep_state.json")
    p.add_argument("--resume", action="store_true",
                   help="continue from an existing state file instead of starting over")
    p.add_argument("--echo", action="store_true",
                   help="mirror container output to this terminal (always logged to file)")
    p.add_argument("--robot", default=None,
                   help="ROBOT model passed to docker.sh (default: docker.sh's own "
                        "default, mogi_bot). Validated against the URDF list below.")
    p.add_argument("--docker-cmd", default="./docker.sh",
                   help="launcher to invoke (override for testing the driver itself)")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    args.methods = [m.strip() for m in args.methods.split(",") if m.strip()]

    if args.target_runs < args.min_runs:
        args.target_runs = args.min_runs

    if args.robot:
        urdf_dir = (REPO / "simulation" / "Week-7-8-ROS2-Navigation"
                    / "bme_ros2_navigation" / "urdf")
        known = sorted(p.stem for p in urdf_dir.glob("*.urdf")) if urdf_dir.is_dir() else []
        if known and args.robot not in known:
            raise SystemExit(
                f"--robot '{args.robot}' has no URDF in {urdf_dir.relative_to(REPO)}.\n"
                f"launch.sh would exit immediately with 'Unknown robot model'.\n"
                f"Available: {', '.join(known)}")

    conf = parse_conf(CONF)
    num_robots = int(conf.get("NUM_ROBOTS", "1"))
    original_method = conf.get("MAP_TRANSPORT")
    if conf.get("RECORD_METRICS", "false").lower() != "true":
        print("WARNING: RECORD_METRICS is not true in experiment.conf -- runs will "
              "produce no bags to compare.", file=sys.stderr)

    SWEEP_DIR.mkdir(parents=True, exist_ok=True)
    state = {"results": []}
    if args.resume and args.state.exists():
        state = json.loads(args.state.read_text())
        print(f"resuming from {rel(args.state)} "
              f"({len(state['results'])} attempts already recorded)")

    def valid_count(m):
        return sum(1 for r in state["results"] if r["method"] == m and r["valid"])

    def save():
        args.state.write_text(json.dumps(state, indent=2))

    print(f"world={args.world}  methods={args.methods}  robots={num_robots}")
    print(f"duration={args.duration}s (from {args.duration_from})  "
          f"cooldown={args.cooldown}s  min={args.min_runs} target={args.target_runs}")
    est = (args.duration + 120 + args.cooldown) * len(args.methods)
    print(f"rough estimate: {timedelta(seconds=int(est * args.min_runs))} for the "
          f"minimum, {timedelta(seconds=int(est * args.target_runs))} for the target")

    deadline = time.monotonic() + args.max_hours * 3600 if args.max_hours else None
    give_up = set()
    consecutive_failures = {m: 0 for m in args.methods}

    try:
        # Two passes: secure --min-runs everywhere, then top up to --target-runs.
        for goal in (args.min_runs, args.target_runs):
            while True:
                todo = [m for m in args.methods
                        if valid_count(m) < goal and m not in give_up]
                if not todo:
                    break
                # Round-robin: always take the method with the fewest valid runs.
                method = min(todo, key=lambda m: (valid_count(m), args.methods.index(m)))
                if deadline and time.monotonic() > deadline:
                    print("\nmax-hours reached -- not starting another run.")
                    raise RunAborted()

                attempt = sum(1 for r in state["results"] if r["method"] == method) + 1
                result = do_one_run(args, method, attempt, num_robots, state)
                state["results"].append(result)
                save()

                if result["valid"]:
                    consecutive_failures[method] = 0
                    print(f"    VALID  ({valid_count(method)}/{goal} for {method})")
                else:
                    consecutive_failures[method] += 1
                    print(f"    INVALID: {result['reason']}")
                    if consecutive_failures[method] >= args.max_attempts:
                        print(f"    giving up on {method} after "
                              f"{args.max_attempts} consecutive failures")
                        give_up.add(method)

                if not args.dry_run and args.cooldown > 0:
                    print(f"    cooling down {args.cooldown:.0f}s...", flush=True)
                    time.sleep(args.cooldown)
    except (RunAborted, KeyboardInterrupt):
        print("\nsweep stopped early.")
    finally:
        if original_method is not None and not args.dry_run:
            set_conf_method(CONF, original_method)
            print(f"restored MAP_TRANSPORT={original_method} in experiment.conf")
        save()
        summarize(state, args)


if __name__ == "__main__":
    main()
