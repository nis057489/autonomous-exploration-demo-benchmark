#!/usr/bin/env python3
"""Drive repeated ./docker.sh runs across map-transport methods.

Automates the manual loop: set MAP_TRANSPORT in experiment.conf, launch
./docker.sh <world>, let it run, Ctrl+C, wait for docker to clean up, repeat.
Everything else in experiment.conf is left exactly as you set it, with one
documented exception: the `oracle` arm is the unimpaired control, and the
launch file refuses to start it while BANDWIDTH_KBPS/LOSS_PCT/DELAY_MS/
LINK_PROFILE would throttle it, so those four are forced to their no-op values
for that arm's runs only (and announced when it happens). Every value this
script touches is restored on exit.

What it adds over doing it by hand:

  * Detects the "a robot never started up" failure and repeats that run instead
    of silently banking a broken one. A run counts as valid only once EVERY
    robot has reported at least one "Sending goal to frontier" -- that single
    line proves the whole chain is alive for that robot (gz bridge -> SLAM ->
    nav2 global_costmap -> explorer). The known failure mode this catches is a
    nav2 lifecycle bringup that leaves global_costmap configured but never
    activated, where the explorer just logs "Still waiting for first costmap".
    In a staggered run the wait is split in two, because a robot holding at its
    start gate has already proven that whole chain and is only waiting on a
    timer that runs on the SIM clock -- see phase 2a/2b in run_once().

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
import glob
import json
import os
import pty
import re
import select
import shlex
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
FIGURES_DIR = REPO / "figures"
FIGURE_SCRIPT = REPO / "generate_comparison_figure.py"

# Same shape replay_gui.py reads out of a bag's metadata.yaml to learn which
# robot namespaces it holds. Parsed with a regex rather than a YAML load so
# this keeps working on the host, which has neither rosbag2_py nor (as
# replay_gui's own docstring notes) tkinter.
TOPIC_NAME_RE = re.compile(r"^\s*name:\s*/([A-Za-z0-9_]+)/", re.MULTILINE)

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
# A robot holding at its deliberate start gate. The explorer only reaches this
# point once its costmap, pose and nav2 action server all exist, so this proves
# the same chain GOAL_RE does -- the robot is healthy and merely waiting out
# explore_start_delay_s. The delay it reports is in SIM seconds.
STAGGER_GATE_RE = re.compile(
    r"\[(?:(robot\d+)\.)?lite_frontier_explorer\]:\s*Staggered start: ready at "
    r"ROS time [\d.]+; release after (?P<delay>[\d.]+)s")
# Wall-clock slack added on top of the sim->wall conversion of a start gate, to
# cover the explorer's own tick period and the first plan after release.
STAGGER_GRACE_S = 30.0
# A launch description that raised. This is fatal and INSTANT: ros2 launch has
# already given up, but docker.sh and the container stay alive, so without this
# the run just sits there and is reported as the generic "no robot ever started
# exploring" after the full startup timeout -- with the actual cause (a bad
# parameter combination, printed once, right here) buried in the log. Every
# retry then reproduces it exactly, because nothing about it is flaky.
LAUNCH_ERROR_RE = re.compile(
    r"\[ERROR\]\s*\[launch\]:\s*Caught exception in launch"
    r"(?:\s*\([^)]*\))?:\s*(?P<msg>.*)"
    # The other way launch dies instantly: the ros2 CLI rejects the command
    # line before any launch file is even loaded, so it never reaches the
    # [ERROR] [launch] path above. Same signature to the harness -- nothing
    # starts, the container stays up, and every retry fails identically.
    r"|(?P<msg2>malformed launch argument\b.*)")


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


def set_conf_values(path, values, required=()):
    """Rewrite each given KEY's value in place, preserving trailing comments.

    Only rewrites lines that already exist -- a key absent from the file keeps
    whatever default the launch file has, which is the same thing that would
    happen if this script had never touched it.
    """
    lines = path.read_text().splitlines(keepends=True)
    seen = set()
    for i, line in enumerate(lines):
        key = line.lstrip().partition("=")[0].strip()
        if key not in values or key in seen:
            continue
        prefix, _, rest = line.partition("=")
        comment = ""
        if "#" in rest:
            value_part, _, comment_part = rest.partition("#")
            # Keep the original column of the comment so the file stays tidy.
            pad = len(value_part) - len(value_part.rstrip())
            comment = " " * max(pad, 1) + "#" + comment_part.rstrip("\n")
        newline = "\n" if line.endswith("\n") else ""
        lines[i] = f"{prefix}={values[key]}{comment}{newline}"
        seen.add(key)
    missing = [k for k in required if k not in seen]
    if missing:
        raise SystemExit(f"no {', '.join(missing)} line(s) found in {path}")
    path.write_text("".join(lines))


# The impairment knobs, at the values that mean "no impairment at all".
# `oracle` is the unimpaired control arm, and the launch file REFUSES to start
# it while any of these would throttle it -- correctly, since an oracle run
# that was actually shaped is worse than no oracle run. But that refusal also
# means a sweep which rewrites MAP_TRANSPORT and nothing else can never launch
# this arm: every other arm wants the impairment left exactly as configured,
# and oracle needs it gone. So oracle -- and only oracle -- runs with these
# written into experiment.conf, and every other arm gets the file's original
# values written back before it starts.
UNIMPAIRED = {"BANDWIDTH_KBPS": "0", "LOSS_PCT": "0.0",
              "DELAY_MS": "0", "LINK_PROFILE": "static"}


def conf_original(baseline):
    """Every value this script may rewrite, at the setting the file arrived with.

    Restore goes through this rather than conf_for_method(original_method): if
    the file's own MAP_TRANSPORT is already `oracle`, the latter would hand
    back the forced-unimpaired values and "restoring" would quietly overwrite
    the user's real BANDWIDTH_KBPS with 0.
    """
    return {k: baseline[k] for k in ("MAP_TRANSPORT", *UNIMPAIRED)
            if k in baseline}


def conf_for_method(method, baseline):
    """What experiment.conf must say to run `method`, given its original values."""
    values = {"MAP_TRANSPORT": method}
    for key, unimpaired in UNIMPAIRED.items():
        if key in baseline:
            values[key] = unimpaired if method == "oracle" else baseline[key]
    return values


def map_message_counts(bag_dir):
    """{topic: message_count} for /<robot>/map topics, straight from
    metadata.yaml. A run whose /map topics are all empty recorded a stack that
    never produced a map -- generate_comparison_figure.py warns about exactly
    this ("no /map or /nav_map messages ... contributes 0 bytes") and silently
    drags a condition's mean toward zero if it is left in the set."""
    md = Path(bag_dir) / "metadata.yaml"
    if not md.is_file():
        return None
    text = md.read_text()
    counts = {}
    # topics_with_message_count entries pair a name: line with a later
    # message_count: line; pair them up in document order.
    entries = re.findall(r"name:\s*(/\S+)|message_count:\s*(\d+)", text)
    pending = None
    for name, count in entries:
        if name:
            pending = name
        elif pending is not None:
            counts[pending] = int(count)
            pending = None
    return {t: c for t, c in counts.items() if re.fullmatch(r"/robot\d+/map", t)}


def prune_invalid_runs(apply_rename, expected_robots=None):
    """Report (and optionally delete) run dirs that did not record a usable run.

    Catches both total failures (nothing recorded) and the partial failure this
    sweep exists to guard against -- one robot never starting, which still
    leaves the other robots' maps in the bag and so looks superficially fine.
    """
    if not RUNS_DIR.is_dir():
        print("no experiment_runs/ directory")
        return []
    dead = []
    print(f"{'run dir':42s} {'size':>8s}  map messages")
    for path in sorted(RUNS_DIR.iterdir()):
        if not path.is_dir() or not RUN_DIR_RE.match(path.name):
            continue
        bag = path / "bag"
        counts = map_message_counts(bag) if bag.is_dir() else None
        size = sum(f.stat().st_size for f in path.rglob("*") if f.is_file())
        detail = ("" if not counts else
                  ", ".join(f"{t.split('/')[1]}={c}" for t, c in sorted(counts.items())))
        if counts is None:
            verdict, bad = "no bag/metadata", True
        elif not counts:
            verdict, bad = "no /robotN/map topics", True
        elif all(c == 0 for c in counts.values()):
            verdict, bad = f"all {len(counts)} map topics empty", True
        elif any(c == 0 for c in counts.values()):
            silent = [t.split("/")[1] for t, c in sorted(counts.items()) if c == 0]
            verdict, bad = f"{detail}  ({', '.join(silent)} never mapped)", True
        elif expected_robots and len(counts) < expected_robots:
            missing = expected_robots - len(counts)
            verdict, bad = f"{detail}  ({missing} robot(s) absent entirely)", True
        else:
            verdict, bad = detail, False
        print(f"  {path.name:40s} {size/1e6:7.1f}M  {verdict}"
              + ("   <-- DEAD" if bad else ""))
        if bad:
            dead.append(path)

    if not dead:
        print("\nnothing to flag -- every run recorded map data")
        return []
    total = sum(sum(f.stat().st_size for f in p.rglob('*') if f.is_file()) for p in dead)
    print(f"\n{len(dead)} dead run(s), {total/1e6:.1f} MB")
    if not apply_rename:
        print("re-run with --prune-invalid --yes to prefix them with "
              f"'{INVALID_PREFIX}' (nothing is ever deleted)")
        return dead
    for path in dead:
        renamed = _rename_invalid(path, path.name)
        if renamed:
            print(f"  renamed -> experiment_runs/{renamed}")
    return dead


def in_container():
    """Same probe replay_gui.py uses to decide whether it already has ROS."""
    return Path("/run/.containerenv").exists()


def ros_cmd(py_args, distrobox_name):
    """Wrap a python invocation so rosbag2_py/rclpy are importable.

    Mirrors replay_gui.py's own dispatch: /usr/bin/python3 explicitly (a
    PATH-shadowing venv in the login shell would otherwise hide the
    interpreter that actually has rosbag2_py), and a distrobox login shell
    when we are on the host, because the ROS setup is sourced from .bashrc.d.
    """
    cmd = ["/usr/bin/python3"] + [str(a) for a in py_args]
    if in_container():
        return cmd
    return ["distrobox", "enter", distrobox_name, "--", "bash", "-lc",
            " ".join(shlex.quote(p) for p in cmd)]


def robots_in_bag(bag_dir, fallback_count):
    """Robot namespaces recorded in a bag, from metadata.yaml.

    Simulator bags hold every robot's topics in one file, so the figure script
    -- which wants one robot=bag_dir pair per robot -- needs all of them
    pointing at the same bag. Falls back to robot1..N if metadata.yaml is
    missing, which happens when a recorder is killed before finalising;
    replay_gui.py handles that case by reindexing a scratch copy, so use it
    if the fallback looks wrong.
    """
    md = Path(bag_dir) / "metadata.yaml"
    if md.is_file():
        names = sorted(set(TOPIC_NAME_RE.findall(md.read_text())))
        robots = [n for n in names if re.fullmatch(r"robot\d+", n)]
        if robots:
            return robots
    return [f"robot{i}" for i in range(1, fallback_count + 1)]


def export_summaries(result, args, num_robots):
    """Cache a valid run's bag-derived series so figures survive bag deletion."""
    made = []
    for run_dir in result["run_dirs"]:
        bag = RUNS_DIR / run_dir / "bag"
        if not bag.is_dir():
            continue
        robots = robots_in_bag(bag, num_robots)
        out = Path(args.summary_dir) / f"{run_dir}.npz"
        py = [REPO / "tools" / "export_run_summary.py",
              "--condition", result["method"], "--out", out, "--bag"]
        py += [f"{r}={bag}" for r in robots]
        if args.max_duration is not None:
            py += ["--max-duration", args.max_duration]
        print(f"    caching summary -> {rel(out)}", flush=True)
        proc = subprocess.run(ros_cmd(py, args.distrobox), cwd=str(REPO))
        if proc.returncode == 0:
            made.append(rel(out))
        else:
            print(f"    WARNING: summary export failed (exit {proc.returncode}); "
                  "the bag is still on disk, so this can be redone later",
                  file=sys.stderr)
    result["summaries"] = made
    return made


def generate_figure(state, args, num_robots):
    """Build the comparison figure from this sweep's valid runs.

    Same invocation replay_gui.py assembles -- one --<condition> occurrence per
    run, each listing every robot=bag_dir -- but the runs are chosen from the
    sweep's own validity record instead of being picked by hand in the GUI.
    """
    by_method = {}
    for r in state["results"]:
        if r["valid"] and not r.get("dry_run"):
            by_method.setdefault(r["method"], []).extend(r["run_dirs"])
    by_method = {m: dirs for m, dirs in by_method.items() if dirs}

    if len(by_method) < 2:
        print("\n  skipping --figure: generate_comparison_figure.py needs at least "
              f"2 conditions with valid runs (have {sorted(by_method) or 'none'}).")
        return None

    py = [FIGURE_SCRIPT]
    for method, dirs in by_method.items():
        for run_dir in dirs:
            bag = RUNS_DIR / run_dir / "bag"
            robots = robots_in_bag(bag, num_robots)
            py += [f"--{method}"] + [f"{r}={bag}" for r in robots]

    out = args.figure_out or (
        FIGURES_DIR / f"sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    py += ["--out", out]
    if args.max_duration is not None:
        py += ["--max-duration", args.max_duration]
    if args.separate_figures:
        py += ["--separate-figures"]
    if args.table:
        py += ["--table", args.table]
    if args.table_file:
        py += ["--table-file", args.table_file]

    counts = ", ".join(f"{m}x{len(d)}" for m, d in by_method.items())
    print(f"\n  generating figure from valid runs ({counts})...", flush=True)
    proc = subprocess.run(ros_cmd(py, args.distrobox), cwd=str(REPO))
    if proc.returncode != 0:
        print(f"  figure generation failed (exit {proc.returncode})", file=sys.stderr)
        return None
    print(f"  figure: {rel(out)}")
    return out


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
        # robot -> (sim-seconds of gate delay, time.monotonic() when it started)
        self.robots_gated = {}
        self.saw_costmap_stall = set()
        self.launch_error = None

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
            self.consume(raw.decode("utf-8", "replace"))
        return True

    def now(self):
        """Indirection so a recorded log can be replayed against its own
        timestamps instead of this process's clock (see tools/tests)."""
        return time.monotonic()

    def consume(self, text):
        """Update run state from one line of stack output."""
        if self.launched_at is None and LAUNCH_ANCHOR in text:
            self.launched_at = self.now()
        m = GOAL_RE.search(text)
        if m:
            self.robots_ready.add(m.group(1) or "robot1")
        m = STAGGER_GATE_RE.search(text)
        if m:
            # setdefault, not assignment: _ready_since is latched once per
            # node, so a second line would mean a relaunched node rather
            # than a restarted timer.
            self.robots_gated.setdefault(
                m.group(1) or "robot1",
                (float(m.group("delay")), self.now()))
        m = COSTMAP_STALL_RE.search(text)
        if m:
            self.saw_costmap_stall.add(m.group(1) or "robot1")
        m = LAUNCH_ERROR_RE.search(text)
        if m and self.launch_error is None:
            self.launch_error = (m.group("msg") or m.group("msg2")).strip()

    def robots_up(self):
        """Robots whose full stack is proven alive: exploring, or holding at
        their start gate (which requires the same costmap/pose/nav2 chain)."""
        return self.robots_ready | set(self.robots_gated)

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


def do_one_run(args, method, attempt, num_robots, state, baseline_conf):
    """Execute a single run. Returns a result dict."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    SWEEP_DIR.mkdir(parents=True, exist_ok=True)
    log_path = SWEEP_DIR / f"{stamp}_{method}_attempt{attempt}.log"

    print(f"\n=== {method}  attempt {attempt}  ({stamp}) ===", flush=True)
    print(f"    log: {rel(log_path)}", flush=True)

    conf_values = conf_for_method(method, baseline_conf)

    if args.dry_run:
        print("    [dry-run] would set "
              + ", ".join(f"{k}={v}" for k, v in conf_values.items())
              + f" and launch ./docker.sh {args.world} for {args.duration}s",
              flush=True)
        return {"method": method, "attempt": attempt, "valid": True,
                "dry_run": True, "log": str(log_path), "run_dirs": []}

    set_conf_values(CONF, conf_values, required=("MAP_TRANSPORT",))
    if method == "oracle":
        # Say it out loud in the sweep's own output: this arm did not run at
        # the impairment the rest of the sweep is configured for, by design.
        print("    oracle: running unimpaired ("
              + ", ".join(f"{k}={UNIMPAIRED[k]}" for k in UNIMPAIRED
                          if k in baseline_conf) + ")", flush=True)
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
            if run.launch_error:
                result["reason"] = f"launch failed: {run.launch_error}"
                result["fatal"] = True
                return _finish(run, result, args, before)
            if time.monotonic() > build_deadline:
                result["reason"] = f"no '{LAUNCH_ANCHOR}' within {args.build_timeout}s"
                return _finish(run, result, args, before)
            run.pump(0.5)
        print("    container started; waiting for all robots to begin exploring...",
              flush=True)

        # --- phase 2a: every robot's stack must come up ---------------------
        # Proven by a frontier goal, or -- in a staggered run -- by the robot
        # reporting that it is holding at its start gate, which requires the
        # same costmap/pose/nav2 chain. The deliberate stagger is deliberately
        # NOT charged against this timeout; phase 2b waits that out separately.
        ready_deadline = run.launched_at + args.startup_timeout
        while len(run.robots_up()) < num_robots:
            if not run.alive() and not run.pump(0.2):
                result["reason"] = "stack exited during startup"
                return _finish(run, result, args, before)
            # Checked before the deadline: ros2 launch has already aborted, so
            # waiting out the remaining startup timeout only delays a failure
            # that has already happened and replaces its cause with a symptom.
            if run.launch_error:
                result["reason"] = f"launch failed: {run.launch_error}"
                result["fatal"] = True
                return _finish(run, result, args, before)
            if time.monotonic() > ready_deadline:
                up = run.robots_up()
                # Only robots that never came up: the stall line is throttled
                # and appears transiently during a healthy startup, so naming a
                # robot that is already exploring sends the diagnosis after the
                # wrong failure.
                stalled = sorted(run.saw_costmap_stall - up)
                result["reason"] = (
                    f"{num_robots - len(up)} robot(s) never brought their stack "
                    f"up within {args.startup_timeout:.0f}s "
                    f"(up: {sorted(up) or 'none'}"
                    + (f"; stuck waiting for costmap: {stalled}" if stalled else "")
                    + ")")
                return _finish(run, result, args, before)
            run.pump(0.5)

        # --- phase 2b: let gated robots wait out their own stagger ----------
        # explore_start_delay_s is enforced on the SIM clock, so it cannot be
        # budgeted in wall-clock seconds: at a real-time factor of 0.6 a 120s
        # stagger costs 200s of wall time, and charging it as 120s fails robots
        # for doing exactly what they were configured to do. --min-rtf is the
        # slowest sim we still call healthy. Being generous costs nothing: this
        # loop exits the moment the last robot sends its first goal.
        if len(run.robots_ready) < num_robots:
            holding = {r: v for r, v in run.robots_gated.items()
                       if r not in run.robots_ready}
            gate_deadline = max(started + delay / args.min_rtf + STAGGER_GRACE_S
                                for delay, started in holding.values())
            print(f"    stack up on all {num_robots} robots; "
                  f"{', '.join(sorted(holding))} holding at the start gate "
                  f"(up to {gate_deadline - time.monotonic():.0f}s more)",
                  flush=True)
            while len(run.robots_ready) < num_robots:
                if not run.alive() and not run.pump(0.2):
                    result["reason"] = "stack exited during staggered start"
                    return _finish(run, result, args, before)
                if run.launch_error:
                    result["reason"] = f"launch failed: {run.launch_error}"
                    result["fatal"] = True
                    return _finish(run, result, args, before)
                if time.monotonic() > gate_deadline:
                    still = sorted(set(run.robots_gated) - run.robots_ready)
                    result["reason"] = (
                        f"{num_robots - len(run.robots_ready)} robot(s) came up "
                        f"but never left the staggered start gate: {still} "
                        f"(sim-time stagger allowed down to rtf "
                        f"{args.min_rtf:g})")
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

    # A discarded run's bag is not analysable, but it is still evidence about
    # why the run failed -- so it is never deleted, only renamed with an
    # "invalid_" prefix. That declutters experiment_runs/ (and drops the run
    # out of replay_gui.py's picker, whose RUN_DIR_RE only matches the
    # timestamp_condition_world convention) while staying fully reversible.
    #
    # The rename is deliberately narrow: only directories this attempt created
    # (diffed against a snapshot taken immediately before launch), whose names
    # match that convention, sitting directly under experiment_runs/.
    if not result["valid"]:
        result["renamed_dirs"], result["kept_dirs"] = [], []
        for name in result["run_dirs"]:
            target = RUNS_DIR / name
            _mark_invalid(target, result)
            if args.no_rename_invalid:
                result["kept_dirs"].append(name)
                continue
            renamed = _rename_invalid(target, name)
            if renamed:
                result["renamed_dirs"].append(renamed)
                print(f"    marked invalid run dir -> experiment_runs/{renamed}")
            else:
                result["kept_dirs"].append(name)
    return result


# timestamp_condition_world, the naming convention launch.sh uses for
# RECORD_METRICS run dirs (and what replay_gui.py parses).
RUN_DIR_RE = re.compile(r"^\d{8}_\d{6}_[a-z0-9]+_\w+$")
INVALID_PREFIX = "invalid_"


def _rename_invalid(target, name):
    """Prefix a run directory with invalid_. Returns the new name, or None.

    Refuses anything that is not a plain run directory sitting directly under
    experiment_runs/, and never overwrites an existing path.
    """
    if not RUN_DIR_RE.match(name):
        print(f"    not renaming unexpected path {target}", file=sys.stderr)
        return None
    try:
        resolved = target.resolve()
        if (resolved.parent != RUNS_DIR.resolve() or not resolved.is_dir()
                or target.is_symlink()):
            print(f"    not renaming unexpected path {target}", file=sys.stderr)
            return None
    except OSError:
        return None

    new_name = INVALID_PREFIX + name
    destination = RUNS_DIR / new_name
    if destination.exists():
        print(f"    {new_name} already exists -- leaving {name} as-is",
              file=sys.stderr)
        return None
    try:
        target.rename(destination)
    except OSError as exc:
        print(f"    could not rename {target}: {exc}", file=sys.stderr)
        return None
    return new_name


def _mark_invalid(target, result):
    """Fallback when a bad run's directory is kept: leave a note saying why."""
    try:
        (target / "INVALID_RUN.txt").write_text(
            "This run was discarded by tools/run_experiment_sweep.py.\n"
            f"Reason: {result.get('reason')}\n"
            f"Log: {result.get('log')}\n")
    except OSError:
        pass


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
    renamed = [d for r in state["results"] if not r["valid"]
               for d in r.get("renamed_dirs", [])]
    kept = [d for r in state["results"] if not r["valid"]
            for d in r.get("kept_dirs", [])]
    if renamed:
        print(f"\n  {len(renamed)} discarded run dir(s) renamed with the "
              f"'{INVALID_PREFIX}' prefix:")
        for d in renamed:
            print(f"      experiment_runs/{d}")
    if kept:
        print("\n  bag dirs from discarded runs, kept and marked with INVALID_RUN.txt:")
        for d in kept:
            print(f"      experiment_runs/{d}")

    manifest = SWEEP_DIR / "valid_runs.json"
    manifest.parent.mkdir(parents=True, exist_ok=True)
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
    p.add_argument("--explore-start-stagger", type=float, default=None,
                   help="Seconds of deliberate per-robot exploration delay, used "
                        "only for the runtime estimate printed at startup. "
                        "Defaults to EXPLORE_START_STAGGER_S from "
                        "experiment.conf. The readiness deadline does not use "
                        "it -- each robot reports its own delay as it starts "
                        "holding, so it cannot drift out of sync with the conf.")
    p.add_argument("--startup-timeout", type=float, default=90.0,
                   help="seconds after container start for every robot to bring "
                        "its stack up -- exploring, or holding at its start "
                        "gate -- before the run is called invalid (default 90). "
                        "A deliberate stagger is not charged against this; see "
                        "--min-rtf")
    p.add_argument("--min-rtf", type=float, default=0.4,
                   help="slowest sim real-time factor still considered healthy "
                        "(default 0.4). Converts a robot's sim-time start "
                        "stagger into a wall-clock cap, so a 120s stagger is "
                        "allowed up to 300s of wall time. Raise it only to fail "
                        "slow hosts sooner: a high floor invalidates good runs.")
    p.add_argument("--build-timeout", type=float, default=3600.0,
                   help="seconds to allow for image build + container start "
                        "(default 3600; the first run may build the image)")
    p.add_argument("--stop-grace", type=float, default=90.0,
                   help="seconds to wait after Ctrl+C before escalating (default 90)")
    p.add_argument("--max-attempts", type=int, default=10,
                   help="consecutive failed attempts per method before giving up "
                        "on it (default 10)")
    p.add_argument("--max-hours", type=float, default=None,
                   help="stop starting new runs after this many hours")
    p.add_argument("--container-name", default=os.environ.get(
        "BENCHMARK_DOCKER_CONTAINER_NAME", "autonomous-exploration-benchmark"))
    p.add_argument("--state", type=Path, default=SWEEP_DIR / "sweep_state.json")
    p.add_argument("--resume", action="store_true",
                   help="continue from an existing state file instead of starting over")
    p.add_argument("--echo", action="store_true",
                   help="mirror container output to this terminal (always logged to file)")
    p.add_argument("--prune-invalid", action="store_true",
                   help="scan experiment_runs/ for runs that recorded no map "
                        "data and report them (add --yes to prefix them with "
                        "'invalid_'); never deletes anything")
    p.add_argument("--yes", action="store_true",
                   help="with --prune-invalid, apply the rename instead of "
                        "just reporting")
    p.add_argument("--no-rename-invalid", action="store_true",
                   help="leave a discarded run's directory name alone (it is "
                        "still marked with INVALID_RUN.txt)")
    p.add_argument("--summary", action="store_true",
                   help="after each valid run, cache its bag-derived series to "
                        "--summary-dir via tools/export_run_summary.py, so the "
                        "figures can be rebuilt after the bags are deleted "
                        "(~0.3 MB per run vs ~50 MB of bag)")
    p.add_argument("--summary-dir", type=Path, default=REPO / "summaries")
    p.add_argument("--figure", action="store_true",
                   help="generate the comparison figure from this sweep's valid "
                        "runs when it finishes (the same generate_comparison_"
                        "figure.py call replay_gui.py builds, but with the runs "
                        "chosen from the sweep's own validity record)")
    p.add_argument("--figure-out", type=Path, default=None,
                   help="figure path (default figures/sweep_<timestamp>.png)")
    p.add_argument("--table", nargs="?", const="markdown",
                   choices=("text", "markdown", "csv"),
                   help="also print a per-condition summary table (forwarded)")
    p.add_argument("--table-file", type=Path, default=None)
    p.add_argument("--separate-figures", action="store_true")
    p.add_argument("--max-duration", type=float, default=None,
                   help="clip each bag to this many seconds for the summary/figure. "
                        "Note bags start recording at container launch while the run "
                        "window starts once every robot is exploring, so this is not "
                        "the same as --duration.")
    p.add_argument("--distrobox", default="jazzy_env",
                   help="distrobox holding rosbag2_py/rclpy (default jazzy_env)")
    p.add_argument("--figure-only", action="store_true",
                   help="skip the runs; just build summaries/figure from --state")
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

    # A zero or negative floor turns the sim->wall conversion of a start gate
    # into a ZeroDivisionError or a deadline in the past, both of which would
    # surface as a mystery invalid run.
    if not (0.0 < args.min_rtf <= 1.0):
        raise SystemExit("--min-rtf must be in (0, 1]: it is the slowest sim "
                         "real-time factor still treated as healthy.")

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
    baseline_conf = dict(conf)
    if args.explore_start_stagger is None:
        args.explore_start_stagger = float(
            conf.get("EXPLORE_START_STAGGER_S", 0) or 0)
    num_robots = int(conf.get("NUM_ROBOTS", "1"))
    original_method = conf.get("MAP_TRANSPORT")
    # The one oracle conflict this script will NOT fix for you. Zeroing the
    # bandwidth/loss/delay knobs is just "no impairment", which is what oracle
    # means. Flipping IMPAIRMENT_MODE from tc to sim is not: under tc every
    # other arm's traffic crosses a veth pair between network namespaces, and
    # that path costs something whether or not netem is shaping it. An oracle
    # that skipped it would differ from the other arms by more than the
    # impairment, which is exactly the confound the arm exists to rule out.
    if "oracle" in args.methods and conf.get("IMPAIRMENT_MODE") == "tc":
        raise SystemExit(
            "IMPAIRMENT_MODE=tc cannot be swept together with the oracle arm: "
            "an unimpaired oracle would also skip the netns/veth path the other "
            "arms run through, so it would not be comparable to them.\n"
            "Either drop oracle from --methods, or run the sweep with "
            "IMPAIRMENT_MODE=sim so every arm shares one impairment mechanism.")
    if conf.get("RECORD_METRICS", "false").lower() != "true":
        print("WARNING: RECORD_METRICS is not true in experiment.conf -- runs will "
              "produce no bags to compare.", file=sys.stderr)

    SWEEP_DIR.mkdir(parents=True, exist_ok=True)
    state = {"results": []}
    # --figure-only exists to work on an earlier sweep's runs, so it implies
    # loading that sweep's state; requiring --resume alongside it just produces
    # a confusing "0 valid runs" report.
    if (args.resume or args.figure_only) and args.state.exists():
        state = json.loads(args.state.read_text())
        print(f"resuming from {rel(args.state)} "
              f"({len(state['results'])} attempts already recorded)")

    def valid_count(m):
        return sum(1 for r in state["results"] if r["method"] == m and r["valid"])

    def save():
        # logs/ is gitignored and routinely deleted, and --state may point
        # anywhere, so never assume the directory is already there -- this
        # ran in a finally block and turned an unrelated error into a
        # confusing FileNotFoundError.
        args.state.parent.mkdir(parents=True, exist_ok=True)
        args.state.write_text(json.dumps(state, indent=2))

    print(f"world={args.world}  methods={args.methods}  robots={num_robots}")
    print(f"duration={args.duration}s (from {args.duration_from})  "
          f"cooldown={args.cooldown}s  min={args.min_runs} target={args.target_runs}")
    # Startup allowance mirrors run_once: bringup, then the last robot's
    # sim-time stagger converted to wall time at the --min-rtf floor.
    startup_est = 120 + (num_robots - 1) * args.explore_start_stagger / args.min_rtf
    est = (args.duration + startup_est + args.cooldown) * len(args.methods)
    print(f"rough estimate: {timedelta(seconds=int(est * args.min_runs))} for the "
          f"minimum, {timedelta(seconds=int(est * args.target_runs))} for the target")

    if args.prune_invalid:
        prune_invalid_runs(apply_rename=args.yes, expected_robots=num_robots)
        return

    if args.figure_only:
        if args.summary:
            for r in state["results"]:
                if r["valid"] and not r.get("dry_run"):
                    export_summaries(r, args, num_robots)
            save()
        summarize(state, args)
        generate_figure(state, args, num_robots)
        return

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
                result = do_one_run(args, method, attempt, num_robots, state,
                                    baseline_conf)
                state["results"].append(result)
                save()

                if result["valid"]:
                    consecutive_failures[method] = 0
                    print(f"    VALID  ({valid_count(method)}/{goal} for {method})")
                    if args.summary and not args.dry_run:
                        # Cache now rather than at the end: an interrupted sweep
                        # still leaves usable data for the runs it completed.
                        export_summaries(result, args, num_robots)
                        save()
                else:
                    consecutive_failures[method] += 1
                    print(f"    INVALID: {result['reason']}")
                    if result.get("fatal"):
                        print(f"    giving up on {method}: launch rejected the "
                              "configuration, so every retry fails identically")
                        give_up.add(method)
                    elif consecutive_failures[method] >= args.max_attempts:
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
            set_conf_values(CONF, conf_original(baseline_conf))
            print(f"restored MAP_TRANSPORT={original_method} (and the impairment "
                  "settings) in experiment.conf")
        save()
        summarize(state, args)
        if args.figure and not args.dry_run:
            generate_figure(state, args, num_robots)


if __name__ == "__main__":
    main()
