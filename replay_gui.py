#!/usr/bin/env python3
"""GUI for browsing experiment_runs/ and launching replay_compare.sh on a
specific pair of (older or latest) baseline/vxch runs, instead of always
comparing the most recent ones.

Run dirs look like experiment_runs/<robot>/<YYYYMMDD_HHMMSS>_<condition>_<robot>,
one per robot per experiment. Robots are started together, so their
timestamps for the "same" experiment land within a few seconds of each
other; this groups runs across robots into "sessions" by clustering nearby
timestamps within a condition.

replay_compare.sh needs docker, which isn't available inside the jazzy_env
distrobox this GUI runs in (tkinter isn't installed on the host Python). So
when running inside a container, the compare script is launched on the host
via `distrobox-host-exec`.
"""
import os
import re
import shlex
import subprocess
import sys
import threading
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(PROJECT_ROOT, "experiment_runs")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "figures")
REPLAY_SCRIPT = os.path.join(PROJECT_ROOT, "replay_compare.sh")
FIGURE_SCRIPT = os.path.join(PROJECT_ROOT, "generate_comparison_figure.py")

RUN_DIR_RE = re.compile(r"^(\d{8}_\d{6})_(baseline|vxch|zstd)_(\w+)$")
SESSION_GAP_SECONDS = 90  # runs across robots within this gap = one session
CONDITIONS = ("baseline", "vxch", "zstd")
CONDITION_LABELS = {"baseline": "Baseline run", "vxch": "Wavestream run", "zstd": "Zstd run"}


def in_container():
    return os.path.exists("/run/.containerenv")


def run_bag_dir(robot, run_dir):
    """Path to a run's bag/ dir. Simulator runs are flat
    (experiment_runs/<run_dir>/bag); real-hardware runs nest under a robot
    dir (experiment_runs/<robot>/<run_dir>/bag). replay_compare.sh resolves
    overrides the same way -- see its RUN_DIR_RE-equivalent check."""
    flat = os.path.join(RUNS_DIR, run_dir)
    if os.path.isdir(flat) and RUN_DIR_RE.match(run_dir):
        return os.path.join(flat, "bag")
    return os.path.join(RUNS_DIR, robot, run_dir, "bag")


TOPIC_NAME_RE = re.compile(r"^\s*name:\s*/([A-Za-z0-9_]+)/", re.MULTILINE)


def robots_in_bag(bag_dir):
    """Robot namespaces recorded in a bag, read straight from metadata.yaml
    (no rosbag2_py needed -- this runs on the host, outside jazzy_env).
    Real-hardware bags hold a single robot's own topics, so this returns
    just that robot; simulator bags hold every robot's topics together in
    one file (all robots launch in a single process, unlike separate
    per-robot recorders on hardware), so this returns all of them --
    needed so generate_comparison_figure.py, which expects one bag per
    robot, gets a robot=bag_dir pair per robot instead of one for the
    session's synthetic "world" key."""
    metadata = os.path.join(bag_dir, "metadata.yaml")
    try:
        with open(metadata) as f:
            text = f.read()
    except OSError:
        return []
    return sorted(set(TOPIC_NAME_RE.findall(text)))


def scan_runs():
    """Return {"baseline": [...], "vxch": [...], "zstd": [...]} of session
    dicts, newest first. Each session dict: {"ts": datetime, "label": str,
    "runs": {robot: dir_name}}."""
    entries = {c: [] for c in CONDITIONS}
    if not os.path.isdir(RUNS_DIR):
        return {c: [] for c in CONDITIONS}

    for robot in sorted(os.listdir(RUNS_DIR)):
        robot_dir = os.path.join(RUNS_DIR, robot)
        if not os.path.isdir(robot_dir):
            continue

        # Simulator runs (launch.sh) are written flat as
        # experiment_runs/<timestamp>_<condition>_<world>/bag, one dir per
        # whole multi-robot run rather than per robot. Treat the world name
        # as a single "robot" so these still form sessions below.
        m = RUN_DIR_RE.match(robot)
        if m and os.path.isdir(os.path.join(robot_dir, "bag")):
            ts_str, condition, world = m.groups()
            try:
                ts = datetime.strptime(ts_str, "%Y%m%d_%H%M%S")
            except ValueError:
                continue
            entries[condition].append((ts, world, robot))
            continue

        # Real-hardware runs (launch_real_hardware.sh / recover_metrics.sh)
        # nest one dir per robot: experiment_runs/<robot>/<timestamp>_<condition>_<robot>.
        for name in os.listdir(robot_dir):
            m = RUN_DIR_RE.match(name)
            if not m:
                continue
            ts_str, condition, run_robot = m.groups()
            if run_robot != robot:
                continue
            if not os.path.isdir(os.path.join(robot_dir, name, "bag")):
                continue
            try:
                ts = datetime.strptime(ts_str, "%Y%m%d_%H%M%S")
            except ValueError:
                continue
            entries[condition].append((ts, robot, name))

    sessions = {}
    for condition, items in entries.items():
        items.sort(key=lambda x: x[0])
        clusters = []
        for ts, robot, name in items:
            # Merge into the current cluster only if this run is both close
            # in time to the most recently added run (not the cluster's
            # frozen start -- comparing against the start let far-apart
            # back-to-back experiments chain-merge whenever every
            # consecutive gap happened to be small) AND from a robot not
            # already in this cluster -- a real session launches each robot
            # exactly once, so a second run from the same robot always means
            # a new, later experiment, never an additional robot joining the
            # same one. Without this, that second run's name would silently
            # overwrite the first under the earlier (and now wrong) label.
            if (clusters
                    and (ts - clusters[-1]["last_ts"]).total_seconds() <= SESSION_GAP_SECONDS
                    and robot not in clusters[-1]["runs"]):
                cluster = clusters[-1]
                cluster["last_ts"] = ts
                cluster["runs"][robot] = name
            else:
                clusters.append({"ts": ts, "last_ts": ts, "runs": {robot: name}})
        for c in clusters:
            c["label"] = c["ts"].strftime("%Y-%m-%d %H:%M:%S")
        clusters.sort(key=lambda c: c["ts"], reverse=True)
        sessions[condition] = clusters

    return sessions


class RunPicker(ttk.Frame):
    def __init__(self, master, title, sessions, all_robots):
        super().__init__(master)
        self.all_robots = all_robots
        ttk.Label(self, text=title, font=("", 11, "bold")).pack(anchor="w")

        columns = ("timestamp", "robots")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", selectmode="browse", height=14)
        self.tree.heading("timestamp", text="Timestamp")
        self.tree.heading("robots", text="Robots included")
        self.tree.column("timestamp", width=160, anchor="w")
        self.tree.column("robots", width=220, anchor="w")
        self.tree.pack(fill="both", expand=True, side="left")

        scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")

        self.sessions = sessions
        for i, session in enumerate(sessions):
            present = sorted(session["runs"].keys())
            missing = sorted(set(all_robots) - set(present))
            robots_text = ", ".join(present)
            if missing:
                robots_text += f"  (missing: {', '.join(missing)})"
            tag = "complete" if not missing else "partial"
            self.tree.insert("", "end", iid=str(i), values=(session["label"], robots_text), tags=(tag,))

        self.tree.tag_configure("partial", foreground="#a15c00")
        if sessions:
            self.tree.selection_set("0")
            self.tree.focus("0")

    def selected_session(self):
        sel = self.tree.selection()
        if not sel:
            return None
        return self.sessions[int(sel[0])]


class ReplayGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Replay Compare")
        self.geometry("1080x560")
        self.proc = None
        self.fig_proc = None

        sessions = scan_runs()
        # Derived from the scanned sessions rather than os.listdir(RUNS_DIR):
        # flat simulator run dirs (experiment_runs/<ts>_<condition>_<world>)
        # aren't robot dirs, so listing RUNS_DIR directly would misreport
        # each sim run as its own "missing" robot.
        all_robots = sorted({
            robot
            for condition_sessions in sessions.values()
            for session in condition_sessions
            for robot in session["runs"]
        })

        picker_frame = ttk.Frame(self, padding=10)
        picker_frame.pack(fill="both", expand=True)
        for col in range(len(CONDITIONS)):
            picker_frame.columnconfigure(col, weight=1)
        picker_frame.rowconfigure(0, weight=1)

        self.pickers = {}
        for col, condition in enumerate(CONDITIONS):
            padx = (0 if col == 0 else 5, 0 if col == len(CONDITIONS) - 1 else 5)
            picker = RunPicker(picker_frame, CONDITION_LABELS[condition], sessions[condition], all_robots)
            picker.grid(row=0, column=col, sticky="nsew", padx=padx)
            self.pickers[condition] = picker

        present = [c for c in CONDITIONS if sessions[c]]
        if len(present) < 2:
            ttk.Label(
                picker_frame,
                text="Need runs for at least 2 conditions under experiment_runs/ to compare.",
                foreground="red",
            ).grid(row=1, column=0, columnspan=len(CONDITIONS), sticky="w", pady=(6, 0))

        controls = ttk.Frame(self, padding=(10, 0, 10, 10))
        controls.pack(fill="x")

        ttk.Label(controls, text="Playback rate:").pack(side="left")
        self.rate_var = tk.StringVar(value="8")
        ttk.Entry(controls, textvariable=self.rate_var, width=6).pack(side="left", padx=(4, 12))

        self.launch_btn = ttk.Button(controls, text="Launch Compare", command=self.launch)
        self.launch_btn.pack(side="left")

        self.stop_btn = ttk.Button(controls, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.pack(side="left", padx=(6, 0))

        self.figure_btn = ttk.Button(controls, text="Generate Figure", command=self.generate_figure)
        self.figure_btn.pack(side="left", padx=(12, 0))

        ttk.Button(controls, text="Refresh runs", command=self.refresh).pack(side="right")

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(self, textvariable=self.status_var, padding=(10, 0)).pack(fill="x")

        log_frame = ttk.Frame(self, padding=(10, 0, 10, 10))
        log_frame.pack(fill="both", expand=True)
        self.log = tk.Text(log_frame, height=10, state="disabled", bg="#111", fg="#ddd")
        self.log.pack(fill="both", expand=True, side="left")
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=log_scroll.set)
        log_scroll.pack(side="right", fill="y")

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def refresh(self):
        for child in list(self.children.values()):
            child.destroy()
        self.__init__()

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _selected_sessions(self):
        """{condition: session} for every condition with a picker selection
        (a condition with no runs at all has an empty picker, so
        selected_session() returns None for it and it's simply omitted)."""
        selected = {}
        for condition, picker in self.pickers.items():
            session = picker.selected_session()
            if session is not None:
                selected[condition] = session
        return selected

    def launch(self):
        if self.proc is not None:
            messagebox.showinfo("Already running", "A replay is already running. Stop it first.")
            return

        selected = self._selected_sessions()
        if len(selected) < 2:
            messagebox.showerror("No selection", "Select runs for at least 2 conditions first.")
            return

        rate = self.rate_var.get().strip() or "8"
        try:
            float(rate)
        except ValueError:
            messagebox.showerror("Invalid rate", f"'{rate}' is not a number.")
            return

        env = os.environ.copy()
        env_args = []
        for condition, session in selected.items():
            override = ",".join(f"{robot}:{name}" for robot, name in session["runs"].items())
            var_name = f"{condition.upper()}_RUN_OVERRIDE"
            env[var_name] = override
            env_args.append(f"{var_name}={override}")
        # Tell replay_compare.sh which conditions to actually show -- without
        # this it defaults to baseline+vxch regardless of what's selected here.
        conditions_arg = ",".join(selected.keys())
        env["REPLAY_CONDITIONS"] = conditions_arg
        env_args.append(f"REPLAY_CONDITIONS={conditions_arg}")

        if in_container():
            cmd = ["distrobox-host-exec", "env", *env_args, REPLAY_SCRIPT, rate]
        else:
            cmd = [REPLAY_SCRIPT, rate]

        labels = ", ".join(f"{c}={s['label']}" for c, s in selected.items())
        self.append_log(f"$ {labels} rate={rate}\n")
        self.status_var.set("Launching...")
        self.launch_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")

        threading.Thread(target=self._run_process, args=(cmd, env), daemon=True).start()

    def _run_process(self, cmd, env):
        try:
            self.proc = subprocess.Popen(
                cmd, cwd=PROJECT_ROOT, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
            )
        except FileNotFoundError as e:
            self.after(0, lambda: self._finish(f"Failed to launch: {e}\n"))
            return

        for line in self.proc.stdout:
            self.after(0, self.append_log, line)
        returncode = self.proc.wait()
        self.after(0, lambda: self._finish(f"[exited with code {returncode}]\n"))

    def _finish(self, message):
        self.append_log(message)
        self.status_var.set("Ready.")
        self.launch_btn.configure(state="normal")
        self.stop_btn.configure(state="disabled")
        self.proc = None

    def stop(self):
        if self.proc is None:
            return
        self.append_log("Stopping...\n")
        try:
            self.proc.terminate()
        except ProcessLookupError:
            pass

    def generate_figure(self):
        """Produce the bandwidth/coverage comparison figure for whichever
        sessions are currently selected in the pickers (any 2 or all 3
        conditions; defaults to the latest of each, since each RunPicker
        preselects row 0 and sessions are sorted newest first)."""
        if self.fig_proc is not None:
            messagebox.showinfo("Already running", "A figure is already being generated.")
            return

        selected = self._selected_sessions()
        if len(selected) < 2:
            messagebox.showerror("No selection", "Select runs for at least 2 conditions first.")
            return

        def bag_args(session):
            args = []
            for robot, run_dir in session["runs"].items():
                bag_dir = run_bag_dir(robot, run_dir)
                robots = robots_in_bag(bag_dir)
                if robot in robots:
                    args.append(f"{robot}={bag_dir}")
                else:
                    # Flat sim bag: its "robot" is a synthetic world key, not
                    # a real namespace, so expand to one arg per actual robot.
                    args.extend(f"{r}={bag_dir}" for r in robots)
            return args

        os.makedirs(FIGURES_DIR, exist_ok=True)
        out_name = "compare_" + "_vs_".join(
            f"{c}-{s['ts'].strftime('%Y%m%d_%H%M%S')}" for c, s in selected.items()
        ) + ".png"
        out_path = os.path.join(FIGURES_DIR, out_name)

        # Always "python3" (never sys.executable): this command runs inside
        # jazzy_env (rosbag2_py/rclpy live there, not on the host), either
        # directly if replay_gui.py itself is already in-container, or via
        # the distrobox wrapper below otherwise -- sys.executable would be
        # the *host* interpreter in that second case, which lacks rosbag2_py.
        py_cmd = ["python3", FIGURE_SCRIPT]
        for condition, session in selected.items():
            py_cmd += [f"--{condition}"] + bag_args(session)
        py_cmd += ["--out", out_path]

        if in_container():
            cmd = py_cmd
        else:
            # generate_comparison_figure.py needs rosbag2_py/rclpy, only
            # importable inside jazzy_env, and only via a login shell (its
            # .bashrc.d sources the ROS setup) -- see FIGURE_SCRIPT's docstring.
            quoted = " ".join(shlex.quote(part) for part in py_cmd)
            cmd = ["distrobox", "enter", "jazzy_env", "--", "bash", "-lc", quoted]

        labels = ", ".join(f"{c}={s['label']}" for c, s in selected.items())
        self.append_log(f"$ generate figure: {labels}\n")
        self.figure_btn.configure(state="disabled")

        threading.Thread(target=self._run_figure_process, args=(cmd, out_path), daemon=True).start()

    def _run_figure_process(self, cmd, out_path):
        try:
            self.fig_proc = subprocess.Popen(
                cmd, cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
            )
        except FileNotFoundError as e:
            self.after(0, lambda: self._finish_figure(f"Failed to launch: {e}\n", None))
            return

        for line in self.fig_proc.stdout:
            self.after(0, self.append_log, line)
        returncode = self.fig_proc.wait()
        message = f"[figure generation exited with code {returncode}]\n"
        self.after(0, lambda: self._finish_figure(message, out_path if returncode == 0 else None))

    def _finish_figure(self, message, out_path):
        self.append_log(message)
        self.figure_btn.configure(state="normal")
        self.fig_proc = None
        if out_path and os.path.isfile(out_path):
            self.append_log(f"Figure saved to {out_path}\n")
            try:
                subprocess.Popen(["xdg-open", out_path])
            except FileNotFoundError:
                pass

    def on_close(self):
        if self.proc is not None:
            if not messagebox.askyesno("Replay running", "A replay is still running. Stop it and quit?"):
                return
            self.stop()
        self.destroy()


def main():
    if not os.path.isdir(RUNS_DIR):
        print(f"No experiment_runs/ directory found at {RUNS_DIR}", file=sys.stderr)
        sys.exit(1)
    ReplayGUI().mainloop()


if __name__ == "__main__":
    main()
