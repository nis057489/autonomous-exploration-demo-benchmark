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
import argparse
import glob
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
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
    session's synthetic "world" key.

    If metadata.yaml is missing (recorder killed before finalizing, e.g. an
    interrupted run), reindex a scratch copy of the raw .mcap first --
    mirrors replay_compare.sh and generate_comparison_figure.py's same
    fallback. Without this, an interrupted bag silently contributes zero
    robots here, which drops that whole condition's args in generate_figure()
    instead of surfacing a clear error."""
    metadata = os.path.join(bag_dir, "metadata.yaml")
    scratch = None
    if not os.path.isfile(metadata):
        mcap_files = glob.glob(os.path.join(bag_dir, "*.mcap"))
        if not mcap_files:
            return []
        scratch = tempfile.mkdtemp(prefix="replay_gui_reindex_")
        for mcap_file in mcap_files:
            # Symlink rather than copy -- reindex only reads the .mcap, and
            # these can be multi-GB; copying would block the GUI's main
            # thread (this runs synchronously in generate_figure(), before
            # the background thread starts) for as long as the copy takes.
            os.symlink(os.path.abspath(mcap_file), os.path.join(scratch, os.path.basename(mcap_file)))
        try:
            subprocess.run(
                ["ros2", "bag", "reindex", "-s", "mcap", scratch],
                check=True, capture_output=True, text=True,
            )
        except (subprocess.CalledProcessError, OSError):
            shutil.rmtree(scratch, ignore_errors=True)
            return []
        metadata = os.path.join(scratch, "metadata.yaml")

    try:
        with open(metadata) as f:
            text = f.read()
    except OSError:
        return []
    finally:
        if scratch is not None:
            shutil.rmtree(scratch, ignore_errors=True)
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
        # "extended" (ctrl/shift-click multi-select) rather than "browse" --
        # generate_figure() uses multiple selected runs per condition to
        # plot mean +/- std error bars across runs; launch() (live replay)
        # still only plays one run per condition, so it errors out if more
        # than one is selected there.
        self.tree = ttk.Treeview(self, columns=columns, show="headings", selectmode="extended", height=14)
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

    def selected_sessions(self):
        """All currently-selected sessions, in tree order (not selection
        order -- ttk.Treeview.selection() already returns iids in tree
        order)."""
        return [self.sessions[int(iid)] for iid in self.tree.selection()]


class ReplayGUI(tk.Tk):
    def __init__(self, initial_max_duration=""):
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

        ttk.Label(
            self,
            text="Ctrl/Shift-click to select several runs per condition -- "
                 "Generate Figure will then plot mean ± std error bars across them "
                 "(Launch Compare still needs exactly one).",
            padding=(10, 6, 10, 0),
            foreground="#555",
        ).pack(fill="x")

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

        ttk.Label(controls, text="Max duration (s):").pack(side="left")
        self.max_duration_var = tk.StringVar(value=initial_max_duration)
        ttk.Entry(controls, textvariable=self.max_duration_var, width=6).pack(side="left", padx=(4, 12))

        self.launch_btn = ttk.Button(controls, text="Launch Compare", command=self.launch)
        self.launch_btn.pack(side="left")

        self.stop_btn = ttk.Button(controls, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.pack(side="left", padx=(6, 0))

        self.figure_btn = ttk.Button(controls, text="Generate Figure", command=self.generate_figure)
        self.figure_btn.pack(side="left", padx=(12, 0))

        self.separate_figures_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls, text="Each panel as its own image", variable=self.separate_figures_var,
        ).pack(side="left", padx=(8, 0))

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
        max_duration = self.max_duration_var.get()
        for child in list(self.children.values()):
            child.destroy()
        self.__init__(initial_max_duration=max_duration)

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _selected_sessions(self):
        """{condition: [session, ...]} for every condition with at least one
        picker selection (a condition with no runs at all has an empty
        picker, so selected_sessions() returns [] for it and it's simply
        omitted). Pickers allow multi-select so generate_figure() can
        average over several runs per condition; launch() below still
        requires exactly one."""
        selected = {}
        for condition, picker in self.pickers.items():
            sessions = picker.selected_sessions()
            if sessions:
                selected[condition] = sessions
        return selected

    def launch(self):
        if self.proc is not None:
            messagebox.showinfo("Already running", "A replay is already running. Stop it first.")
            return

        selected_multi = self._selected_sessions()
        if len(selected_multi) < 2:
            messagebox.showerror("No selection", "Select runs for at least 2 conditions first.")
            return
        multi = {c: s for c, s in selected_multi.items() if len(s) > 1}
        if multi:
            messagebox.showerror(
                "Multiple runs selected",
                "Live replay only plays one run per condition. Select a single run for: "
                + ", ".join(sorted(multi)) + ".",
            )
            return
        selected = {c: s[0] for c, s in selected_multi.items()}

        rate = self.rate_var.get().strip() or "8"
        try:
            float(rate)
        except ValueError:
            messagebox.showerror("Invalid rate", f"'{rate}' is not a number.")
            return

        max_duration = self.max_duration_var.get().strip()
        if max_duration:
            try:
                float(max_duration)
            except ValueError:
                messagebox.showerror("Invalid max duration", f"'{max_duration}' is not a number.")
                return

        env = os.environ.copy()
        env_args = []
        if max_duration:
            env["REPLAY_MAX_DURATION"] = max_duration
            env_args.append(f"REPLAY_MAX_DURATION={max_duration}")
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
        duration_suffix = f" max_duration={max_duration}s" if max_duration else ""
        self.append_log(f"$ {labels} rate={rate}{duration_suffix}\n")
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
        preselects row 0 and sessions are sorted newest first). Selecting
        several runs for a condition (ctrl/shift-click) plots that
        condition's mean +/- std across those runs instead of a single
        run's values -- see generate_comparison_figure.py's --max-duration
        docstring update for the --<condition> repeat-per-run CLI shape."""
        if self.fig_proc is not None:
            messagebox.showinfo("Already running", "A figure is already being generated.")
            return

        selected = self._selected_sessions()
        if len(selected) < 2:
            messagebox.showerror("No selection", "Select runs for at least 2 conditions first.")
            return

        max_duration = self.max_duration_var.get().strip()
        if max_duration:
            try:
                float(max_duration)
            except ValueError:
                messagebox.showerror("Invalid max duration", f"'{max_duration}' is not a number.")
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
            c + "-" + "+".join(s["ts"].strftime("%Y%m%d_%H%M%S") for s in sessions)
            for c, sessions in selected.items()
        ) + ".png"
        out_path = os.path.join(FIGURES_DIR, out_name)

        # Always the container's system "/usr/bin/python3" -- never
        # sys.executable (that'd be the *host* interpreter when we're not
        # already in-container, which lacks rosbag2_py) and never a bare
        # "python3" (PATH inside the distrobox login shell below inherits
        # whatever the host shell had prepended -- e.g. an activated venv --
        # which can shadow the system interpreter that actually has
        # matplotlib/rosbag2_py installed).
        py_cmd = ["/usr/bin/python3", FIGURE_SCRIPT]
        for condition, sessions in selected.items():
            # One --<condition> flag per selected run -- generate_comparison_figure.py
            # treats each occurrence as a separate run to average over.
            for session in sessions:
                py_cmd += [f"--{condition}"] + bag_args(session)
        py_cmd += ["--out", out_path]
        if max_duration:
            py_cmd += ["--max-duration", max_duration]
        separate_figures = self.separate_figures_var.get()
        if separate_figures:
            py_cmd += ["--separate-figures"]

        if in_container():
            cmd = py_cmd
        else:
            # generate_comparison_figure.py needs rosbag2_py/rclpy, only
            # importable inside jazzy_env, and only via a login shell (its
            # .bashrc.d sources the ROS setup) -- see FIGURE_SCRIPT's docstring.
            quoted = " ".join(shlex.quote(part) for part in py_cmd)
            cmd = ["distrobox", "enter", "jazzy_env", "--", "bash", "-lc", quoted]

        labels = ", ".join(
            f"{c}=[{', '.join(s['label'] for s in sessions)}]" for c, sessions in selected.items()
        )
        self.append_log(f"$ generate figure: {labels}\n")
        self.figure_btn.configure(state="disabled")

        threading.Thread(
            target=self._run_figure_process, args=(cmd, out_path, separate_figures), daemon=True,
        ).start()

    def _run_figure_process(self, cmd, out_path, separate_figures):
        try:
            self.fig_proc = subprocess.Popen(
                cmd, cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
            )
        except FileNotFoundError as e:
            self.after(0, lambda: self._finish_figure(f"Failed to launch: {e}\n", None, separate_figures))
            return

        for line in self.fig_proc.stdout:
            self.after(0, self.append_log, line)
        returncode = self.fig_proc.wait()
        message = f"[figure generation exited with code {returncode}]\n"
        self.after(0, lambda: self._finish_figure(message, out_path if returncode == 0 else None, separate_figures))

    def _finish_figure(self, message, out_path, separate_figures):
        self.append_log(message)
        self.figure_btn.configure(state="normal")
        self.fig_proc = None
        if not out_path:
            return
        if separate_figures:
            # --separate-figures writes <stem>_<panel><suffix> per panel
            # instead of out_path itself -- see generate_comparison_figure.py.
            stem, suffix = os.path.splitext(out_path)
            panel_paths = sorted(glob.glob(f"{stem}_*{suffix}"))
            for panel_path in panel_paths:
                self.append_log(f"Figure saved to {panel_path}\n")
            for panel_path in panel_paths:
                try:
                    subprocess.Popen(["xdg-open", panel_path])
                except FileNotFoundError:
                    break
        elif os.path.isfile(out_path):
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
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--max-duration", type=float, default=None,
        help="Clip both playback and the generated figure to this many seconds of bag time.",
    )
    args = parser.parse_args()

    if not os.path.isdir(RUNS_DIR):
        print(f"No experiment_runs/ directory found at {RUNS_DIR}", file=sys.stderr)
        sys.exit(1)
    initial_max_duration = "" if args.max_duration is None else str(args.max_duration)
    ReplayGUI(initial_max_duration=initial_max_duration).mainloop()


if __name__ == "__main__":
    main()
