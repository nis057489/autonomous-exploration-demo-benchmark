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

RUN_DIR_RE = re.compile(r"^(\d{8}_\d{6})_(baseline|vxch)_(\w+)$")
SESSION_GAP_SECONDS = 90  # runs across robots within this gap = one session


def in_container():
    return os.path.exists("/run/.containerenv")


def scan_runs():
    """Return {"baseline": [...], "vxch": [...]} of session dicts, newest
    first. Each session dict: {"ts": datetime, "label": str,
    "runs": {robot: dir_name}}."""
    entries = {"baseline": [], "vxch": []}
    if not os.path.isdir(RUNS_DIR):
        return {"baseline": [], "vxch": []}

    for robot in sorted(os.listdir(RUNS_DIR)):
        robot_dir = os.path.join(RUNS_DIR, robot)
        if not os.path.isdir(robot_dir):
            continue
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
        self.geometry("760x560")
        self.proc = None
        self.fig_proc = None

        sessions = scan_runs()
        all_robots = sorted(os.listdir(RUNS_DIR)) if os.path.isdir(RUNS_DIR) else []

        picker_frame = ttk.Frame(self, padding=10)
        picker_frame.pack(fill="both", expand=True)
        picker_frame.columnconfigure(0, weight=1)
        picker_frame.columnconfigure(1, weight=1)
        picker_frame.rowconfigure(0, weight=1)

        self.baseline_picker = RunPicker(picker_frame, "Baseline run", sessions["baseline"], all_robots)
        self.baseline_picker.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        self.vxch_picker = RunPicker(picker_frame, "Wavestream run", sessions["vxch"], all_robots)
        self.vxch_picker.grid(row=0, column=1, sticky="nsew", padx=(5, 0))

        if not sessions["baseline"] or not sessions["vxch"]:
            ttk.Label(
                picker_frame,
                text="No runs found under experiment_runs/ for one or both conditions.",
                foreground="red",
            ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))

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

    def launch(self):
        if self.proc is not None:
            messagebox.showinfo("Already running", "A replay is already running. Stop it first.")
            return

        baseline_session = self.baseline_picker.selected_session()
        vxch_session = self.vxch_picker.selected_session()
        if not baseline_session or not vxch_session:
            messagebox.showerror("No selection", "Select a baseline run and a Wavestream run first.")
            return

        rate = self.rate_var.get().strip() or "8"
        try:
            float(rate)
        except ValueError:
            messagebox.showerror("Invalid rate", f"'{rate}' is not a number.")
            return

        baseline_override = ",".join(f"{robot}:{name}" for robot, name in baseline_session["runs"].items())
        vxch_override = ",".join(f"{robot}:{name}" for robot, name in vxch_session["runs"].items())

        env = os.environ.copy()
        env["BASELINE_RUN_OVERRIDE"] = baseline_override
        env["VXCH_RUN_OVERRIDE"] = vxch_override

        if in_container():
            cmd = [
                "distrobox-host-exec", "env",
                f"BASELINE_RUN_OVERRIDE={baseline_override}",
                f"VXCH_RUN_OVERRIDE={vxch_override}",
                REPLAY_SCRIPT, rate,
            ]
        else:
            cmd = [REPLAY_SCRIPT, rate]

        self.append_log(f"$ baseline={baseline_session['label']} wavestream={vxch_session['label']} rate={rate}\n")
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
        baseline and vxch sessions are currently selected in the pickers
        (defaults to the latest of each, since each RunPicker preselects
        row 0 and sessions are sorted newest first)."""
        if self.fig_proc is not None:
            messagebox.showinfo("Already running", "A figure is already being generated.")
            return

        baseline_session = self.baseline_picker.selected_session()
        vxch_session = self.vxch_picker.selected_session()
        if not baseline_session or not vxch_session:
            messagebox.showerror("No selection", "Select a baseline run and a Wavestream run first.")
            return

        def bag_args(session):
            return [
                f"{robot}={os.path.join(RUNS_DIR, robot, run_dir, 'bag')}"
                for robot, run_dir in session["runs"].items()
            ]

        os.makedirs(FIGURES_DIR, exist_ok=True)
        out_name = f"compare_{baseline_session['ts'].strftime('%Y%m%d_%H%M%S')}_vs_{vxch_session['ts'].strftime('%Y%m%d_%H%M%S')}.png"
        out_path = os.path.join(FIGURES_DIR, out_name)

        # Always "python3" (never sys.executable): this command runs inside
        # jazzy_env (rosbag2_py/rclpy live there, not on the host), either
        # directly if replay_gui.py itself is already in-container, or via
        # the distrobox wrapper below otherwise -- sys.executable would be
        # the *host* interpreter in that second case, which lacks rosbag2_py.
        py_cmd = (
            ["python3", FIGURE_SCRIPT, "--baseline"] + bag_args(baseline_session)
            + ["--vxch"] + bag_args(vxch_session) + ["--out", out_path]
        )

        if in_container():
            cmd = py_cmd
        else:
            # generate_comparison_figure.py needs rosbag2_py/rclpy, only
            # importable inside jazzy_env, and only via a login shell (its
            # .bashrc.d sources the ROS setup) -- see FIGURE_SCRIPT's docstring.
            quoted = " ".join(shlex.quote(part) for part in py_cmd)
            cmd = ["distrobox", "enter", "jazzy_env", "--", "bash", "-lc", quoted]

        self.append_log(f"$ generate figure: baseline={baseline_session['label']} wavestream={vxch_session['label']}\n")
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
