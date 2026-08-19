#!/usr/bin/env python3
"""Interactive vxch tester: generate a synthetic occupancy grid, encode it
with the real vxch Haar-band codec (via ../build/vxch_cli, which links the
actual voxelcodec_ros/codec.cpp -- see ../CMakeLists.txt), then click "Send
Next Band" to push one band at a time across a simulated bandwidth-limited
link and watch the receiver's reconstruction fill in, coarsest-first, next
to the sender's ground-truth map. No ROS, no robots, no DDS discovery.

Needs tkinter + numpy + Pillow, which live in the jazzy_env distrobox
(the host venv doesn't have them -- see replay_gui.py's docstring for the
same constraint), so run this via:
    distrobox enter jazzy_env -- python3 gui/vxch_gui.py
"""
import json
import os
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
from PIL import Image, ImageTk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grid_io  # noqa: E402

GUI_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(GUI_DIR)
CLI_BIN = os.path.join(PROJECT_ROOT, "build", "vxch_cli")
MAPS_DIR = os.path.join(PROJECT_ROOT, "maps")
MAP_PATH = os.path.join(MAPS_DIR, "synthetic_map.grid")
SESSION_PATH = os.path.join(MAPS_DIR, "session.vxch")
STATE_PATH = os.path.join(MAPS_DIR, "state.json")
RECEIVER_PATH = os.path.join(MAPS_DIR, "receiver.grid")

CANVAS_PX = 480

# kbps presets lifted from ../../wifi_profiles.json so this matches the same
# link conditions the real hardware experiments run under.
WIFI_PRESETS = {
    "Good (250 mbps)": 250_000,
    "Degraded (1 mbps)": 1_000,
    "Denied (1 kbps)": 1,
}


def run_cli(*args):
    if not os.path.isfile(CLI_BIN):
        raise RuntimeError(
            f"{CLI_BIN} not found -- build it first:\n"
            f"  cd {PROJECT_ROOT} && distrobox enter jazzy_env -- "
            f"bash -lc 'cmake -S . -B build && cmake --build build -j'"
        )
    proc = subprocess.run(
        [CLI_BIN, *[str(a) for a in args]],
        cwd=PROJECT_ROOT, capture_output=True, text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"vxch_cli {' '.join(map(str, args))} failed:\n{proc.stderr.strip()}")
    return json.loads(proc.stdout)


def array_to_photo(cells):
    rgb = grid_io.grid_to_rgb(cells)
    img = Image.fromarray(rgb, mode="RGB")
    # Always fit the longer side to CANVAS_PX, in both directions -- the old
    # integer-floor-div-clamped-to-1 version only ever upscaled, so a map
    # larger than the display box (e.g. a 640x480 generated map) rendered at
    # full native pixel size and grew the whole window (Tk auto-sizes to its
    # widgets) past the screen. NEAREST keeps small maps crisp/blocky when
    # enlarging; BOX area-averages when shrinking, so a heavily downscaled
    # large map still shows where mixed free/occupied/unknown regions are
    # (as gray blends) instead of aliasing into an arbitrary sparse subsample.
    scale = CANVAS_PX / max(img.width, img.height)
    new_w = max(1, round(img.width * scale))
    new_h = max(1, round(img.height * scale))
    resample = Image.NEAREST if scale >= 1 else Image.BOX
    img = img.resize((new_w, new_h), resample)
    return ImageTk.PhotoImage(img)


def fmt_bytes(n):
    return f"{n/1024:.1f} KB" if n >= 1024 else f"{n} B"


class MapPanel(ttk.Frame):
    def __init__(self, master, title):
        super().__init__(master)
        ttk.Label(self, text=title, font=("", 11, "bold")).pack(anchor="w")
        self.canvas_label = tk.Label(self, bg="#222")
        self.canvas_label.pack()
        # Tk only treats a Label's width/height as pixels once it holds an
        # image (otherwise they're text-cell units) -- showing a blank
        # placeholder immediately avoids a geometry jump on the first real
        # image and keeps every panel the same size before any map exists.
        self.show(np.full((CANVAS_PX // 3, CANVAS_PX // 3), -1, dtype=np.int8))

    def show(self, cells):
        self._photo = array_to_photo(cells)
        self.canvas_label.configure(image=self._photo)


class VxchGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("vxch visual test")
        self.geometry("1080x760")
        os.makedirs(MAPS_DIR, exist_ok=True)

        self.width = self.height = 0
        self.sender_cells = None
        self.total_entries = 0
        self.raw_bytes = 0
        self.total_compressed_bytes = 0
        self.sent_index = 0
        self.cumulative_bytes = 0
        self._sending = False
        self._stop_requested = False

        self._build_widgets()
        self._refresh_from_disk()

    # ---------------------------------------------------------------- UI

    def _build_widgets(self):
        gen = ttk.LabelFrame(self, text="1. Synthetic map", padding=8)
        gen.pack(fill="x", padx=10, pady=(10, 4))
        ttk.Label(gen, text="Width").pack(side="left")
        self.width_var = tk.StringVar(value="160")
        ttk.Entry(gen, textvariable=self.width_var, width=6).pack(side="left", padx=(2, 8))
        ttk.Label(gen, text="Height").pack(side="left")
        self.height_var = tk.StringVar(value="120")
        ttk.Entry(gen, textvariable=self.height_var, width=6).pack(side="left", padx=(2, 8))
        ttk.Label(gen, text="Seed").pack(side="left")
        self.seed_var = tk.StringVar(value="42")
        ttk.Entry(gen, textvariable=self.seed_var, width=6).pack(side="left", padx=(2, 12))
        self.gen_btn = ttk.Button(gen, text="Generate Map", command=self.generate_map)
        self.gen_btn.pack(side="left")

        enc = ttk.LabelFrame(self, text="2. Encode with vxch", padding=8)
        enc.pack(fill="x", padx=10, pady=4)
        ttk.Label(enc, text="Haar levels").pack(side="left")
        self.levels_var = tk.StringVar(value="4")
        ttk.Entry(enc, textvariable=self.levels_var, width=4).pack(side="left", padx=(2, 8))
        ttk.Label(enc, text="Tile size (cells)").pack(side="left")
        self.tile_var = tk.StringVar(value="32")
        ttk.Entry(enc, textvariable=self.tile_var, width=5).pack(side="left", padx=(2, 8))
        ttk.Label(enc, text="Compression").pack(side="left")
        self.compression_var = tk.StringVar(value="zstd")
        ttk.Combobox(
            enc, textvariable=self.compression_var, values=["zstd", "none"],
            width=6, state="readonly",
        ).pack(side="left", padx=(2, 12))
        # Ablation knob, independent of compression above: on = zigzag-varint pack each
        # band's Haar coefficients before compression (default); off = fixed-width int32
        # instead. With compression=none this isolates varint packing's own contribution --
        # compression=none alone still leaves varint packing in place. Mirrors
        # occupancy_grid_vxch_node's varint_encoding parameter.
        self.varint_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            enc, text="Varint packing", variable=self.varint_var,
        ).pack(side="left", padx=(0, 12))
        self.encode_btn = ttk.Button(enc, text="Encode", command=self.encode_map, state="disabled")
        self.encode_btn.pack(side="left")

        send = ttk.LabelFrame(self, text="3. Send bands over a simulated link", padding=8)
        send.pack(fill="x", padx=10, pady=4)
        ttk.Label(send, text="Bandwidth (kbps)").pack(side="left")
        self.bandwidth_var = tk.StringVar(value="50")
        ttk.Entry(send, textvariable=self.bandwidth_var, width=8).pack(side="left", padx=(2, 8))
        for label, kbps in WIFI_PRESETS.items():
            ttk.Button(
                send, text=label, width=16,
                command=lambda k=kbps: self.bandwidth_var.set(str(k)),
            ).pack(side="left", padx=2)
        self.send_btn = ttk.Button(
            send, text="Send Next Band", command=self.send_next_clicked, state="disabled")
        self.send_btn.pack(side="left", padx=(12, 4))
        self.send_all_btn = ttk.Button(
            send, text="Send All", command=self.send_all_clicked, state="disabled")
        self.send_all_btn.pack(side="left", padx=4)
        self.stop_btn = ttk.Button(send, text="Stop", command=self.stop_sending, state="disabled")
        self.stop_btn.pack(side="left", padx=4)
        self.reset_btn = ttk.Button(
            send, text="Reset Receiver", command=self.reset_receiver, state="disabled")
        self.reset_btn.pack(side="left", padx=4)

        panels = ttk.Frame(self, padding=(10, 6))
        panels.pack(fill="both", expand=True)
        self.sender_panel = MapPanel(panels, "Sender -- ground truth map")
        self.sender_panel.pack(side="left", padx=(0, 8))
        self.receiver_panel = MapPanel(panels, "Receiver -- reconstruction so far")
        self.receiver_panel.pack(side="left", padx=(8, 0))

        self.status_var = tk.StringVar(value="Generate a map to begin.")
        ttk.Label(self, textvariable=self.status_var, padding=(10, 0)).pack(fill="x")

        log_frame = ttk.Frame(self, padding=(10, 4, 10, 10))
        log_frame.pack(fill="both", expand=False)
        self.log = tk.Text(log_frame, height=8, state="disabled", bg="#111", fg="#ddd")
        self.log.pack(fill="both", expand=True, side="left")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")

    def _log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ------------------------------------------------------------ state

    def _blank_receiver(self):
        if self.width and self.height:
            self.receiver_panel.show(np.full((self.height, self.width), -1, dtype=np.int8))

    def _load_receiver_if_present(self):
        if os.path.isfile(RECEIVER_PATH):
            _, _, _, cells = grid_io.read_grid(RECEIVER_PATH)
            self.receiver_panel.show(cells)
        else:
            self._blank_receiver()

    def _refresh_from_disk(self):
        """Pick up whatever the last run left behind, so the GUI is
        resumable across restarts instead of forcing a fresh map every time."""
        if os.path.isfile(MAP_PATH):
            self.width, self.height, _, self.sender_cells = grid_io.read_grid(MAP_PATH)
            self.sender_panel.show(self.sender_cells)
            self.encode_btn.configure(state="normal")

        if os.path.isfile(SESSION_PATH):
            with open(SESSION_PATH) as f:
                session = json.load(f)
            self.total_entries = len(session["entries"])
            self.raw_bytes = session["raw_bytes"]
            self.total_compressed_bytes = session["total_compressed_bytes"]
            self.send_btn.configure(state="normal")
            self.send_all_btn.configure(state="normal")
            self.reset_btn.configure(state="normal")

        if os.path.isfile(STATE_PATH):
            with open(STATE_PATH) as f:
                state = json.load(f)
            self.sent_index = state.get("sent_index", 0)
            self.cumulative_bytes = state.get("cumulative_bytes", 0)

        self._load_receiver_if_present()
        self._update_status()

    def _update_status(self):
        if self.total_entries:
            ratio = (
                100.0 * self.total_compressed_bytes / self.raw_bytes
                if self.raw_bytes else 0.0
            )
            self.status_var.set(
                f"Sent {self.sent_index}/{self.total_entries} bands  "
                f"({fmt_bytes(self.cumulative_bytes)} / "
                f"{fmt_bytes(self.total_compressed_bytes)} of encoded traffic, "
                f"raw map = {fmt_bytes(self.raw_bytes)}, "
                f"full vxch stream = {ratio:.0f}% of raw)"
            )
            if self.sent_index >= self.total_entries:
                self.send_btn.configure(state="disabled")
                self.send_all_btn.configure(state="disabled")

    # -------------------------------------------------------- 1. generate

    def generate_map(self):
        try:
            width = int(self.width_var.get())
            height = int(self.height_var.get())
            seed = int(self.seed_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Width/height/seed must be integers.")
            return
        self.gen_btn.configure(state="disabled")
        self.status_var.set("Generating synthetic map...")
        threading.Thread(
            target=self._generate_map_thread, args=(width, height, seed), daemon=True,
        ).start()

    def _generate_map_thread(self, width, height, seed):
        try:
            result = run_cli("gen-map", "--out", MAP_PATH, "--width", width,
                              "--height", height, "--seed", seed)
        except Exception as e:  # noqa: BLE001
            self.after(0, lambda: self._generate_map_failed(str(e)))
            return
        self.after(0, lambda: self._generate_map_done(result))

    def _generate_map_failed(self, message):
        messagebox.showerror("Generate failed", message)
        self.gen_btn.configure(state="normal")

    def _generate_map_done(self, result):
        for path in (SESSION_PATH, STATE_PATH, RECEIVER_PATH):
            if os.path.isfile(path):
                os.remove(path)
        self.total_entries = self.sent_index = self.cumulative_bytes = 0
        self.total_compressed_bytes = 0
        self.send_btn.configure(state="disabled")
        self.send_all_btn.configure(state="disabled")
        self.reset_btn.configure(state="disabled")
        self.width, self.height, _, self.sender_cells = grid_io.read_grid(MAP_PATH)
        self.sender_panel.show(self.sender_cells)
        self._blank_receiver()
        self.gen_btn.configure(state="normal")
        self.encode_btn.configure(state="normal")
        self._log(
            f"Generated {result['width']}x{result['height']} map: "
            f"{result['free_cells']} free, {result['occupied_cells']} occupied, "
            f"{result['unknown_cells']} unknown cells."
        )
        self.status_var.set("Map generated. Encode it to build the band send queue.")

    # ---------------------------------------------------------- 2. encode

    def encode_map(self):
        try:
            levels = int(self.levels_var.get())
            tile_size = int(self.tile_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Haar levels/tile size must be integers.")
            return
        self.encode_btn.configure(state="disabled")
        self.status_var.set("Encoding with vxch...")
        threading.Thread(
            target=self._encode_thread,
            args=(levels, tile_size, self.compression_var.get(), self.varint_var.get()),
            daemon=True,
        ).start()

    def _encode_thread(self, levels, tile_size, compression, use_varint):
        try:
            result = run_cli(
                "encode", "--map", MAP_PATH, "--out", SESSION_PATH,
                "--levels", levels, "--tile-size-cells", tile_size,
                "--compression", compression,
                "--varint", "true" if use_varint else "false",
            )
        except Exception as e:  # noqa: BLE001
            self.after(0, lambda: self._encode_failed(str(e)))
            return
        self.after(0, lambda: self._encode_done(result))

    def _encode_failed(self, message):
        messagebox.showerror("Encode failed", message)
        self.encode_btn.configure(state="normal")

    def _encode_done(self, result):
        for path in (STATE_PATH, RECEIVER_PATH):
            if os.path.isfile(path):
                os.remove(path)
        self.total_entries = result["total_entries"]
        self.raw_bytes = result["raw_bytes"]
        self.total_compressed_bytes = result["total_compressed_bytes"]
        self.sent_index = self.cumulative_bytes = 0
        self._blank_receiver()
        self.encode_btn.configure(state="normal")
        self.send_btn.configure(state="normal")
        self.send_all_btn.configure(state="normal")
        self.reset_btn.configure(state="normal")
        ratio = 100.0 * self.total_compressed_bytes / self.raw_bytes if self.raw_bytes else 0
        varint_label = "varint" if result["varint"] else "fixed-width int32"
        self._log(
            f"Encoded {result['tiles']} tiles x {result['bands_per_tile']} bands "
            f"= {self.total_entries} band messages queued ({varint_label} packing). "
            f"Full stream = {fmt_bytes(self.total_compressed_bytes)} vs "
            f"{fmt_bytes(self.raw_bytes)} raw ({ratio:.0f}%)."
        )
        self._update_status()

    def reset_receiver(self):
        for path in (STATE_PATH, RECEIVER_PATH):
            if os.path.isfile(path):
                os.remove(path)
        self.sent_index = self.cumulative_bytes = 0
        self._blank_receiver()
        self.send_btn.configure(state="normal")
        self.send_all_btn.configure(state="normal")
        self._log("Receiver reset -- same encoded session, replaying from band 0.")
        self._update_status()

    # ------------------------------------------------------------ 3. send

    def _bandwidth_kbps(self):
        try:
            kbps = float(self.bandwidth_var.get())
        except ValueError:
            kbps = 50.0
        return max(kbps, 0.001)

    def send_next_clicked(self):
        if self._sending:
            return
        self._sending = True
        self._set_send_controls(active=False)
        threading.Thread(target=self._send_n_thread, daemon=True).start()

    def send_all_clicked(self):
        if self._sending:
            return
        self._sending = True
        self._stop_requested = False
        self._set_send_controls(active=False)
        self.stop_btn.configure(state="normal")
        threading.Thread(target=self._send_all_thread, daemon=True).start()

    def stop_sending(self):
        self._stop_requested = True

    def _set_send_controls(self, active):
        state = "normal" if active else "disabled"
        self.send_btn.configure(state=state)
        self.send_all_btn.configure(state=state)
        self.reset_btn.configure(state=state)

    def _send_all_thread(self):
        while not self._stop_requested:
            done = self._send_one_band_blocking()
            if done or self._stop_requested:
                break
        self.after(0, self._finish_sending)

    def _send_n_thread(self):
        self._send_one_band_blocking()
        self.after(0, self._finish_sending)

    def _send_one_band_blocking(self):
        """Pop the next queued band, simulate its transfer time over the
        configured bandwidth, then reveal the receiver's updated
        reconstruction. Runs on a background thread; only touches Tk state
        via self.after."""
        kbps = self._bandwidth_kbps()
        try:
            result = run_cli(
                "step", "--session", SESSION_PATH, "--state", STATE_PATH,
                "--out-receiver", RECEIVER_PATH, "--n", 1,
            )
        except Exception as e:  # noqa: BLE001
            self.after(0, lambda: messagebox.showerror("Send failed", str(e)))
            return True

        sent = result["sent"]
        if sent:
            band = sent[0]
            delay_s = band["compressed_size"] * 8 / (kbps * 1000)
            label = (
                "LL (coarsest)" if band["band_index"] == 0
                else f"detail band {band['band_index']}"
            )
            self.after(0, lambda: self.status_var.set(
                f"Sending tile ({band['tile_row']},{band['tile_col']}) {label}: "
                f"{fmt_bytes(band['compressed_size'])} @ {kbps:.0f} kbps "
                f"-> ~{delay_s*1000:.0f} ms in flight..."
            ))
            time.sleep(delay_s)

            self.sent_index = result["sent_index"]
            self.cumulative_bytes = result["cumulative_bytes"]
            self.total_entries = result["total_entries"]
            self.raw_bytes = result["raw_bytes"]
            self.total_compressed_bytes = result["total_compressed_bytes"]

            _, _, _, receiver_cells = grid_io.read_grid(RECEIVER_PATH)

            def apply():
                self.receiver_panel.show(receiver_cells)
                self._log(
                    f"tile({band['tile_row']},{band['tile_col']}) {label}: "
                    f"{fmt_bytes(band['compressed_size'])} compressed "
                    f"({fmt_bytes(band['uncompressed_size'])} raw) "
                    f"delivered after {delay_s*1000:.0f} ms @ {kbps:.0f} kbps"
                )
                self._update_status()

            self.after(0, apply)

        return result["done"]

    def _finish_sending(self):
        self._sending = False
        self.stop_btn.configure(state="disabled")
        done = self.sent_index >= self.total_entries
        self._set_send_controls(active=not done)
        self.reset_btn.configure(state="normal")
        self._update_status()


def main():
    VxchGui().mainloop()


if __name__ == "__main__":
    main()
