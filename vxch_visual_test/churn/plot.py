#!/usr/bin/env python3
"""Plot the churn experiment written by churn.py."""
import json, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

MAP_KB = 596        # New College @0.2 m, fully encoded
SCHEMES = ["global", "slab-8", "slab-32", "cube-8", "cube-4"]
COLORS = {"global": "#d62728", "slab-8": "#ff7f0e", "slab-32": "#bcbd22",
          "cube-8": "#1f77b4", "cube-4": "#2ca02c"}

h = json.load(open(sys.argv[1] if len(sys.argv) > 1 else "hist.json"))
out = sys.argv[2] if len(sys.argv) > 2 else "churn.png"
vox = [r["voxels"] / 1000 for r in h["global"]]
fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))

for s in SCHEMES:
    ax[0].plot(vox, [r["dirty_kb"] for r in h[s]], label=s, color=COLORS[s], lw=2)
ax[0].axhline(MAP_KB, ls=":", c="k", lw=1)
ax[0].text(vox[1], MAP_KB * 1.1, "whole map", fontsize=8)
ax[0].set_yscale("log")
ax[0].set(xlabel="map size (thousand voxels)", ylabel="re-sent per update (KB, log)")
ax[0].set_title("Cost of one map update", fontweight="bold", loc="left")

for s in SCHEMES:
    cum, t = [], 0
    for r in h[s]:
        t += r["dirty_kb"]; cum.append(t / 1024)
    ax[1].plot(vox, cum, label=s, color=COLORS[s], lw=2)
ax[1].set(xlabel="map size (thousand voxels)", ylabel="cumulative re-sent (MB)")
ax[1].set_title("Total traffic over the run", fontweight="bold", loc="left")

for s in SCHEMES:   # three channels per unit, so normalise by 3*units
    ax[2].plot(vox, [100 * r["dirty_units"] / max(1, 3 * r["total_units"]) for r in h[s]],
               label=s, color=COLORS[s], lw=2)
ax[2].set(xlabel="map size (thousand voxels)", ylabel="% of channels invalidated")
ax[2].set_ylim(-3, 105)
ax[2].set_title("Fraction of the map invalidated", fontweight="bold", loc="left")

for a in ax:
    a.legend(frameon=False, fontsize=9); a.grid(alpha=.3)
plt.tight_layout(); plt.savefig(out, dpi=110)

print(f"{'scheme':9s} {'units':>6s} {'total re-sent':>14s} {'= x map':>8s} {'% chans dirty':>14s}")
for s in SCHEMES:
    tot = sum(r["dirty_kb"] for r in h[s]) / 1024
    du = sum(100 * r["dirty_units"] / max(1, 3 * r["total_units"]) for r in h[s]) / len(h[s])
    print(f"{s:9s} {h[s][-1]['total_units']:6d} {tot:11.1f} MB {tot*1024/MAP_KB:7.0f}x {du:13.1f}%")
print(f"\nwrote {out}")
