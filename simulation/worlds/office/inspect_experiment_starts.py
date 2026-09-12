#!/usr/bin/env python3
"""Generate a collision-based office layout and validate shared-hall starts.

Run from any directory. Requires the dependencies of check_spawn_points and
matplotlib. This is a static geometry check, not a Nav2/Gazebo rollout.
"""
from collections import deque
from pathlib import Path
import os

os.environ.setdefault('MPLCONFIGDIR', '/tmp/office_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
import check_spawn_points as geometry


def main():
    volumes = geometry.load_volumes()
    xs, ys, labels, sizes, _ = geometry.interior_map(volumes)
    with open(geometry.PRESETS) as stream:
        starts = yaml.safe_load(stream)['office']['shared_hall']
    def cell(x, y):
        return (round((y - ys[0]) / geometry.GRID_RES),
                round((x - xs[0]) / geometry.GRID_RES))
    region = labels[cell(starts[0]['x'], starts[0]['y'])]
    assert region >= 0
    # Inflate raster obstacles by 0.35 m, exceeding the configured 0.291 m
    # circumscribed footprint radius. Include raster half-cell padding already
    # applied by interior_map; this connectivity test is conservative.
    occupied = labels < 0
    blocked = occupied.copy()
    radius = .35
    for dr in range(-4, 5):
        for dc in range(-4, 5):
            if np.hypot(dr, dc) * geometry.GRID_RES <= radius:
                blocked |= np.roll(occupied, (dr, dc), axis=(0, 1))
    allowed = (labels == region) & ~blocked
    seed = cell(starts[0]['x'], starts[0]['y'])
    assert allowed[seed]
    reached = np.zeros_like(allowed)
    reached[seed] = True
    queue = deque([seed])
    while queue:
        r, c = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if (0 <= nr < allowed.shape[0] and 0 <= nc < allowed.shape[1]
                    and allowed[nr, nc] and not reached[nr, nc]):
                reached[nr, nc] = True
                queue.append((nr, nc))
    for i, start in enumerate(starts):
        x, y = start['x'], start['y']
        assert reached[cell(x, y)], (i, 'not connected with footprint padding')
        for z in np.arange(.1, .61, .1):
            assert not geometry.containing(volumes, x, y, z=z, margin=radius), (i, z)
        print(f'robot{i+1}: ({x:.2f}, {y:.2f}), yaw={start["yaw"]:.4f}; padded body clear')
    for i, a in enumerate(starts):
        for b in starts[i+1:]:
            assert np.hypot(a['x']-b['x'], a['y']-b['y']) > 2*radius
    probes = [(-15, 5.5), (-15, 10), (-15, 14), (-15, 18.7), (6, 18)]
    for x, y in probes:
        assert reached[cell(x, y)], (x, y, 'circulation probe unreachable')
    print(f'{len(volumes)} collision volumes; connected point-floor area '
          f'{sizes[region] * geometry.GRID_RES**2:.1f} m²')
    print('All starts and five circulation probes connect with 0.35 m obstacle inflation.')
    fig, ax = plt.subplots(figsize=(12, 8))
    image = np.where(labels == region, 1., np.where(occupied, 0., .65))
    ax.imshow(image, origin='lower', extent=[xs[0], xs[-1], ys[0], ys[-1]],
              cmap='gray', vmin=0, vmax=1)
    for i, start in enumerate(starts):
        x, y = start['x'], start['y']
        color = ['tab:red', 'tab:blue', 'tab:green'][i]
        ax.add_patch(plt.Circle((x, y), radius, color=color))
        ax.arrow(x, y, -.9, 0, color=color, head_width=.3, length_includes_head=True)
        ax.annotate(f'R{i+1}', (x, y), xytext=(x, y-2.8), ha='center', color=color,
                    arrowprops={'arrowstyle': '-', 'color': color})
    ax.text(-18, 11.8, 'Long aisles and cross-corridors', fontsize=10)
    ax.set(xlim=(-29, 23), ylim=(-4, 24), xlabel='World x (m)', ylabel='World y (m)',
           title='Office: collision geometry and shared-hall starts\n'
                 'White: connected floor; black: obstacles; gray: exterior/disconnected')
    ax.grid(alpha=.15)
    fig.tight_layout()
    output = Path(__file__).with_name('office_start_layout.svg')
    fig.savefig(output)
    print(output)


if __name__ == '__main__':
    main()
