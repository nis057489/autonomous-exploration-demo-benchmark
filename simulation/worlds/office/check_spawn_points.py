#!/usr/bin/env python3
"""Check office spawn points against office.world's actual collision geometry.

Motivation: the office world is unusually easy to pick bad spawn points in.
ServiceSim models some rooms as a single solid box filling the whole room
(PublicMeetingRoomC, PrivateMeetingRoomB-E, PrivateOfficeA/B, PrivateBathroomB),
so a point that looks like open floor on the visual floorplan -- or reads as
"free" in the shipped office_part1/map/map.png, which does not correspond to
this world's generation -- can be solidly inside geometry. A robot spawned there
is embedded in a static box and simply cannot move, while Nav2 happily plans
paths for it, so the symptom looks like a controller problem rather than a
spawn problem.

Usage:
    python3 simulation/worlds/office/check_spawn_points.py            # check presets
    python3 simulation/worlds/office/check_spawn_points.py X Y ...    # check points

Exits non-zero if any checked point is inside collision geometry.
"""

import os
import sys
from collections import deque

import numpy as np
import yaml
from lxml import etree

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD = os.path.join(HERE, "office.world")
PRESETS = os.path.abspath(os.path.join(HERE, "..", "..", "..", "spawn_presets.yaml"))

ROBOT_Z = 0.10       # spawn height used by launch.sh
GRID_RES = 0.10
# Walls in this world are 0.01 m thin boxes; a grid sampled at cell centres
# passes straight through them. Inflating each box by the cell half-diagonal
# makes rasterisation conservative so thin walls are never missed.
CELL_PAD = GRID_RES * 0.7071


def _pose(el):
    p = el.find("pose")
    if p is None or not (p.text or "").strip():
        return np.zeros(6)
    vals = [float(v) for v in p.text.split()]
    return np.array(vals + [0.0] * (6 - len(vals)))


def _mat(p):
    x, y, z, roll, pitch, yaw = p
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    m = np.eye(4)
    m[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])
    m[:3, 3] = [x, y, z]
    return m


def _half_extents(shape):
    if shape.tag == "box":
        return np.array([float(v) for v in shape.find("size").text.split()]) / 2
    if shape.tag == "cylinder":
        rad = float(shape.find("radius").text)
        return np.array([rad, rad, float(shape.find("length").text) / 2])
    return None


def _link_volumes(root, m_parent, label, volumes):
    for link in root.findall("link"):
        m_link = m_parent @ _mat(_pose(link))
        for col in link.findall("collision"):
            geom = col.find("geometry")
            if geom is None or len(geom) == 0:
                continue
            half = _half_extents(geom[0])
            if half is None:
                continue
            m_col = m_link @ _mat(_pose(col))
            volumes.append((label, np.linalg.inv(m_col), half))


def load_volumes():
    """Every solid collision volume, as (model name, world->local matrix, half extents).

    Includes are resolved and their model.sdf collisions composed in. Skipping
    them would miss the furniture entirely -- desks, chairs and tables are all
    <include>d, so a spawn point could sit inside a desk and still be reported
    clear.
    """
    world = etree.parse(WORLD).getroot().find("world")
    models_dir = os.path.join(HERE, "models")
    volumes = []
    cache = {}

    for model in world.iter("model"):
        if model.get("name") == "floor":
            continue  # the ground plane is what the robot stands on
        m_model = _mat(_pose(model))
        _link_volumes(model, m_model, model.get("name"), volumes)

    for inc in world.iter("include"):
        uri = inc.find("uri")
        if uri is None or not (uri.text or "").strip().startswith("model://"):
            continue
        name = (uri.text or "").strip()[len("model://"):].split("/")[0]
        if name not in cache:
            sdf_path = os.path.join(models_dir, name, "model.sdf")
            cache[name] = (etree.parse(sdf_path).getroot().find("model")
                           if os.path.isfile(sdf_path) else None)
        model_root = cache[name]
        if model_root is None:
            continue

        # Compose: world -> enclosing <model> wrapper (if any) -> include pose.
        m_world = np.eye(4)
        parent = inc.getparent()
        if parent is not None and parent.tag == "model":
            m_world = _mat(_pose(parent))
        m_inc = m_world @ _mat(_pose(inc)) @ _mat(_pose(model_root))
        label = inc.find("name").text.strip() if inc.find("name") is not None else name
        _link_volumes(model_root, m_inc, f"{label} ({name})", volumes)

    return volumes


def containing(volumes, x, y, z=ROBOT_Z, margin=0.0):
    """Names of collision volumes containing (x, y, z), expanded by margin."""
    point = np.array([x, y, z, 1.0])
    hits = []
    for name, inv, half in volumes:
        local = inv @ point
        if np.all(np.abs(local[:3]) <= half + margin):
            hits.append(name)
    return sorted(set(hits))


def interior_map(volumes):
    """Conservative occupancy grid plus connected-component labels."""
    xs = np.arange(-40.0, 35.0, GRID_RES)
    ys = np.arange(-12.0, 36.0, GRID_RES)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx, gy, np.full_like(gx, ROBOT_Z), np.ones_like(gx)], axis=-1)

    occ = np.zeros(gx.shape, bool)
    for _, inv, half in volumes:
        local = pts @ inv.T
        h = half + CELL_PAD
        occ |= ((np.abs(local[..., 0]) <= h[0]) &
                (np.abs(local[..., 1]) <= h[1]) &
                (np.abs(local[..., 2]) <= h[2]))

    free = ~occ
    labels = np.full(free.shape, -1, int)
    sizes = []
    for i in range(free.shape[0]):
        for j in range(free.shape[1]):
            if free[i, j] and labels[i, j] < 0:
                cid = len(sizes)
                queue = deque([(i, j)])
                labels[i, j] = cid
                count = 0
                while queue:
                    a, b = queue.popleft()
                    count += 1
                    for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        p, q = a + da, b + db
                        if (0 <= p < free.shape[0] and 0 <= q < free.shape[1]
                                and free[p, q] and labels[p, q] < 0):
                            labels[p, q] = cid
                            queue.append((p, q))
                sizes.append(count)

    # Distance to nearest occupied cell, in metres.
    dist = np.full(occ.shape, np.inf)
    queue = deque()
    for i in range(occ.shape[0]):
        for j in range(occ.shape[1]):
            if occ[i, j]:
                dist[i, j] = 0
                queue.append((i, j))
    while queue:
        a, b = queue.popleft()
        for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            p, q = a + da, b + db
            if (0 <= p < occ.shape[0] and 0 <= q < occ.shape[1]
                    and dist[p, q] > dist[a, b] + 1):
                dist[p, q] = dist[a, b] + 1
                queue.append((p, q))
    return xs, ys, labels, sizes, dist * GRID_RES


def main():
    volumes = load_volumes()
    print(f"{len(volumes)} solid collision volumes in office.world")

    args = sys.argv[1:]
    if args:
        if len(args) % 2:
            raise SystemExit("give X Y pairs")
        points = [(f"arg[{i}]", float(args[2 * i]), float(args[2 * i + 1]))
                  for i in range(len(args) // 2)]
    else:
        with open(PRESETS) as handle:
            presets = yaml.safe_load(handle).get("office", {})
        points = [(f"{preset}[{i}]", pos["x"], pos["y"])
                  for preset, positions in presets.items()
                  for i, pos in enumerate(positions)]

    xs, ys, labels, sizes, dist = interior_map(volumes)

    def cell(x, y):
        return (int(round((y - ys[0]) / GRID_RES)), int(round((x - xs[0]) / GRID_RES)))

    # The interior is the largest component that is not the unbounded outdoor one.
    outside = labels[cell(-38.0, -10.0)]
    interior = max((c for c in range(len(sizes)) if c != outside), key=lambda c: sizes[c])
    print(f"interior region: {sizes[interior] * GRID_RES ** 2:.0f} m^2 "
          f"({'sealed from outside' if interior != outside else 'LEAKS to outside'})")

    failures = 0
    for name, x, y in points:
        inside = containing(volumes, x, y)
        i, j = cell(x, y)
        clear = dist[i, j]
        region = labels[i, j]
        if inside:
            print(f"  FAIL {name:16s} ({x:7.2f},{y:6.2f})  INSIDE {', '.join(inside)}")
            failures += 1
        elif region != interior:
            print(f"  FAIL {name:16s} ({x:7.2f},{y:6.2f})  not in the interior region")
            failures += 1
        else:
            print(f"  ok   {name:16s} ({x:7.2f},{y:6.2f})  clearance {clear:.2f} m")

    if failures:
        print(f"\n{failures} bad spawn point(s): robots there are embedded in geometry "
              "and will not move.", file=sys.stderr)
        return 1
    print("\nall spawn points are clear")
    return 0


if __name__ == "__main__":
    sys.exit(main())
