"""Geometry and information-to-decision checks, independent of ROS/Gazebo."""

from collections import deque
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import generate_long_t as generator
from check_long_t_decisions import Scene, check, collision_boxes


def test_checked_in_artifacts_match_generator():
    for name, text in generator.artifacts().items():
        assert (generator.OUT / name).read_text() == text


def test_floor_is_connected_and_all_boundary_edges_have_walls():
    cells = generator.floor_cells()
    reached = {next(iter(cells))}
    queue = deque(reached)
    boxes = collision_boxes()['walls']
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (x + dx, y + dy)
            if neighbor in cells:
                if neighbor not in reached:
                    reached.add(neighbor)
                    queue.append(neighbor)
            else:
                # Exposed floor edge lies on the wall's inner surface.
                px = (x + 0.5 + dx / 2) * generator.STEP
                py = (y + 0.5 + dy / 2) * generator.STEP
                assert any(x0 - 1e-6 <= px <= x1 + 1e-6 and y0 - 1e-6 <= py <= y1 + 1e-6
                           for x0, y0, x1, y1 in boxes)
    assert reached == cells


def test_starts_are_separated_and_have_footprint_clearance():
    scene = Scene(resolution=0.25)
    walls = collision_boxes()['walls']
    for i, start in enumerate(generator.STARTS):
        x, y = start['x'], start['y']
        assert scene.truth[scene.cell(x, y)] == 0
        clearance = min(np.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1))
                        for x0, y0, x1, y1 in walls)
        assert clearance >= 1.4
        for other in generator.STARTS[i+1:]:
            assert np.hypot(x-other['x'], y-other['y']) >= 4


def test_work_tips_reachable_with_robot_radius():
    scene = Scene(resolution=0.25)
    free = scene.truth == 0
    safe = free.copy()
    # Conservative square footprint, half-width 0.5 m, larger than the robots.
    padded = np.pad(free, 2, constant_values=False)
    for dr in range(5):
        for dc in range(5):
            safe &= padded[dr:dr+free.shape[0], dc:dc+free.shape[1]]
    start = scene.cell(**{k: generator.STARTS[0][k] for k in ('x', 'y')})
    visited = {start}
    queue = deque(visited)
    while queue:
        r, c = queue.popleft()
        for nr, nc in ((r-1, c), (r+1, c), (r, c-1), (r, c+1)):
            if 0 <= nr < safe.shape[0] and 0 <= nc < safe.shape[1] and safe[nr, nc] and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append((nr, nc))
    for region in generator.regions():
        if region['name'].endswith('_tip'):
            x0, y0, x1, _ = region['bounds']
            assert scene.cell((x0+x1)/2, y0+1) in visited
    for start in generator.STARTS:
        assert scene.cell(start['x'], start['y']) in visited


def test_sdf_visuals_match_collisions_and_floor_is_at_zero():
    tree = ET.parse(generator.OUT / 'long_t.sdf')
    for link in tree.findall('.//model/link'):
        for collision in link.findall('collision'):
            visual = link.find(f"visual[@name='{collision.attrib['name'].replace('_collision', '_visual')}']")
            assert visual.findtext('pose') == collision.findtext('pose')
            assert visual.findtext('geometry/box/size') == collision.findtext('geometry/box/size')
            size = list(map(float, collision.findtext('geometry/box/size').split()))
            assert min(size) > 0
            if link.attrib['name'] == 'floor':
                z = float(collision.findtext('pose').split()[2])
                assert z + size[2]/2 == 0


def test_spawn_yaml_matches_geometry():
    # Read only this fixture's deliberately inline {x, y, yaw} YAML records.
    stanza = (ROOT / 'spawn_presets.yaml').read_text().split('\nlong_t:\n')[1].split('\nsmall_maze:')[0]
    distributed = stanza.split('  distributed:\n')[1].split('  reversed:')[0]
    records = [json.loads(re.sub(r'(\w+):', r'"\1":', entry))
               for entry in re.findall(r'\{[^}]+\}', distributed)]
    assert records == generator.STARTS


@pytest.fixture(scope='module')
def decisions():
    return check()['results']


def test_oracle_enters_unfinished_work_in_all_checkpoints(decisions):
    assert all(row['outcome'] == 'work_spine' for row in decisions if row['arm'] == 'oracle')
    # This occurs before first team coverage, not in the post-coverage tail.
    assert all(row['team_unobserved_m2'] > 450 for row in decisions)


def test_none_revisits_peer_branches_before_work(decisions):
    for radius in (3.5, 10.0):
        rows = [r for r in decisions if r['arm'] == 'none' and r['sensor_range_m'] == radius]
        assert sum(r['outcome'] == 'peer_start' for r in rows) >= 2
