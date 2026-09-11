#!/usr/bin/env python3
"""Compare first discovery and overlap at equal budgets from exported summaries.

Example: python3 tools/analyze_discovery.py summaries/none.npz summaries/oracle.npz
Uses raw local-map cell sets only. Times follow the summary's bag-record clock.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def discovery_series(run):
    """Return (time, unique area, summed per-robot area, overlap area).

    Count each cell once per robot and once per team, including when multiple
    messages arrive at the same timestamp. Overlap is sum minus union, not
    repeated sensor hits or distance spent revisiting a place.
    """
    resolutions = {entry[6] for entry in run.values()}
    if len(resolutions) != 1 or next(iter(resolutions)) <= 0:
        raise ValueError('All robots must have the same positive cell resolution')
    area = next(iter(resolutions)) ** 2
    seen = {robot: set() for robot in run}
    team = set()
    total = 0
    output = []
    events = sorted(((t, robot, cells) for robot, entry in run.items()
                     for t, cells in entry[4]), key=lambda event: (event[0], event[1]))
    for t, robot, cells in events:
        new = set(cells.tolist()) - seen[robot]
        seen[robot].update(new)
        team.update(new)
        total += len(new)
        row = (t, len(team) * area, total * area, (total - len(team)) * area)
        if output and output[-1][0] == t:
            output[-1] = row
        else:
            output.append(row)
    return output


def at_time(series, time):
    index = np.searchsorted([row[0] for row in series], time, side='right') - 1
    return series[index][1:] if index >= 0 else (0.0, 0.0, 0.0)


def report(run, budgets, targets, window):
    if window <= 0:
        raise ValueError('Rate window must be positive')
    series = discovery_series(run)
    # Last raw map observation, not last discovery: a stationary robot can
    # keep observing without discovering anything new.
    if not run or any(not entry[3] for entry in run.values()):
        raise ValueError('Every robot must have raw local-map observations')
    end = min(entry[3][-1][0] for entry in run.values())
    samples = []
    for t in budgets:
        if t < 0 or t > end:
            samples.append({'time_s': t, 'observed': False})
            continue
        unique, total, overlap = at_time(series, t)
        start = max(0, t - window)
        previous = at_time(series, start)[0]
        samples.append(dict(time_s=t, observed=True, unique_m2=unique,
                            summed_local_m2=total, overlap_m2=overlap,
                            discovery_m2_s=(unique - previous)/(t-start) if t > start else 0,
                            unique_fraction=unique/total if total else None))
    return dict(common_observed_end_s=end, clock='bag recording elapsed seconds',
                samples=samples, first_passage_s={str(target): next(
                    (t for t, unique, _, _ in series if unique >= target and t <= end), None)
                    for target in targets})


def main():
    from export_run_summary import load_summary
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summaries', nargs='+', type=Path)
    parser.add_argument('--budgets', nargs='+', type=float, default=[60,120,180,240,300])
    parser.add_argument('--targets', nargs='+', type=float, default=[200,300,400,450])
    parser.add_argument('--window', type=float, default=30)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    output = {}
    for path in args.summaries:
        meta, run = load_summary(path)
        output[str(path)] = dict(condition=meta['condition'], world=meta['world'],
                                 **report(run, args.budgets, args.targets, args.window))
    rendered = json.dumps(output, indent=2)
    if args.out:
        args.out.write_text(rendered + '\n')
    print(rendered)


if __name__ == '__main__':
    main()
