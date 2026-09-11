#!/usr/bin/env python3
"""Download every Gazebo Fuel model a world references into the Fuel cache.

WHY THIS EXISTS. Gazebo resolves a Fuel <uri> by downloading it synchronously
while the world loads, so a world built from dozens of Fuel models (the SubT
worlds: final_prelim_03, tunnel_qualification) spends its first launch blocked
on the network before the /create service ever comes up -- long enough for
robot spawning to time out, and on a sweep it would land on whichever run
happened to be first. Fetching once up front keeps every launch identical.

The Dockerfile's own `gz fuel download` layer does not cover this: docker.sh
bind-mounts the host's ~/.gz/fuel over /root/.gz/fuel, which hides whatever the
image baked in. The host cache is the one that counts, so run this wherever
`gz` is on PATH and $HOME is the host's (the jazzy_env distrobox, or inside the
container started by docker.sh -- both write through to ~/.gz/fuel).

URIs are passed to `gz fuel download` exactly as the world spells them. Fuel
caches a model under its name and host as requested, so "Tunnel%20Tile%205"
and "Tunnel Tile 5" land in different cache folders, as do the same model via
fuel.ignitionrobotics.org and fuel.gazebosim.org -- only the spelling the SDF
uses is found at load time.

Models can <include> other models (every SubT artifact nests an "Artifact
Proximity Detector", still addressed via fuel.ignitionrobotics.org), and those
are resolved at load time too, so each cached model.sdf is scanned in turn.
model.config <depend> entries are not: nothing loads them.

Usage:
  tools/prefetch_fuel_models.py tunnel_qualification final_prelim_03
  tools/prefetch_fuel_models.py --list final_prelim_03   # print, don't fetch
"""

import argparse
import os
import re
import subprocess
import sys
from urllib.parse import urlparse

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS_ROOT = os.path.join(PROJECT_ROOT, "simulation", "worlds")


def world_file(name):
    # Same rule as launch.sh / world.launch.py: worlds/<name>/<name>.{world,sdf}.
    for ext in (".world", ".sdf"):
        path = os.path.join(WORLDS_ROOT, name, name + ext)
        if os.path.isfile(path):
            return path
    raise SystemExit(f"Unknown world '{name}' (no {WORLDS_ROOT}/{name}/{name}.world|.sdf)")


def fuel_model_uris(path):
    with open(path) as f:
        text = f.read()
    uris = []
    # Some SubT includes wrap the URI across lines or pad it with spaces.
    for raw in re.findall(r"<uri>(.*?)</uri>", text, re.S):
        uri = raw.strip()
        parts = urlparse(uri)
        if parts.scheme in ("http", "https") and "/models/" in parts.path:
            # Only the model root: .../<owner>/models/<name>[/<version>|/tip/files/...]
            owner_models, _, rest = uri.partition("/models/")
            uri = f"{owner_models}/models/{rest.split('/')[0]}"
            if uri not in uris:
                uris.append(uri)
    return uris


def cache_dir(uri):
    # <cache>/<host>/<owner>/models/<name>, all lowercased -- Fuel's layout.
    root = os.environ.get("GZ_FUEL_CACHE_PATH", os.path.expanduser("~/.gz/fuel"))
    parts = urlparse(uri)
    owner, _, name = parts.path.split("/1.0/", 1)[1].partition("/models/")
    return os.path.join(root, parts.netloc, owner.lower(), "models", name.lower())


def cached_model_sdf(uri):
    # Newest cached version's model.sdf, or None.
    d = cache_dir(uri)
    versions = sorted((v for v in os.listdir(d) if v.isdigit()), key=int) if os.path.isdir(d) else []
    path = os.path.join(d, versions[-1], "model.sdf") if versions else None
    return path if path and os.path.isfile(path) else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("worlds", nargs="+")
    ap.add_argument("--list", action="store_true", help="print the URIs and exit")
    args = ap.parse_args()

    uris = []
    for name in args.worlds:
        for uri in fuel_model_uris(world_file(name)):
            if uri not in uris:
                uris.append(uri)

    if args.list:
        for uri in uris:
            print(uri)
        return 0

    failed = []
    i = 0
    # uris grows as nested includes are discovered.
    while i < len(uris):
        uri = uris[i]
        i += 1
        if os.path.isdir(cache_dir(uri)):
            print(f"[{i}/{len(uris)}] cached   {uri}")
        else:
            print(f"[{i}/{len(uris)}] fetching {uri}", flush=True)
            if subprocess.run(["gz", "fuel", "download", "-u", uri]).returncode != 0:
                failed.append(uri)
                continue
        sdf = cached_model_sdf(uri)
        if sdf:
            uris.extend(u for u in fuel_model_uris(sdf) if u not in uris)

    if failed:
        print(f"\n{len(failed)} model(s) failed to download:", file=sys.stderr)
        for uri in failed:
            print(f"  {uri}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
