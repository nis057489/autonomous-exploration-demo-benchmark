#!/usr/bin/env python3
"""Generate a deterministic capacity-vs-time schedule for the DDIL links.

WHY THIS EXISTS. A constant-bandwidth run measures steady-state throughput: how
much map each transport pushes through a fixed pipe. But vxch's actual claim is
not throughput, it is *graceful degradation* -- under a starved link it still
delivers coarse coverage of the whole map, and when capacity returns it refines.
Both halves of that are TRANSIENTS, and a constant-capacity run is structurally
blind to a transient. Varying capacity turns the link into an excitation signal
so those transients become measurable.

THE TWO RULES THE SHAPE FOLLOWS FROM.

1. The schedule is an independent variable: exogenous, deterministic, identical
   across arms. It must never be a function of robot state, map state, or which
   transport is running -- otherwise capacity becomes endogenous to the policy
   under test and a baseline-vs-vxch comparison means nothing. Hence: the whole
   trace is sampled UP FRONT from RANDOM_SEED and written to disk before the run
   starts, rather than sampled live. Statistically identical, but it makes the
   trace an auditable pre-run input instead of a side effect, it cannot drift
   with node-startup jitter, and two arms' schedules can be diffed byte-for-byte
   to prove they saw the same conditions.

2. Excite the regimes; don't average them. Real links fail in bursts, not
   smoothly, and burstiness is also what makes them measurable -- a hard
   transition gives every transient metric an edge to align to. A sine wave is
   the worst of both worlds: it is not how radios behave, its time-at-value
   distribution is arcsine (it already spends most of its time near the
   extremes, i.e. it is a smeared square wave), and because it sweeps
   continuously through the codec's regime boundaries no sample is ever in
   steady state -- every measurement is contaminated by the one before it. So:
   two-state Gilbert-Elliott with exponential dwells. Realistic AND step-shaped.

Profiles:
  static            one segment at bandwidth_kbps -- today's behavior, and the
                    control condition you need in order to attribute anything.
  gilbert_elliott   seeded two-state good/bad link, exponential dwells.
  staircase         deterministic descent/ascent through the codec's documented
                    regime boundaries. Not more realistic than gilbert_elliott,
                    but the cleanest thing to read by eye off a single run.

Usage:
    python3 tools/gen_link_schedule.py --profile gilbert_elliott \\
        --seed 42 --duration 900 --out experiment_runs/<run>/link_schedule.json

No ROS imports -- runnable and inspectable outside the container.
"""

import argparse
import json
import random
import sys

SCHEDULE_VERSION = 1

# Defaults come from the measured regime boundaries documented in
# experiment.conf's BANDWIDTH_KBPS block, not from round numbers:
#   >= ~80 kbps : nothing shed at all (one robot's encoder offers ~81 kbps)
#   ~20-40      : coarse + mid detail
#   <  ~19      : the COARSE layer alone saturates the link
# "good" is the no-shedding point and "bad" is below the coarse-saturation
# floor, because the bad state has to genuinely starve the link or there is
# nothing for the recovery transient to recover FROM.
DEFAULT_GOOD_KBPS = 80.0
DEFAULT_BAD_KBPS = 15.0
DEFAULT_GOOD_DWELL_S = 90.0
DEFAULT_BAD_DWELL_S = 30.0

# Floor on any segment's length. An exponential draw will happily produce a
# 0.4s blip, and anything at that scale sits BELOW the stack's own time
# constants -- max_queue_seconds=4.0, manifest_min_interval_s=3.0, the
# encoder's send_rate_hz=1 tick -- so what you would be measuring is the relay
# queue ringing, not the transport's behavior. Enforced as a hard floor on
# every segment without distorting the requested mean dwell (see _dwell).
DEFAULT_MIN_DWELL_S = 15.0

# Hold the good state for this long at the start of every run before the first
# transition. Same hazard the existing IMPAIRMENT_DELAY_S (experiment.conf) and
# set_wifi_bandwidth.sh --delay-s guards already exist for: DDS discovery needs
# several SPDP announce/response round-trips, FastDDS does not reliably retry a
# handshake that failed at startup, and a pair that never discovered each other
# stays silently unmatched for the rest of the process's life -- which is
# indistinguishable from "the link was too slow" unless you go read the proxy's
# rcvd counters. Never open a run in the bad state.
DEFAULT_WARMUP_S = 60.0

# staircase rungs: the documented regime boundaries themselves, walked down and
# back up so each regime is visited twice (once while draining a backlog built
# at a higher rate, once while filling one at a lower rate -- those are not the
# same operating point, and the difference is exactly the queue behavior under
# test).
STAIRCASE_RUNGS_KBPS = [80.0, 50.0, 30.0, 15.0, 30.0, 50.0, 80.0]


def _dwell(rng, mean_s, min_s):
    """Draw one segment length: a shifted exponential with mean exactly mean_s.

    The obvious implementation -- draw Exp(mean_s), resample anything under the
    floor -- is WRONG in a way that quietly misreports the experiment. By
    memorylessness, an exponential conditioned on X >= m is distributed exactly
    as m + Exp(same rate), so rejection sampling inflates the realized mean by
    a full min_s: ask for a 30s mean outage with a 15s floor and you actually
    get 45s (measured, not theorized). The knob would then not mean what the
    conf file says it means, and the duty cycle in the run notes would be wrong.

    So draw the excess over the floor with mean (mean_s - min_s) instead. Same
    left-truncated-exponential shape, same hard floor, but E[dwell] == mean_s.
    """
    if mean_s <= 0.0:
        return max(min_s, 0.0)
    if mean_s <= min_s:
        # No room for an exponential tail above the floor -- the distribution
        # collapses to the floor itself. A caller who sets min_dwell above the
        # mean dwell has asked for a fixed-period square wave; give them that
        # rather than silently stretching their mean.
        return min_s
    return min_s + rng.expovariate(1.0 / (mean_s - min_s))


def gen_static(bandwidth_kbps, duration_s):
    return [{"t": 0.0, "kbps": float(bandwidth_kbps), "state": "static"}]


def gen_gilbert_elliott(
    duration_s, seed, good_kbps, bad_kbps,
    good_dwell_s, bad_dwell_s, min_dwell_s, warmup_s,
):
    """Two-state Markov link, sampled to completion up front.

    Note the whole chain is realized here, not stepped live -- see rule 1 in the
    module docstring.
    """
    rng = random.Random(seed)
    segments = [{"t": 0.0, "kbps": float(good_kbps), "state": "good"}]

    t = float(warmup_s)
    state = "good"
    while t < duration_s:
        # Leaving whichever state we are in -> flip, then draw the new state's
        # dwell to find the NEXT transition time.
        state = "bad" if state == "good" else "good"
        kbps = bad_kbps if state == "bad" else good_kbps
        segments.append({"t": round(t, 3), "kbps": float(kbps), "state": state})
        mean = bad_dwell_s if state == "bad" else good_dwell_s
        t += _dwell(rng, mean, min_dwell_s)

    # A run that ends mid-outage would confound "vxch never recovered" with
    # "the run stopped before it had the chance to". If the final segment is
    # bad and there is not room for a full min-dwell recovery window after it,
    # drop it -- better to under-sample outages than to end on an unresolved
    # one.
    if len(segments) > 1 and segments[-1]["state"] == "bad":
        if duration_s - segments[-1]["t"] < min_dwell_s:
            segments.pop()

    return segments


def gen_staircase(duration_s, warmup_s, rungs=None):
    """Even dwell on each rung across whatever time is left after warmup."""
    rungs = rungs or STAIRCASE_RUNGS_KBPS
    segments = [{"t": 0.0, "kbps": float(rungs[0]), "state": "staircase"}]
    usable = max(duration_s - warmup_s, 0.0)
    step = usable / len(rungs) if rungs else 0.0
    for i, kbps in enumerate(rungs):
        t = warmup_s + i * step
        if i == 0:
            continue  # already covered by the warmup hold at rungs[0]
        segments.append({"t": round(t, 3), "kbps": float(kbps), "state": "staircase"})
    return segments


def generate(
    profile, duration_s, seed=-1, bandwidth_kbps=0.0,
    good_kbps=DEFAULT_GOOD_KBPS, bad_kbps=DEFAULT_BAD_KBPS,
    good_dwell_s=DEFAULT_GOOD_DWELL_S, bad_dwell_s=DEFAULT_BAD_DWELL_S,
    min_dwell_s=DEFAULT_MIN_DWELL_S, warmup_s=DEFAULT_WARMUP_S,
):
    if profile == "static":
        segments = gen_static(bandwidth_kbps, duration_s)
    elif profile == "gilbert_elliott":
        if seed < 0:
            raise ValueError(
                "gilbert_elliott needs a non-negative seed: the schedule has to "
                "replay identically across baseline/zstd/vxch or the arms did "
                "not see the same conditions. Set RANDOM_SEED >= 0.")
        segments = gen_gilbert_elliott(
            duration_s, seed, good_kbps, bad_kbps,
            good_dwell_s, bad_dwell_s, min_dwell_s, warmup_s)
    elif profile == "staircase":
        segments = gen_staircase(duration_s, warmup_s)
    else:
        raise ValueError(f"unknown profile '{profile}'")

    return {
        "version": SCHEDULE_VERSION,
        "profile": profile,
        "seed": seed,
        "duration_s": float(duration_s),
        "params": {
            "bandwidth_kbps": float(bandwidth_kbps),
            "good_kbps": float(good_kbps),
            "bad_kbps": float(bad_kbps),
            "good_dwell_s": float(good_dwell_s),
            "bad_dwell_s": float(bad_dwell_s),
            "min_dwell_s": float(min_dwell_s),
            "warmup_s": float(warmup_s),
        },
        "segments": segments,
    }


def describe(schedule):
    """One line per segment, plus a duty-cycle summary."""
    segs = schedule["segments"]
    duration = schedule["duration_s"]
    lines = []
    time_at = {}
    for i, s in enumerate(segs):
        end = segs[i + 1]["t"] if i + 1 < len(segs) else duration
        held = max(end - s["t"], 0.0)
        time_at[s["state"]] = time_at.get(s["state"], 0.0) + held
        lines.append(f"  t={s['t']:8.1f}s  {s['kbps']:6.1f} kbps  {s['state']} ({held:.1f}s)")
    lines.append(f"  -- {len(segs)} segments over {duration:.0f}s")
    for state, held in sorted(time_at.items()):
        lines.append(f"  -- {state}: {held:.0f}s ({100.0 * held / duration:.0f}%)")
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--profile", default="static",
                   choices=["static", "gilbert_elliott", "staircase"])
    p.add_argument("--duration", type=float, required=True,
                   help="run length in seconds the schedule must cover")
    p.add_argument("--seed", type=int, default=-1)
    p.add_argument("--bandwidth-kbps", type=float, default=0.0,
                   help="static profile only: the constant rate")
    p.add_argument("--good-kbps", type=float, default=DEFAULT_GOOD_KBPS)
    p.add_argument("--bad-kbps", type=float, default=DEFAULT_BAD_KBPS)
    p.add_argument("--good-dwell-s", type=float, default=DEFAULT_GOOD_DWELL_S)
    p.add_argument("--bad-dwell-s", type=float, default=DEFAULT_BAD_DWELL_S)
    p.add_argument("--min-dwell-s", type=float, default=DEFAULT_MIN_DWELL_S)
    p.add_argument("--warmup-s", type=float, default=DEFAULT_WARMUP_S)
    p.add_argument("--out", help="write JSON here (default: stdout)")
    p.add_argument("--describe", action="store_true",
                   help="also print a human-readable trace to stderr")
    args = p.parse_args()

    try:
        schedule = generate(
            args.profile, args.duration, seed=args.seed,
            bandwidth_kbps=args.bandwidth_kbps,
            good_kbps=args.good_kbps, bad_kbps=args.bad_kbps,
            good_dwell_s=args.good_dwell_s, bad_dwell_s=args.bad_dwell_s,
            min_dwell_s=args.min_dwell_s, warmup_s=args.warmup_s)
    except ValueError as e:
        print(f"gen_link_schedule: {e}", file=sys.stderr)
        return 2

    # sort_keys so the same inputs give a byte-identical file across arms --
    # that byte-identity is the fairness check (see the plan's verification
    # step 2), so it must not depend on dict ordering.
    text = json.dumps(schedule, indent=2, sort_keys=True)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text + "\n")
        print(f"wrote {args.out} ({len(schedule['segments'])} segments, "
              f"profile={args.profile}, seed={args.seed})", file=sys.stderr)
    else:
        print(text)

    if args.describe:
        print(describe(schedule), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
