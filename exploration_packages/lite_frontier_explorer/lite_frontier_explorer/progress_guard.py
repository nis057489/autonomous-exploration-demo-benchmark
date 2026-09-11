"""Long-horizon navigation progress checks, independent of ROS."""
import math


class ProgressGuard:
    def __init__(self, timeout_s=45.0, radius_m=1.0, unreachable_checks=4):
        if (not math.isfinite(timeout_s) or timeout_s <= 0
                or not math.isfinite(radius_m) or radius_m <= 0
                or unreachable_checks < 1):
            raise ValueError('Progress guard limits must be positive and finite')
        self.timeout_s = timeout_s
        self.radius_m = radius_m
        self.unreachable_checks = unreachable_checks
        self.reset()

    def reset(self):
        self.anchor = None
        self.since = None
        self.missing = 0

    def update(self, now, xy, reachable=None):
        """Return a failure reason, or None; reachable=None means unassessed.

        Small movements cannot reset the long-horizon timer. Moving outside
        the anchor radius counts as progress even when taking a detour away
        from the goal. Simulation-clock rollback starts a fresh window.
        """
        if self.since is not None and now < self.since:
            self.reset()
        self.missing = self.missing + 1 if reachable is False else 0
        if self.missing >= self.unreachable_checks:
            return 'active goal remained unreachable across consecutive checks'
        if (self.anchor is None
                or math.hypot(xy[0] - self.anchor[0], xy[1] - self.anchor[1]) >= self.radius_m):
            self.anchor = xy
            self.since = now
        if now - self.since >= self.timeout_s:
            return f'failed to leave a {self.radius_m:g}m radius for {self.timeout_s:g}s'
        return None
