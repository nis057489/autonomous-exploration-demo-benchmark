"""Expiring goal reservations; inputs are only messages actually delivered."""
import math


class GoalReservations:
    def __init__(self, ttl=8.0):
        self.ttl = ttl
        self.records = {}

    def receive(self, peer, stamp, goal, now):
        if not math.isfinite(stamp) or stamp > now + .1:
            return
        previous = self.records.get(peer)
        if previous is not None and stamp <= previous[0]:
            return
        if goal is not None and not all(math.isfinite(v) for v in goal):
            return
        self.records[peer] = (stamp, goal)

    def active(self, now):
        return {peer: goal for peer, (stamp, goal) in self.records.items()
                if goal is not None and 0 <= now - stamp <= self.ttl}
