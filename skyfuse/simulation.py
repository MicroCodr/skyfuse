"""Ground-truth world model: aircraft flying waypoint-free patrol patterns.

Aircraft fly straight at constant speed, occasionally performing coordinated
turns. If one approaches the edge of the surveillance area it is commanded
to turn back toward the center, so the scenario runs forever.
"""
import math
import random
from typing import List

from . import config


class Aircraft:
    def __init__(self, ac_id: int, x: float, y: float, heading: float,
                 speed: float, cooperative: bool, rng: random.Random):
        self.id = ac_id
        self.x = x
        self.y = y
        self.heading = heading
        self.speed = speed
        self.cooperative = cooperative
        self._rng = rng
        self._turn_rate = 0.0
        self._turn_time_left = 0.0
        self._next_maneuver = rng.uniform(*config.MANEUVER_INTERVAL)

    @property
    def vx(self) -> float:
        return self.speed * math.cos(self.heading)

    @property
    def vy(self) -> float:
        return self.speed * math.sin(self.heading)

    def step(self, dt: float) -> None:
        rng = self._rng

        # boundary avoidance takes priority over random maneuvers
        margin = 6_000.0
        near_edge = (abs(self.x) > config.AREA_HALF - margin or
                     abs(self.y) > config.AREA_HALF - margin)
        if near_edge:
            to_center = math.atan2(-self.y, -self.x)
            err = _wrap(to_center - self.heading)
            if abs(err) > 0.2:
                self._turn_rate = math.copysign(config.TURN_RATE_RANGE[1], err)
                self._turn_time_left = dt * 2
        elif self._turn_time_left <= 0:
            self._next_maneuver -= dt
            if self._next_maneuver <= 0:
                self._turn_rate = (rng.choice([-1, 1]) *
                                   rng.uniform(*config.TURN_RATE_RANGE))
                self._turn_time_left = rng.uniform(*config.MANEUVER_DURATION)
                self._next_maneuver = rng.uniform(*config.MANEUVER_INTERVAL)

        if self._turn_time_left > 0:
            self.heading = _wrap(self.heading + self._turn_rate * dt)
            self._turn_time_left -= dt

        self.x += self.vx * dt
        self.y += self.vy * dt


class Simulation:
    def __init__(self, seed: int = None):
        self._rng = random.Random(seed)
        self.time = 0.0
        self.aircraft: List[Aircraft] = []
        for i in range(config.NUM_AIRCRAFT):
            self.aircraft.append(self._spawn(i))

    def _spawn(self, ac_id: int) -> Aircraft:
        rng = self._rng
        r = config.AREA_HALF * 0.8
        return Aircraft(
            ac_id=ac_id,
            x=rng.uniform(-r, r),
            y=rng.uniform(-r, r),
            heading=rng.uniform(-math.pi, math.pi),
            speed=rng.uniform(*config.SPEED_RANGE),
            cooperative=rng.random() < config.COOPERATIVE_FRACTION,
            rng=rng,
        )

    def step(self, dt: float) -> None:
        self.time += dt
        for ac in self.aircraft:
            ac.step(dt)


def _wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
