"""The three sensor models. Each one is bad in a different way (radar has
clutter + so-so bearing, EO has great bearing but bad range, ADS-B is
precise but only sees cooperative aircraft) so the fusion actually has
something to do.

Note detections don't carry the aircraft id - figuring out which
measurement belongs to which track is the tracker's job.
"""
import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .simulation import Aircraft


@dataclass
class Detection:
    sensor: str
    time: float
    kind: str                     # 'polar' or 'cart'
    z: np.ndarray                 # (r, theta) or (x, y)
    R: np.ndarray                 # 2x2 measurement noise covariance
    sensor_pos: Tuple[float, float] = (0.0, 0.0)

    def position(self) -> Tuple[float, float]:
        """Cartesian position implied by the measurement (for display/init)."""
        if self.kind == 'cart':
            return float(self.z[0]), float(self.z[1])
        r, th = float(self.z[0]), float(self.z[1])
        return (self.sensor_pos[0] + r * math.cos(th),
                self.sensor_pos[1] + r * math.sin(th))


class Sensor:
    name = 'sensor'
    period = 1.0

    def __init__(self, rng: random.Random):
        self.rng = rng
        self.enabled = True
        self._last_scan = -1e9

    def due(self, t: float) -> bool:
        return self.enabled and t - self._last_scan >= self.period

    def scan(self, t: float, aircraft: List[Aircraft]) -> List[Detection]:
        self._last_scan = t
        dets = []
        for ac in aircraft:
            d = self._observe(t, ac)
            if d is not None:
                dets.append(d)
        dets.extend(self._false_alarms(t))
        return dets

    def _observe(self, t: float, ac: Aircraft) -> Optional[Detection]:
        raise NotImplementedError

    def _false_alarms(self, t: float) -> List[Detection]:
        return []


class Radar(Sensor):
    name = 'radar'
    period = 1.0
    pos = (0.0, 0.0)
    sigma_r = 80.0                        # range noise, m
    sigma_th = math.radians(0.4)          # bearing noise
    max_range = 75_000.0
    pd = 0.9                              # probability of detection
    clutter_rate = 2.0                    # mean false alarms per scan

    def _observe(self, t, ac):
        dx, dy = ac.x - self.pos[0], ac.y - self.pos[1]
        r = math.hypot(dx, dy)
        if r > self.max_range or self.rng.random() > self.pd:
            return None
        z = np.array([r + self.rng.gauss(0, self.sigma_r),
                      math.atan2(dy, dx) + self.rng.gauss(0, self.sigma_th)])
        R = np.diag([self.sigma_r ** 2, self.sigma_th ** 2])
        return Detection(self.name, t, 'polar', z, R, self.pos)

    def _false_alarms(self, t):
        out = []
        n = _poisson(self.rng, self.clutter_rate)
        for _ in range(n):
            r = self.max_range * math.sqrt(self.rng.random()) * 0.75
            th = self.rng.uniform(-math.pi, math.pi)
            R = np.diag([self.sigma_r ** 2, self.sigma_th ** 2])
            out.append(Detection(self.name, t, 'polar',
                                 np.array([r, th]), R, self.pos))
        return out


class ElectroOptical(Sensor):
    """IRST-style sensor: superb angles, terrible range estimates."""
    name = 'eo'
    period = 1.5
    pos = (-30_000.0, 32_000.0)
    sigma_r = 1_500.0
    sigma_th = math.radians(0.05)
    max_range = 60_000.0
    pd = 0.85

    def _observe(self, t, ac):
        dx, dy = ac.x - self.pos[0], ac.y - self.pos[1]
        r = math.hypot(dx, dy)
        if r > self.max_range or self.rng.random() > self.pd:
            return None
        z = np.array([r + self.rng.gauss(0, self.sigma_r),
                      math.atan2(dy, dx) + self.rng.gauss(0, self.sigma_th)])
        R = np.diag([self.sigma_r ** 2, self.sigma_th ** 2])
        return Detection(self.name, t, 'polar', z, R, self.pos)


class AdsB(Sensor):
    """Transponder position reports — precise, but cooperative traffic only."""
    name = 'adsb'
    period = 1.0
    sigma = 30.0
    pd = 0.95

    def _observe(self, t, ac):
        if not ac.cooperative or self.rng.random() > self.pd:
            return None
        z = np.array([ac.x + self.rng.gauss(0, self.sigma),
                      ac.y + self.rng.gauss(0, self.sigma)])
        R = np.eye(2) * self.sigma ** 2
        return Detection(self.name, t, 'cart', z, R)


def _poisson(rng: random.Random, lam: float) -> int:
    """Knuth's algorithm — good enough for small lambda."""
    l = math.exp(-lam)
    k, p = 0, 1.0
    while True:
        p *= rng.random()
        if p <= l:
            return k
        k += 1


def default_sensors(rng: random.Random) -> List[Sensor]:
    return [Radar(rng), ElectroOptical(rng), AdsB(rng)]
