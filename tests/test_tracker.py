"""Track lifecycle: confirm on consistent evidence, coast on silence,
drop when stale, and never confirm scattered clutter."""
import random

import numpy as np

from skyfuse import config
from skyfuse.fusion.tracker import TrackManager, TrackStatus
from skyfuse.sensors import Detection


def cart_det(t, x, y, sigma=30.0):
    return Detection('adsb', t, 'cart', np.array([x, y]), np.eye(2) * sigma ** 2)


def feed_target(mgr, t0, n, x0=0.0, y0=0.0, vx=200.0):
    for k in range(n):
        t = t0 + k
        mgr.process_scan(t, [cart_det(t, x0 + vx * k, y0)])
    return t0 + n - 1


def test_track_confirms():
    mgr = TrackManager()
    feed_target(mgr, 0.0, config.CONFIRM_HITS)
    assert len(mgr.confirmed) == 1
    assert mgr.confirmed[0].status is TrackStatus.CONFIRMED


def test_track_coasts_then_drops():
    mgr = TrackManager()
    t = feed_target(mgr, 0.0, 6)

    mgr.process_scan(t + config.COAST_AFTER + 0.5, [])
    assert mgr.confirmed[0].status is TrackStatus.COASTING

    mgr.process_scan(t + config.DROP_CONFIRMED_AFTER + 0.5, [])
    assert mgr.tracks == []


def test_coasting_track_recovers():
    mgr = TrackManager()
    t = feed_target(mgr, 0.0, 6, vx=0.0)
    mgr.process_scan(t + 3.0, [])
    assert mgr.confirmed[0].status is TrackStatus.COASTING
    mgr.process_scan(t + 3.5, [cart_det(t + 3.5, 0.0, 0.0)])
    assert mgr.confirmed[0].status is TrackStatus.CONFIRMED


def test_scattered_clutter_never_confirms():
    rng = random.Random(11)
    mgr = TrackManager()
    for k in range(30):
        t = float(k)
        clutter = [cart_det(t, rng.uniform(-40_000, 40_000),
                            rng.uniform(-40_000, 40_000))
                   for _ in range(3)]
        mgr.process_scan(t, clutter)
    assert len(mgr.confirmed) == 0, 'random clutter produced a confirmed track'


def test_two_targets_two_tracks():
    mgr = TrackManager()
    for k in range(8):
        t = float(k)
        mgr.process_scan(t, [
            cart_det(t, 200.0 * k, 0.0),
            cart_det(t, -10_000 - 150.0 * k, 5_000),
        ])
    assert len(mgr.confirmed) == 2
