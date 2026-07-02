"""Gating + global assignment behavior."""
import math
import random

import numpy as np

from skyfuse import config
from skyfuse.fusion.association import associate
from skyfuse.fusion.tracker import Track
from skyfuse.sensors import Detection


def cart_det(t, x, y, sigma=30.0):
    return Detection('adsb', t, 'cart', np.array([x, y]), np.eye(2) * sigma ** 2)


def make_track(x, y, t=0.0):
    return Track(cart_det(t, x, y), t)


def test_obvious_pairing():
    tracks = [make_track(0, 0), make_track(10_000, 0)]
    dets = [cart_det(1.0, 10_020, 30), cart_det(1.0, 15, -20)]
    matches, ut, ud = associate(tracks, dets, config.GATE_CHI2)
    assert sorted(matches) == [(0, 1), (1, 0)]
    assert not ut and not ud


def test_far_detection_is_not_gated_in():
    tracks = [make_track(0, 0)]
    dets = [cart_det(1.0, 20_000, 20_000)]
    matches, ut, ud = associate(tracks, dets, config.GATE_CHI2)
    assert matches == []
    assert ut == [0] and ud == [0]


def test_global_beats_greedy():
    """Two tracks, two detections placed so a greedy matcher would steal the
    wrong one. Hungarian minimizes total cost and gets both right."""
    ta, tb = make_track(0, 0), make_track(400, 0)
    for tr in (ta, tb):                    # predict to scan time, as the manager does
        tr.predict_to(1.0)
    # d0 sits between the tracks but closer to B; d1 is right of B.
    d0, d1 = cart_det(1.0, 260, 0), cart_det(1.0, 430, 0)
    matches, _, _ = associate([ta, tb], [d0, d1], config.GATE_CHI2)
    assert sorted(matches) == [(0, 0), (1, 1)]


def test_empty_inputs():
    assert associate([], [], config.GATE_CHI2) == ([], [], [])
    t = [make_track(0, 0)]
    m, ut, ud = associate(t, [], config.GATE_CHI2)
    assert m == [] and ut == [0] and ud == []
