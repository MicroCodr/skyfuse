"""Global nearest-neighbor data association.

Given the current set of tracks and one sensor scan's detections, decide
which detection belongs to which track. Two stages:

1. Gating — a detection is only a candidate for a track if its squared
   Mahalanobis distance falls inside a chi-square gate (99%, 2 DOF). This
   accounts for *both* track uncertainty and sensor noise, so an uncertain
   coasting track has a wide-open gate while a well-fed track is picky.

2. Assignment — the Hungarian algorithm (scipy's linear_sum_assignment)
   finds the globally optimal one-to-one pairing. Greedy nearest-neighbor
   fails when targets cross; global assignment handles it.
"""
from typing import List, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

BIG = 1e9


def associate(tracks: list, detections: list,
              gate: float) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """Returns (matches, unmatched_track_idxs, unmatched_detection_idxs).

    matches is a list of (track_index, detection_index) pairs.
    """
    if not tracks or not detections:
        return [], list(range(len(tracks))), list(range(len(detections)))

    cost = np.full((len(tracks), len(detections)), BIG)
    for i, tr in enumerate(tracks):
        for j, det in enumerate(detections):
            d2 = tr.ekf.mahalanobis2(det)
            if d2 < gate:
                cost[i, j] = d2

    rows, cols = linear_sum_assignment(cost)
    matches = [(i, j) for i, j in zip(rows, cols) if cost[i, j] < BIG]
    matched_t = {i for i, _ in matches}
    matched_d = {j for _, j in matches}
    unmatched_t = [i for i in range(len(tracks)) if i not in matched_t]
    unmatched_d = [j for j in range(len(detections)) if j not in matched_d]
    return matches, unmatched_t, unmatched_d
