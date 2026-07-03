"""Matching detections to tracks.

Gate first (mahalanobis distance inside a chi-square threshold), then use
scipy's hungarian algorithm for the assignment. Tried greedy nearest
neighbor first but it grabs the wrong detection when two targets cross,
global assignment fixes that.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment

BIG = 1e9


def associate(tracks, detections, gate):
    """Returns (matches, unmatched_track_idxs, unmatched_det_idxs)."""
    if not tracks or not detections:
        return [], list(range(len(tracks))), list(range(len(detections)))

    # TODO this is O(tracks * dets), fine at this scale
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
