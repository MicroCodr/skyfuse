"""Scores the tracker against the truth (which the tracker itself never
sees). Tracks get matched to the nearest truth aircraft within 2km,
closest pairs first, then RMSE over the matched ones.
"""
import math
from typing import List

from . import config
from .fusion.tracker import Track
from .simulation import Aircraft


def evaluate(tracks: List[Track], aircraft: List[Aircraft]) -> dict:
    pairs = []
    for tr in tracks:
        tx, ty = tr.ekf.x[0], tr.ekf.x[1]
        for ac in aircraft:
            d = math.hypot(tx - ac.x, ty - ac.y)
            if d < config.TRUTH_MATCH_RADIUS:
                pairs.append((d, tr.id, ac.id))

    pairs.sort()
    used_tracks, used_truth, matched = set(), set(), []
    for d, tid, aid in pairs:
        if tid in used_tracks or aid in used_truth:
            continue
        used_tracks.add(tid)
        used_truth.add(aid)
        matched.append(d)

    rmse = math.sqrt(sum(d * d for d in matched) / len(matched)) if matched else 0.0
    return {
        'rmse': round(rmse, 1),
        'matched': len(matched),
        'missed_targets': len(aircraft) - len(matched),
        'false_tracks': len(tracks) - len(matched),
    }
