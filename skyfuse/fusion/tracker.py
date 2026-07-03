"""Track manager. Owns all the tracks and their lifecycle.

Each sensor scan comes in through process_scan whenever it happens - the
tracks get predicted forward to the scan's timestamp before matching, so
sensors with different rates all fuse into the same tracks.

Lifecycle: tentative -> confirmed (4 hits) -> coasting (silent for a bit)
-> dropped. Clutter makes tentative tracks all the time but they never
get 4 consistent hits so they just die off.
"""
import itertools
from collections import Counter
from enum import Enum

from .. import config
from .association import associate
from .ekf import EKF


class TrackStatus(Enum):
    TENTATIVE = 'tentative'
    CONFIRMED = 'confirmed'
    COASTING = 'coasting'


class Track:
    _ids = itertools.count(1)

    def __init__(self, det, t):
        self.id = next(Track._ids)
        self.ekf = EKF.from_detection(det, config.INIT_VEL_SIGMA)
        self.status = TrackStatus.TENTATIVE
        self.time = t          # timestamp the state is valid for
        self.last_update = t
        self.created = t
        self.hits = 1
        self.sensor_counts = Counter({det.sensor: 1})

    def predict_to(self, t):
        dt = t - self.time
        if dt > 0:
            self.ekf.predict(dt, config.PROCESS_NOISE_ACCEL)
            self.time = t

    def update(self, det, t):
        self.ekf.update(det)
        self.hits += 1
        self.last_update = t
        self.sensor_counts[det.sensor] += 1
        if self.status is TrackStatus.TENTATIVE and self.hits >= config.CONFIRM_HITS:
            self.status = TrackStatus.CONFIRMED
        elif self.status is TrackStatus.COASTING:
            # got a detection again, back to normal
            self.status = TrackStatus.CONFIRMED

    def silent_for(self, t):
        return t - self.last_update


class TrackManager:
    def __init__(self):
        self.tracks = []

    def process_scan(self, t, detections):
        # predict everything to the scan time first, important! gating
        # against stale states gives wrong distances
        for tr in self.tracks:
            tr.predict_to(t)

        matches, _, unmatched_dets = associate(
            self.tracks, detections, config.GATE_CHI2)

        for ti, di in matches:
            self.tracks[ti].update(detections[di], t)

        # every unmatched detection could be a new target
        for di in unmatched_dets:
            self.tracks.append(Track(detections[di], t))

        self._age(t)

    def _age(self, t):
        keep = []
        for tr in self.tracks:
            silent = tr.silent_for(t)
            if tr.status is TrackStatus.TENTATIVE:
                if silent <= config.DROP_TENTATIVE_AFTER:
                    keep.append(tr)
            else:
                if silent > config.DROP_CONFIRMED_AFTER:
                    continue
                if silent > config.COAST_AFTER:
                    tr.status = TrackStatus.COASTING
                keep.append(tr)
        self.tracks = keep

    @property
    def confirmed(self):
        return [t for t in self.tracks
                if t.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)]
