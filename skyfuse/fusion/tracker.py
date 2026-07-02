"""Track manager: owns track lifecycle and fuses asynchronous sensor scans.

Every sensor scan (radar sweep, EO frame, ADS-B batch) is pushed through
`process_scan`. Because the EKF predicts each track forward to the scan
timestamp before updating, sensors with different rates and different
measurement models all fuse into the same state estimate — that is the
whole trick of asynchronous multi-sensor fusion.

Lifecycle (M-of-N style):

    TENTATIVE --4 hits--> CONFIRMED --2.5 s silent--> COASTING
        |                                                  |
        +--2.5 s silent--> dropped        6 s silent --> dropped

Clutter creates tentative tracks constantly; they die quietly before
confirmation because random false alarms don't line up scan after scan.
"""
import itertools
from collections import Counter
from enum import Enum
from typing import List

from .. import config
from .association import associate
from .ekf import EKF


class TrackStatus(Enum):
    TENTATIVE = 'tentative'
    CONFIRMED = 'confirmed'
    COASTING = 'coasting'


class Track:
    _ids = itertools.count(1)

    def __init__(self, det, t: float):
        self.id = next(Track._ids)
        self.ekf = EKF.from_detection(det, config.INIT_VEL_SIGMA)
        self.status = TrackStatus.TENTATIVE
        self.time = t                    # time the state estimate is valid for
        self.last_update = t
        self.created = t
        self.hits = 1
        self.sensor_counts = Counter({det.sensor: 1})

    def predict_to(self, t: float) -> None:
        dt = t - self.time
        if dt > 0:
            self.ekf.predict(dt, config.PROCESS_NOISE_ACCEL)
            self.time = t

    def update(self, det, t: float) -> None:
        self.ekf.update(det)
        self.hits += 1
        self.last_update = t
        self.sensor_counts[det.sensor] += 1
        if self.status is TrackStatus.TENTATIVE and self.hits >= config.CONFIRM_HITS:
            self.status = TrackStatus.CONFIRMED
        elif self.status is TrackStatus.COASTING:
            self.status = TrackStatus.CONFIRMED

    def silent_for(self, t: float) -> float:
        return t - self.last_update


class TrackManager:
    def __init__(self):
        self.tracks: List[Track] = []

    def process_scan(self, t: float, detections: list) -> None:
        # 1. bring every track's state up to the scan time
        for tr in self.tracks:
            tr.predict_to(t)

        # 2. globally assign detections to tracks
        matches, _, unmatched_dets = associate(
            self.tracks, detections, config.GATE_CHI2)

        # 3. update matched tracks
        for ti, di in matches:
            self.tracks[ti].update(detections[di], t)

        # 4. unmatched detections seed new tentative tracks
        for di in unmatched_dets:
            self.tracks.append(Track(detections[di], t))

        # 5. lifecycle transitions
        self._age(t)

    def _age(self, t: float) -> None:
        keep = []
        for tr in self.tracks:
            silent = tr.silent_for(t)
            if tr.status is TrackStatus.TENTATIVE:
                if silent <= config.DROP_TENTATIVE_AFTER:
                    keep.append(tr)
            else:
                if silent > config.DROP_CONFIRMED_AFTER:
                    continue
                tr.status = (TrackStatus.COASTING if silent > config.COAST_AFTER
                             else tr.status)
                keep.append(tr)
        self.tracks = keep

    @property
    def confirmed(self) -> List[Track]:
        return [t for t in self.tracks
                if t.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)]
