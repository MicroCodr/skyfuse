"""Extended Kalman filter for a 2D constant-velocity target.

State vector: x = [px, py, vx, vy]

Two measurement models are supported, which is what makes this a *fusion*
filter — measurements from different sensors flow through the same state:

* cartesian  z = [px, py]           (linear — ADS-B)
* polar      z = [range, bearing]   (nonlinear — radar, EO/IR)

The polar update linearizes h(x) around the current estimate (the "E" in
EKF) and wraps the bearing innovation to [-pi, pi] — skipping that wrap is
the classic bug that makes tracks explode when a target crosses the -pi/pi
boundary behind the sensor.
"""
import math
from typing import Tuple

import numpy as np


class EKF:
    def __init__(self, x0: np.ndarray, P0: np.ndarray):
        self.x = x0.astype(float).copy()
        self.P = P0.astype(float).copy()

    # --- prediction ------------------------------------------------------

    def predict(self, dt: float, sigma_a: float) -> None:
        """Propagate with a constant-velocity model + white-noise acceleration."""
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        q = sigma_a ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                          [0, dt4 / 4, 0, dt3 / 2],
                          [dt3 / 2, 0, dt2, 0],
                          [0, dt3 / 2, 0, dt2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    # --- measurement models ----------------------------------------------

    def innovation_cart(self, z: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        nu = z - H @ self.x
        S = H @ self.P @ H.T + R
        return nu, S, H

    def innovation_polar(self, z: np.ndarray, R: np.ndarray,
                         sensor_pos: Tuple[float, float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        dx = self.x[0] - sensor_pos[0]
        dy = self.x[1] - sensor_pos[1]
        r = math.hypot(dx, dy)
        r = max(r, 1e-6)
        h = np.array([r, math.atan2(dy, dx)])
        H = np.array([[dx / r, dy / r, 0, 0],
                      [-dy / r ** 2, dx / r ** 2, 0, 0]])
        nu = z - h
        nu[1] = _wrap(nu[1])                    # bearing innovation wrap
        S = H @ self.P @ H.T + R
        return nu, S, H

    def innovation(self, det) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if det.kind == 'cart':
            return self.innovation_cart(det.z, det.R)
        return self.innovation_polar(det.z, det.R, det.sensor_pos)

    def mahalanobis2(self, det) -> float:
        """Squared Mahalanobis distance of a detection from the track."""
        nu, S, _ = self.innovation(det)
        return float(nu @ np.linalg.solve(S, nu))

    # --- update ------------------------------------------------------------

    def update(self, det) -> None:
        nu, S, H = self.innovation(det)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        # Joseph form: numerically stable, keeps P symmetric positive-definite
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ det.R @ K.T

    # --- init helpers --------------------------------------------------------

    @classmethod
    def from_detection(cls, det, vel_sigma: float) -> 'EKF':
        """Start a track from a single detection: measured position,
        unknown velocity (zero mean, large covariance)."""
        px, py = det.position()
        x0 = np.array([px, py, 0.0, 0.0])
        if det.kind == 'cart':
            P_pos = det.R.copy()
        else:
            # push polar noise through the polar->cartesian jacobian
            r, th = float(det.z[0]), float(det.z[1])
            J = np.array([[math.cos(th), -r * math.sin(th)],
                          [math.sin(th), r * math.cos(th)]])
            P_pos = J @ det.R @ J.T
        P0 = np.zeros((4, 4))
        P0[:2, :2] = P_pos
        P0[2, 2] = P0[3, 3] = vel_sigma ** 2
        return cls(x0, P0)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
