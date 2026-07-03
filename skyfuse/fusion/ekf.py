"""EKF for a 2D target. State is [px, py, vx, vy].

Handles two measurement types so different sensors can update the same
track: cartesian (x, y) from ADS-B and polar (range, bearing) from
radar/EO. The polar one is the nonlinear case, that's the "extended" part.
"""
import math

import numpy as np


class EKF:
    def __init__(self, x0, P0):
        self.x = x0.astype(float).copy()
        self.P = P0.astype(float).copy()

    def predict(self, dt, sigma_a):
        # constant velocity model, process noise = white noise acceleration
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

    def innovation_cart(self, z, R):
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        nu = z - H @ self.x
        S = H @ self.P @ H.T + R
        return nu, S, H

    def innovation_polar(self, z, R, sensor_pos):
        dx = self.x[0] - sensor_pos[0]
        dy = self.x[1] - sensor_pos[1]
        r = math.hypot(dx, dy)
        r = max(r, 1e-6)
        h = np.array([r, math.atan2(dy, dx)])
        # jacobian of h(x) around the current estimate
        H = np.array([[dx / r, dy / r, 0, 0],
                      [-dy / r ** 2, dx / r ** 2, 0, 0]])
        nu = z - h
        # wrap the bearing residual! learned this the hard way, without it
        # tracks explode when a target crosses the +/-pi line
        nu[1] = _wrap(nu[1])
        S = H @ self.P @ H.T + R
        return nu, S, H

    def innovation(self, det):
        if det.kind == 'cart':
            return self.innovation_cart(det.z, det.R)
        return self.innovation_polar(det.z, det.R, det.sensor_pos)

    def mahalanobis2(self, det):
        nu, S, _ = self.innovation(det)
        return float(nu @ np.linalg.solve(S, nu))

    def update(self, det):
        nu, S, H = self.innovation(det)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        # Joseph form update, keeps P symmetric (the simple form drifts)
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ det.R @ K.T

    @classmethod
    def from_detection(cls, det, vel_sigma):
        """Start a track from one detection. Position from the measurement,
        velocity unknown so zero with a big covariance."""
        px, py = det.position()
        x0 = np.array([px, py, 0.0, 0.0])
        if det.kind == 'cart':
            P_pos = det.R.copy()
        else:
            # convert the polar noise to cartesian with the jacobian, so a
            # far away detection starts with appropriately wide uncertainty
            r, th = float(det.z[0]), float(det.z[1])
            J = np.array([[math.cos(th), -r * math.sin(th)],
                          [math.sin(th), r * math.cos(th)]])
            P_pos = J @ det.R @ J.T
        P0 = np.zeros((4, 4))
        P0[:2, :2] = P_pos
        P0[2, 2] = P0[3, 3] = vel_sigma ** 2
        return cls(x0, P0)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
