"""The EKF should converge on a constant-velocity target and stay honest
about its uncertainty."""
import math
import random

import numpy as np
import pytest

from skyfuse.fusion.ekf import EKF
from skyfuse.sensors import Detection


def radar_det(t, tx, ty, rng, sigma_r=80.0, sigma_th=math.radians(0.4)):
    r = math.hypot(tx, ty) + rng.gauss(0, sigma_r)
    th = math.atan2(ty, tx) + rng.gauss(0, sigma_th)
    return Detection('radar', t, 'polar', np.array([r, th]),
                     np.diag([sigma_r ** 2, sigma_th ** 2]), (0.0, 0.0))


def test_converges_on_straight_target():
    rng = random.Random(7)
    x, y, vx, vy = 10_000.0, 5_000.0, 200.0, -50.0
    ekf = EKF.from_detection(radar_det(0.0, x, y, rng), vel_sigma=200.0)

    for k in range(1, 30):
        t = float(k)
        x += vx; y += vy
        ekf.predict(1.0, sigma_a=8.0)
        ekf.update(radar_det(t, x, y, rng))

    pos_err = math.hypot(ekf.x[0] - x, ekf.x[1] - y)
    vel_err = math.hypot(ekf.x[2] - vx, ekf.x[3] - vy)
    assert pos_err < 300, f'position error too large: {pos_err:.0f} m'
    assert vel_err < 40, f'velocity error too large: {vel_err:.0f} m/s'


def test_update_shrinks_uncertainty():
    rng = random.Random(1)
    ekf = EKF.from_detection(radar_det(0.0, 20_000, 0, rng), vel_sigma=200.0)
    ekf.predict(1.0, sigma_a=8.0)
    before = np.trace(ekf.P[:2, :2])
    ekf.update(radar_det(1.0, 20_000, 0, rng))
    after = np.trace(ekf.P[:2, :2])
    assert after < before


def test_prediction_grows_uncertainty():
    rng = random.Random(2)
    ekf = EKF.from_detection(radar_det(0.0, 20_000, 0, rng), vel_sigma=200.0)
    before = np.trace(ekf.P[:2, :2])
    ekf.predict(3.0, sigma_a=8.0)
    assert np.trace(ekf.P[:2, :2]) > before


def test_bearing_wrap_across_pi():
    """A target sitting just past the +/-pi bearing seam must not produce a
    huge bogus innovation."""
    rng = random.Random(3)
    # target behind the sensor: bearing ~ pi
    det0 = radar_det(0.0, -20_000, 100, rng)
    ekf = EKF.from_detection(det0, vel_sigma=200.0)
    # measurement on the other side of the seam (bearing ~ -pi)
    det1 = radar_det(1.0, -20_000, -100, rng)
    nu, _, _ = ekf.innovation(det1)
    assert abs(nu[1]) < 0.1, 'bearing innovation was not wrapped'


def test_covariance_stays_symmetric():
    rng = random.Random(4)
    ekf = EKF.from_detection(radar_det(0.0, 15_000, -8_000, rng), vel_sigma=200.0)
    for k in range(1, 50):
        ekf.predict(1.0, sigma_a=8.0)
        ekf.update(radar_det(float(k), 15_000, -8_000, rng))
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-6)
    assert np.all(np.linalg.eigvalsh(ekf.P) > 0), 'P lost positive definiteness'
