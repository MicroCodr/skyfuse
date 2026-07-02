"""Tunable parameters for the simulation and fusion engine.

Everything is SI units (meters, seconds, radians) unless noted.
"""
import math

# --- world -------------------------------------------------------------
AREA_HALF = 50_000.0          # surveillance area is a 100 x 100 km square
SIM_DT = 0.1                  # truth propagation step
NUM_AIRCRAFT = 8
COOPERATIVE_FRACTION = 0.6    # fraction of aircraft with a transponder

# --- aircraft dynamics -------------------------------------------------
SPEED_RANGE = (150.0, 300.0)          # m/s
TURN_RATE_RANGE = (math.radians(1.0), math.radians(3.0))   # rad/s
MANEUVER_INTERVAL = (15.0, 40.0)      # seconds between maneuvers
MANEUVER_DURATION = (5.0, 15.0)

# --- fusion ------------------------------------------------------------
PROCESS_NOISE_ACCEL = 8.0     # sigma_a for the CV model (targets maneuver)
INIT_VEL_SIGMA = 200.0        # velocity uncertainty for a brand-new track
GATE_CHI2 = 9.21              # chi-square 99% gate, 2 degrees of freedom

CONFIRM_HITS = 4              # detections needed to confirm a track
COAST_AFTER = 2.5             # seconds without update -> COASTING
DROP_CONFIRMED_AFTER = 6.0    # seconds without update -> dropped
DROP_TENTATIVE_AFTER = 2.5

# --- metrics -----------------------------------------------------------
TRUTH_MATCH_RADIUS = 2_000.0  # a track within 2 km of a truth target counts
