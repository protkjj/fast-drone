"""Legacy import-name compatibility; defaults to the FIXED v2 aircraft, not 8 kg.

Prefer baseline_params() for a fresh dictionary. Do not mutate this singleton.
"""
from models.team_light.control.baseline_v2 import baseline_params
vehicle_params = baseline_params()
