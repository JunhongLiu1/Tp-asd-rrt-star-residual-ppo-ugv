#!/usr/bin/env python3
"""Shared normalisation and Final Score model.

All formal, pilot and multi-goal experiments must import this module.
Normalised values are intentionally NOT capped at 1.0.
Contact is reported separately and does not modify the Final Score.
"""

from dataclasses import dataclass
from typing import Dict


RADIATION_REFERENCE = 10.7134511
TERRAIN_REFERENCE = 12.9489118
EXECUTION_TIME_REFERENCE_S = 89.012

RADIATION_WEIGHT = 0.4
TERRAIN_WEIGHT = 0.4
EXECUTION_TIME_WEIGHT = 0.2


@dataclass(frozen=True)
class FinalScoreResult:
    normalized_radiation: float
    normalized_terrain: float
    normalized_execution_time: float
    radiation_contribution: float
    terrain_contribution: float
    execution_time_contribution: float
    final_score: float

    def as_dict(self) -> Dict[str, float]:
        return {
            "radiation_reference": RADIATION_REFERENCE,
            "terrain_reference": TERRAIN_REFERENCE,
            "execution_time_reference_s": EXECUTION_TIME_REFERENCE_S,
            "radiation_weight": RADIATION_WEIGHT,
            "terrain_weight": TERRAIN_WEIGHT,
            "execution_time_weight": EXECUTION_TIME_WEIGHT,
            "normalized_radiation": self.normalized_radiation,
            "normalized_terrain": self.normalized_terrain,
            "normalized_execution_time": self.normalized_execution_time,
            "radiation_contribution": self.radiation_contribution,
            "terrain_contribution": self.terrain_contribution,
            "execution_time_contribution": self.execution_time_contribution,
            "final_score": self.final_score,
        }



def normalize_metric(value: float, reference: float) -> float:
    """Shared uncapped normalisation used by every experiment.

    Values are constrained only at the lower boundary of zero.
    Values above the reference are allowed to exceed 1.0.
    """
    value = float(value)
    reference = float(reference)

    if reference <= 0.0:
        raise ValueError("reference must be greater than zero")

    return max(value / reference, 0.0)

def calculate_final_score(
    radiation_cost: float,
    terrain_cost: float,
    execution_time_s: float,
) -> FinalScoreResult:
    """Calculate the shared uncapped Final Score."""

    radiation_cost = float(radiation_cost)
    terrain_cost = float(terrain_cost)
    execution_time_s = float(execution_time_s)

    if radiation_cost < 0.0:
        raise ValueError("radiation_cost must be non-negative")

    if terrain_cost < 0.0:
        raise ValueError("terrain_cost must be non-negative")

    if execution_time_s <= 0.0:
        raise ValueError("execution_time_s must be greater than zero")

    normalized_radiation = normalize_metric(
        radiation_cost,
        RADIATION_REFERENCE,
    )

    normalized_terrain = normalize_metric(
        terrain_cost,
        TERRAIN_REFERENCE,
    )

    normalized_execution_time = normalize_metric(
        execution_time_s,
        EXECUTION_TIME_REFERENCE_S,
    )

    radiation_contribution = (
        RADIATION_WEIGHT * normalized_radiation
    )

    terrain_contribution = (
        TERRAIN_WEIGHT * normalized_terrain
    )

    execution_time_contribution = (
        EXECUTION_TIME_WEIGHT
        * normalized_execution_time
    )

    final_score = 100.0 * (
        radiation_contribution
        + terrain_contribution
        + execution_time_contribution
    )

    return FinalScoreResult(
        normalized_radiation=normalized_radiation,
        normalized_terrain=normalized_terrain,
        normalized_execution_time=normalized_execution_time,
        radiation_contribution=radiation_contribution,
        terrain_contribution=terrain_contribution,
        execution_time_contribution=execution_time_contribution,
        final_score=final_score,
    )
