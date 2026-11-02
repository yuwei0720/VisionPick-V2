"""Small, explainable Preferred-IK scoring helpers retained in DEV8.

This module intentionally does *not* claim collision awareness.  It ranks a
small set of IK solutions already produced by the robot backend.  The operator
reference posture is a soft preference, not a waypoint or hard joint limit.
"""
from __future__ import annotations

from dataclasses import dataclass


def angular_delta_deg(a: float, b: float) -> float:
    """Shortest absolute angular difference in degrees."""
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


@dataclass(frozen=True)
class CandidateScore:
    angles: tuple[float, float, float, float, float, float]
    seed_source: str
    continuity_cost: float
    posture_cost: float
    wrist_flip_cost: float
    total_cost: float

    def as_dict(self) -> dict:
        return {
            "angles_deg": [round(v, 3) for v in self.angles],
            "seed_source": self.seed_source,
            "continuity_cost": round(self.continuity_cost, 3),
            "posture_cost": round(self.posture_cost, 3),
            "wrist_flip_cost": round(self.wrist_flip_cost, 3),
            "total_cost": round(self.total_cost, 3),
        }


def score_candidate(angles, *, current_angles, reference_angles, seed_source: str) -> CandidateScore:
    """Score one IK result; lower cost is better.

    - continuity: prefer less total joint motion from the current posture;
    - posture: softly prefer J2/J3/J4 near the operator-taught outward reference;
    - wrist flip: penalize very large J5/J6 changes (>90 deg).

    The weights are deliberately simple and exposed in event telemetry.  This
    is a posture selector, not a collision checker.
    """
    q = tuple(float(v) for v in angles[:6])
    current = tuple(float(v) for v in current_angles[:6])
    reference = tuple(float(v) for v in reference_angles[:6])

    deltas = [angular_delta_deg(q[i], current[i]) for i in range(6)]
    continuity = sum(deltas) / 6.0
    posture = sum(angular_delta_deg(q[i], reference[i]) for i in (1, 2, 3)) / 3.0
    wrist_flip = sum(max(0.0, deltas[i] - 90.0) for i in (4, 5)) / 2.0
    total = 0.35 * continuity + 0.55 * posture + 0.10 * wrist_flip
    return CandidateScore(q, seed_source, continuity, posture, wrist_flip, total)


def deduplicate_candidates(candidates, *, tolerance_deg: float = 0.5):
    """Keep the first candidate from each near-identical joint solution."""
    unique = []
    for candidate in candidates:
        if not any(
            max(angular_delta_deg(candidate.angles[i], prior.angles[i]) for i in range(6)) <= tolerance_deg
            for prior in unique
        ):
            unique.append(candidate)
    return unique
