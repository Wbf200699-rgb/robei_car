from __future__ import annotations

import math
from typing import Mapping


REQUIRED_PAIRS = ("AB", "AC", "AD", "BC", "BD", "CD")


def solve_anchor_coordinates(distances_cm: Mapping[str, float]) -> dict[str, object]:
    missing = [pair for pair in REQUIRED_PAIRS if float(distances_cm.get(pair, 0)) <= 0]
    if missing:
        return {"quality": "incomplete", "missing_pairs": missing}

    ab, ac, ad = (float(distances_cm[name]) for name in ("AB", "AC", "AD"))
    bc, bd, cd = (float(distances_cm[name]) for name in ("BC", "BD", "CD"))
    if ac <= 1.0:
        return {"quality": "invalid", "reason": "AC baseline is too short"}

    bx = (ab * ab + ac * ac - bc * bc) / (2.0 * ac)
    dx = (ad * ad + ac * ac - cd * cd) / (2.0 * ac)
    by_sq = ab * ab - bx * bx
    dy_sq = ad * ad - dx * dx
    if by_sq < 0 or dy_sq < 0:
        return {"quality": "invalid", "reason": "pair distances violate triangle geometry"}

    by = math.sqrt(by_sq)
    dy = math.sqrt(dy_sq)
    predicted_bd = math.hypot(bx - dx, by - dy)
    residual_cm = abs(predicted_bd - bd)
    # UWB pair ranging is noisy near metal and antennas. Ten centimeters is a
    # useful acceptance gate without pretending the result is survey-grade.
    quality = "full" if residual_cm <= 10.0 else "rejected"
    return {
        "quality": quality,
        "coordinates_cm": {
            "A": {"x": 0.0, "y": 0.0},
            "B": {"x": bx, "y": by},
            "C": {"x": ac, "y": 0.0},
            "D": {"x": dx, "y": dy},
        },
        "distances_cm": {name: float(distances_cm[name]) for name in REQUIRED_PAIRS},
        "predicted_bd_cm": predicted_bd,
        "measured_bd_cm": bd,
        "residual_cm": residual_cm,
    }
