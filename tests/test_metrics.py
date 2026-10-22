"""Benchmark metric definitions (risk ratio, encounter detection)."""

from __future__ import annotations

import pytest

from kestrel.bench import risk_ratio


def _summary(seed0: int, runs: int, violations: int) -> dict[str, object]:
    return {"seed0": seed0, "runs": runs, "separation_violations": violations}


def test_risk_ratio_is_mitigated_over_unmitigated() -> None:
    assert risk_ratio(_summary(3000, 24, 3), _summary(3000, 24, 12)) == 0.25


def test_risk_ratio_undefined_without_baseline_losses() -> None:
    assert risk_ratio(_summary(3000, 24, 0), _summary(3000, 24, 0)) is None


def test_risk_ratio_refuses_different_encounter_sets() -> None:
    with pytest.raises(ValueError, match="same encounter set"):
        risk_ratio(_summary(3000, 24, 3), _summary(2000, 24, 12))
