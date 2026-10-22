from __future__ import annotations

from pathlib import Path

import pytest

from kestrel.config import Config, Scenario, load_scenario

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def cfg() -> Config:
    return Config()


@pytest.fixture()
def airfield() -> Scenario:
    return load_scenario(ROOT / "scenarios" / "airfield.yaml")
