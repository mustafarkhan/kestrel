# Contributing to Kestrel

Kestrel is an open sensor node and air-picture stack for counter-drone defence. It detects,
tracks, identifies and reports. Contributions that keep it honest and make it easier to
test with real sensors and aircraft are welcome.

## What will and will not be merged

* **In scope:** sensing, tracking, classification, fusion, identification, interoperability
  (SAPIENT, Cursor on Target, MAVLink), safety, simulation fidelity, real-data evaluation,
  documentation, tests.
* **Out of scope, always:** intercept, engagement, targeting or weapon functions of any
  kind; anything that sets a track *hostile* without a human decision; anything that
  presents a simulated result as a flight result.

## Set up

```bash
git clone https://github.com/mustafarkhan/kestrel && cd kestrel
pip install -e ".[dev,camera]"
pytest -m "not slow and not sitl"     # about a minute
```

## Before opening a pull request

1. Work on a branch; one topic per pull request.
2. `ruff check . && ruff format --check . && mypy` must pass, and so must
   `pytest -m "not sitl"` (the slow closed-loop tests included).
3. New behaviour comes with a test that fails without it. A fixed fault comes with the
   test that would have caught it.
4. If a change moves any published number, re-run the benchmark it belongs to with the
   published seeds, run `python scripts/collect_results.py`, and update the README table;
   `tests/test_published_results.py` checks that the README matches `docs/results/`.
5. A design choice with alternatives gets a short decision record in `docs/decisions/`:
   what was found, what was chosen, the numbers before and after.
6. Say what is not done. A limitation belongs in the README and in
   [docs/readiness.md](docs/readiness.md), not only in the code.

CI runs lint, the unit tests on three Python versions, the closed-loop tests, an
ArduCopter SITL flight, and a build of the wheel and the container. A pull request is
merged when all of it is green.

## Reporting results from real hardware or data

These are the most valuable contributions. Follow
[docs/field-test-plan.md](docs/field-test-plan.md) and
[docs/real-data.md](docs/real-data.md), and include the logs, the software commit, the
data set version and the exact command. Results that contradict the simulation are as
welcome as ones that confirm it.

## Security

Do not open a public issue for a vulnerability: see [SECURITY.md](SECURITY.md).

## Licence

Contributions are accepted under the repository's Apache-2.0 licence.
