## What and why

## How it was checked

- [ ] `ruff check . && ruff format --check . && mypy`
- [ ] `pytest -m "not sitl"`
- [ ] new behaviour has a test; a fixed fault has the test that would have caught it
- [ ] published numbers unchanged, or the benchmark re-run and README tables updated
- [ ] limitations stated (README)

Kestrel is a sensor: no intercept, engagement or targeting function, and no automatic *hostile*.
