#!/usr/bin/env bash
# Build ArduCopter SITL for the integration test.
# Usage: build_sitl.sh <ardupilot-ref> <output-dir>
# On failure, the failing command and the tail of its output are emitted as
# GitHub annotations, so the cause is visible without opening the raw log.
set -euo pipefail

REF="$1"
OUT="$2"
WORK="${RUNNER_TEMP:-/tmp}/sitl-build"
LOG="$WORK/build.log"
mkdir -p "$WORK" "$OUT"
: > "$LOG"

step() {
  echo "== $*" | tee -a "$LOG"
  if ! "$@" >>"$LOG" 2>&1; then
    echo "::error title=SITL build failed::command: $*"
    tail -n 40 "$LOG" | sed 's/%/%25/g' | while IFS= read -r line; do echo "::error::${line}"; done
    exit 1
  fi
}

python -m pip install --quiet pexpect future
# empy 3.3.4 is a single module; vendor it (its setup.py does not build on new setuptools)
mkdir -p "$WORK/pylib" "$WORK/empy"
step python -m pip download --no-deps --no-binary :all: "empy==3.3.4" -d "$WORK/empy"
step tar xzf "$WORK/empy/empy-3.3.4.tar.gz" -C "$WORK/empy"
cp "$WORK/empy/empy-3.3.4/em.py" "$WORK/pylib/"
export PYTHONPATH="$WORK/pylib"
step python -c "import em, pexpect, future; print('python deps ok', em.__version__)"

step git clone --depth 1 --branch "$REF" https://github.com/ArduPilot/ardupilot.git "$WORK/ardupilot"
cd "$WORK/ardupilot"
step git submodule update --init --depth 1 modules/mavlink modules/waf modules/DroneCAN/libcanard \
  modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/pydronecan modules/DroneCAN/DSDL
step python ./waf configure --board sitl
step python ./waf copter
cp build/sitl/bin/arducopter Tools/autotest/default_params/copter.parm "$OUT/"
echo "built $(ls -la "$OUT/arducopter")"
