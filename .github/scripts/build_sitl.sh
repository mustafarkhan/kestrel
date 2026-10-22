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
    # One multi-line annotation (GitHub shows only a few per step): error lines first, then the tail.
    {
      echo "command: $*"
      grep -iE "error|no module|failed|not found" "$LOG" | grep -v "Generating C" | tail -n 15
      echo "--- last lines ---"
      tail -n 15 "$LOG"
    } | sed -e 's/%/%25/g' -e 's/\r/%0D/g' | awk 'BEGIN{ORS="%0A"} {print}' | sed 's/^/::error title=SITL build failed::/'
    echo
    exit 1
  fi
}

# ArduPilot's DroneCAN generator imports pkg_resources, which Python 3.12+ environments no longer
# include and setuptools >= 81 removes.
python -m pip install --quiet pexpect future "setuptools<81"
# empy 3.3.4 is a single module; vendor it (its setup.py does not build on new setuptools)
mkdir -p "$WORK/pylib" "$WORK/empy"
step python -m pip download --no-deps --no-binary :all: "empy==3.3.4" -d "$WORK/empy"
step tar xzf "$WORK/empy/empy-3.3.4.tar.gz" -C "$WORK/empy"
cp "$WORK/empy/empy-3.3.4/em.py" "$WORK/pylib/"
export PYTHONPATH="$WORK/pylib"
step python -c "import em, pexpect, future, pkg_resources; print('python deps ok', em.__version__)"

step git clone --depth 1 --branch "$REF" https://github.com/ArduPilot/ardupilot.git "$WORK/ardupilot"
cd "$WORK/ardupilot"
step git submodule update --init --depth 1 modules/mavlink modules/waf modules/DroneCAN/libcanard \
  modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/pydronecan modules/DroneCAN/DSDL
step python ./waf configure --board sitl
step python ./waf copter
cp build/sitl/bin/arducopter Tools/autotest/default_params/copter.parm "$OUT/"
echo "built $(ls -la "$OUT/arducopter")"
