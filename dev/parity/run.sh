#!/usr/bin/env bash
# Engine parity: the crate has two numeric engines for the linear filter — the
# default dense row-major one and the nalgebra one behind `--features nalgebra`.
# Both are expected to produce the same estimates. This script runs the same
# differential harness in both configurations and compares the results case by
# case, which is what keeps the second engine from drifting away from the first.
#
# The harness is installed as an untracked test target, run in each
# configuration, and removed again. Run from anywhere inside the crate.
#
# Exit status is non-zero when the engines disagree on a value, or on an outcome
# for a case that is not listed as a reviewed boundary case below.
set -euo pipefail

cd "$(dirname "$0")/../.."

# Cases where the two engines may return different outcomes (error vs Ok) on a
# numerically ill-conditioned model. The contract allows either outcome there,
# and the engines sum in a different order, so their relative pivot tests can
# land on opposite sides of the boundary. Adding a case here is a reviewed
# decision, not a way to silence a difference: values are never allowed to
# differ, and a case that starts disagreeing in value still fails.
REVIEWED_BOUNDARY="illcond04 illcond14"

OUT="${TMPDIR:-/tmp}/kalman-engine-parity"
mkdir -p "$OUT"

remove_harness() { rm -f tests/zz_parity.rs; }
trap remove_harness EXIT

# run_engine <label> [cargo args...]
run_engine() {
  local label=$1; shift
  cp dev/parity/harness.rs tests/zz_parity.rs
  cargo test --quiet --test zz_parity "$@" -- --nocapture 2>/dev/null | grep '^{' > "$OUT/$label.jsonl" || true
  remove_harness
  printf '%-10s %s cases\n' "$label" "$(wc -l < "$OUT/$label.jsonl")"
}

run_engine default
run_engine nalgebra --features nalgebra

allow=$(printf '%s' "$REVIEWED_BOUNDARY" | tr ' ' ',')
python3 dev/parity/compare.py "$OUT/default.jsonl" "$OUT/nalgebra.jsonl" --allow-status "$allow"
