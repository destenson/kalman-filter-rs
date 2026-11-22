#!/usr/bin/env bash
# Record one probe run whose implementation was done by an in-session subagent.
# usage: record-probe.sh <target> <probe> <notes-file>
# The subagent must have left its work uncommitted in the probe worktree.
set -euo pipefail
T=$1; P=$2; NOTES=$3
REPO=/home/dennis/src/kalman-filters
RD=$REPO/.tournament/linear-core
DIR=$RD/worktrees/probe-$T-$P
BR=tourney/linear-core/probe/$T/$P
LIB=/home/dennis/.dsh/skills/design-tournament/scripts/lib.py
export CARGO_HOME=$RD/cargo-home

BASE=$(git -C "$REPO" rev-parse "tourney/linear-core/$T")

git -C "$DIR" add -A
git -C "$DIR" -c user.name=tourney -c user.email=tourney@localhost \
  commit -q --allow-empty --no-verify -m "probe $P"

# test run after the probe
t_rc=0
source "$RD/config.sh"
(cd "$DIR" && bash -c "$TEST_CMD") > "$RD/probe/${T}__${P}.test.log" 2>&1 || t_rc=$?

git -C "$REPO" diff "$BASE" "$BR" > "$RD/probe/${T}__${P}.diff"

# agent log in the shape lib.py expects (no cost/turns: a DSH subagent does not report them)
python3 - "$NOTES" > "$RD/logs/probe-$T-$P.json" <<'PY'
import json, sys, pathlib
print(json.dumps({"result": pathlib.Path(sys.argv[1]).read_text()}))
PY

python3 -I "$LIB" probe-record "$DIR" "$T" "$P" "$BASE" "$RD/state/frozen.json" \
  --test-rc "$t_rc" --frozen-paths "$FROZEN_PATHS" --agent-log "$RD/logs/probe-$T-$P.json" \
  --notes-out "$RD/probe/${T}__${P}.notes.md" > "$RD/probe/${T}__${P}.json"

python3 -c "import json,sys; d=json.load(open('$RD/probe/${T}__${P}.json')); print(f\"{d['target']}/{d['probe']}: tests_ok={d['tests_ok']} files={d['files_changed']} +{d['insertions']} -{d['deletions']} frozen={len(d['frozen_violations'])}\")"
