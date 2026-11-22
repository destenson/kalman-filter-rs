#!/usr/bin/env bash
# Build the blind export for judge agents, for every candidate that actually has a branch.
# Same layout as the skill's build_blind, restricted to executed candidates.
set -euo pipefail
REPO=/home/dennis/src/kalman-filters
RD=$REPO/.tournament/linear-core
BR=tourney/linear-core
B=$RD/blind

rm -rf "$B"; mkdir -p "$B/_evidence"

mapfile -t LETTERS < <(cut -f1 "$RD/mapping.tsv")
for L in "${LETTERS[@]}"; do
  git -C "$REPO" show-ref --verify --quiet "refs/heads/$BR/$L" || continue
  mkdir -p "$B/$L"
  git -C "$REPO" archive "$BR/$L" | tar -x -C "$B/$L"
done

mkdir -p "$B/_baseline"
git -C "$REPO" archive "$(cat "$RD/base_ref")" | tar -x -C "$B/_baseline"

cp "$RD/metrics/summary.blind.md" "$B/_evidence/metrics.md" 2>/dev/null || echo "(not measured)" > "$B/_evidence/metrics.md"
python3 -I /home/dennis/.dsh/skills/design-tournament/scripts/lib.py probe-summary "$RD" --blind > "$B/_evidence/probes.md"
cp "$RD/difftest/summary.blind.md" "$B/_evidence/difftest.md" 2>/dev/null || true

for p in "$RD"/probe/*__*.diff "$RD"/probe/*__*.notes.md; do
  [ -e "$p" ] || continue
  cp "$p" "$B/_evidence/"
done
for p in "$RD"/probes/*.md; do
  case "$(basename "$p")" in _*) ;; *) cp "$p" "$B/_evidence/probe-request-$(basename "$p")";; esac
done
cp "$RD/brief.md" "$B/BRIEF.md"
cp "$RD"/notes/full-test-status.md "$B/_evidence/full-test-status.md" 2>/dev/null || true
cp "$RD"/notes/FINDING-A-psd.md "$B/_evidence/finding-A-psd.md" 2>/dev/null || true

echo "blind export at $B"
ls "$B" "$B/_evidence" | head -40
