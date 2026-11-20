---
name: design-tournament
description: Run a multi-agent design tournament on a project or module. 5-10 independent agents each build the same spec under a different design stance in isolated git worktrees. The results are measured (frozen acceptance tests, metrics, and a hidden "change probe" that measures maintainability), compared by blinded judge agents, and synthesized into one design document that a fresh agent implements. Use when starting a new project or module whose architecture is uncertain, when a project (often agent-written) has drifted and deserves a reboot, or when the user asks for "multiple agents", "best of N", "competing implementations", "design bake-off", "tournament", or "reboot/rewrite this module properly".
---

# Design Tournament

Diverge → measure → judge → synthesize → rebuild. The output is a better
*design*, not a merge of code. Merging code from N candidates produces a
Frankenstein with mismatched abstractions. Instead, ideas are extracted into
`DESIGN.md` and one fresh agent implements it cleanly.

Everything is driven by `scripts/tourney` (bash + python3, git worktrees, headless
`claude -p`). State lives in `.tournament/<run>/` at the repo root. Init adds that
path to `.git/info/exclude`, so the tracked tree is never touched.
`references/rationale.md` explains *why* each phase exists. Read it when the user
asks or wants to adapt the workflow.

## Phase map

| # | Command | Who | Output | Human checkpoint |
|---|---------|-----|--------|------------------|
| 0 | `tourney init <run> --mode greenfield\|reboot` | you | `config.sh`, `brief.md`, prompt templates | edit config + brief |
| 1 | `tourney spec <run>` | 1 agent | branch `tourney/<run>/spec`: `SPEC.md` + acceptance tests | **approve spec** → `tourney freeze <run>` |
| 2 | `tourney diverge <run>` | N agents ∥ | branches `tourney/<run>/A..`, each with `DESIGN_NOTES.md` | — |
| 3 | `tourney measure <run>` | script | `metrics/*.json`, `metrics/summary.md` | glance at it |
| 4 | `tourney probe <run>` | N×P agents ∥ | `probe/*.json`, `probe/summary.md` | — |
| 5 | `tourney judge <run>` | 2-3 agents ∥, blinded | `reports/judge-*.md` | read reports |
| 6 | `tourney synthesize <run>` | 1 agent | `synthesis/DESIGN.md` | **approve design** |
| 7 | `tourney build <run>` | 1 agent | branch `tourney/<run>/final` | review; `measure`/`probe` it as `final` |

`tourney all <run>` runs phases 2-6 back to back after the spec is frozen, then
stops for design review. `tourney status <run>` shows progress. `tourney clean <run>`
removes worktrees and keeps branches and reports.

The script lives at `${CLAUDE_SKILL_DIR}/scripts/tourney`. If that variable is unset,
use the directory containing this file. Run it from inside the target repo.

## How to drive it (your job as the orchestrating agent)

1. **Scope it.** Ask the user, or infer from context, for: the run name, the scope
   (whole repo vs. one module, i.e. the paths that candidates rewrite), the mode, and
   the budget. Mode is `greenfield` for a new spec or `reboot` to characterize existing
   behavior first. A tournament costs roughly N+P·N+J+3 agent runs. Suggest one module
   for a first run, not a whole 14k-line crate.
2. **Init**, then fill in `.tournament/<run>/config.sh`: the build, test, and lint
   commands, `SCOPE_PATHS`, `FROZEN_PATHS`, `ALLOWED_TOOLS`, `MODEL`, budget, and
   parallelism. Then write `brief.md`: the goal, the constraints, what is wrong today,
   and the non-goals. The brief is the one thing every agent reads, so make it concrete.
3. **Pick stances.** `templates/stances/` has general ones. Delete stances that make no
   sense for the problem and add domain-specific ones. *Stances must produce genuinely
   different architectures*; "same thing but cleaner" is not a stance. Use 5-8.
4. **Write the hidden probes** in `.tournament/<run>/probes/`. Each one is a realistic
   follow-up feature the spec does *not* mention. Probes measure the cost of the next
   change, which is what maintainability is. They must stay out of everything the
   candidates see (they live outside the worktrees, and nothing copies them in).
5. **Spec → human approves → freeze.** Show the user `SPEC.md` and the test list from
   the spec branch. Do not freeze until they agree. Freezing records hashes of
   `FROZEN_PATHS`, and `measure` flags any candidate that edited them (agents love
   weakening tests).
6. **Run the long phases in the background** (`run_in_background: true`). Do not poll.
   Read `logs/` only if something fails. Each agent's JSON result (cost, turns, and
   errors) lands in `logs/<name>.json`.
7. **After judging**, summarize for the user: the metrics table, the probe table, where
   the judges agreed, where they disagreed, and the proposed `DESIGN.md`. Get approval
   (or edits) before `build`.
8. **Verify the final result** with `tourney measure <run> final` and
   `tourney probe <run> final`, and compare it against the candidates. If it does not
   beat the best candidate on probes, say so plainly. Do not dress it up.

## Rules that keep it honest

- **Independence until judging.** Candidates never see each other. Only judges and the
  synthesizer read all of them.
- **Blinding.** Branches are letters. The stance → letter map (`mapping.tsv`) is read by
  the scripts only. Judges get a `git archive` export with no history. The implement
  prompt tells candidates not to name their stance. This blinding is soft: judges can
  still infer a stance from the code, and that's fine.
- **The spec and tests are the yardstick.** If the spec turns out to be wrong
  mid-tournament, stop, fix it, re-freeze, and re-run `diverge`. Don't let candidates
  diverge on what to build; only on how.
- **Don't merge candidate code.** The synthesizer writes prose and interfaces. The
  builder may *read* candidates (`--add-dir` on the blind export) but rewrites everything.
- **Report faithfully.** Failed candidates, tampered tests, and a final build that loses
  on the probes all go in your summary.

## No headless `claude`? (in-session fallback)

If `claude -p` is unavailable, run phases 2/4/5 with the Agent tool instead: one
`Agent` call per candidate with `isolation: "worktree"` and the rendered prompt from
`tourney render <run> implement <stance-file>`. Then commit each worktree to
`tourney/<run>/<letter>` and continue with `measure`/`judge` as normal. This is slower
to set up but needs no CLI.

## Files

- `scripts/tourney`: the CLI (all phases).
- `scripts/lib.py`: template rendering, metrics/probe summaries, and freeze checks.
- `templates/`: copied into each run at `init` so they can be edited per run.
- `references/rationale.md`: why each phase exists, failure modes, and cost guidance.
- `references/example-kalman-filter-rs.md`: a worked example (a reboot of an
  agent-written Rust crate) with config, brief, stances, and probes.
