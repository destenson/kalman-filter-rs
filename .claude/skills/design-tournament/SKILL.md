---
name: design-tournament
description: Run a multi-agent design tournament on a project or module. Auditors first harden a frozen spec. Then 5-10 independent agents each build it under a different design stance in isolated git worktrees. The results are measured (frozen acceptance tests, metrics, differential testing across implementations, and hidden "change probes" that measure maintainability), compared by blinded judge agents, and synthesized into one design document that a fresh agent implements. Use when starting a new project or module whose architecture is uncertain, when a project (often agent-written) has drifted and deserves a reboot, or when the user asks for "multiple agents", "best of N", "competing implementations", "design bake-off", "tournament", or "reboot/rewrite this module properly".
---

# Design Tournament

Spec → audit → diverge → measure → judge → synthesize → rebuild. The output is a
better *design*, not a merge of code. Merging code from N candidates produces a
Frankenstein with mismatched abstractions. Instead, ideas are extracted into
`DESIGN.md` and one fresh agent implements it cleanly.

Everything is driven by `scripts/tourney` (bash + python3, git worktrees, headless
`claude -p`). State lives in `.tournament/<run>/` at the repo root. Init adds that
path to `.git/info/exclude`, so the tracked tree is never touched. The skill itself
is project-agnostic: everything project-specific (brief, stances, probes, the
difftest harness) lives in the run directory.

- `references/rationale.md`: *why* each phase exists, failure modes, tuning, and cost.
- `references/authoring-guide.md`: how to write a good brief, stances, probes,
  auditor lenses, and a differential-test harness.

## Phase map

| # | Command | Who | Output | Human checkpoint |
|---|---------|-----|--------|------------------|
| 0 | `tourney init <run> --mode greenfield\|reboot` | you | `config.sh`, `brief.md`, templates | edit config + brief |
| 1 | `tourney spec <run>` | 1 agent | branch `tourney/<run>/spec`: `SPEC.md` + acceptance tests | read the draft |
| 1a | `tourney audit <run>` | 3 agents ∥ | `audit/<lens>.md` reports + `audit_*` tests | read the reports |
| 1b | `tourney fold <run>` | 1 agent | confirmed findings folded into the spec | **approve spec** → `tourney freeze <run>` |
| 2 | `tourney diverge <run>` | N agents ∥ | branches `tourney/<run>/A..`, each with `DESIGN_NOTES.md` | — |
| 3 | `tourney measure <run>` | script | `metrics/summary.md` | glance at it |
| 3a | `tourney difftest <run>` | script | `difftest/summary.md`: where implementations disagree | read the disagreements |
| 4 | `tourney probe <run>` | N×P agents ∥ | `probe/summary.md`, diffs, maintainer notes | — |
| 5 | `tourney judge <run>` | 3 agents ∥, blinded | `reports/judge-*.md` | read the reports |
| 6 | `tourney synthesize <run>` | 1 agent | `synthesis/DESIGN.md` | **approve design** |
| 7 | `tourney build <run>` | 1 agent | branch `tourney/<run>/final` | review; `measure`/`difftest`/`probe` it as `final` |

`tourney all <run>` runs phases 2-6 (including difftest when configured) after the
spec is frozen, then stops for design review. Audit and difftest are optional but
cheap relative to what they catch. `tourney status <run>` shows progress and spend.
`tourney clean <run>` removes worktrees and keeps branches and reports.

The script lives at `${CLAUDE_SKILL_DIR}/scripts/tourney`. If that variable is unset,
use the directory containing this file. Run it from inside the target repo.

## How to drive it (your job as the orchestrating agent)

1. **Scope it.** Ask the user, or infer from context, for: the run name, the scope
   (whole repo vs. one module, i.e. the paths that candidates rewrite), the mode, and
   the budget. Mode is `greenfield` for a new spec or `reboot` to characterize existing
   behavior first. Suggest one module of 1-4k lines for a first run.
2. **Init**, then fill in `.tournament/<run>/config.sh` (commands, `SCOPE_PATHS`,
   `FROZEN_PATHS`, tools, model, budget, parallelism) and `brief.md`. The brief is the
   one thing every agent reads, so make it concrete. Verify any defect you list in it:
   a wrong claim in the brief misleads every agent.
3. **Pick stances** (5-8) that produce genuinely different architectures. Delete
   generic ones that don't fit and add domain-specific ones.
4. **Write the hidden probes** in `.tournament/<run>/probes/`: realistic follow-up
   features that the spec does not mention. They must never reach the candidates.
5. **Spec, then audit, then fold.** Check the spec agent's claims yourself (run the
   tests and confirm they pass and fail as reported). Run `audit`: each auditor lens
   hunts for defects the draft missed and proves each one with a failing test. `fold`
   merges the confirmed findings into the spec. Show the user what was added and
   rejected, and the final `SPEC.md` and test list. **Do not freeze until they agree.**
6. **Optionally write a differential-test harness** for the run (see the
   authoring guide and the `DIFFTEST_*` settings). It runs the original code, every
   candidate, and optionally a reference oracle on the same generated cases. It finds
   bugs nobody wrote a test for.
7. **Run the long phases in the background** (`run_in_background: true`). Do not poll.
   Read `logs/` only if something fails.
8. **After judging**, summarize for the user: the metrics, difftest disagreements,
   probe results, where the judges agreed and disagreed, and the proposed
   `DESIGN.md`. Get approval (or edits) before `build`.
9. **Verify the final result** with `measure`, `difftest`, and `probe` on `final`, and
   compare it against the candidates. If it does not beat the best candidate, say so
   plainly.

## Rules that keep it honest

- **Independence until judging.** Candidates never see each other. Only judges and the
  synthesizer read all of them.
- **Blinding.** Branches are letters. The stance → letter map (`mapping.tsv`) is read by
  the scripts only. Judges get a `git archive` export with no history and evidence
  with no stance names.
- **The spec and tests are the yardstick.** If the spec turns out to be wrong
  mid-tournament, stop, fix it, re-freeze, and re-run `diverge`.
- **Don't merge candidate code.** The synthesizer writes prose and interfaces. The
  builder may *read* candidates but rewrites everything.
- **Report faithfully.** Failed candidates, tampered tests, difftest disagreements,
  and a final build that loses on the probes all go in your summary.
- **Pushing.** Tournament branches are local by default. If the user wants them on the
  remote, push the spec and final branches and ask before pushing dozens of
  candidate and probe branches.

## No headless `claude`? (in-session fallback)

If `claude -p` is unavailable, run the agent phases with the Agent tool instead: one
`Agent` call per candidate (or auditor or prober) with `isolation: "worktree"` and the
prompt from `tourney render <run> <template> [file]`. Commit each worktree to the
branch name the script would use, then continue with the script phases as normal.

## Files

- `scripts/tourney`: the CLI (all phases).
- `scripts/lib.py`: templating, metrics/probe/difftest summaries, freeze checks.
- `templates/`: copied into each run at `init` (prompts, stances, judge lenses,
  auditor lenses, a probe example, a brief skeleton).
- `references/`: rationale and the authoring guide.
