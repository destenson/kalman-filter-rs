# Why the tournament is shaped this way

## The problem it solves
A single agent (or a single person) commits early to an architecture and then defends
it through every later refactor. Trial and error happens *serially*, so each failed
idea costs a full rewrite and leaves scars in the code. The tournament runs the trials
*in parallel*, throws the code away, and keeps the knowledge.

## Phase by phase

| Phase | Exists because… | Fails if… |
|---|---|---|
| Spec + frozen tests | Without a shared yardstick, "best" means whichever version the judge read last. | Tests check internals (they bias toward one design) or are so loose that a wrong implementation passes. |
| Stances | Same prompt × N agents ≈ 2-3 distinct designs. Diversity has to be *assigned*. | Stances are cosmetic ("clean", "well-documented"). They must change the architecture. |
| Isolation | Seeing each other's work anchors agents on the first plausible design (groupthink). | Shared worktree, shared notes, or a candidate peeking at a sibling's branch. |
| DESIGN_NOTES | Agents learn things they never put in code: what fought back, what they would redo. | Notes advertise instead of report. The prompt demands candor; judges verify against code. |
| Measure | Cheap and objective; catches broken builds and tampered tests before expensive judging. | Treated as a ranking. LOC and lint are signals, not verdicts. |
| Change probe | Maintainability *is* the cost of the next change, so measure it directly with features nobody designed for. | Probes leak into the spec, or are so small that every design handles them trivially. |
| Blind judges with lenses | One judge overweights one lens; named stances bias judges. Disagreement between lenses is the interesting output. | Judges skim notes instead of code. The prompt requires `file:line` citations. |
| Synthesize into prose | Merged code from N designs is a Frankenstein of mismatched abstractions. Ideas transfer cleanly; code does not. | The synthesizer unions every feature. Rule: one skeleton, justified grafts. |
| Fresh build | A clean implementation of a known-good design is fast, and consistent from top to bottom. | The builder copies files wholesale. It may read candidates, not paste them. |
| Re-measure final | Keeps the process honest: the synthesis should beat the best candidate on probes. | Skipped. If the final loses, say so and consider shipping the best candidate instead. |

## Tuning
- **N candidates**: 5-8. Beyond 8, judges skim and the extra designs add little.
  With fewer than 4 there is not much design space to compare.
- **Probes**: 2-3 per run, stressing different axes. Probes cost N × P agent runs, so
  for large N use 2.
- **Judges**: 3 lenses (correctness, simplicity, extensibility) is the default. Add
  performance only when the spec has real performance requirements.
- **Scope**: one module or subsystem of 1-4k lines per run. For a large codebase, run
  successive tournaments, core first, each later one with the previous winner frozen
  as the spec baseline.
- **Model mix**: candidates can run on a cheaper model; judges and the synthesizer
  benefit most from the strongest model (set `MODEL` per phase by editing `config.sh`
  between phases).

## Cost estimate
Agent runs = 1 (spec) + N (diverge) + N·P (probes) + J (judges) + 1 (synth) + 1 (build)
+ P (final probes). With N=6, P=2, J=3, that is 26 runs. Use `BUDGET_USD` to cap each
run and `tourney status` to watch total spend.

## When NOT to use it
- The architecture is already obvious, or the work is a leaf feature: one agent with
  good tests is enough.
- No one can write a meaningful spec yet (pure exploration): first do one throwaway
  spike, *then* run a tournament informed by it.
- The scope cannot be isolated, because everything depends on everything. First carve
  out an interface boundary (that refactor can itself be a small tournament) and then
  run the tournament on one side of it.

## Reboot mode specifics
An off-track codebase is usually rich in *knowledge* (edge cases found, requirements
discovered) and poor in *structure*. Reboot mode harvests the knowledge into the spec
(KEEP/CHANGE/DROP behaviors plus characterization tests) and lets candidates discard
the structure. `_baseline/` is shown to judges so that regressions against the
original are visible.
