You are the lead architect closing a design tournament. Several engineers independently
implemented the same spec with different design stances (candidates {{LETTERS}}).
Reviewers have compared them blind, each through a different lens. Your job is to write
the **blueprint** that one fresh engineer will implement from scratch. It is not a merge.

# Inputs (current directory, read-only)
- `BRIEF.md`, and `SPEC.md` inside any candidate directory.
- `_evidence/judge-*.md`: the reviewer reports. Start here.
- `_evidence/metrics.md`, `_evidence/probes.md`, `_evidence/difftest.md` (if present),
  and the probe diffs and notes.
- The candidates `{{LETTERS}}` and `_baseline/`. Read the code wherever the reports
  disagree or make a claim you need to check.

# Owner amendment (made after the spec was frozen; it applies to this step)
The owner wants **ergonomic construction**: `new` may keep its eight parameters, but
there must also be easy-to-use helper constructors and/or builder objects that take
fewer parameters (defaults, diagonal or scalar noise, common models such as constant
velocity). This **relaxes B16/Q5's "at most one front end"**: several helpers or builders
are allowed, provided every one of them delegates to the same validation as `new`, adds
no behavior of its own, and never panics. Candidates were not asked for this, so do not
count its absence against them. The `ergonomic-constructors` change probe is the
evidence for which design makes it easy.

**Naming rule (owner):** a constructor that can fail is named `try_*` and returns
`KalmanResult<Self>`: `try_new(..)` is the validated constructor, a builder finishes
with `try_build()`, a fallible helper is `try_constant_velocity(..)`, and so on. Only
constructors that cannot fail may use plain names (`new`, `build`, …). Because the frozen
tests still call `new(..) -> KalmanResult<Self>`, keep `new` as a one-line forward to
`try_new` with a doc comment saying it is temporary. It is removed in a mechanical
cleanup after the final build has been verified. Do not add `new` anywhere else.

# Principles
- **Pick one skeleton and graft onto it.** Do not union features or average designs.
- **Every abstraction must earn its place** with a spec requirement or probe evidence
  (a seam that made a hidden follow-up feature cheap). Delete speculative generality.
- **Prefer what won on the probes** over what merely looks elegant.
- Where reviewers disagree, decide, and record the reasoning.
- Learn from failures too: a stance that fought the problem tells you what to avoid.

# Output: the complete DESIGN.md as your final message
1. **Decision summary** (≤ 8 lines): the skeleton candidate and why; the 3-5 most
   important grafts.
2. **Module layout**: files/modules with one-line responsibilities.
3. **Core types and public interface**: actual signatures in the project's language,
   consistent with SPEC.md.
4. **Key algorithms and invariants**: including the numerical and error-handling
   policy (when to return errors, never panic, etc.).
5. **Grafts**: a table of idea | source (`X/path:line`) | where it lands | why.
6. **Rejected**: ideas and designs deliberately left out, with the evidence.
7. **Judge disagreements**: how each was resolved.
8. **Open questions for the human**: decisions that are the owner's to make, each with
   your recommendation.
9. **Implementation order**: steps with the acceptance tests expected to go green at
   each step.

Write it so an engineer who has seen none of the candidates can build it. Cite
candidate code as *reference*, not as text to copy.
