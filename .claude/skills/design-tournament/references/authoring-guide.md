# Authoring guide: brief, stances, probes, auditors, difftest harness

Everything here goes into `.tournament/<run>/`, not into the skill.

## Brief (`brief.md`)
Every agent reads it, so its errors propagate. Write:
- **Goal and scope**: which paths candidates own, and what must keep working outside them
  (e.g. "the whole crate must still build; other modules get only mechanical edits").
- **Hard constraints**: language/edition, dependencies allowed or forbidden, whether the
  public API may break (and if so, ask for a migration note).
- **What is wrong today** (reboot): concrete defects with `file:line`. *Verify each one*:
  read the code or run a quick test. Unverified suspicions go in as "suspected".
- **Quality bar**: measurable where possible ("no allocation in the steady-state loop",
  "no panic reachable from the public API", "p99 < 2 ms at 10k items").
- **Non-goals**: agents gold-plate; this section is what stops it.
- In reboot mode, when the public API may change, tell the spec author it may add a
  *thin adapter* that exposes the new contract interface over today's code, so that
  tests for KEEP behaviors run now and tests for CHANGE behaviors fail for the right
  reason.

## Stances (`stances/*.md`)
A stance must change the *architecture*, not the polish. Test: would two agents that
follow different stances produce different module boundaries or core types? Each file
should state the principle, what it means concretely here, what to avoid, and its
likely failure mode (which keeps the candidates honest). Add 1-3 domain-specific
stances. For example:
- numerical code: "factored representation" (keep a decomposition, never the raw form)
- a web service: "event-sourced" or "CQRS"
- a parser: "parser combinators" vs. "hand-written recursive descent" vs. "table-driven"
- a UI: "state machine first" vs. "unidirectional data flow"

## Probes (`probes/*.md`)
2-3 hidden follow-up feature requests. Each should be:
- absent from the spec and the brief,
- one agent session in size (roughly 50-300 lines in a reasonable design),
- cutting across the core abstraction, not a leaf addition,
- stated with a testable acceptance criterion.

Cover different axes: *extend the model* (a new input or variant), *a new algorithm
that reuses internals* (a second consumer of the core), and *a cross-cutting concern*
(observability, persistence, cancellation, or configuration).

## Auditor lenses (`auditors/*.md`)
The defaults are `reference-oracle`, `inputs-and-panics`, and `claims-vs-code`.
Add a domain lens when the domain has a known failure family: concurrency
("interleavings, cancellation, lock ordering"), security ("untrusted input reaching
X"), or persistence ("crash between steps, partial writes"). Auditors only add
`audit_*` test files. `fold` decides what enters the frozen spec.

## Differential-test harness (`DIFFTEST_*` in `config.sh`)
One file in the run dir, written against the spec's public interface only, so it
compiles against the original code (via the spec adapter) and every candidate.
- Generate cases deterministically: a fixed seed and your own small PRNG, so library
  version changes can't alter the cases. Cover the random bulk, boundary sizes,
  extreme scales, and error-triggering inputs.
- Print one JSON object per case to stdout:
  `{"case":"c042","ok":true,"values":[...]}` or
  `{"case":"c043","ok":false,"error":"DimensionMismatch"}`. A case may set its own
  `"tol"` (relative) and `"abs"` tolerance, e.g. for a 32-bit variant or a scaled case.
  Other stdout lines are ignored.
- Optional reference oracle: if the harness contains a simple, obviously-correct
  implementation, switch to it on an environment variable (`DIFFTEST_REFERENCE_ENV`).
  The comparison then uses the oracle as ground truth instead of the majority.
- Error *kinds* are compared as categories (the text before `(` or `{`), so emit the
  error variant name rather than the full message.
- Keep the run time to a minute or so per target; difftest runs on every candidate and
  on `final`.
