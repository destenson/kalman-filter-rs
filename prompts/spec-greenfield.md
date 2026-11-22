You are writing the **contract** for a design tournament. Several independent agents
will later implement it, each with a different architecture, and they will be compared
against what you write. You define *what* is built, never *how*.

# Brief
{{BRIEF}}

# Your deliverables (in this repository)

1. `SPEC.md` with these sections:
   - **Purpose**: 3-5 sentences.
   - **Public interface**: the exact entry points (signatures/types in the project's
     language) that tests will call. Keep it as small as the brief allows. Do not
     specify internal modules, traits, or helper types; those are the candidates' design space.
   - **Behaviors**: numbered, testable statements (`B1`, `B2`, …), including error cases
     and edge cases (empty input, dimension mismatch, singular/degenerate data, NaN,
     very large/small values, as relevant).
   - **Quality requirements**: numerical tolerance, performance budgets, panics policy,
     dependency limits, taken from the brief.
   - **Non-goals**.
   - **Test index**: each acceptance test name → the behavior IDs it covers.
2. **Acceptance tests** under `{{FROZEN_PATHS}}` that exercise *only* the public
   interface (black-box). They must be deterministic (fixed seeds) and use tolerances
   for floating point. Prefer many small named tests over one big one, and cover every
   behavior ID.
3. **Minimal stubs** under `{{SCOPE_PATHS}}`, just enough for the tests to compile
   and fail (e.g. `todo!()` / `raise NotImplementedError`). Stubs must not suggest an
   architecture: one flat file is fine.

# Rules
- Run `{{TEST_CMD}}`. The tests must compile/collect and then fail on the stubs, not
  error out on syntax.
- Do not implement the functionality.
- If the brief is ambiguous, choose the simplest reading and record it under
  "Assumptions" in SPEC.md.
- Do not commit; the harness commits.

Finish with a short summary: the behavior count, the test count, and assumptions made.
