You are implementing the final, approved design from a design tournament.

# Inputs
- `TOURNAMENT_DESIGN.md` (repository root): the blueprint. Follow it.
- `SPEC.md` and the acceptance tests in `{{FROZEN_PATHS}}`: the contract.
- `{{BLIND_DIR}}`: the read-only candidate implementations (`A/`, `B/`, …) that the
  design cites as references, plus the judges' reports in `_evidence/`.

# Brief
{{BRIEF}}

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

# Starting point: candidate {{BASE}}
This worktree already contains candidate {{BASE}}'s code, which passes all 32 frozen tests.
**Graft, don't rewrite.** `TOURNAMENT_DESIGN.md` is a graft plan: apply exactly its grafts
(§2), fixes (§3), removals (§4), amendment design (§6) and out-of-scope fixes (§7), in its
build order (§8), running the checks it lists after each step. Keep the base's structure
and style. Anything the plan does not mention stays as it is. If a planned change turns
out wrong when you run the code, don't apply it; record why under "Deviations".
**No bloat**: never keep two ways of doing the same thing.

The plan's step 7 (deleting the temporary `new` forward) is NOT yours: the frozen tests
must keep compiling. Stop after step 6.

# Rules
- Work in the order given under "Implementation order" in the design, running
  `{{TEST_CMD}}` at each checkpoint.
- Consult the cited candidate code for ideas, edge cases, and tricky algorithms. Do not
  copy whole files: write code that fits *this* design, with consistent naming and
  error handling throughout.
- You own `{{SCOPE_PATHS}}`. **Never modify `{{FROZEN_PATHS}}`.** Outside scope, make
  only the minimal mechanical edits.
- Build nothing beyond the spec and the design. If the design is wrong or impossible
  somewhere, deviate minimally and document it.
- Build: `{{BUILD_CMD}}`. Tests: `{{TEST_CMD}}`. Lint: `{{LINT_CMD}}`. All must be clean.
- Do not commit; the harness commits.

# DESIGN_NOTES.md (required, at the repository root)
1. Architecture in ≤ 10 lines.
2. **Deviations from TOURNAMENT_DESIGN.md** and the reason for each.
3. Known weaknesses and next steps.

Finish with: test status, LOC, and a list of deviations.
