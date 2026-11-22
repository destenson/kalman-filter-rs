You are the lead engineer closing a design tournament for the linear Kalman filter core of
the `kalman_filters` crate. Three candidates implemented the same frozen contract
(`SPEC.md` and `tests/spec_linear_kf.rs`, identical in every candidate directory):

- `A/`: data-oriented, with its own small matrix module. **The owner has chosen A as the base.**
- `B/`: functional core. Interrupted before finishing: one old test file was not ported.
- `C/`: nalgebra-native. **Ruled out as a base**, because the owner requires that nalgebra stay an
  optional feature, never a required dependency.

All three pass all 32 frozen tests. `_evidence/metrics.md` and `_evidence/difftest.md` hold
the measurements (difftest = 270 generated cases against a textbook oracle). `_baseline/` is
the original code. `BRIEF.md` is the brief. Only C wrote `DESIGN_NOTES.md`.

# Your task: a graft plan, not code
The final implementation starts from **A's code** and takes in only the specific,
worthwhile improvements found in B or C. Read the in-scope code of all three (`src/filter.rs`,
`src/matrix/`, `src/validation.rs`, `src/error.rs`, `src/types.rs`, `src/lib.rs`) and compare
them in detail.

For every candidate graft, ask:
1. Is it **better** in a way that matters: correctness, robustness, clarity, less code, a
   cleaner seam? Cite `file:line` in both A and the source.
2. Does it stay **in scope** (SPEC.md plus the owner amendments below), with no new features?
3. Does it **avoid bloat**? Prefer grafts that delete or simplify code. A graft that adds
   code must earn it with correctness or a clear simplification elsewhere. Reject "nice to have".
4. Does it keep nalgebra optional? Nothing may make it a required dependency.

Also list **A's own weaknesses** that no candidate does better but that a small fix would
address (bugs, unclear code, needless complexity, unwraps or panics, dead code). And list
anything in A that should be *removed*.

# Owner amendments (they apply to the final build)
- **Naming:** a constructor that can fail is named `try_*` and returns `KalmanResult<Self>`.
  `try_new(..)` is the validated constructor, a builder finishes with `try_build()`, and so on.
  Plain names are only for infallible constructors. Keep `new` as a temporary one-line
  forward to `try_new`, only so the frozen tests compile; it is removed afterwards.
- **Ergonomic construction:** easy helpers and/or a builder that take fewer parameters
  (default x0 = 0; P0, Q and R given as a full matrix, a diagonal, or a scalar σ²·I; a
  constant-velocity helper). Every one delegates to the same validation and never panics.
  Keep this small: one builder plus one or two helpers, no framework.
- **Error variants:** the candidates disagree on variant names for the same failure (see the
  difftest edge cases). Propose one consistent, minimal set. `DimensionMismatch`,
  `SingularMatrix`, `InvalidCovariance` and `InvalidNoiseCovariance` must stay.

# Output: your final message, markdown
1. **Verdict on A** (≤ 6 lines): what makes it a good base, and its main weaknesses.
2. **Grafts**: a table of graft | source (`X/path:line`) | where in A | why it is better | net
   lines (+/−). Order by value. Only include ones that pass all four questions.
3. **Fixes to A** with no source candidate: a table of issue | `A/path:line` | fix | net lines.
4. **Removals from A**: dead or unnecessary code.
5. **Rejected ideas**: tempting grafts you rejected, with the reason (bloat, scope, nalgebra, worse).
6. **Amendment design**: the exact public signatures for `try_new`, the builder, and the
   helper(s), plus the error variant set.
7. **Out-of-scope edits**: does A's churn outside its scope (examples, benches,
   `src/information/*`, `src/extended.rs`, `src/builders/information.rs`) look minimal and
   mechanical? Flag anything that isn't.
8. **Build order**, with the checks to run after each step.

Be decisive and concrete, and keep it under about 250 lines.
