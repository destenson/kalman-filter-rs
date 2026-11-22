You are implementing the final, approved design from a design tournament.

# Inputs
- `TOURNAMENT_DESIGN.md` (repository root): the blueprint. Follow it.
- `SPEC.md` and the acceptance tests in `SPEC.md tests/spec_linear_kf.rs`: the contract.
- `/home/user/kalman-filter-rs/.tournament/linear-core/blind`: the read-only candidate implementations (`A/`, `B/`, …) that the
  design cites as references, plus the judges' reports in `_evidence/`.

# Brief

## Goal
Rebuild the *linear* Kalman filter core of the `kalman_filters` crate (state model,
predict, update, matrix layer, construction API) as one clean, numerically robust
implementation that the nonlinear filters (EKF/UKF/CKF/IF) can later be rebuilt on.

## Scope
Candidates own: src/filter.rs, src/matrix/, src/types.rs, src/error.rs, src/builder.rs, src/validation.rs,
src/unified.rs, src/legacy.rs. Every other module (extended, unscented, scented,
information, particle, ensemble, builders/, logging, metrics, validation, examples,
benches, existing tests/) may receive only minimal mechanical edits to keep compiling;
those are rebuilt in later tournaments. `cargo build --all-targets` with default
features must stay green.

## Hard constraints
- Rust edition 2024. Default features must build with no system libraries.
- No panic reachable from the public API on bad input: return `KalmanError`.
- f32 and f64 supported.
- nalgebra may become a required dependency, or the matrix layer may stay own-code;
  the candidate decides and justifies it. ndarray/BLAS backends are out of scope
  (may be removed or left untouched behind their features).
- The public API may break. Give a short migration note in DESIGN_NOTES.md.

## What is wrong today (commit c93b68c)
- Two parallel implementations of one algorithm: `KalmanFilter<T>` with flat `Vec<T>`
  matrices (src/filter.rs:32) and `StaticKalmanFilter<T, M, N>` on nalgebra
  (src/filter.rs:460) duplicate predict/update. On top: src/unified.rs (720 lines: an
  enum, `IntoUnified`, `KalmanFilterTrait`), src/legacy.rs, src/builder.rs and
  src/builders/*. Four ways to construct and wrap one algorithm.
- Covariance update is the simple form `P = (I − KH)P` plus symmetrization
  (src/filter.rs:328-347 and :533), not Joseph form. `test_joseph_form`
  (src/filter.rs:1065) only asserts P shrinks, which the simple form also satisfies —
  a test that cannot fail.
- Innovation covariance S is explicitly inverted (src/filter.rs:297) instead of solved
  (e.g. Cholesky).
- ~10 heap allocations per predict+update (`vec![…]` temporaries) and a hand-rolled
  triple loop for P·Hᵀ (src/filter.rs:309) bypassing the backend.
- Prometheus metrics and `log` trace blocks are interleaved with every numeric step
  of predict/update.
- src/legacy.rs accepts `control_dim` and documents it as "unused" — input silently
  ignored. Control input is not supported.
- Public mutable fields (`kf.x`, `kf.P`, …): invariants (dimensions, symmetric PSD
  covariance) cannot be enforced.
- Panic paths reachable from the public API: integer division by a zero dimension
  (src/filter.rs:136) and indexing before length checks (src/builder.rs:377). (The
  `.unwrap()`s in filter.rs/matrix/ are in test modules; the non-test ones are in the
  feature-gated ndarray backend.)
- src/legacy.rs is not declared in lib.rs, so it is dead code that never compiles.

## Quality bar
- Joseph-form (or equally stable) covariance update; solve with S rather than invert it.
- Covariance stays symmetric PSD over 10⁴ steps on an ill-conditioned model, f32 and f64.
- No heap allocation in steady-state predict/update for fixed dimensions.
- Logging/metrics are never interleaved with the math.
- One validated construction path (see SPEC.md); dimensions and inputs validated there.

## Non-goals
Nonlinear filters, particle/ensemble filters, OpenCV, redesigning prometheus
integration, ndarray/BLAS backends, no_std (keep it possible; don't implement it).
Do not add features beyond what SPEC.md requires.

## Note to the spec author: the contract's public interface
Because the API may break, SPEC.md defines a *new, minimal* public interface for the
linear filter (construction with validated dimensions, predict, update, state and
covariance accessors, error type), small enough not to dictate internal design.
The frozen tests in tests/spec_linear_kf.rs target only that interface. To make KEEP
tests compile and pass on today's code, you may add a thin adapter (e.g. a
`kalman_filters::linear` module in src/filter.rs) that maps the new interface onto
the current implementation. Candidates will replace it. Don't fix any defects in
the adapter: CHANGE tests should fail on it.

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

# Starting point: candidate A
This worktree already contains candidate A's code, which passes all 32 frozen tests.
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
  `cargo test --quiet --test spec_linear_kf && cargo test --quiet --lib` at each checkpoint.
- Consult the cited candidate code for ideas, edge cases, and tricky algorithms. Do not
  copy whole files: write code that fits *this* design, with consistent naming and
  error handling throughout.
- You own `src/filter.rs src/matrix src/types.rs src/error.rs src/builder.rs src/unified.rs src/legacy.rs src/validation.rs`. **Never modify `SPEC.md tests/spec_linear_kf.rs`.** Outside scope, make
  only the minimal mechanical edits.
- Build nothing beyond the spec and the design. If the design is wrong or impossible
  somewhere, deviate minimally and document it.
- Build: `cargo build --all-targets --quiet`. Tests: `cargo test --quiet --test spec_linear_kf && cargo test --quiet --lib`. Lint: `cargo clippy --all-targets --quiet`. All must be clean.
- Do not commit; the harness commits.

# DESIGN_NOTES.md (required, at the repository root)
1. Architecture in ≤ 10 lines.
2. **Deviations from TOURNAMENT_DESIGN.md** and the reason for each.
3. Known weaknesses and next steps.

Finish with: test status, LOC, and a list of deviations.
