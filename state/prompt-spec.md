You are writing the **contract** for a design-tournament *reboot* of existing code.
Several independent agents will later rewrite the code under `src/filter.rs src/matrix src/types.rs src/error.rs src/builder.rs src/unified.rs src/legacy.rs`, each
with a different architecture, and they will be compared against what you write. You
define *what* the code must do, never *how*.

# Brief

## Goal
Rebuild the *linear* Kalman filter core of the `kalman_filters` crate (state model,
predict, update, matrix layer, construction API) as one clean, numerically robust
implementation that the nonlinear filters (EKF/UKF/CKF/IF) can later be rebuilt on.

## Scope
Candidates own: src/filter.rs, src/matrix/, src/types.rs, src/error.rs, src/builder.rs,
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
- `.unwrap()` in library code (27 in filter.rs + matrix/).

## Quality bar
- Joseph-form (or equally stable) covariance update; solve with S rather than invert it.
- Covariance stays symmetric PSD over 10⁴ steps on an ill-conditioned model, f32 and f64.
- No heap allocation in steady-state predict/update for fixed dimensions.
- Logging/metrics are never interleaved with the math.
- One way to construct a filter, with dimensions validated at construction.

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

# Step 1: understand the current code
Read the code under `src/filter.rs src/matrix src/types.rs src/error.rs src/builder.rs src/unified.rs src/legacy.rs`, its callers elsewhere in the repo, its existing
tests, and its docs. Build a list of the public behaviors it actually has. Then sort
each one into:
- **KEEP**: correct and wanted.
- **CHANGE**: a defect, or a behavior the brief says must change (wrong math, panics,
  silently ignored inputs, misleading names, tests that test the wrong thing).
- **DROP**: dead, duplicated, or out-of-scope functionality the brief allows removing.

Verify suspected defects by reading carefully and, where cheap, by writing a quick
failing test. Do not guess.

# Step 2: deliverables (in this repository)
1. `SPEC.md` with these sections:
   - **Purpose**.
   - **Public interface**: what callers may rely on after the rewrite. State whether
     the existing API must be preserved exactly, may change with a migration note, or
     is free; follow the brief.
   - **Behaviors**: numbered (`B1`…), each tagged KEEP or CHANGE, with the defect
     explained for CHANGE items (cite file:line in the current code).
   - **Dropped**: what candidates may delete.
   - **Quality requirements** and **Non-goals**.
   - **Test index**: test name → behavior IDs.
2. **Acceptance tests** under `SPEC.md tests/spec_linear_kf.rs`, against the public interface only:
   - Characterization tests for KEEP behaviors. These must pass on the current code.
   - Tests for CHANGE behaviors, named with the prefix `spec_change_`. These are
     expected to fail on the current code; list them in SPEC.md.
   - If existing tests in these paths are wrong (assert buggy behavior, or test names
     that lie), fix or delete them and record that in SPEC.md under "Test curation".
   - Deterministic (fixed seeds), with float tolerances.

# Rules
- Do not fix or refactor the implementation code. Only the contract.
- Run `cargo test --quiet --test spec_linear_kf && cargo test --quiet --lib`. KEEP tests pass; `spec_change_*` tests fail for the documented
  reason, not because of compile errors (if a CHANGE needs an API that does not exist
  yet, put the test behind the smallest stub needed to compile, and say so).
- Do not commit; the harness commits.

Finish with a summary: the counts of KEEP/CHANGE/DROP, the test counts, and the most
important defects found.
