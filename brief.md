
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
