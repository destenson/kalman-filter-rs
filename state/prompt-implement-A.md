You are one of several engineers independently implementing the same specification.
Each engineer was assigned a different design stance so that the team can explore the
design space. Afterwards, reviewers compare all the implementations on:

1. **Correctness**: the frozen acceptance tests, plus a careful read for edge cases.
2. **Simplicity**: how little a newcomer must understand to work in the code.
3. **Ease of change**: other engineers will later implement *unseen* follow-up
   features in your codebase. How much they have to touch, and how much the design
   fights them, is measured.

The best ideas from every implementation will be harvested, so a clear, honest
implementation of your stance is more useful than a hedge toward the average.

# Your design stance
**Data-oriented / performance-first.** Start from the data layout and the hot path.
Preallocate; aim for zero allocation in steady-state operations; reuse buffers; keep
data contiguous; batch operations; know the cost of every call. Abstractions are
allowed only where they compile away.
Avoid: per-call heap allocation, needless copies, dynamic dispatch on hot paths.
Likely failure mode: API ergonomics and readability suffer; premature specialization.

Commit to the stance. If, after a genuine attempt, part of it clearly cannot work for
this problem, deviate only there and explain why in DESIGN_NOTES.md. That finding is
valuable.

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

# The contract
Read `SPEC.md` and the acceptance tests in `SPEC.md tests/spec_linear_kf.rs` first.

# Rules
- You own `src/filter.rs src/matrix src/types.rs src/error.rs src/builder.rs src/unified.rs src/legacy.rs src/validation.rs`: restructure, rename, delete, or add files there freely.
- **Never modify anything under `SPEC.md tests/spec_linear_kf.rs`.** Edits there are detected
  automatically and disqualify the candidate. If you believe a test is wrong, say so
  in DESIGN_NOTES.md and leave it failing.
- Outside your scope, make only the minimal mechanical edits needed to keep the
  project building. Every such file is counted and reported.
- You may add your own tests in new files.
- Build only what the spec asks for. Speculative features and abstractions with one
  implementation count against you, unless your stance specifically calls for them.
- Iterate until these pass or you have exhausted reasonable approaches:
  - build: `cargo build --all-targets --quiet`
  - tests: `cargo test --quiet --test spec_linear_kf && cargo test --quiet --lib`
  - lint: `cargo clippy --all-targets --quiet` (fix warnings in your code)
- Do not commit; the harness commits your worktree.

# DESIGN_NOTES.md (required, at the repository root, ≤ 150 lines)
Reviewers read this blind, so **do not name or label your stance**. Describe the design itself.
1. **Architecture in ≤ 10 lines**: the main types/modules and how data flows.
2. **Key decisions**: each with the alternative you rejected and why.
3. **What fought back**: where the problem resisted this design.
4. **Known weaknesses**: be candid; reviewers will find them anyway.
5. **Next changes**: what you expect to be easy or hard to add later.
6. (reboot) **What was worth keeping** from the original code, and what was not.

Finish with a 5-line summary: test status, LOC, and the single most important design idea.
