# SPEC — linear Kalman filter core (design-tournament contract)

Baseline: commit `c93b68c`. Candidate-owned paths: `src/filter.rs`, `src/matrix/`,
`src/types.rs`, `src/error.rs`, `src/builder.rs`, `src/unified.rs`, `src/legacy.rs`.
Frozen acceptance tests: `tests/spec_linear_kf.rs`.

## 1. Purpose

Provide **one** linear Kalman filter (state model, predict, update, matrix layer,
construction) for `f32` and `f64`. It must be numerically robust, must not panic, and must
not allocate in steady state. The nonlinear filters (EKF/UKF/CKF/IF) will later be rebuilt
on top of it. This spec says *what* the filter must do, not *how* it is built.

## 2. Public interface

**The existing API may break**, provided `DESIGN_NOTES.md` has a short migration note
(old type or method → new one). Callers may rely only on the items below. Anything else a
candidate exposes is the candidate's choice, but there must still be one validated way to
construct the linear filter, plus at most one delegating front end (see B16, Q5).

```rust
// crate root
pub use kalman_filters::{KalmanError, KalmanResult};        // KalmanResult<T> = Result<T, KalmanError>
pub mod kalman_filters::linear;

// kalman_filters::linear
pub struct LinearKalmanFilter<T>;          // T = f32 and T = f64 must work. No public fields.

impl LinearKalmanFilter<T> {
    /// All matrices are dense, row-major slices:
    /// x0: n, p0: n×n, f: n×n, q: n×n, h: m×n, r: m×m.
    pub fn new(state_dim: usize, measurement_dim: usize,
               x0: &[T], p0: &[T], f: &[T], q: &[T], h: &[T], r: &[T])
        -> KalmanResult<Self>;

    pub fn state_dim(&self) -> usize;                  // n
    pub fn measurement_dim(&self) -> usize;            // m
    pub fn predict(&mut self) -> KalmanResult<()>;
    pub fn update(&mut self, z: &[T]) -> KalmanResult<()>;
    pub fn state(&self) -> impl AsRef<[T]> + '_;       // length n (e.g. &[T] or Vec<T>)
    pub fn covariance(&self) -> impl AsRef<[T]> + '_;  // n×n, row-major
}
```

- Any bound on `T` is allowed, as long as `f32` and `f64` satisfy it. The tests only use
  concrete `f32` and `f64`.
- `KalmanError` must keep these variants, which the tests match on:
  `DimensionMismatch { .. }`, `SingularMatrix`, `InvalidCovariance`,
  `InvalidNoiseCovariance`. The exact field values of `DimensionMismatch` are not
  specified. Variants may be added. Variants used only by out-of-scope modules must stay
  while those modules use them.
- `state()` and `covariance()` may return borrowed or owned data. Tests only call
  `.as_ref()` on the result.
- The adapter in `src/filter.rs` (`pub mod linear`, re-exported from `lib.rs` as
  `kalman_filters::linear`) maps this interface onto the current `KalmanFilter<T>` and
  fixes nothing. Candidates replace it.
- Other modules use today's internals: `matrix::{DefaultBackend, MatrixOps,
  MatrixDecomposition}`, `filter::KalmanFilter::{initialize, invert_matrix}`, and
  `KalmanScalar`. Examples include `extended.rs`, `information/*`, `builders/information.rs`,
  examples, benches and `tests/*`. Candidates may change those call sites only with
  minimal mechanical edits, or keep thin shims, so that `cargo build --all-targets` stays
  green with default features. Older tests in `tests/*.rs` that use removed APIs may be
  mechanically ported or `#[cfg]`-gated out. Record any such change in `DESIGN_NOTES.md`.

## 3. Behaviors

Notation: n = state_dim and m = measurement_dim. A matrix is **PSD** if it is symmetric and
positive semi-definite, within a small relative tolerance appropriate to `T`. Exactly
symmetric input must always pass the symmetry check.

### Construction

- **B1 (KEEP): Construction and accessors.** A valid model constructs successfully.
  Immediately after construction, `state()` returns `x0` and `covariance()` returns `p0`
  exactly. `state_dim()` and `measurement_dim()` report n and m. Works for `f32` and `f64`.
  Today: `KalmanFilter::initialize`, `src/filter.rs:76-183`.
- **B2 (KEEP): Dimension validation.** If any of `x0, p0, f, q, h, r` has the wrong length,
  construction returns `Err(KalmanError::DimensionMismatch { .. })`. Today:
  `src/filter.rs:92-138`.
- **B3 (CHANGE): Zero dimensions are rejected, without panicking.** `state_dim == 0` or
  `measurement_dim == 0` → `Err(_)`.
  *Defect:* `initialize` builds error payloads with `len / n` and `len / m`
  (`src/filter.rs:112,118,124,130,136`), so for example `(n=1, m=0, r=[1.0])` panics with
  "attempt to divide by zero". With consistent empty inputs, zero dimensions are silently
  accepted and produce a degenerate filter.
- **B4 (CHANGE): `p0` must be PSD**, otherwise `Err(KalmanError::InvalidCovariance)`. This
  covers asymmetric, negative-diagonal and indefinite `p0`. A singular PSD `p0`, including
  all zeros, is valid.
  *Defect:* `initialize` runs `validate_covariance` but only logs a warning on failure
  (`src/filter.rs:141-147`). It never checks definiteness. The builder checks symmetry
  only (`src/builder.rs:169`). The builder's symmetry check also indexes the matrix
  *before* checking its length, so a short `initial_covariance` panics
  (`src/builder.rs:377`).
- **B5 (CHANGE, partly KEEP): `q` and `r` must be PSD**, otherwise
  `Err(KalmanError::InvalidNoiseCovariance)`. A singular PSD `q` or `r` (for example
  `q = 0`, or a rank-deficient `r`) is valid at construction.
  - KEEP: an `r` with a negative diagonal is rejected (`src/validation.rs:56` via
    `src/filter.rs:150`).
  - CHANGE: `q` is never validated by `initialize`, so an asymmetric, negative or
    indefinite `q` is accepted. `r` is only checked on its diagonal
    (`src/validation.rs:45-65`), so an asymmetric `r`, or an indefinite `r` with a
    positive diagonal, is accepted.
  - Not specified: whether `r` with a zero diagonal is accepted. Today it is rejected, and
    the tests do not cover it.
- **B6 (CHANGE): Non-finite input is rejected.** A NaN or ±∞ in any construction input →
  `Err(_)`. A NaN or ±∞ in the `update` measurement → `Err(_)`, with state and covariance
  unchanged.
  *Defect:* NaN passes every comparison in `validate_covariance` and
  `validate_measurement_noise` (`src/validation.rs:12-42,45-65`). `update` copies `z`
  without checking it (`src/filter.rs:267`), so one NaN measurement permanently poisons
  `x` and `P`.

### Predict / update

- **B7 (KEEP): Predict.** `x ← F x`, `P ← F P Fᵀ + Q` (`src/filter.rs:186-233`).
  `predict` must not panic. It may return `Err(KalmanError::FilterDivergence(_))` if the
  result is non-finite. That case is not tested.
- **B8 (KEEP): Update equations.** `y = z − Hx`, `S = HPHᵀ + R`, `K = PHᵀS⁻¹`,
  `x ← x + Ky`. The posterior `P` is mathematically equal to `(I−KH)P(I−KH)ᵀ + KRKᵀ`.
  Results must match a textbook f64 oracle to a relative 1e-9 (f64) or 1e-4 (f32) over 50
  steps of a 4-state/2-measurement model with correlated `R`. Further consequences that
  must hold: the scalar closed form, convergence of a scalar random walk to the algebraic
  Riccati steady state, and an unobserved state's variance growing as `P0 + kQ`.
  Monte-Carlo consistency must hold (average NEES ≈ n, NIS ≈ m) with a fixed seed.
- **B9 (KEEP): Update errors are atomic.** If `z.len() != m`, `update` returns
  `Err(DimensionMismatch { .. })`. If `S` is singular or not positive definite, it returns
  `Err(KalmanError::SingularMatrix)`. In both cases `state()` and `covariance()` are
  unchanged and the filter stays usable. Today: `src/filter.rs:250-262,299-305`. All
  mutation happens after these checks.
- **B10 (CHANGE): Singularity detection must be scale-invariant.** A well-conditioned `S`
  must never be reported singular just because its entries are small in absolute terms.
  *Defect:* `S` is inverted by Gauss–Jordan elimination, with an absolute pivot threshold
  of `KalmanScalar::epsilon()` (1e-10 for f64, 1e-6 for f32): `src/matrix/naive.rs:147`,
  reached via `src/filter.rs:299`. The same absolute threshold is in
  `KalmanFilter::invert_matrix` (`src/filter.rs:400,415`). With f64 `P = R = 4e-12` (`S = 8e-12`), or f32
  `P = R = 1e-7`, `update` returns `SingularMatrix`. In f32, ordinary models hit this
  thousands of times per 10⁴ steps.
- **B11 (CHANGE): The covariance stays symmetric PSD on ill-conditioned models.** This
  must hold over 10⁴ predict+update steps, for both `f32` and `f64`, on the two frozen
  models below. Every step must succeed, and `P` must stay finite, with relative asymmetry
  ≤ 1e-10 (f64) / 1e-4 (f32) and `λ_min ≥ −tol·λ_max`, where tol = 1e-9 (f64) / 1e-5
  (f32). The update must use the Joseph form or an equally stable one (UD or
  square-root, for example), and must *solve* with `S` (Cholesky/LDLᵀ) rather than
  invert it.
  - **Model A:** constant velocity, `P0 = 1e8·I`, `R = 1e-8`, tiny `Q`.
  - **Model B:** nearly collinear double measurement, `H = [[1,1],[1,1.01]]`,
    `R = 1e-4·I`, `Q = 0`.

  *Defect:* the update uses the simple form `P = (I−KH)P` followed by symmetrization
  (`src/filter.rs:329-347`; also `StaticKalmanFilter` at `src/filter.rs:530-533`), and
  explicitly inverts `S` (`src/filter.rs:296-305`, `:520`). Measured on the current code:
  model A in f64 reaches `λ_min = −6.9e-11` at step 1 (with `λ_max = 1.3e-8`); model B in
  f32 reaches `λ = −5.1e-5` at step 0. A reference Joseph + Cholesky implementation passes
  both models in both precisions. The old unit test `test_joseph_form`
  (`src/filter.rs:1065`) could not fail; see the test curation list.
- **B12 (CHANGE): No heap allocation in steady state.** After construction plus one
  warm-up cycle, `predict` and `update` perform **zero** heap allocations. This is
  measured with a counting global allocator over 100 cycles of a 4×2 model in f32 and f64.
  *Defect:* each predict+update cycle does 15 `vec![]` / `to_vec()` allocations
  (`src/filter.rs:198,204,208,265,267,288,292,298,309,319,323,331,335,343` and the
  `invert` scratch buffer at `src/matrix/naive.rs:123`). Measured: 1500 allocations per
  100 cycles.

### Non-test behaviors (checked in review)

- **B13 (CHANGE): No mutable public state.** No public fields. The invariants in B2–B6
  cannot be bypassed after construction.
  *Defect:* `KalmanFilter` has `pub x, P, F, Q, H, R, state_dim, measurement_dim`
  (`src/filter.rs:37-51`), and `StaticKalmanFilter` is the same (`src/filter.rs:465-475`).
- **B14 (CHANGE): No silently ignored input.** No public API may accept a parameter and
  then ignore it.
  *Defect:* `legacy::KalmanFilter::new(.., control_dim)` stores `control_dim` and never
  uses it, and `predict(_control)` drops the control vector (`src/legacy.rs:22,58`).
  `correct` discards update errors (`src/legacy.rs:63-65`). Control input is out of scope
  for this tournament (see Non-goals). The fix is to *not offer* it.
- **B15 (CHANGE): No panic on bad input from the public API.** Every invalid input must
  produce a `KalmanError`. That includes wrong lengths, zero dimensions, non-finite
  values and singular `S`.
  *Defect:* B3 (divide by zero). The builder panics on short slices (`src/builder.rs:377`;
  `SMatrix::from_row_slice` at `src/builder.rs:268-322` under `nalgebra`). The public
  `matrix::MatrixOps` / `MatrixDecomposition` functions index caller slices unchecked
  (for example `src/matrix/naive.rs:14-20`). Correction to the brief: with default
  features, `src/filter.rs` and `src/matrix/naive.rs` have no `.unwrap()` outside
  `#[cfg(test)]`. The non-test `unwrap`/`expect` calls are in
  `src/matrix/ndarray_backend.rs` (`blas-backend` feature, 26 occurrences). The real
  default-build panic paths are integer division and out-of-bounds indexing.
- **B16 (CHANGE): One construction path.** Today there are `KalmanFilter::new` (all-zero
  matrices plus public fields), `KalmanFilter::initialize`, `StaticKalmanFilter::new`
  (no validation at all, `src/filter.rs:484`), `KalmanFilterBuilder::build`
  (`src/builder.rs:128,200`; it picks different backends by dimension and validates
  differently per path), `UnifiedKalmanFilter`/`IntoUnified`, and `legacy::KalmanFilter`.
  These collapse to one *validated* path: `LinearKalmanFilter::new`. A candidate may
  add **one** typed or ergonomic front end (e.g. compile-time dimensions, or a
  builder) provided it delegates to the same validation and adds no behavior of its
  own. Wrappers, unified enums and per-backend constructors are not allowed.
- **B17 (KEEP): `f32` and `f64` are both supported** (`KalmanScalar` impls,
  `src/types.rs:51-62`).

## 4. Dropped (candidates may delete)

- `src/legacy.rs`. It is dead code: it is not declared in `src/lib.rs`, so it is never
  compiled.
- `src/unified.rs`: the `UnifiedKalmanFilter` enum, `IntoUnified`, `KalmanFilterTrait`
  and its `set_*` model setters. Time-varying model setters are not part of this
  contract.
- `src/builder.rs`: `KalmanFilterBuilder` (replaced by `LinearKalmanFilter::new`). The
  re-exports in `lib.rs` may be removed or replaced.
- `StaticKalmanFilter<T, M, N>` as a separate public algorithm. A candidate may still use
  const-generic or nalgebra storage *internally*.
- `KalmanFilter<T>`'s public API: `new` with zeroed matrices, the public fields, and the
  public `invert_matrix`.
- The public `matrix` traits (`MatrixOps`, `MatrixDecomposition`, `MatrixBackend`) and
  the backend-selection features, as a public API. Explicit inversion of `S`. The
  ndarray/BLAS backends may be deleted or left untouched behind their features.
- `KalmanScalar::Epsilon(&self)` (deprecated) and the deprecated `types::{KalmanError,
  KalmanResult}` re-export, if no out-of-scope module needs them.
- Logging and metrics calls inside predict/update. Any instrumentation must sit outside
  the numeric code (see Q4).

## 5. Quality requirements

- **Q1 Numerical robustness:** B10 and B11. Use the Joseph form or an equally stable
  update. Solve with `S`; never invert it.
- **Q2 Allocation:** B12. Zero heap allocations in steady-state predict and update.
- **Q3 Safety:** B15. No `.unwrap()`, `.expect()`, unchecked indexing that can fail on
  user input, or `panic!` reachable from the public API. Integer division by a
  user-supplied dimension must be guarded.
- **Q4 Separation of concerns:** `log` and Prometheus calls are never interleaved with the
  numeric steps of predict/update. If instrumentation is kept, it wraps the math (for
  example before and after a call) or lives in a separate layer.
- **Q5 One validated construction path** (B16, B2–B6), plus at most one front end that delegates to it.
- **Q6 Build:** Rust edition 2024. `cargo build --all-targets` is green with default
  features and needs no system libraries. nalgebra may become a required dependency, or
  the matrix layer may stay own-code. The choice must be justified in `DESIGN_NOTES.md`.
- **Q7 Docs:** `DESIGN_NOTES.md` contains the architecture rationale, the nalgebra
  decision, and the migration note.

## 6. Non-goals

- Nonlinear filters (EKF/UKF/CKF/IF), particle and ensemble filters. They only need to
  keep compiling.
- Control input (`B·u`), time-varying model setters, smoothing, and adaptive noise.
- OpenCV, a Prometheus redesign, ndarray/BLAS backends, and implementing `no_std` (but do
  not foreclose it).
- Any feature beyond this spec.

## 7. Test index (`tests/spec_linear_kf.rs`)

| Test | Behaviors | Status on c93b68c |
|---|---|---|
| `construction_reports_dimensions_and_initial_estimate` | B1, B17 | pass |
| `construction_rejects_wrong_length_inputs` | B2 | pass |
| `rejects_negative_measurement_noise` | B5 | pass |
| `accepts_singular_positive_semidefinite_matrices` | B4, B5 | pass |
| `predict_propagates_state_and_covariance` | B7 | pass |
| `scalar_update_matches_closed_form` | B8 | pass |
| `matches_reference_filter_f64` | B8 | pass |
| `matches_reference_filter_f32` | B8, B17 | pass |
| `scalar_random_walk_reaches_riccati_steady_state` | B8 | pass |
| `unobserved_state_keeps_growing_uncertainty` | B8 | pass |
| `estimates_are_statistically_consistent_nees_nis` | B8 | pass |
| `update_rejects_wrong_measurement_length_and_leaves_filter_unchanged` | B9 | pass |
| `update_with_singular_innovation_covariance_fails_atomically` | B9 | pass |
| `covariance_stays_symmetric_psd_well_conditioned` | B8, B11 (benign baseline) | pass |
| `spec_change_zero_dimensions_are_rejected_without_panic` | B3, B15 | **fail**: panic "attempt to divide by zero" at `src/filter.rs:136` |
| `spec_change_rejects_asymmetric_initial_covariance` | B4 | **fail**: accepted (`Ok`) |
| `spec_change_rejects_indefinite_initial_covariance` | B4 | **fail**: accepted (`Ok`) |
| `spec_change_rejects_invalid_process_noise` | B5 | **fail**: accepted (`Ok`) |
| `spec_change_rejects_asymmetric_or_indefinite_measurement_noise` | B5 | **fail**: accepted (`Ok`) |
| `spec_change_rejects_non_finite_construction_inputs` | B6 | **fail**: NaN accepted |
| `spec_change_update_rejects_non_finite_measurement` | B6 | **fail**: NaN measurement accepted |
| `spec_change_small_scale_update_is_not_reported_singular` | B10 | **fail**: `SingularMatrix` for `S = 8e-12` |
| `spec_change_covariance_stays_psd_ill_conditioned_f64` | B11 | **fail**: model A, step 1, `λ_min = −6.9e-11` |
| `spec_change_covariance_stays_psd_ill_conditioned_f32` | B11, B10 | **fail**: model B, step 0, `λ_min = −5.1e-5` |
| `spec_change_no_heap_allocation_in_steady_state` | B12 | **fail**: 1500 allocations per 100 cycles |

Totals: 25 tests, of which 14 are KEEP (pass) and 11 are `spec_change_*` (fail on the
adapter for the reasons listed, never because of compile errors). No stub was needed: the
whole interface maps onto the current code.

Behaviors B13, B14 and B16 are checked in review. They cannot be observed through the
public interface; B13, for example, would need a compile-fail test.

## 8. Test curation (existing tests in candidate-owned paths)

- `src/filter.rs::tests::test_joseph_form` was renamed to
  `test_scalar_update_shrinks_covariance`. It asserted only that `P > 0` and that `P`
  shrinks, which the simple (non-Joseph) form also satisfies, so it could never fail and
  its name was false. Joseph-level stability is now tested by
  `spec_change_covariance_stays_psd_ill_conditioned_{f32,f64}`.
- `src/filter.rs::tests::test_measurement_outlier_robustness` was renamed to
  `test_measurement_outlier_pulls_estimate`. It asserts that the filter is *not* robust,
  so the old name was inverted.
- Kept, with notes for candidates:
  - The other `src/filter.rs` unit tests use the public fields and `KalmanFilter::new`.
    They characterize behaviors now covered by B7–B9 and may be deleted or ported along
    with the dropped API.
  - `test_matrix_inversion` and `test_singular_matrix_detection` test the dropped
    `invert_matrix`.
  - The `src/builder.rs` tests test the dropped builder.
  - `src/matrix/tests.rs` and `naive.rs` test the dropped public backend traits.
  - The `src/error.rs` and `src/types.rs` tests remain valid as long as those items
    exist.
