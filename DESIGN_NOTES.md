# Linear Kalman filter core — design notes

## 1. Architecture

- `src/matrix/mod.rs`: crate-private, dependency-free, allocation-free row-major kernels (matmul, upper-triangle `A·Bᵀ`, mirror, Cholesky + row solve) and the crate's one rounding tolerance `rel_tol(k) = 64·k·ε`. A marked "Transitional" block keeps `matvec`/`matmul_transpose`/`symmetrize`/`invert_matrix` for the nonlinear filters.
- `src/validation.rs`: `is_psd` (construction: unit-diagonal-scaled, so units don't matter) and `psd_to_working_precision` (step acceptance, SPEC B11/B21: `λ_min ≥ −ε^¾·max p_ii`).
- `src/filter.rs`: `LinearKalmanFilter<T>` owns the model (F, Q, H, R), the estimate (x, P) and a fixed `Workspace`. `try_new` is the only validator. The order is dimensions → lengths → finiteness → PSD, one cause per error variant. Stored P0/Q/R are made exactly symmetric by mirroring, which needs no arithmetic.
- predict: `x ← Fx`, `P ← FPFᵀ + Q`, computing the upper triangle and mirroring it. update: Cholesky of S with a pivot test relative to each diagonal, `Kᵀ = S⁻¹HP` by triangular solves, Joseph form. Both write only into the workspace. `commit(on_err)` checks finiteness and PSD, then swaps the candidate in (O(1)). A failed step leaves the filter untouched, and steady state does no heap allocation.
- `src/builder.rs`: `Covariance::{Full, Diagonal, Scalar}`, `LinearKalmanFilter::builder(..).…​.try_build()` and `try_constant_velocity(..)`. They only expand their inputs into dense slices, using `checked_mul` and `try_reserve_exact`, then call `try_new`.
- The public path is `kalman_filters::linear::{LinearKalmanFilter, LinearKalmanFilterBuilder, Covariance}`. `mod filter` is private, and the model accessors are `pub(crate)`.

### Migration
- Use `kalman_filters::linear::LinearKalmanFilter`. The root re-export and the `kalman_filters::filter` path are gone.
- `new(..)` → `try_new(..)`, which takes the same 8 arguments. `new` is a temporary `#[doc(hidden)]` forward that exists only for the frozen tests.
- `KalmanError::InvalidDimension{..}` is gone. Zero n/m now gives `ZeroDimension`, and an overflowing `rows×cols` gives `DimensionMismatch`. NaN/∞ in `p0`, `q` or `r` now gives `NonFiniteInput`, not a covariance error.
- `transition()`, `process_noise()`, `observation()` and `measurement_noise()` are no longer public.

## 2. Deviations from TOURNAMENT_DESIGN.md

1. **The post-step guard is a real PSD test, not `variances_consistent`** (§3 "keep the guard", step 2 check). `tests/linear_kf_robustness.rs` failed with A's O(n²) guard, which only checks 2×2 minors. f32 reached `λ_min/λ_max = −1.2e-3` at step 1 (n=5, m=4), and f64 reached `−2e-2` (n=5, m=3). The f32 failure is bit-identical with every one of my grafts reverted, so it already existed in A's WIP. That violates SPEC B21: "every update that returns Ok leaves P … PSD within the B11 tolerances, on any valid model".
   - The guard is now `psd_to_working_precision`: a Cholesky factorisation of `P + τ·max(p_ii)·I` in the `ws.a` scratch buffer, so there is still no allocation. Its cost is O(n³), the same order as the step itself.
   - I first tried the unit-diagonal `is_psd`. It is stricter than the spec, which measures relative to λ_max, so I did not keep it.
   - `τ = ε^¾` (6.4e-6 for f32, 1.8e-12 for f64) is chosen from the spec bounds (1e-5 / 1e-9), not tuned to tests. `rel_tol` (64·n·ε) can't be used here because in f32 it exceeds the spec's 1e-5 once n ≥ 2. It is the only tolerance besides `rel_tol`, and it is an acceptance criterion, not a rounding allowance.
   - `variances_consistent` is deleted, so there is no second PSD notion.
2. **Step 3 triage (illcond).** I ran the 270-case difftest harness locally (copied as an untracked test, then removed).
   - `illcond00`: the Cholesky pivot test fires. That is B21-correct; kept.
   - `illcond04`, `10`, `18`, `19`: the step guard fires. Measured by bisection, these posteriors need τ between 2e-9 and 3.4e-8 relative to max p_ii, which is outside the spec's own f64 tolerance of 1e-9. A committed them, and its `illcond18`/`19` values disagreed with the oracle by 6.7e-2 and 2.7e-4.
   - Result: 255 agree, **0 value mismatches** (A had 2), 5 status mismatches (all `illcond*`, now `SingularMatrix`). All `edge_*` statuses match the consensus variant set.
   - The design expected "no new mismatches elsewhere". `illcond10` is new, which follows from item 1 and the spec criterion, so I accepted it.
3. **`rel_tol` is bounded on `KalmanScalar`, not `Float`.** It uses the crate's `from_usize`, so no `unwrap_or_else(T::max_value)`-style fallback is needed. The one `<T as Float>::epsilon()` lives inside it, because `KalmanScalar::epsilon` shadows the `Float` one. `is_psd` therefore takes `T: KalmanScalar`.
4. **The early `y` finiteness check is removed (graft 4), but the `CholeskyError::NonFinite → FilterDivergence` mapping stays.** That path is not a duplicate of commit's check.
5. **`check_dims(n, m)` is shared by `try_new` and `try_build`.** The builder must check F/H lengths before it allocates from n or m (§6), and sharing the function keeps the zero-dimension rule in one place.
6. **Allocation failure maps to `DimensionMismatch`.** In the builder and helper, overflow and failed `try_reserve_exact` both return `DimensionMismatch { expected: (rows, cols), actual: (0, 1) }`, following the §6 table ("dimension product overflows"). There is no new variant.
7. **Non-frozen call sites were switched to `try_new`**: benches, examples, `information/conversion.rs`, unit tests and `linear_kf_robustness.rs`. Only the frozen spec tests use `new` now, so step 7 is a one-function deletion.
8. Graft 2's `Workspace::new(n, m)` was applied in step 1, together with `check_len`, because they are coupled.

## 3. Known weaknesses and next steps
- The Joseph form is not a square-root filter. On very ill-conditioned updates the step guard turns rounding failure into `Err(SingularMatrix)` instead of producing a usable P. A UD or square-root update would let those steps succeed.
- The builder and helper can't guard against huge but *non-overflowing* sizes on overcommitting systems. `try_reserve_exact` may succeed and touching the memory may then OOM. This only affects `Diagonal`/`Scalar` R with m ≫ n, or `try_constant_velocity` with a huge `axes`.
- §7 out-of-scope items:
  - `matrix::invert_matrix` now uses a relative `n·ε·max|A|` pivot test where the baseline used an absolute ε. This changes the EKF/UKF/CKF/IF singularity behaviour, probably for the better.
  - `extended.rs` aliases `use crate::matrix as DefaultBackend;`. It is transitional.
  - `examples/sensor_fusion.rs` rebuilds the filter on every sensor switch, which re-allocates and re-validates per measurement. That is acceptable in an example.
  - `information/conversion.rs` now propagates `predict()?` errors, and `information_to_kalman` rejects non-PSD input. Both are intended.
  - The Cargo features `naive-matrix`, `blas-backend` and `ndarray-linalg*` gate nothing and are dead.
- `cargo clippy --all-targets` is clean on every owned file. Repo-wide it still reports baseline errors in `examples/particle_filter_robot.rs:75` and `examples/sensor_fusion.rs:189`, plus warnings in the nonlinear modules and benches. All of them are already in `_baseline` and out of scope.
- Step 7, deleting `new`, is left for after the frozen tests switch to `try_new`.
