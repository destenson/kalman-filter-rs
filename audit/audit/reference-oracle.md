I found 3 defects that SPEC.md doesn't list, and a fourth (AUD-2) that extends its B10. Each has a failing test in `tests/audit_reference_oracle.rs`, checked against an f64 reference written inside that test file: textbook equations, Joseph covariance update, S solved with complete pivoting. The file has 7 tests. The 3 sanity tests pass on the current code; the 4 defect tests fail for the reasons below. Run it with `cargo test --test audit_reference_oracle`.

The sanity tests show the core math is right when inputs are well scaled. That covers every shape from n, m = 1 to 6 (including m > n) over 50 steps in f32 and f64, a 10⁴-step f64 run with no drift, and uniform scaling up to 1e140 (f64) and 1e15 (f32). All of the defects come from **fixed thresholds that ignore the scale of the numbers**: `KalmanScalar::epsilon()` in `src/matrix/naive.rs:147` and the 1e-12 floor in `src/validation.rs:59`.

## Confirmed

**AUD-1** | A tiny but strictly positive measurement noise R is rejected.
- **Where:** `src/validation.rs:59`, reached from `src/filter.rs:150`. Every diagonal entry of R must be at least an absolute 1e-12.
- **Test:** `audit_tiny_positive_measurement_noise_is_accepted`
- **Evidence:** with R = 1e-13 (f64, scalar), `new` returns `Err(InvalidNoiseCovariance)` where Ok was expected. The same happens for R = diag(1, 1e-13). The f32 case uses the same floor but is never reached, because the test stops at the f64 failure. The model is a perfectly conditioned 1-D problem; the only thing that changed is the units.
- **Proposed spec behavior:** any finite symmetric PSD (positive semi-definite) `r` with strictly positive diagonal is accepted, whatever its absolute magnitude. Any rejection of a tiny diagonal must be relative to the scale of `r`, not an absolute floor.

**AUD-2** | The result changes when one measurement component is expressed in different units. This extends B10 (scale-invariant singularity detection).
- **Where:** `src/matrix/naive.rs:147`, the absolute pivot test in Gauss–Jordan, reached via `src/filter.rs:299`.
- **Tests:** `audit_measurement_unit_change_does_not_change_estimate_f64` and `_f32`
- **Evidence:**
  - The test multiplies measurement component 2 by c, scaling row 2 of H, R as D·R·D, and z to match. The exact posterior does not change.
  - f64 with c = 3e-6: `update` returns `SingularMatrix` at step 0. I chose c so that R stays above the AUD-1 floor.
  - f32 with c = 1e-3: `SingularMatrix` at step 1.
  - The reference oracle run on the rescaled model agrees with the unscaled one to within 1e-10, so a correct implementation can pass.
  - B10's frozen test only scales a 1-D problem uniformly. Here the largest entry of S is about 1, so a fix that only compares against max|S| still fails.
- **Proposed spec behavior:** whether S is reported singular, and the resulting x and P, must not change under per-component rescaling of the measurement (S → D·S·D with D diagonal and positive).

**AUD-3** | A numerically singular S of large magnitude is "inverted", and the update returns Ok with a **negative variance**.
- **Where:** `src/matrix/naive.rs:147`. The absolute 1e-10 threshold lets a rounding residual of about 1e-16·|S| pass as a valid pivot.
- **Test:** `audit_large_scale_singular_innovation_is_detected`
- **Setup:** n = 1, m = 2, H = h, R = scale·h·hᵀ, P0 = scale. This makes S rank-1 at every scale, and z = [0, 1] has a component outside the range of S.
- **Evidence:** 3 of 322 cases return `Ok(())`:
  - h = [1, 0.1] at scale 1e8: P goes from 1e8 to **−5e7**.
  - Two random h at scale 1e12: P = **−5e11**.
  - B9 requires `Err(SingularMatrix)` with the state unchanged.
  - A reference Cholesky whose pivot tolerance is relative to the diagonal (d_k ≤ 1e3·eps·S_kk) flags all 322 cases, which the test also asserts.
- **Proposed spec behavior:** B9's "S singular → `SingularMatrix`" uses a tolerance relative to S's own diagonal. A successful `update` must never produce a covariance with a negative diagonal.

## Suspected, not confirmed
- **Large finite measurements can produce a non-finite state while `update` returns Ok.** For example, z near f64::MAX with K·y overflowing. In the cases I tried the result stayed finite, and the spec doesn't define behavior for non-finite *results* of `update`; it only allows `FilterDivergence` for `predict`, and doesn't test it.
- **`predict` returns Ok even when P overflows to ∞**, and later updates then run on a non-finite P. B7 explicitly leaves this untested, so I didn't count it as a defect.
- **Overflow of `n*n` / `m*n` in `initialize` (`src/filter.rs:103` onward) is not reachable.** `x0` is checked first and would need a slice of length n ≥ 2³², which can't be allocated.

## Spec problems
- B5 says that rejecting an `r` with a zero diagonal is "not specified", but says nothing about the absolute 1e-12 floor (AUD-1). As written, a candidate that keeps `validate_measurement_noise` passes every frozen test and stays unit-dependent. The spec should say "strictly positive at any scale".
- B10's only test scales a 1-D problem uniformly (`spec_change_small_scale_update_is_not_reported_singular`). A threshold relative to max|S| passes it but still fails mixed-unit models (AUD-2). The wording should also cover per-component (diagonal) scaling.
- B9 doesn't say what counts as "singular" in floating point. Without a relative criterion, AUD-3's negative-variance result arguably satisfies the current text. Pinning it to "relative to diag(S)" makes it testable.
- B11 checks that P stays PSD only along successful update paths. It should also say that `update` must never return Ok with an indefinite P, which would close AUD-3 from the other side.

Out of scope, not tested: `validate_covariance` (`src/validation.rs:26,35`) has the same absolute floors, 1e-12 on the diagonal and 1e-10 on symmetry, which are wrong for f32 and for large-magnitude covariances. Other modules still use it.
