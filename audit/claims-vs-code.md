I added 8 audit tests in `tests/audit_claims_vs_code.rs`, and all 8 fail on the current code (`cargo test --quiet --test audit_claims_vs_code`: 0 passed, 8 failed). They cover 5 defects that SPEC.md does not list. I changed nothing else and committed nothing.

## Confirmed

**AUD-1** | A measurement noise R that is positive definite but small is rejected with `InvalidNoiseCovariance`. The check is an absolute floor (diagonal < 1e-12), so whether R is valid depends on the units chosen. The error text says R "must be positive semi-definite", and this R is.
- **Where:** `src/validation.rs:58`, reached from `src/filter.rs:150`.
- **Tests:** `audit_small_but_positive_definite_measurement_noise_is_accepted_f64`, `audit_small_but_positive_definite_measurement_noise_is_accepted_f32`, `audit_noise_validation_verdict_is_unit_invariant`.
- **Evidence:** f64 R = 1e-14 (σ = 100 nm) and f32 R = 1e-13 are both rejected. The same model is accepted in metres (R = 1e-8) and rejected in km (R = 1e-14 km²). In the f64 test, P0 = 1, so S ≈ 1 is well conditioned and the known B10 problem plays no part. Expected: the filter constructs, and `update` matches the closed form x = z/(1+R), P = R/(1+R).
- **Proposed spec behavior:** whether P0, Q or R is accepted must not depend on scale; any R that passes the PSD test with a relative tolerance must be accepted, however small its entries.

**AUD-2** | A huge `measurement_dim` panics with "attempt to multiply with overflow" in the length check `m * n`.
- **Where:** `src/filter.rs:127` (`m * m` at `:133` could also overflow, but only after an `h` of length m·n has been passed).
- **Test:** `audit_overflowing_dimensions_return_error_not_panic`.
- **Evidence:** with n = 2, both m = usize::MAX and m = 2⁶³ panic in debug builds. Release builds wrap around instead, so the outcome depends on the build profile. SPEC B3/B15 only mention division by a zero dimension.
- **Proposed spec behavior:** dimension arithmetic is overflow-checked, and dimensions whose sizes overflow return `Err(DimensionMismatch)`.

**AUD-3** | Detecting a singular S also depends on scale, which is the mirror image of B10. A rank-1 S is reported as singular at small scale but is "inverted" once the variances are larger.
- **Where:** absolute pivot threshold at `src/matrix/naive.rs:147`, reached via `src/filter.rs:299`.
- **Test:** `audit_singular_innovation_covariance_detected_at_any_scale`.
- **Evidence:** the model is n = 1, m = 2, H = [1; 1.1], R = r·[[1, 1.1], [1.1, 1.21]]. That R is rank-1 PSD, so it is valid per B5, and S is exactly rank 1 at every scale. A reference check of the Cholesky pivot with a relative tolerance confirms S is singular at every scale.
  - At scales 1, 1e3 and 1e6, `update` returns `SingularMatrix`.
  - At scale 1e9 it returns `Ok`, x jumps from 0 to 15811, and P = 7.25e7 instead of the rank-consistent 6.67e7.
  - At scale 1e12 it returns `Ok` and x and P do not change.
- **Proposed spec behavior:** B9 should define "singular" relatively, for example a Cholesky/LDLᵀ pivot ≤ c·ε·max diag(S), so the verdict is the same at every scale. Note that a Cholesky with no tolerance at all could also accept this S.

**AUD-4** | All-finite inputs that overflow make `predict` or `update` return `Ok(())` while leaving inf in the state. `FilterDivergence` exists and `check_numerical_stability` runs after every step, but that check only logs.
- **Where:** `src/filter.rs:186-233` and `:236-370`; the adapter's `predict` always returns `Ok`.
- **Tests:** `audit_predict_overflow_is_not_silently_accepted`, `audit_update_overflow_is_not_silently_accepted`.
- **Evidence:**
  - x0 = 1e200, F = 1e200: after `predict`, state = [inf] and covariance = [inf], with `Ok`.
  - x0 = −1e308, z = 1e308: after `update`, state = [inf], with `Ok`.
- **Expected:** either `Ok` with finite values, or `Err` with the state left unchanged.
- **Proposed spec behavior:** `predict` and `update` never commit a non-finite x or P. They return `Err(FilterDivergence)` and leave the filter untouched, the same guarantee B6 already gives for a NaN measurement.

**AUD-5** | `DimensionMismatch.actual` is documented as "Actual matrix dimensions (rows, columns)", but it is computed as `(len / n, n)`, which truncates. The error can even report exactly the expected shape.
- **Where:** `src/filter.rs:112,118,124,130,136`.
- **Test:** `audit_dimension_mismatch_reports_supplied_size_truthfully`.
- **Evidence:** for n = 2, a 3-element p0 is reported as "got 1x2", and a 5-element p0 as "got 2x2", which equals `expected`.
- **Proposed spec behavior:** `actual` must account for every element supplied (rows·cols = len, for example `(len, 1)`), or the error should carry the length. This is low severity, since SPEC says the field values are unspecified.

## Suspected, not confirmed
- **The nalgebra backend gets rectangular matrices wrong.** `NalgebraBackend` converts with `DMatrix::from_iterator(rows, cols, ..).transpose()` (`src/matrix/nalgebra_backend.rs:19,28,42`). That gives a cols×rows matrix, which is only correct for square inputs, so any m ≠ n would panic or give wrong results. I couldn't observe this: `--no-default-features --features std,nalgebra` doesn't compile (26 × `T: RealField` errors), and with default features `naive-matrix` always wins.
- **The backend docs are false.** `src/matrix/mod.rs` calls nalgebra "(default)" and says the priority is "blas > nalgebra > naive". In the code, `naive-matrix` is the default and takes precedence whenever it is enabled. This isn't observable through the spec interface.
- **Two in-scope unit tests cannot fail.** `test_covariance_symmetry_maintained` (`src/filter.rs:919`) can never fail because `update` force-symmetrizes P, and it doesn't check PSD. `test_steady_state_gain_convergence` (`:1070`) says it checks the algebraic Riccati equation, but only asserts 0 < P < 10.
- **Out of scope, noted only:** `src/validation.rs:34` uses an absolute 1e-10 symmetry tolerance. The README promises "Builder Pattern … with compile-time validation", but the builder only validates at runtime.

## Spec problems
- **B9/B10 define "singular" from one side only.** They say what must *not* be reported as singular, but give no relative criterion for what *must* be (AUD-3). The frozen singular test only uses exact zeros, so a Cholesky with no tolerance passes it and still accepts numerically singular S.
- **B7 allows `Ok` with a non-finite result** ("may return FilterDivergence … not tested"). That contradicts the intent of B6 that one bad step must not poison the filter (AUD-4).
- **B15/Q3 only name division by a dimension.** They should say "all dimension arithmetic is overflow-checked" (AUD-2). Note that `spec_change_zero_dimensions…` would pass in release builds even if the overflow were left in.
- **The PSD tolerance for construction inputs is described only qualitatively** ("small relative tolerance appropriate to T"). Nothing tests that tiny but valid magnitudes are accepted (AUD-1). A candidate that copies `validation.rs`'s absolute floors would pass every frozen test.
