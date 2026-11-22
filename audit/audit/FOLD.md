I've updated `SPEC.md` and `tests/spec_linear_kf.rs` with the audit findings: 5 new CHANGE behaviors (B18–B22) and 7 new tests. Running `cargo test --quiet --test spec_linear_kf` gives 14 passed and 18 failed. The 14 passes are all the KEEP tests, and every `spec_change_*` test fails for the reason listed in the test index, with no compile errors. `cargo test --quiet --lib` passes 151/151.

I ran every audit test myself in a scratch crate outside the repo, and all the confirmed findings reproduced. Only `SPEC.md` and the frozen test file changed. No `audit_*` file was copied in, nothing under `src/` was touched, and nothing is committed.

### Added

| Behavior | Test(s) | Defect (current code) | Auditors merged |
|---|---|---|---|
| **B18** Covariance validation is scale-invariant | `spec_change_noise_covariance_validation_is_scale_invariant` | An absolute floor of 1e-12 on the diagonal of R (`src/validation.rs:6,58-60` via `src/filter.rs:150`). Valid R values such as 1e-14 (f64), 1e-13 (f32) and diag(1, 1e-13) are rejected, so the same model passes in metres and fails in km. | claims AUD-1, difftest AUD-D1, inputs AUD-2, oracle AUD-1 |
| **B19** Dimension arithmetic is overflow-checked | `spec_change_overflowing_dimensions_are_rejected_without_panic` | The `m*n` length check overflows at `src/filter.rs:127` (`m*m` is at `:133`), so `new` panics. In release builds it wraps instead, then reads R out of bounds. | claims AUD-2, inputs AUD-1 |
| **B20** `predict`/`update` never commit a non-finite estimate | `spec_change_predict_never_commits_non_finite_estimate`, `spec_change_update_never_commits_non_finite_estimate` | All-finite inputs that overflow return `Ok` with an inf or NaN state (`src/filter.rs:186-233`, `:266-270`, `:315-317`). The stability check only logs. | claims AUD-4, inputs AUD-3, inputs AUD-4 |
| **B21** A singular S is detected at any scale, and an `Ok` update never leaves P indefinite | `spec_change_large_scale_singular_innovation_is_detected`, `spec_change_successful_update_never_yields_indefinite_covariance` | The absolute pivot threshold (`src/matrix/naive.rs:147`) misses rank-1 S at large scale. Updates then return `Ok` with P = −5e7, or P = [[0,−1e30],[−1e30,−2.7e15]]. | claims AUD-3, oracle AUD-3, inputs AUD-5 |
| **B22** Changing one measurement component's units doesn't change the result | `spec_change_measurement_unit_change_does_not_change_estimate` | Same absolute pivot test. Scaling one component by 3e-6 (f64) or 1e-3 (f32) gives `SingularMatrix`, and B10's uniform-scaling test can't catch this. | oracle AUD-2 |

The B20 and B21 tests accept either outcome: the documented error with the filter unchanged, or `Ok` with a valid result. The B21 and B22 tests each carry an in-test reference showing a correct implementation can pass.

### Rejected

| Finding | Reason |
|---|---|
| claims AUD-5: `DimensionMismatch.actual` reports a truncated shape | Not a defect under the contract: the spec deliberately leaves the field values unspecified. |
| inputs control test: huge `m` with no overflow returns `DimensionMismatch` | Already covered by B2, and it passes today. |
| oracle sanity tests (random shapes, 10⁴-step drift, uniform scaling up to 1e140) | Not defects: they pass, and B8 already covers this. |
| nalgebra backend gets rectangular matrices wrong; backend docs are false | Out of scope: the nalgebra backend doesn't compile and isn't observable through the spec interface. |
| `n*n` overflow | Duplicate of B19, and it can't be reached in a test without allocating a slice of 2³² elements. |
| Accuracy loss at P0 = 1e20 with nearly collinear H | Flaky: no tolerance exists that a correct implementation is sure to meet. |
| Absolute floors in `validation.rs` used by other modules; README "compile-time validation" | Out of scope. |

### Curated

None of the existing frozen tests was wrong or too strict, so no test changed. I corrected the spec text instead:

- **B5:** it contradicted itself on a rank-deficient R with a zero diagonal (e.g. diag(1, 0)). It now says a rank-deficient R with a positive diagonal must be accepted, and an exactly zero diagonal stays unspecified.
- **B7:** it allowed `predict` to return `Ok` with a non-finite result. It now points to B20.
- **B9, B10, B11:** "singular" is now defined relative to S's own scale, with cross-references to B21 and B22.
- **B15, Q3:** integer overflow of dimension products is now listed.
- **Q1:** absolute magnitude thresholds are now explicitly forbidden.
- **Unit tests:** I added a note that `test_covariance_symmetry_maintained` and `test_steady_state_gain_convergence` in `src/filter.rs` cannot fail. They're left unedited because that's candidate-owned code.

### Totals

| | Behaviors | Tests |
|---|---|---|
| KEEP | 7 (B1, B2, B7, B8, B9, B17, plus the KEEP part of B5) | 14 |
| CHANGE | 16 (B3, B4, B5, B6, B10–B16, B18–B22) | 18 `spec_change_*` |
| Total | 22 | 32 |

B13, B14 and B16 are still checked in review only.
