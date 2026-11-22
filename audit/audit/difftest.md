# Differential-test finding (orchestrator; harness vs. textbook f64 oracle, 270 cases)

## Confirmed
AUD-D1 | Validation is not scale-invariant: P0 and R diagonals below an absolute 1e-12 are
rejected, so a valid model expressed in small units fails construction |
src/validation.rs:6 (`MIN_CONDITION_NUMBER = 1e-12`, an absolute magnitude despite its
name), used at :26 (P) and :59 (R) | `audit_validation_is_scale_invariant` |
all 10 difftest cases at scale 1e-6 fail with `InvalidNoiseCovariance`; the oracle accepts
them; at scale 1e-7 the 1-D test model is rejected at construction |
Proposed: validity of P0/Q/R is judged relative to their own scale (e.g. symmetric PSD by
eigenvalues relative to the largest), never against an absolute magnitude.

## Already covered by the spec
- Ill-conditioned models (P0 = 1e8·I, R = 1e-8·I): original code's x/P deviate from the
  Joseph-form oracle by up to 6.5e-2 relative (B11).
