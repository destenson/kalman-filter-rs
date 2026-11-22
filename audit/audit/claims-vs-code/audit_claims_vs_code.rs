//! Audit tests (lens: promises the project makes that the code does not keep).
//!
//! These target ONLY the SPEC.md public interface
//! (`kalman_filters::linear::LinearKalmanFilter`, `KalmanError`).
//! Each test documents the promise it checks and why it fails on c93b68c.
//! Deterministic: no randomness.

use kalman_filters::KalmanError;
use kalman_filters::linear::LinearKalmanFilter;
use std::panic::{AssertUnwindSafe, catch_unwind};

fn v<T: Copy, A: AsRef<[T]>>(a: A) -> Vec<T> {
    a.as_ref().to_vec()
}

// ---------------------------------------------------------------------------
// AUD-1: a strictly positive-definite R is rejected because it is "small".
//
// Promise: `KalmanError::InvalidNoiseCovariance` displays "Noise covariance
// matrix must be positive semi-definite" (src/error.rs), and SPEC B5 says a
// PSD (even singular) R is valid. But `validate_measurement_noise`
// (src/validation.rs:58, reached from src/filter.rs:150) rejects any diagonal
// entry below the absolute constant MIN_CONDITION_NUMBER = 1e-12, so the
// verdict depends on the units the user chose. R = 1e-14 m² (σ = 100 nm) is a
// perfectly ordinary, well-posed model.
// ---------------------------------------------------------------------------

#[test]
fn audit_small_but_positive_definite_measurement_noise_is_accepted_f64() {
    // P0 = 1 so that S = 1 + 1e-14 is perfectly conditioned: this isolates the
    // construction check from the (already known) B10 inversion threshold.
    let r = 1e-14;
    let kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[r]);
    let mut kf = match kf {
        Ok(kf) => kf,
        Err(e) => panic!("R = {r:e} is positive definite but construction returned {e:?}"),
    };
    kf.update(&[1.0]).expect("S = 1 + 1e-14 is well conditioned");
    // Closed form: K = P/(P+R), x = K z, P+ = P R/(P+R).
    let k = 1.0 / (1.0 + r);
    let x = v(kf.state())[0];
    let p = v(kf.covariance())[0];
    assert!((x - k).abs() <= 1e-15, "x = {x:e}, expected {k:e}");
    assert!((p - r / (1.0 + r)).abs() <= 1e-9 * r, "P = {p:e}, expected {:e}", r / (1.0 + r));
}

#[test]
fn audit_small_but_positive_definite_measurement_noise_is_accepted_f32() {
    // f32 represents 1e-13 with full precision (min normal ~1.2e-38).
    let r = 1e-13f32;
    let kf = LinearKalmanFilter::<f32>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[r]);
    assert!(kf.is_ok(), "f32 R = {r:e} is positive definite but rejected: {:?}", kf.err());
}

#[test]
fn audit_noise_validation_verdict_is_unit_invariant() {
    // Same physical model expressed in metres and in kilometres (all variances
    // scale by 1e-6). Validity of a covariance cannot depend on the unit.
    let metres = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[1e-4], &[1.0], &[1e-8]);
    assert!(metres.is_ok(), "metres model must be valid: {:?}", metres.err());
    let s = 1e-6; // (1 m / 1 km)^2
    let km = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0 * s], &[1.0], &[1e-4 * s], &[1.0], &[1e-8 * s]);
    assert!(km.is_ok(), "same model in km (R = 1e-14 km²) rejected: {:?}", km.err());
}

// ---------------------------------------------------------------------------
// AUD-2: huge `measurement_dim` panics with integer overflow.
//
// Promise: SPEC B15 / Q3 "No panic on bad input from the public API", and the
// brief's "integer division by a user-supplied dimension must be guarded". The
// spec only covers division; the length checks `m * n` and `m * m`
// (src/filter.rs:127,133) overflow first and panic ("attempt to multiply with
// overflow") in debug builds. Release builds wrap instead, so the outcome is
// build-profile dependent.
// ---------------------------------------------------------------------------

#[test]
fn audit_overflowing_dimensions_return_error_not_panic() {
    let x0 = [0.0, 1.0];
    let p0 = [1.0, 0.0, 0.0, 1.0];
    let f = [1.0, 0.1, 0.0, 1.0];
    let q = [0.01, 0.0, 0.0, 0.01];
    let h = [1.0, 0.0];
    let r = [0.1];
    // m * n overflows (n = 2) at src/filter.rs:127. (`m * m` at :133 can also
    // overflow, but only after an h of length m*n has been supplied.)
    let mut failures = Vec::new();
    for m in [usize::MAX, (usize::MAX / 2) + 1] {
        let res = catch_unwind(AssertUnwindSafe(|| {
            LinearKalmanFilter::<f64>::new(2, m, &x0, &p0, &f, &q, &h, &r).map(|_| ())
        }));
        match res {
            Ok(Err(_)) => {}
            Ok(Ok(())) => failures.push(format!("measurement_dim = {m} accepted")),
            Err(p) => {
                let msg = p
                    .downcast_ref::<&str>()
                    .map(|s| s.to_string())
                    .or_else(|| p.downcast_ref::<String>().cloned())
                    .unwrap_or_default();
                failures.push(format!("measurement_dim = {m} panicked ({msg}) instead of returning Err"));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

// ---------------------------------------------------------------------------
// AUD-3: singularity verdict depends on the unit scale (mirror of B10).
//
// Promise: SPEC B9 "If S is singular or not positive definite, update returns
// SingularMatrix"; the `invert` doc (src/matrix/mod.rs) "Returns
// SingularMatrix if matrix is singular". The absolute pivot threshold
// (src/matrix/naive.rs:147, 1e-10) catches a rank-1 S at unit scale but, once
// all variances are multiplied by 1e12, the rounding residual of the zero
// pivot exceeds 1e-10 and the *same* singular S is "inverted". The filter then
// accepts a measurement that has probability zero under the model and the
// state jumps by many orders of magnitude.
//
// Model: n = 1, m = 2, H = [1; 1.1], R = r·[[1,1.1],[1.1,1.21]] (rank-1 PSD,
// valid per B5), so S = (p + r)·[[1,1.1],[1.1,1.21]] has rank 1 at every
// scale. Observed on c93b68c: scales 1, 1e3, 1e6 -> SingularMatrix; scale 1e9
// -> Ok, x jumps 0 -> 15811 and P = 7.25e7 (rank-1-consistent value 6.67e7);
// scale 1e12 -> Ok with x and P unchanged.
// ---------------------------------------------------------------------------

const HH: f64 = 1.1;

fn rank1_model(scale: f64) -> LinearKalmanFilter<f64> {
    let p = 0.1 * scale;
    let r = 0.2 * scale;
    LinearKalmanFilter::<f64>::new(
        1,
        2,
        &[0.0],
        &[p],
        &[1.0],
        &[0.0],
        &[1.0, HH],
        &[r, HH * r, HH * r, HH * HH * r],
    )
    .expect("rank-1 PSD R is valid (SPEC B5)")
}

/// Reference: S = c·[[1,h],[h,h²]] has det 0 exactly; with a relative test on
/// its Cholesky pivot it is singular at every scale.
fn reference_is_singular(scale: f64) -> bool {
    let c = 0.1 * scale + 0.2 * scale;
    let s = [c, HH * c, HH * c, HH * HH * c];
    let l11 = s[0].sqrt();
    let l21 = s[2] / l11;
    let d2 = s[3] - l21 * l21;
    d2 <= 1e-12 * s[3]
}

#[test]
fn audit_singular_innovation_covariance_detected_at_any_scale() {
    for scale in [1.0, 1e3, 1e6, 1e9, 1e12] {
        assert!(reference_is_singular(scale));
        let mut kf = rank1_model(scale);
        let (x, p) = (v(kf.state()), v(kf.covariance()));
        let sigma = scale.sqrt();
        // z2 != 1.1 z1: impossible under the model.
        let res = kf.update(&[1.0 * sigma, 2.0 * sigma]);
        assert!(
            matches!(res, Err(KalmanError::SingularMatrix)),
            "scale {scale:e}: S is rank-1 (singular) but update returned {res:?}; \
             state became {:?} (prior {:?}, prior σ = {:e})",
            v(kf.state()),
            x,
            p[0].sqrt()
        );
        assert_eq!(v(kf.state()), x);
        assert_eq!(v(kf.covariance()), p);
    }
}

// ---------------------------------------------------------------------------
// AUD-4: finite inputs silently produce a non-finite estimate.
//
// Promise: `KalmanError::FilterDivergence` ("Filter divergence detected") and
// `check_numerical_stability` (called after every predict/update,
// src/filter.rs:216,350) suggest divergence is detected; SPEC B6 already
// requires that a NaN measurement must not poison x and P. Yet with all-finite
// inputs that overflow, predict()/update() return Ok(()) and leave inf/NaN in
// the state, which then poisons every later step. `check_numerical_stability`
// only logs.
// ---------------------------------------------------------------------------

fn all_finite(a: &[f64]) -> bool {
    a.iter().all(|x| x.is_finite())
}

#[test]
fn audit_predict_overflow_is_not_silently_accepted() {
    // x0 = 1e200, F = 1e200: F x overflows to +inf.
    let mut kf =
        LinearKalmanFilter::<f64>::new(1, 1, &[1e200], &[1.0], &[1e200], &[0.0], &[1.0], &[1.0])
            .expect("all inputs finite and valid");
    let (x, p) = (v(kf.state()), v(kf.covariance()));
    let res = kf.predict();
    let ok_and_finite = res.is_ok() && all_finite(&v(kf.state())) && all_finite(&v(kf.covariance()));
    let err_and_unchanged = res.is_err() && v(kf.state()) == x && v(kf.covariance()) == p;
    assert!(
        ok_and_finite || err_and_unchanged,
        "predict returned {res:?} with state {:?}, covariance {:?}",
        v(kf.state()),
        v(kf.covariance())
    );
}

#[test]
fn audit_update_overflow_is_not_silently_accepted() {
    // Finite x0 and finite z whose difference overflows: y = z - Hx = +inf.
    let mut kf =
        LinearKalmanFilter::<f64>::new(1, 1, &[-1e308], &[1.0], &[1.0], &[0.0], &[1.0], &[1.0])
            .expect("all inputs finite and valid");
    let (x, p) = (v(kf.state()), v(kf.covariance()));
    let res = kf.update(&[1e308]);
    let ok_and_finite = res.is_ok() && all_finite(&v(kf.state())) && all_finite(&v(kf.covariance()));
    let err_and_unchanged = res.is_err() && v(kf.state()) == x && v(kf.covariance()) == p;
    assert!(
        ok_and_finite || err_and_unchanged,
        "update returned {res:?} with state {:?}, covariance {:?}",
        v(kf.state()),
        v(kf.covariance())
    );
}

// ---------------------------------------------------------------------------
// AUD-5: DimensionMismatch reports a shape the caller never supplied.
//
// Promise: `KalmanError::DimensionMismatch { actual: (rows, cols) }` is
// documented as "Actual matrix dimensions (rows, columns)" and displays
// "expected NxN, got RxC" (src/error.rs). `initialize` fabricates
// `actual = (len / n, n)` (src/filter.rs:112-136), truncating: a 3-element p0
// for n = 2 is reported as "got 1x2" (2 elements), a 5-element p0 as "got
// 2x2", i.e. the error claims the caller passed exactly the expected shape.
// Since inputs are flat slices, no honest (rows, cols) exists; the reported
// value must at least account for every element (rows*cols == len) or the
// error must carry the length instead.
// ---------------------------------------------------------------------------

#[test]
fn audit_dimension_mismatch_reports_supplied_size_truthfully() {
    let x0 = [0.0, 1.0];
    let f = [1.0, 0.1, 0.0, 1.0];
    let q = [0.01, 0.0, 0.0, 0.01];
    let h = [1.0, 0.0];
    let r = [0.1];
    for len in [3usize, 5] {
        let p0 = vec![1.0; len];
        match LinearKalmanFilter::<f64>::new(2, 1, &x0, &p0, &f, &q, &h, &r) {
            Err(KalmanError::DimensionMismatch { expected, actual }) => {
                assert_ne!(actual, expected, "len {len}: error says the expected shape was supplied");
                assert_eq!(
                    actual.0 * actual.1,
                    len,
                    "len {len}: reported actual {actual:?} does not describe {len} elements"
                );
            }
            other => panic!("len {len}: expected DimensionMismatch, got {:?}", other.err()),
        }
    }
}
