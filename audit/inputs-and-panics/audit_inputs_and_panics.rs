//! Audit tests (input-space / panic lens) against the SPEC.md public interface.
//! Each test documents the defect it targets. All deterministic.

use kalman_filters::KalmanError;
use kalman_filters::linear::LinearKalmanFilter;
use std::panic::{AssertUnwindSafe, catch_unwind};

fn v<T: Copy, A: AsRef<[T]>>(a: A) -> Vec<T> {
    a.as_ref().to_vec()
}

// ---------------------------------------------------------------------------
// AUD-1: dimension products overflow -> panic instead of Err (B15)
// ---------------------------------------------------------------------------

/// n = 2, m = usize::MAX/2 + 1: `m * n` overflows. x0/p0/f/q are valid for
/// n = 2, so construction reaches the `observation_matrix.len() != m * n` check
/// (src/filter.rs:127), which panics with "attempt to multiply with overflow"
/// in debug builds (in release it wraps to 0, the empty h/r pass the length
/// checks, and validate_measurement_noise indexes R out of bounds).
#[test]
fn audit_huge_measurement_dim_overflow_returns_error_not_panic() {
    let m = usize::MAX / 2 + 1;
    let i2 = [1.0f64, 0.0, 0.0, 1.0];
    let res = catch_unwind(AssertUnwindSafe(|| {
        LinearKalmanFilter::<f64>::new(2, m, &[0.0, 0.0], &i2, &i2, &i2, &[], &[]).map(|_| ())
    }));
    match res {
        Ok(Err(_)) => {}
        Ok(Ok(())) => panic!("accepted measurement_dim = {m} with empty h and r"),
        Err(_) => panic!("LinearKalmanFilter::new panicked on measurement_dim = {m} (dimension product overflow)"),
    }
}

/// Same with m*m overflowing while m*n does not: n = 1, m = 2^32, h has m
/// entries would be needed, so instead use n = 1 and m = usize::MAX: m*n is
/// fine, h.len() (0) != m -> DimensionMismatch. This one must already pass;
/// it documents that the panic needs a product overflow, not just a large m.
#[test]
fn audit_huge_measurement_dim_without_overflow_returns_error() {
    let res = catch_unwind(AssertUnwindSafe(|| {
        LinearKalmanFilter::<f64>::new(1, usize::MAX, &[0.0], &[1.0], &[1.0], &[1.0], &[], &[]).map(|_| ())
    }));
    assert!(matches!(res, Ok(Err(KalmanError::DimensionMismatch { .. }))), "got {res:?}");
}

// ---------------------------------------------------------------------------
// AUD-2: positive-definite but small R rejected (absolute threshold) (B5)
// ---------------------------------------------------------------------------

/// validate_measurement_noise (src/validation.rs:58-60, via src/filter.rs:150)
/// rejects any R diagonal < 1e-12 in absolute terms. R = 1e-14 is a perfectly
/// valid (PD) noise covariance, e.g. variances in km² of a µm-precise sensor.
/// Reference: scalar closed form K = P/(P+R).
#[test]
fn audit_small_positive_measurement_noise_is_accepted_f64() {
    let (p0, r, z) = (3e-14f64, 1e-14f64, 2e-7f64);
    let mut kf = match LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[p0], &[1.0], &[0.0], &[1.0], &[r]) {
        Ok(kf) => kf,
        Err(e) => panic!("R = {r:e} (positive definite) rejected: {e:?}"),
    };
    kf.update(&[z]).expect("well-conditioned update");
    let k = p0 / (p0 + r);
    let (x, p) = (v(kf.state())[0], v(kf.covariance())[0]);
    assert!(((x - k * z) / (k * z)).abs() < 1e-9, "x = {x:e}, expected {:e}", k * z);
    assert!(((p - (1.0 - k) * p0) / ((1.0 - k) * p0)).abs() < 1e-9, "P = {p:e}");
}

/// Same with a 2-D diagonal R where only one channel is very precise.
#[test]
fn audit_small_positive_measurement_noise_is_accepted_f32_and_multichannel() {
    let r32 = LinearKalmanFilter::<f32>::new(1, 1, &[0.0], &[1e-13], &[1.0], &[0.0], &[1.0], &[1e-13]);
    assert!(r32.is_ok(), "f32 R = 1e-13 (PD, normal f32 number) rejected: {:?}", r32.err());

    let i2 = [1.0f64, 0.0, 0.0, 1.0];
    let r = [1.0, 0.0, 0.0, 1e-13];
    let res = LinearKalmanFilter::<f64>::new(2, 2, &[0.0, 0.0], &i2, &i2, &[0.0; 4], &i2, &r);
    assert!(res.is_ok(), "R = diag(1, 1e-13) rejected: {:?}", res.err());
}

// ---------------------------------------------------------------------------
// AUD-3: finite inputs -> Ok(()) with a non-finite state (silent divergence)
// ---------------------------------------------------------------------------

/// x0 = -3e38, z = +3e38 (both finite f32). The innovation z - Hx overflows
/// to +inf, update returns Ok(()) and the state becomes +inf permanently.
/// Proposed: an update that returns Ok leaves state/covariance finite;
/// otherwise Err (e.g. FilterDivergence) with state and covariance unchanged.
#[test]
fn audit_update_overflowing_innovation_is_not_silently_accepted() {
    let mut kf = LinearKalmanFilter::<f32>::new(1, 1, &[-3e38], &[1.0], &[1.0], &[0.0], &[1.0], &[1.0]).unwrap();
    let (x, p) = (v(kf.state()), v(kf.covariance()));
    let res = kf.update(&[3e38]);
    let (x1, p1) = (v(kf.state()), v(kf.covariance()));
    match res {
        Ok(()) => assert!(
            x1.iter().chain(&p1).all(|a| a.is_finite()),
            "update returned Ok but state = {x1:?}, P = {p1:?}"
        ),
        Err(_) => {
            assert_eq!(x1, x, "state changed on error");
            assert_eq!(p1, p, "covariance changed on error");
        }
    }
}

/// predict with finite F, P0 whose product overflows: P becomes +inf and
/// predict returns Ok(()) (adapter + KalmanFilter::predict never fail).
/// The following update then returns Ok(()) with a NaN state (S = inf,
/// inverted to 0, K = inf*0 = NaN): finite inputs, Ok results, NaN output.
#[test]
fn audit_overflow_in_predict_does_not_lead_to_ok_nan_update() {
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[1.0], &[1e200], &[1e200], &[0.0], &[1.0], &[1.0]).unwrap();
    let (x0, p0) = (v(kf.state()), v(kf.covariance()));
    match kf.predict() {
        Ok(()) => {
            let (x, p) = (v(kf.state()), v(kf.covariance()));
            if x.iter().chain(&p).all(|a| a.is_finite()) {
                return; // a correct, finite result: fine
            }
            // Spec B7 tolerates this case, but the next update must not
            // turn it into an Ok with NaN state.
            let res = kf.update(&[0.0]);
            let (x1, p1) = (v(kf.state()), v(kf.covariance()));
            assert!(
                res.is_err() || x1.iter().chain(&p1).all(|a| !a.is_nan()),
                "predict Ok -> P = {p:?}; update returned {res:?} with state {x1:?}, P {p1:?}"
            );
        }
        Err(_) => {
            assert_eq!(v(kf.state()), x0, "state changed on failed predict");
            assert_eq!(v(kf.covariance()), p0, "covariance changed on failed predict");
        }
    }
}

// ---------------------------------------------------------------------------
// AUD-4: one update with valid input returns Ok with a NEGATIVE variance
// ---------------------------------------------------------------------------

fn eig2(p: &[f64]) -> (f64, f64) {
    let (a, b, d) = (p[0], 0.5 * (p[1] + p[2]), p[3]);
    let disc = ((a - d) * (a - d) / 4.0 + b * b).sqrt();
    ((a + d) / 2.0 - disc, (a + d) / 2.0 + disc)
}

/// Textbook-stable reference: S solved by Cholesky (relative pivot check),
/// Joseph-form covariance. 2 states, 2 measurements, row-major.
fn joseph_cholesky_ref(x: &[f64], p: &[f64], h: &[f64], r: &[f64], z: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
    let mm = |a: &[f64], b: &[f64]| -> Vec<f64> {
        (0..4).map(|k| { let (i, j) = (k / 2, k % 2); a[2 * i] * b[j] + a[2 * i + 1] * b[2 + j] }).collect()
    };
    let t = |a: &[f64]| vec![a[0], a[2], a[1], a[3]];
    let hp = mm(h, p);
    let s: Vec<f64> = mm(&hp, &t(h)).iter().zip(r).map(|(a, b)| a + b).collect();
    let l00 = s[0].sqrt();
    let l10 = s[2] / l00;
    let d = s[3] - l10 * l10;
    if !(d > 1e-15 * s[3].abs().max(s[0].abs())) { return None; }
    let l11 = d.sqrt();
    // solve S Kᵀ = H P column by column
    let mut kt = vec![0.0; 4];
    for c in 0..2 {
        let y0 = hp[c] / l00;
        let y1 = (hp[2 + c] - l10 * y0) / l11;
        let k1 = y1 / l11;
        let k0 = (y0 - l10 * k1) / l00;
        kt[c] = k0; kt[2 + c] = k1;
    }
    let k = t(&kt);
    let hx = [h[0] * x[0] + h[1] * x[1], h[2] * x[0] + h[3] * x[1]];
    let y = [z[0] - hx[0], z[1] - hx[1]];
    let xn = vec![x[0] + k[0] * y[0] + k[1] * y[1], x[1] + k[2] * y[0] + k[3] * y[1]];
    let kh = mm(&k, h);
    let a = vec![1.0 - kh[0], -kh[1], -kh[2], 1.0 - kh[3]];
    let apa = mm(&mm(&a, p), &t(&a));
    let krk = mm(&mm(&k, r), &t(&k));
    let mut pn: Vec<f64> = apa.iter().zip(&krk).map(|(u, w)| u + w).collect();
    let off = 0.5 * (pn[1] + pn[2]);
    pn[1] = off; pn[2] = off;
    Some((xn, pn))
}

fn acceptable_posterior(x: &[f64], p: &[f64]) -> Result<(), String> {
    let (lmin, lmax) = eig2(p);
    let finite = x.iter().chain(p).all(|a| a.is_finite());
    let asym = (p[1] - p[2]).abs() / lmax.abs().max(f64::MIN_POSITIVE);
    if finite && p[0] >= 0.0 && p[3] >= 0.0 && asym <= 1e-10 && lmin >= -1e-9 * lmax {
        Ok(())
    } else {
        Err(format!("x = {x:?}, P = {p:?}, λ = ({lmin:e}, {lmax:e})"))
    }
}

/// P0 = 1e30·I (legal: huge but finite prior), R = I, two nearly collinear
/// position-sum sensors H = [[1,1],[1,1+1e-15]]. All inputs are valid under
/// B2-B6. Current code (Gauss–Jordan inverse of S, absolute pivot threshold,
/// simple-form P update) returns Ok with P = [[0, -1e30], [-1e30, -2.7e15]]:
/// a negative variance from one update. B11 only checks this class on its
/// two frozen models; nothing in SPEC states the post-condition for a single
/// update. The reference Joseph + Cholesky update in this test either
/// rejects S or yields a PSD posterior (asserted below), so the bar is met by
/// a correct implementation.
#[test]
fn audit_single_update_never_returns_ok_with_negative_variance() {
    let p0 = [1e30f64, 0.0, 0.0, 1e30];
    let h = [1.0, 1.0, 1.0, 1.0 + 1e-15];
    let i2 = [1.0, 0.0, 0.0, 1.0];
    let x0 = [0.0, 0.0];
    let z = [h[0] + h[1], h[2] + h[3]];

    // A correct implementation can meet the bar:
    if let Some((xr, pr)) = joseph_cholesky_ref(&x0, &p0, &h, &i2, &z) {
        acceptable_posterior(&xr, &pr).expect("reference Joseph+Cholesky must be acceptable");
    }

    let mut kf = LinearKalmanFilter::<f64>::new(2, 2, &x0, &p0, &i2, &[0.0; 4], &h, &i2).unwrap();
    match kf.update(&z) {
        Err(KalmanError::SingularMatrix) => {
            assert_eq!(v(kf.state()), x0.to_vec());
            assert_eq!(v(kf.covariance()), p0.to_vec());
        }
        Err(e) => panic!("unexpected error {e:?}"),
        Ok(()) => {
            if let Err(msg) = acceptable_posterior(&v(kf.state()), &v(kf.covariance())) {
                panic!("update returned Ok but posterior is not PSD: {msg}");
            }
        }
    }
}

