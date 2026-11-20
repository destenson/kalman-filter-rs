//! Frozen acceptance tests for the linear Kalman filter core (see SPEC.md).
//!
//! These tests target ONLY the public interface defined in SPEC.md:
//!
//! ```text
//! kalman_filters::{KalmanError, KalmanResult}
//! kalman_filters::linear::LinearKalmanFilter<T>   (T = f32 | f64)
//!     ::new(n, m, x0, p0, f, q, h, r) -> KalmanResult<Self>   // row-major slices
//!     .state_dim() / .measurement_dim() -> usize
//!     .predict() -> KalmanResult<()>
//!     .update(z: &[T]) -> KalmanResult<()>
//!     .state() / .covariance() -> impl AsRef<[T]>             // covariance row-major n×n
//! ```
//!
//! Tests named `spec_change_*` describe behavior that must CHANGE; they are
//! expected to fail on commit c93b68c (through the adapter in src/filter.rs).
//! All other tests characterize KEEP behavior and must pass before and after.
//!
//! Everything is deterministic: no OS randomness, fixed-seed SplitMix64.

use kalman_filters::KalmanError;
use kalman_filters::linear::LinearKalmanFilter;

// ---------------------------------------------------------------------------
// Allocation counter (thread-local, so parallel tests do not interfere).
// Used only by the no-allocation test; it is a transparent wrapper otherwise.
// ---------------------------------------------------------------------------
mod alloc_counter {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::cell::Cell;

    pub struct Counting;

    thread_local! {
        static COUNT: Cell<usize> = const { Cell::new(0) };
    }

    unsafe impl GlobalAlloc for Counting {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let _ = COUNT.try_with(|c| c.set(c.get() + 1));
            unsafe { System.alloc(layout) }
        }
        unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
            unsafe { System.dealloc(ptr, layout) }
        }
        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            let _ = COUNT.try_with(|c| c.set(c.get() + 1));
            unsafe { System.alloc_zeroed(layout) }
        }
        unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
            let _ = COUNT.try_with(|c| c.set(c.get() + 1));
            unsafe { System.realloc(ptr, layout, new_size) }
        }
    }

    pub fn count() -> usize {
        COUNT.with(|c| c.get())
    }
}

#[global_allocator]
static GLOBAL: alloc_counter::Counting = alloc_counter::Counting;

// ---------------------------------------------------------------------------
// Helpers (f64 reference math, deterministic RNG, matrix checks)
// ---------------------------------------------------------------------------

fn v<T: Copy, A: AsRef<[T]>>(a: A) -> Vec<T> {
    a.as_ref().to_vec()
}

fn to_f32(a: &[f64]) -> Vec<f32> {
    a.iter().map(|&x| x as f32).collect()
}

fn to_f64(a: &[f32]) -> Vec<f64> {
    a.iter().map(|&x| x as f64).collect()
}

fn assert_close(actual: &[f64], expected: &[f64], tol: f64, what: &str) {
    assert_eq!(actual.len(), expected.len(), "{what}: length");
    let scale = expected.iter().fold(1.0f64, |m, x| m.max(x.abs()));
    for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (a - e).abs() <= tol * scale,
            "{what}[{i}]: got {a:e}, expected {e:e} (tol {tol:e} * scale {scale:e})"
        );
    }
}

fn mat_mul(a: &[f64], b: &[f64], r: usize, k: usize, c: usize) -> Vec<f64> {
    let mut o = vec![0.0; r * c];
    for i in 0..r {
        for j in 0..c {
            o[i * c + j] = (0..k).map(|p| a[i * k + p] * b[p * c + j]).sum();
        }
    }
    o
}

fn transpose(a: &[f64], r: usize, c: usize) -> Vec<f64> {
    let mut o = vec![0.0; r * c];
    for i in 0..r {
        for j in 0..c {
            o[j * r + i] = a[i * c + j];
        }
    }
    o
}

/// Gauss-Jordan inverse with partial pivoting (reference only, f64).
fn inverse(a: &[f64], n: usize) -> Vec<f64> {
    let mut m = a.to_vec();
    let mut inv = vec![0.0; n * n];
    for i in 0..n {
        inv[i * n + i] = 1.0;
    }
    for col in 0..n {
        let piv = (col..n)
            .max_by(|&x, &y| m[x * n + col].abs().total_cmp(&m[y * n + col].abs()))
            .unwrap();
        for k in 0..n {
            m.swap(col * n + k, piv * n + k);
            inv.swap(col * n + k, piv * n + k);
        }
        let d = m[col * n + col];
        for k in 0..n {
            m[col * n + k] /= d;
            inv[col * n + k] /= d;
        }
        for r in 0..n {
            if r != col {
                let f = m[r * n + col];
                for k in 0..n {
                    m[r * n + k] -= f * m[col * n + k];
                    inv[r * n + k] -= f * inv[col * n + k];
                }
            }
        }
    }
    inv
}

/// Textbook linear KF in f64, used as the oracle for KEEP numerics.
struct RefKf {
    n: usize,
    m: usize,
    x: Vec<f64>,
    p: Vec<f64>,
    f: Vec<f64>,
    q: Vec<f64>,
    h: Vec<f64>,
    r: Vec<f64>,
}

impl RefKf {
    fn predict(&mut self) {
        let (n, f) = (self.n, &self.f);
        self.x = mat_mul(f, &self.x, n, n, 1);
        let fp = mat_mul(f, &self.p, n, n, n);
        let mut p = mat_mul(&fp, &transpose(f, n, n), n, n, n);
        for (pi, qi) in p.iter_mut().zip(&self.q) {
            *pi += qi;
        }
        self.p = p;
    }

    /// Returns (innovation y, innovation covariance S).
    fn update(&mut self, z: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (n, m) = (self.n, self.m);
        let hx = mat_mul(&self.h, &self.x, m, n, 1);
        let y: Vec<f64> = z.iter().zip(&hx).map(|(a, b)| a - b).collect();
        let ht = transpose(&self.h, m, n);
        let pht = mat_mul(&self.p, &ht, n, n, m);
        let mut s = mat_mul(&self.h, &pht, m, n, m);
        for (si, ri) in s.iter_mut().zip(&self.r) {
            *si += ri;
        }
        let k = mat_mul(&pht, &inverse(&s, m), n, m, m);
        let ky = mat_mul(&k, &y, n, m, 1);
        for (xi, d) in self.x.iter_mut().zip(&ky) {
            *xi += d;
        }
        // Joseph form in the oracle: P = (I-KH) P (I-KH)^T + K R K^T
        let kh = mat_mul(&k, &self.h, n, m, n);
        let mut a = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                a[i * n + j] = if i == j { 1.0 } else { 0.0 } - kh[i * n + j];
            }
        }
        let apa = mat_mul(&mat_mul(&a, &self.p, n, n, n), &transpose(&a, n, n), n, n, n);
        let krk = mat_mul(&mat_mul(&k, &self.r, n, m, m), &transpose(&k, n, m), n, m, n);
        self.p = apa.iter().zip(&krk).map(|(a, b)| a + b).collect();
        (y, s)
    }
}

/// Symmetric Jacobi eigenvalues (f64), on the symmetric part of `a`.
fn eigenvalues(a: &[f64], n: usize) -> Vec<f64> {
    let mut a = a.to_vec();
    for i in 0..n {
        for j in 0..i {
            let m = 0.5 * (a[i * n + j] + a[j * n + i]);
            a[i * n + j] = m;
            a[j * n + i] = m;
        }
    }
    for _ in 0..100 {
        let off: f64 = (0..n)
            .flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| a[i * n + j] * a[i * n + j])
            .sum();
        if off == 0.0 {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                let apq = a[p * n + q];
                if apq == 0.0 {
                    continue;
                }
                let theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let (akp, akq) = (a[k * n + p], a[k * n + q]);
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p * n + k], a[q * n + k]);
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    (0..n).map(|i| a[i * n + i]).collect()
}

/// Checks: finite, symmetric (relative), PSD (min eig >= -tol * max eig).
fn check_covariance(p: &[f64], n: usize, sym_tol: f64, psd_tol: f64) -> Result<(), String> {
    if p.len() != n * n {
        return Err(format!("covariance length {} != {}", p.len(), n * n));
    }
    if p.iter().any(|x| !x.is_finite()) {
        return Err(format!("non-finite covariance {p:?}"));
    }
    let amax = p.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    for i in 0..n {
        for j in 0..n {
            let d = (p[i * n + j] - p[j * n + i]).abs();
            if d > sym_tol * amax {
                return Err(format!("asymmetric at ({i},{j}): {d:e} > {sym_tol:e}*{amax:e}"));
            }
        }
    }
    let eig = eigenvalues(p, n);
    let mx = eig.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let mn = eig.iter().cloned().fold(f64::INFINITY, f64::min);
    if mn < -psd_tol * mx.abs() {
        return Err(format!("not PSD: eigenvalues {eig:?}"));
    }
    Ok(())
}

/// SplitMix64 + Box-Muller: deterministic standard normals.
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

fn is_dimension_mismatch<T>(r: &Result<T, KalmanError>) -> bool {
    matches!(r, Err(KalmanError::DimensionMismatch { .. }))
}

// A valid baseline model: n = 2 (constant velocity), m = 1 (position).
const X0: [f64; 2] = [0.0, 1.0];
const P0: [f64; 4] = [1.0, 0.0, 0.0, 1.0];
const F: [f64; 4] = [1.0, 0.1, 0.0, 1.0];
const Q: [f64; 4] = [0.01, 0.0, 0.0, 0.01];
const H: [f64; 2] = [1.0, 0.0];
const R: [f64; 1] = [0.1];

fn baseline() -> LinearKalmanFilter<f64> {
    LinearKalmanFilter::<f64>::new(2, 1, &X0, &P0, &F, &Q, &H, &R).expect("valid baseline")
}

// A richer model for the oracle comparison: n = 4, m = 2, correlated R,
// non-diagonal P0 and Q, non-trivial H.
fn rich_model() -> RefKf {
    let dt = 0.5;
    RefKf {
        n: 4,
        m: 2,
        x: vec![1.0, -2.0, 0.5, 0.25],
        p: vec![
            4.0, 0.5, 0.2, 0.0, //
            0.5, 3.0, 0.0, 0.1, //
            0.2, 0.0, 2.0, 0.3, //
            0.0, 0.1, 0.3, 1.0,
        ],
        f: vec![
            1.0, 0.0, dt, 0.0, //
            0.0, 1.0, 0.0, dt, //
            0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 1.0,
        ],
        q: vec![
            0.02, 0.0, 0.01, 0.0, //
            0.0, 0.02, 0.0, 0.01, //
            0.01, 0.0, 0.04, 0.0, //
            0.0, 0.01, 0.0, 0.04,
        ],
        h: vec![
            1.0, 0.0, 0.0, 0.0, //
            0.5, 1.0, 0.0, 0.2,
        ],
        r: vec![0.3, 0.05, 0.05, 0.2],
    }
}

fn rich_measurement(k: usize) -> Vec<f64> {
    vec![1.0 + 0.3 * k as f64 + 0.1 * (k as f64).sin(), -2.0 + 0.05 * k as f64]
}

// ===========================================================================
// B1: construction & accessors (KEEP)
// ===========================================================================

#[test]
fn construction_reports_dimensions_and_initial_estimate() {
    let kf = baseline();
    assert_eq!(kf.state_dim(), 2);
    assert_eq!(kf.measurement_dim(), 1);
    assert_eq!(v(kf.state()), X0.to_vec());
    assert_eq!(v(kf.covariance()), P0.to_vec());

    let kf32 =
        LinearKalmanFilter::<f32>::new(2, 1, &to_f32(&X0), &to_f32(&P0), &to_f32(&F), &to_f32(&Q), &to_f32(&H), &to_f32(&R))
            .expect("valid f32 model");
    assert_eq!(v(kf32.state()), to_f32(&X0));
    assert_eq!(v(kf32.covariance()), to_f32(&P0));
}

// ===========================================================================
// B2: dimension validation at construction (KEEP)
// ===========================================================================

#[test]
fn construction_rejects_wrong_length_inputs() {
    let n = 2;
    let m = 1;
    let short2 = [1.0];
    let long4 = [1.0, 0.0, 0.0, 1.0, 0.0];
    assert!(is_dimension_mismatch(&LinearKalmanFilter::<f64>::new(n, m, &[0.0], &P0, &F, &Q, &H, &R)), "x0");
    assert!(is_dimension_mismatch(&LinearKalmanFilter::<f64>::new(n, m, &X0, &short2, &F, &Q, &H, &R)), "p0 short");
    assert!(is_dimension_mismatch(&LinearKalmanFilter::<f64>::new(n, m, &X0, &long4, &F, &Q, &H, &R)), "p0 long");
    assert!(is_dimension_mismatch(&LinearKalmanFilter::<f64>::new(n, m, &X0, &P0, &[1.0, 0.0], &Q, &H, &R)), "f");
    assert!(is_dimension_mismatch(&LinearKalmanFilter::<f64>::new(n, m, &X0, &P0, &F, &[0.1], &H, &R)), "q");
    assert!(is_dimension_mismatch(&LinearKalmanFilter::<f64>::new(n, m, &X0, &P0, &F, &Q, &[1.0], &R)), "h");
    assert!(is_dimension_mismatch(&LinearKalmanFilter::<f64>::new(n, m, &X0, &P0, &F, &Q, &H, &[0.1, 0.0])), "r");
}

// ===========================================================================
// B3: zero dimensions (CHANGE)
// ===========================================================================

#[test]
fn spec_change_zero_dimensions_are_rejected_without_panic() {
    // m = 0 with a non-empty R: current code divides by m (src/filter.rs:135) and panics.
    let r = LinearKalmanFilter::<f64>::new(1, 0, &[0.0], &[1.0], &[1.0], &[0.1], &[], &[1.0]);
    assert!(r.is_err(), "measurement_dim = 0 must be rejected");
    // n = 0 / m = 0 with consistent (empty) inputs: currently accepted.
    let r = LinearKalmanFilter::<f64>::new(1, 0, &[0.0], &[1.0], &[1.0], &[0.1], &[], &[]);
    assert!(r.is_err(), "measurement_dim = 0 must be rejected");
    let r = LinearKalmanFilter::<f64>::new(0, 1, &[], &[], &[], &[], &[], &[1.0]);
    assert!(r.is_err(), "state_dim = 0 must be rejected");
    let r = LinearKalmanFilter::<f64>::new(0, 0, &[], &[], &[], &[], &[], &[]);
    assert!(r.is_err(), "zero dimensions must be rejected");
}

// ===========================================================================
// B4: initial covariance must be symmetric PSD (CHANGE)
// ===========================================================================

#[test]
fn spec_change_rejects_asymmetric_initial_covariance() {
    let p0 = [1.0, 0.5, 0.0, 1.0];
    let r = LinearKalmanFilter::<f64>::new(2, 1, &X0, &p0, &F, &Q, &H, &R);
    assert!(matches!(r, Err(KalmanError::InvalidCovariance)), "asymmetric P0 must be InvalidCovariance");
}

#[test]
fn spec_change_rejects_indefinite_initial_covariance() {
    // Negative diagonal.
    let r = LinearKalmanFilter::<f64>::new(2, 1, &X0, &[-1.0, 0.0, 0.0, 1.0], &F, &Q, &H, &R);
    assert!(matches!(r, Err(KalmanError::InvalidCovariance)), "negative-diagonal P0");
    // Positive diagonal but indefinite (eigenvalues 3 and -1).
    let r = LinearKalmanFilter::<f64>::new(2, 1, &X0, &[1.0, 2.0, 2.0, 1.0], &F, &Q, &H, &R);
    assert!(matches!(r, Err(KalmanError::InvalidCovariance)), "indefinite P0");
}

// ===========================================================================
// B5: noise covariances must be symmetric PSD (CHANGE for Q, partly KEEP for R)
// ===========================================================================

#[test]
fn spec_change_rejects_invalid_process_noise() {
    for q in [[0.01, 0.005, 0.0, 0.01], [-0.01, 0.0, 0.0, 0.01], [0.01, 0.02, 0.02, 0.01]] {
        let r = LinearKalmanFilter::<f64>::new(2, 1, &X0, &P0, &F, &q, &H, &R);
        assert!(matches!(r, Err(KalmanError::InvalidNoiseCovariance)), "Q = {q:?} must be InvalidNoiseCovariance");
    }
}

#[test]
fn rejects_negative_measurement_noise() {
    let r = LinearKalmanFilter::<f64>::new(2, 1, &X0, &P0, &F, &Q, &H, &[-0.1]);
    assert!(matches!(r, Err(KalmanError::InvalidNoiseCovariance)));
}

#[test]
fn spec_change_rejects_asymmetric_or_indefinite_measurement_noise() {
    let h = [1.0, 0.0, 0.0, 1.0];
    for r_bad in [[1.0, 0.5, 0.0, 1.0], [1.0, 2.0, 2.0, 1.0]] {
        let r = LinearKalmanFilter::<f64>::new(2, 2, &X0, &P0, &F, &Q, &h, &r_bad);
        assert!(matches!(r, Err(KalmanError::InvalidNoiseCovariance)), "R = {r_bad:?} must be InvalidNoiseCovariance");
    }
}

#[test]
fn accepts_singular_positive_semidefinite_matrices() {
    // P0 = 0 (state known exactly), Q = 0 (deterministic dynamics) and a
    // rank-deficient PSD R are all valid models.
    let h = [1.0, 0.0, 0.0, 1.0];
    let r_singular = [1.0, 1.0, 1.0, 1.0];
    let kf = LinearKalmanFilter::<f64>::new(2, 2, &X0, &[0.0; 4], &F, &[0.0; 4], &h, &r_singular);
    assert!(kf.is_ok(), "PSD (singular) P0, Q, R must be accepted: {:?}", kf.err());
}

// ===========================================================================
// B6: non-finite inputs (CHANGE)
// ===========================================================================

#[test]
fn spec_change_rejects_non_finite_construction_inputs() {
    let nan = f64::NAN;
    let inf = f64::INFINITY;
    let cases: Vec<(&str, Result<LinearKalmanFilter<f64>, KalmanError>)> = vec![
        ("x0", LinearKalmanFilter::new(2, 1, &[nan, 0.0], &P0, &F, &Q, &H, &R)),
        ("p0", LinearKalmanFilter::new(2, 1, &X0, &[nan, 0.0, 0.0, 1.0], &F, &Q, &H, &R)),
        ("f", LinearKalmanFilter::new(2, 1, &X0, &P0, &[1.0, inf, 0.0, 1.0], &Q, &H, &R)),
        ("q", LinearKalmanFilter::new(2, 1, &X0, &P0, &F, &[0.01, 0.0, 0.0, nan], &H, &R)),
        ("h", LinearKalmanFilter::new(2, 1, &X0, &P0, &F, &Q, &[nan, 0.0], &R)),
        ("r", LinearKalmanFilter::new(2, 1, &X0, &P0, &F, &Q, &H, &[nan])),
    ];
    for (what, r) in cases {
        assert!(r.is_err(), "non-finite {what} must be rejected at construction");
    }
}

#[test]
fn spec_change_update_rejects_non_finite_measurement() {
    let mut kf = baseline();
    kf.predict().unwrap();
    let (x, p) = (v(kf.state()), v(kf.covariance()));
    assert!(kf.update(&[f64::NAN]).is_err(), "NaN measurement must be rejected");
    assert!(kf.update(&[f64::INFINITY]).is_err(), "infinite measurement must be rejected");
    assert_eq!(v(kf.state()), x, "state must be unchanged after a rejected update");
    assert_eq!(v(kf.covariance()), p, "covariance must be unchanged after a rejected update");
}

// ===========================================================================
// B7: predict equations (KEEP)
// ===========================================================================

#[test]
fn predict_propagates_state_and_covariance() {
    let p0 = [2.0, 0.5, 0.5, 1.0];
    let q = [0.01, 0.0, 0.0, 0.02];
    let mut kf = LinearKalmanFilter::<f64>::new(2, 1, &[1.0, 2.0], &p0, &F, &q, &H, &R).unwrap();
    kf.predict().unwrap();
    // x = F x ; P = F P F^T + Q  (hand-computed)
    assert_close(&v(kf.state()), &[1.2, 2.0], 1e-12, "x");
    assert_close(&v(kf.covariance()), &[2.12, 0.6, 0.6, 1.02], 1e-12, "P");
}

// ===========================================================================
// B8: update equations (KEEP)
// ===========================================================================

#[test]
fn scalar_update_matches_closed_form() {
    // 1-D: K = P/(P+R), x+ = x + K (z - x), P+ = (1-K) P
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[0.25]).unwrap();
    kf.update(&[2.0]).unwrap();
    let k = 1.0 / 1.25;
    assert_close(&v(kf.state()), &[k * 2.0], 1e-12, "x");
    assert_close(&v(kf.covariance()), &[(1.0 - k) * 1.0], 1e-12, "P");
}

#[test]
fn matches_reference_filter_f64() {
    let mut oracle = rich_model();
    let (n, m) = (oracle.n, oracle.m);
    let mut kf = LinearKalmanFilter::<f64>::new(n, m, &oracle.x, &oracle.p, &oracle.f, &oracle.q, &oracle.h, &oracle.r).unwrap();
    for k in 0..50 {
        kf.predict().unwrap();
        oracle.predict();
        assert_close(&v(kf.state()), &oracle.x, 1e-9, &format!("x after predict {k}"));
        assert_close(&v(kf.covariance()), &oracle.p, 1e-9, &format!("P after predict {k}"));
        let z = rich_measurement(k);
        kf.update(&z).unwrap();
        oracle.update(&z);
        assert_close(&v(kf.state()), &oracle.x, 1e-9, &format!("x after update {k}"));
        assert_close(&v(kf.covariance()), &oracle.p, 1e-9, &format!("P after update {k}"));
    }
}

#[test]
fn matches_reference_filter_f32() {
    let mut oracle = rich_model();
    let (n, m) = (oracle.n, oracle.m);
    let mut kf = LinearKalmanFilter::<f32>::new(
        n,
        m,
        &to_f32(&oracle.x),
        &to_f32(&oracle.p),
        &to_f32(&oracle.f),
        &to_f32(&oracle.q),
        &to_f32(&oracle.h),
        &to_f32(&oracle.r),
    )
    .unwrap();
    for k in 0..50 {
        kf.predict().unwrap();
        oracle.predict();
        let z = rich_measurement(k);
        kf.update(&to_f32(&z)).unwrap();
        oracle.update(&z);
        assert_close(&to_f64(&v(kf.state())), &oracle.x, 1e-4, &format!("x after step {k}"));
        assert_close(&to_f64(&v(kf.covariance())), &oracle.p, 1e-4, &format!("P after step {k}"));
    }
}

#[test]
fn scalar_random_walk_reaches_riccati_steady_state() {
    // Random walk: Pp = P + Q, P = Pp R / (Pp + R).
    // Steady-state predicted covariance solves Pp^2 - Q Pp - Q R = 0.
    let (q, r) = (0.1, 1.0);
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[10.0], &[1.0], &[q], &[1.0], &[r]).unwrap();
    for _ in 0..200 {
        kf.predict().unwrap();
        kf.update(&[0.0]).unwrap();
    }
    let pp = (q + (q * q + 4.0 * q * r).sqrt()) / 2.0;
    let p_post = pp * r / (pp + r);
    assert_close(&v(kf.covariance()), &[p_post], 1e-9, "steady-state P");
}

#[test]
fn unobserved_state_keeps_growing_uncertainty() {
    // 3 independent random walks, only the first two observed.
    let eye3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    let q = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
    let h = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let r = [0.1, 0.0, 0.0, 0.1];
    let mut kf = LinearKalmanFilter::<f64>::new(3, 2, &[0.0; 3], &eye3, &eye3, &q, &h, &r).unwrap();
    for _ in 0..50 {
        kf.predict().unwrap();
        kf.update(&[5.0, 3.0]).unwrap();
    }
    let (x, p) = (v(kf.state()), v(kf.covariance()));
    assert!((x[0] - 5.0).abs() < 0.05 && (x[1] - 3.0).abs() < 0.05, "observed states converge: {x:?}");
    assert!(x[2].abs() < 1e-12, "unobserved state is not moved: {x:?}");
    assert_close(&[p[8]], &[1.0 + 50.0 * 0.01], 1e-12, "unobserved variance = P0 + k Q");
}

#[test]
fn estimates_are_statistically_consistent_nees_nis() {
    // Monte-Carlo consistency on a simulated constant-velocity target:
    // average NEES ~ n = 2, average NIS ~ m = 1.
    let dt: f64 = 1.0;
    let qs: f64 = 0.01;
    let f = [1.0, dt, 0.0, 1.0];
    let q = [qs * dt * dt * dt / 3.0, qs * dt * dt / 2.0, qs * dt * dt / 2.0, qs * dt];
    // Cholesky of Q (2x2) for sampling process noise.
    let l00 = q[0].sqrt();
    let l10 = q[2] / l00;
    let l11 = (q[3] - l10 * l10).sqrt();
    let h = [1.0, 0.0];
    let rv: f64 = 1.0;
    let p0 = [10.0, 0.0, 0.0, 1.0];
    let x0 = [0.0, 1.0];
    let mut rng = Rng(0x5EED_0001);
    let (runs, steps) = (300, 50);
    let (mut nees_sum, mut nis_sum, mut count) = (0.0, 0.0, 0usize);
    for _ in 0..runs {
        let mut xt = [x0[0] + 10f64.sqrt() * rng.normal(), x0[1] + rng.normal()];
        let mut kf = LinearKalmanFilter::<f64>::new(2, 1, &x0, &p0, &f, &q, &h, &[rv]).unwrap();
        for _ in 0..steps {
            let (w0, w1) = (rng.normal(), rng.normal());
            xt = [xt[0] + dt * xt[1] + l00 * w0, xt[1] + l10 * w0 + l11 * w1];
            let z = xt[0] + rv.sqrt() * rng.normal();
            kf.predict().unwrap();
            let pp = v(kf.covariance());
            let y = z - v(kf.state())[0];
            nis_sum += y * y / (pp[0] + rv);
            kf.update(&[z]).unwrap();
            let (x, p) = (v(kf.state()), v(kf.covariance()));
            let e = [xt[0] - x[0], xt[1] - x[1]];
            let det = p[0] * p[3] - p[1] * p[2];
            nees_sum += (e[0] * e[0] * p[3] - e[0] * e[1] * (p[1] + p[2]) + e[1] * e[1] * p[0]) / det;
            count += 1;
        }
    }
    let nees = nees_sum / count as f64;
    let nis = nis_sum / count as f64;
    assert!((1.8..=2.2).contains(&nees), "average NEES {nees} not ~2");
    assert!((0.9..=1.1).contains(&nis), "average NIS {nis} not ~1");
}

// ===========================================================================
// B9: update error handling (KEEP)
// ===========================================================================

#[test]
fn update_rejects_wrong_measurement_length_and_leaves_filter_unchanged() {
    let mut kf = baseline();
    kf.predict().unwrap();
    let (x, p) = (v(kf.state()), v(kf.covariance()));
    assert!(is_dimension_mismatch(&kf.update(&[1.0, 2.0])));
    assert!(is_dimension_mismatch(&kf.update(&[])));
    assert_eq!(v(kf.state()), x);
    assert_eq!(v(kf.covariance()), p);
    // Still usable afterwards.
    kf.update(&[0.5]).unwrap();
}

#[test]
fn update_with_singular_innovation_covariance_fails_atomically() {
    // P0 = 0 and a rank-deficient R make S = H P H^T + R singular.
    let h = [1.0, 0.0, 0.0, 1.0];
    let r = [1.0, 1.0, 1.0, 1.0];
    let mut kf = LinearKalmanFilter::<f64>::new(2, 2, &X0, &[0.0; 4], &[1.0, 0.0, 0.0, 1.0], &[0.0; 4], &h, &r).unwrap();
    let (x, p) = (v(kf.state()), v(kf.covariance()));
    let res = kf.update(&[1.0, 2.0]);
    assert!(matches!(res, Err(KalmanError::SingularMatrix)), "expected SingularMatrix, got {res:?}");
    assert_eq!(v(kf.state()), x);
    assert_eq!(v(kf.covariance()), p);
}

// ===========================================================================
// B10: scale-invariant singularity detection (CHANGE)
// ===========================================================================

#[test]
fn spec_change_small_scale_update_is_not_reported_singular() {
    // A perfectly conditioned 1-D problem whose variances are small in the
    // chosen units. Current code inverts S with an absolute pivot threshold
    // (KalmanScalar::epsilon: 1e-10 for f64, 1e-6 for f32; src/matrix/naive.rs:143)
    // and reports SingularMatrix.
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[4e-12], &[1.0], &[0.0], &[1.0], &[4e-12]).unwrap();
    kf.update(&[1e-6]).expect("f64: S = 8e-12 is well conditioned");
    assert_close(&v(kf.state()), &[0.5e-6], 1e-9, "f64 x");
    assert_close(&v(kf.covariance()), &[2e-12], 1e-9, "f64 P");

    let mut kf = LinearKalmanFilter::<f32>::new(1, 1, &[0.0], &[1e-7], &[1.0], &[0.0], &[1.0], &[1e-7]).unwrap();
    kf.update(&[1e-3]).expect("f32: S = 2e-7 is well conditioned");
    assert_close(&to_f64(&v(kf.state())), &[0.5e-3], 1e-5, "f32 x");
    assert_close(&to_f64(&v(kf.covariance())), &[0.5e-7], 1e-5, "f32 P");
}

// ===========================================================================
// B11: covariance stays symmetric PSD on ill-conditioned models (CHANGE)
// ===========================================================================

/// Model A: constant velocity, huge prior (1e8) and very precise position
/// measurements (R = 1e-8), tiny process noise. Condition number ~1e16.
fn model_a() -> (usize, usize, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let qv = 1e-10;
    (
        2,
        1,
        vec![1e8, 0.0, 0.0, 1e8],
        vec![1.0, 1.0, 0.0, 1.0],
        vec![qv * 0.25, qv * 0.5, qv * 0.5, qv],
        vec![1.0, 0.0],
    )
}
const MODEL_A_R: [f64; 1] = [1e-8];

/// Model B: two nearly collinear precise measurements of a static state
/// (H = [[1,1],[1,1+d]], R = d^2 I, d = 1e-2), Q = 0.
const D: f64 = 1e-2;
const MODEL_B_H: [f64; 4] = [1.0, 1.0, 1.0, 1.0 + D];
const MODEL_B_R: [f64; 4] = [D * D, 0.0, 0.0, D * D];

fn psd_measurement(k: usize, i: usize) -> f64 {
    ((k * 7 + i * 3) % 11) as f64 * 0.01
}

macro_rules! psd_run {
    ($name:ident, $t:ty, $conv_in:expr, $conv_out:expr) => {
        fn $name(label: &str, n: usize, m: usize, p0: &[f64], f: &[f64], q: &[f64], h: &[f64], r: &[f64], sym_tol: f64, psd_tol: f64) {
            let c = $conv_in;
            let back = $conv_out;
            let x0 = vec![0.0; n];
            let mut kf = LinearKalmanFilter::<$t>::new(n, m, &c(&x0), &c(p0), &c(f), &c(q), &c(h), &c(r))
                .unwrap_or_else(|e| panic!("{label}: construction failed: {e:?}"));
            for k in 0..10_000 {
                kf.predict().unwrap_or_else(|e| panic!("{label}: predict {k} failed: {e:?}"));
                let z: Vec<f64> = (0..m).map(|i| psd_measurement(k, i)).collect();
                kf.update(&c(&z)).unwrap_or_else(|e| panic!("{label}: update {k} failed: {e:?}"));
                let p = back(&v(kf.covariance()));
                if let Err(msg) = check_covariance(&p, n, sym_tol, psd_tol) {
                    panic!("{label}: step {k}: {msg}");
                }
            }
        }
    };
}

psd_run!(psd_run_f64, f64, |a: &[f64]| a.to_vec(), |a: &[f64]| a.to_vec());
psd_run!(psd_run_f32, f32, |a: &[f64]| to_f32(a), |a: &[f32]| to_f64(a));

#[test]
fn spec_change_covariance_stays_psd_ill_conditioned_f64() {
    let (n, m, p0, f, q, h) = model_a();
    psd_run_f64("model A f64", n, m, &p0, &f, &q, &h, &MODEL_A_R, 1e-10, 1e-9);
    psd_run_f64("model B f64", 2, 2, &P0, &[1.0, 0.0, 0.0, 1.0], &[0.0; 4], &MODEL_B_H, &MODEL_B_R, 1e-10, 1e-9);
}

#[test]
fn spec_change_covariance_stays_psd_ill_conditioned_f32() {
    let (n, m, p0, f, q, h) = model_a();
    psd_run_f32("model B f32", 2, 2, &P0, &[1.0, 0.0, 0.0, 1.0], &[0.0; 4], &MODEL_B_H, &MODEL_B_R, 1e-4, 1e-5);
    psd_run_f32("model A f32", n, m, &p0, &f, &q, &h, &MODEL_A_R, 1e-4, 1e-5);
}

#[test]
fn covariance_stays_symmetric_psd_well_conditioned() {
    // KEEP: a benign model must keep a symmetric PSD covariance (both precisions).
    let o = rich_model();
    let r = o.r.clone();
    psd_run_f64("rich f64", o.n, o.m, &o.p, &o.f, &o.q, &o.h, &r, 1e-10, 1e-9);
    psd_run_f32("rich f32", o.n, o.m, &o.p, &o.f, &o.q, &o.h, &r, 1e-4, 1e-5);
}

// ===========================================================================
// B12: no heap allocation in steady-state predict/update (CHANGE)
// ===========================================================================

#[test]
fn spec_change_no_heap_allocation_in_steady_state() {
    let o = rich_model();
    let mut kf64 = LinearKalmanFilter::<f64>::new(o.n, o.m, &o.x, &o.p, &o.f, &o.q, &o.h, &o.r).unwrap();
    let mut kf32 = LinearKalmanFilter::<f32>::new(
        o.n,
        o.m,
        &to_f32(&o.x),
        &to_f32(&o.p),
        &to_f32(&o.f),
        &to_f32(&o.q),
        &to_f32(&o.h),
        &to_f32(&o.r),
    )
    .unwrap();
    let zs: Vec<Vec<f64>> = (0..100).map(rich_measurement).collect();
    let zs32: Vec<Vec<f32>> = zs.iter().map(|z| to_f32(z)).collect();
    // Warm-up (first call may lazily initialize anything it likes).
    kf64.predict().unwrap();
    kf64.update(&zs[0]).unwrap();
    kf32.predict().unwrap();
    kf32.update(&zs32[0]).unwrap();

    let before = alloc_counter::count();
    for z in &zs {
        kf64.predict().unwrap();
        kf64.update(z).unwrap();
    }
    let f64_allocs = alloc_counter::count() - before;

    let before = alloc_counter::count();
    for z in &zs32 {
        kf32.predict().unwrap();
        kf32.update(z).unwrap();
    }
    let f32_allocs = alloc_counter::count() - before;

    assert_eq!(f64_allocs, 0, "f64: heap allocations in 100 predict+update cycles");
    assert_eq!(f32_allocs, 0, "f32: heap allocations in 100 predict+update cycles");
}

// ===========================================================================
// B18: covariance validation is scale-invariant (CHANGE)
// ===========================================================================

/// 1-D random walk with every variance multiplied by s² (and states by s).
/// Returns (x / s, P / s²) after 10 predict+update cycles.
fn scaled_random_walk(s: f64) -> Result<(f64, f64), String> {
    let s2 = s * s;
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0 * s2], &[1.0], &[0.01 * s2], &[1.0], &[0.5 * s2])
        .map_err(|e| format!("new: {e:?}"))?;
    for k in 0..10 {
        kf.predict().map_err(|e| format!("predict {k}: {e:?}"))?;
        kf.update(&[(k as f64 * 0.1) * s]).map_err(|e| format!("update {k}: {e:?}"))?;
    }
    Ok((v(kf.state())[0] / s, v(kf.covariance())[0] / s2))
}

#[test]
fn spec_change_noise_covariance_validation_is_scale_invariant() {
    // The same model in different units is equally valid and gives the same
    // result in scaled units. Today validate_measurement_noise rejects any
    // diag(R) below an ABSOLUTE 1e-12 (src/validation.rs:58-60, via
    // src/filter.rs:150).
    let unit = scaled_random_walk(1.0).expect("unit-scale model must be accepted");
    for s in [1e-7, 1e-4, 1e4, 1e7] {
        let r = scaled_random_walk(s).unwrap_or_else(|e| panic!("scale {s:e}: valid model rejected: {e}"));
        assert!((r.0 - unit.0).abs() <= 1e-9 * unit.0.abs().max(1.0), "scale {s:e}: state {r:?} vs {unit:?}");
        assert!((r.1 - unit.1).abs() <= 1e-9 * unit.1.abs(), "scale {s:e}: covariance {r:?} vs {unit:?}");
    }

    // A tiny R next to P0 = 1: S = 1 + 1e-14 is perfectly conditioned, so this
    // isolates construction from B10. Closed form: x = z/(1+R), P = R/(1+R).
    let r = 1e-14;
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[r])
        .unwrap_or_else(|e| panic!("f64 R = {r:e} is positive definite but rejected: {e:?}"));
    kf.update(&[1.0]).expect("S = 1 + 1e-14 is well conditioned");
    assert_close(&v(kf.state()), &[1.0 / (1.0 + r)], 1e-15, "x");
    let p = v(kf.covariance())[0];
    assert!((p - r / (1.0 + r)).abs() <= 1e-9 * r, "P = {p:e}, expected {:e}", r / (1.0 + r));

    // f32 (1e-13 is a normal f32) and a multichannel R with one precise channel.
    let r32 = LinearKalmanFilter::<f32>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[1e-13]);
    assert!(r32.is_ok(), "f32 R = 1e-13 rejected: {:?}", r32.err());
    let i2 = [1.0, 0.0, 0.0, 1.0];
    let res = LinearKalmanFilter::<f64>::new(2, 2, &[0.0, 0.0], &i2, &i2, &[0.0; 4], &i2, &[1.0, 0.0, 0.0, 1e-13]);
    assert!(res.is_ok(), "R = diag(1, 1e-13) rejected: {:?}", res.err());
}

// ===========================================================================
// B19: dimension arithmetic cannot overflow into a panic (CHANGE)
// ===========================================================================

#[test]
fn spec_change_overflowing_dimensions_are_rejected_without_panic() {
    use std::panic::{AssertUnwindSafe, catch_unwind};
    // n = 2 and a huge m: the length check `m * n` (src/filter.rs:127)
    // overflows ("attempt to multiply with overflow"). In release builds it
    // wraps instead, and validate_measurement_noise then indexes R out of bounds.
    let empty: [f64; 0] = [];
    let cases: [(usize, &[f64], &[f64]); 3] = [
        (usize::MAX, &H, &R),
        (usize::MAX / 2 + 1, &H, &R),
        (usize::MAX / 2 + 1, &empty, &empty),
    ];
    let mut failures = Vec::new();
    for (m, h, r) in cases {
        let res = catch_unwind(AssertUnwindSafe(|| {
            LinearKalmanFilter::<f64>::new(2, m, &X0, &P0, &F, &Q, h, r).map(|_| ())
        }));
        match res {
            Ok(Err(_)) => {}
            Ok(Ok(())) => failures.push(format!("measurement_dim = {m}, h.len() = {}: accepted", h.len())),
            Err(_) => failures.push(format!("measurement_dim = {m}, h.len() = {}: panicked", h.len())),
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

// ===========================================================================
// B20: predict/update never commit a non-finite estimate (CHANGE)
// ===========================================================================

/// `Ok` must leave a finite state and covariance; `Err` must leave both unchanged.
fn assert_finite_or_unchanged(
    what: &str,
    res: Result<(), KalmanError>,
    before: (Vec<f64>, Vec<f64>),
    after: (Vec<f64>, Vec<f64>),
) {
    match res {
        Ok(()) => assert!(
            after.0.iter().chain(&after.1).all(|a| a.is_finite()),
            "{what}: returned Ok but state = {:?}, covariance = {:?}",
            after.0,
            after.1
        ),
        Err(e) => assert!(before == after, "{what}: returned {e:?} but changed the filter: {before:?} -> {after:?}"),
    }
}

#[test]
fn spec_change_predict_never_commits_non_finite_estimate() {
    // F x overflows: x0 = 1e200, F = 1e200 (all inputs finite and valid).
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[1e200], &[1.0], &[1e200], &[0.0], &[1.0], &[1.0]).unwrap();
    let before = (v(kf.state()), v(kf.covariance()));
    let res = kf.predict();
    assert_finite_or_unchanged("predict, F x overflows", res, before, (v(kf.state()), v(kf.covariance())));

    // F P Fᵀ overflows: P0 = 1e200, F = 1e200. Today P becomes inf with Ok, and
    // the next update returns Ok with a NaN state.
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[1.0], &[1e200], &[1e200], &[0.0], &[1.0], &[1.0]).unwrap();
    let before = (v(kf.state()), v(kf.covariance()));
    let res = kf.predict();
    assert_finite_or_unchanged("predict, F P F^T overflows", res, before, (v(kf.state()), v(kf.covariance())));
    let before = (v(kf.state()), v(kf.covariance()));
    let res = kf.update(&[0.0]);
    assert_finite_or_unchanged("update after overflowing predict", res, before, (v(kf.state()), v(kf.covariance())));
}

#[test]
fn spec_change_update_never_commits_non_finite_estimate() {
    // Finite x0 and z whose difference overflows: y = z - H x = +inf
    // (src/filter.rs:266-270, :315-317; nothing is checked before committing).
    let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[-1e308], &[1.0], &[1.0], &[0.0], &[1.0], &[1.0]).unwrap();
    let before = (v(kf.state()), v(kf.covariance()));
    let res = kf.update(&[1e308]);
    assert_finite_or_unchanged("f64 update, innovation overflows", res, before, (v(kf.state()), v(kf.covariance())));

    let mut kf = LinearKalmanFilter::<f32>::new(1, 1, &[-3e38], &[1.0], &[1.0], &[0.0], &[1.0], &[1.0]).unwrap();
    let before = (to_f64(&v(kf.state())), to_f64(&v(kf.covariance())));
    let res = kf.update(&[3e38]);
    let after = (to_f64(&v(kf.state())), to_f64(&v(kf.covariance())));
    assert_finite_or_unchanged("f32 update, innovation overflows", res, before, after);
}

// ===========================================================================
// B21: singular S is detected relative to its own scale; a successful
// update never yields an indefinite covariance (CHANGE)
// ===========================================================================

/// Cholesky with a pivot tolerance RELATIVE to the diagonal: d_k <= 1e3·eps·S_kk
/// means singular. Feasibility reference for the test below.
fn ref_cholesky_is_singular(s: &[f64], m: usize) -> bool {
    let mut l = vec![0.0; m * m];
    for j in 0..m {
        let mut d = s[j * m + j];
        for k in 0..j {
            d -= l[j * m + k] * l[j * m + k];
        }
        if !(d > 1e3 * f64::EPSILON * s[j * m + j]) {
            return true;
        }
        l[j * m + j] = d.sqrt();
        for i in j + 1..m {
            let mut acc = s[i * m + j];
            for k in 0..j {
                acc -= l[i * m + k] * l[j * m + k];
            }
            l[i * m + j] = acc / l[j * m + j];
        }
    }
    false
}

#[test]
fn spec_change_large_scale_singular_innovation_is_detected() {
    // n = 1, m = 2, H = h, R = scale·h hᵀ (rank-1 PSD, valid per B5), P0 = scale:
    // S = 2·scale·h hᵀ has rank 1 at every scale, and z = [0, 1] has a component
    // outside range(S). The absolute pivot threshold (src/matrix/naive.rs:147)
    // lets the rounding residual of the zero pivot through at large scale, and
    // the update returns Ok with a NEGATIVE variance (e.g. h = [1, 0.1] at 1e8).
    let mut hbs = vec![1.0 / 3.0, 0.1, 0.7, 1.0 / 7.0, 0.123456789, std::f64::consts::SQRT_2, 1.1];
    let mut rng = Rng(0xA0D1_7006);
    hbs.extend((0..40).map(|_| 2.0 * rng.uniform() - 1.0));
    let scales = [1.0f64, 1e2, 1e4, 1e6, 1e8, 1e9, 1e10, 1e12];
    let (mut bad, mut total) = (Vec::new(), 0);
    for &hb in &hbs {
        for &scale in &scales {
            total += 1;
            let h = [1.0, hb];
            let r = [scale, hb * scale, hb * scale, hb * hb * scale];
            let s = [scale + r[0], scale * hb + r[1], scale * hb + r[2], scale * hb * hb + r[3]];
            assert!(ref_cholesky_is_singular(&s, 2), "reference failed to flag h=[1,{hb}] scale={scale:e}");
            let mut kf = LinearKalmanFilter::<f64>::new(1, 2, &[0.0], &[scale], &[1.0], &[0.0], &h, &r).unwrap();
            let res = kf.update(&[0.0, 1.0]);
            let (x, p) = (v(kf.state())[0], v(kf.covariance())[0]);
            match res {
                Err(KalmanError::SingularMatrix) if (x, p) == (0.0, scale) => {}
                // Tolerated only if the result is still a sane posterior.
                Ok(()) if x.is_finite() && x.abs() < 10.0 && p >= -1e-9 * scale && p <= scale => {}
                other => bad.push(format!("h=[1,{hb}] scale={scale:e}: {other:?}, x={x:e}, P={p:e}")),
            }
        }
    }
    assert!(bad.is_empty(), "singular S mishandled in {}/{total} cases:\n{}", bad.len(), bad.join("\n"));
}

/// Joseph form with S solved by Cholesky (relative pivot test); 2×2 row-major.
/// Returns None if S is numerically singular. Feasibility reference.
fn joseph_cholesky_ref_2x2(x: &[f64], p: &[f64], h: &[f64], r: &[f64], z: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
    let pht = mat_mul(p, &transpose(h, 2, 2), 2, 2, 2);
    let hp = transpose(&pht, 2, 2);
    let s: Vec<f64> = mat_mul(h, &pht, 2, 2, 2).iter().zip(r).map(|(a, b)| a + b).collect();
    let l00 = s[0].sqrt();
    let l10 = s[2] / l00;
    let d = s[3] - l10 * l10;
    if !(d > 1e-15 * s[3].abs().max(s[0].abs())) {
        return None;
    }
    let l11 = d.sqrt();
    // Solve S Kᵀ = H P column by column.
    let mut kt = vec![0.0; 4];
    for c in 0..2 {
        let y0 = hp[c] / l00;
        let y1 = (hp[2 + c] - l10 * y0) / l11;
        let k1 = y1 / l11;
        kt[c] = (y0 - l10 * k1) / l00;
        kt[2 + c] = k1;
    }
    let k = transpose(&kt, 2, 2);
    let hx = mat_mul(h, x, 2, 2, 1);
    let y = [z[0] - hx[0], z[1] - hx[1]];
    let ky = mat_mul(&k, &y, 2, 2, 1);
    let xn = vec![x[0] + ky[0], x[1] + ky[1]];
    let kh = mat_mul(&k, h, 2, 2, 2);
    let a = [1.0 - kh[0], -kh[1], -kh[2], 1.0 - kh[3]];
    let apa = mat_mul(&mat_mul(&a, p, 2, 2, 2), &transpose(&a, 2, 2), 2, 2, 2);
    let krk = mat_mul(&mat_mul(&k, r, 2, 2, 2), &kt, 2, 2, 2);
    let mut pn: Vec<f64> = apa.iter().zip(&krk).map(|(u, w)| u + w).collect();
    let off = 0.5 * (pn[1] + pn[2]);
    pn[1] = off;
    pn[2] = off;
    Some((xn, pn))
}

#[test]
fn spec_change_successful_update_never_yields_indefinite_covariance() {
    // P0 = 1e30·I (huge but finite prior), R = I, two nearly collinear sensors
    // H = [[1,1],[1,1+1e-15]]. All inputs are valid under B2-B6. Today one
    // update returns Ok with P = [[0, -1e30], [-1e30, -2.7e15]]: the absolute
    // pivot test (src/matrix/naive.rs:147) accepts the numerically singular S
    // and the simple-form update (src/filter.rs:329-347) produces garbage.
    let p0 = [1e30f64, 0.0, 0.0, 1e30];
    let h = [1.0, 1.0, 1.0, 1.0 + 1e-15];
    let i2 = [1.0, 0.0, 0.0, 1.0];
    let x0 = [0.0, 0.0];
    let z = [h[0] + h[1], h[2] + h[3]];

    // A correct implementation can meet the bar.
    if let Some((xr, pr)) = joseph_cholesky_ref_2x2(&x0, &p0, &h, &i2, &z) {
        assert!(xr.iter().all(|a| a.is_finite()), "reference state not finite");
        check_covariance(&pr, 2, 1e-10, 1e-9).expect("reference Joseph+Cholesky must be PSD");
    }

    let mut kf = LinearKalmanFilter::<f64>::new(2, 2, &x0, &p0, &i2, &[0.0; 4], &h, &i2).unwrap();
    match kf.update(&z) {
        Err(KalmanError::SingularMatrix) => {
            assert_eq!(v(kf.state()), x0.to_vec());
            assert_eq!(v(kf.covariance()), p0.to_vec());
        }
        Err(e) => panic!("unexpected error {e:?}"),
        Ok(()) => {
            let (x, p) = (v(kf.state()), v(kf.covariance()));
            assert!(x.iter().all(|a| a.is_finite()), "state not finite: {x:?}");
            assert!(p[0] >= 0.0 && p[3] >= 0.0, "negative variance: P = {p:?}");
            if let Err(msg) = check_covariance(&p, 2, 1e-10, 1e-9) {
                panic!("update returned Ok but posterior is not PSD: {msg}");
            }
        }
    }
}

// ===========================================================================
// B22: results do not depend on the units of individual measurement
// components (CHANGE)
// ===========================================================================

/// Rescale measurement component i by c_i: row i of H by c_i, R -> D R D.
fn rich_model_unit_scaled(c: &[f64]) -> RefKf {
    let mut md = rich_model();
    let (n, m) = (md.n, md.m);
    for i in 0..m {
        for j in 0..n {
            md.h[i * n + j] *= c[i];
        }
        for j in 0..m {
            md.r[i * m + j] *= c[i] * c[j];
        }
    }
    md
}

fn rel_err(a: &[f64], b: &[f64]) -> f64 {
    let d = a.iter().zip(b).fold(0.0f64, |acc, (x, y)| acc.max((x - y).abs()));
    d / b.iter().fold(f64::MIN_POSITIVE, |acc, x| acc.max(x.abs()))
}

#[test]
fn spec_change_measurement_unit_change_does_not_change_estimate() {
    // z_i -> c_i z_i, H_i -> c_i H_i, R -> D R D leaves the exact posterior
    // unchanged. Today the absolute pivot test (src/matrix/naive.rs:147)
    // reports SingularMatrix when one component is in small units, although
    // S's largest entry is O(1), so B10's uniform-scaling test does not cover
    // it. The factors keep every diag(R) >= 1e-12, so B18 plays no part.
    for c1 in [3e-6, 1e6] {
        let c = [1.0, c1];
        let sm = rich_model_unit_scaled(&c);
        let mut oracle = rich_model();
        // Feasibility: the textbook oracle run directly on the rescaled model.
        let mut oracle_scaled = rich_model_unit_scaled(&c);
        let mut kf = LinearKalmanFilter::<f64>::new(sm.n, sm.m, &sm.x, &sm.p, &sm.f, &sm.q, &sm.h, &sm.r)
            .unwrap_or_else(|e| panic!("f64 c={c1:e}: construction failed: {e:?}"));
        for k in 0..20 {
            let z = rich_measurement(k);
            let zc: Vec<f64> = z.iter().zip(&c).map(|(a, b)| a * b).collect();
            oracle.predict();
            oracle.update(&z);
            oracle_scaled.predict();
            oracle_scaled.update(&zc);
            let eo = rel_err(&oracle_scaled.x, &oracle.x).max(rel_err(&oracle_scaled.p, &oracle.p));
            assert!(eo < 1e-10, "oracle itself not unit-invariant: {eo:e}");
            kf.predict().unwrap();
            kf.update(&zc).unwrap_or_else(|e| panic!("f64 c={c1:e} step {k}: update failed: {e:?}"));
            let ex = rel_err(&v(kf.state()), &oracle.x).max(rel_err(&v(kf.covariance()), &oracle.p));
            assert!(ex < 1e-8, "f64 c={c1:e} step {k}: rel err {ex:e}");
        }
    }

    // f32: component 2 in units 1e3 times smaller.
    let c = [1.0, 1e-3];
    let sm = rich_model_unit_scaled(&c);
    let mut oracle = rich_model();
    let mut kf = LinearKalmanFilter::<f32>::new(
        sm.n,
        sm.m,
        &to_f32(&sm.x),
        &to_f32(&sm.p),
        &to_f32(&sm.f),
        &to_f32(&sm.q),
        &to_f32(&sm.h),
        &to_f32(&sm.r),
    )
    .unwrap_or_else(|e| panic!("f32: construction failed: {e:?}"));
    for k in 0..20 {
        let z = rich_measurement(k);
        let zc: Vec<f64> = z.iter().zip(&c).map(|(a, b)| a * b).collect();
        oracle.predict();
        oracle.update(&z);
        kf.predict().unwrap();
        kf.update(&to_f32(&zc)).unwrap_or_else(|e| panic!("f32 c=1e-3 step {k}: update failed: {e:?}"));
        let ex = rel_err(&to_f64(&v(kf.state())), &oracle.x).max(rel_err(&to_f64(&v(kf.covariance())), &oracle.p));
        assert!(ex < 1e-3, "f32 c=1e-3 step {k}: rel err {ex:e}");
    }
}
