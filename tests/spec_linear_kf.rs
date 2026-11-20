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
