//! The linear Kalman filter.
//!
//! All storage is allocated once in [`LinearKalmanFilter::new`]: the model
//! (F, Q, H, R), the estimate (x, P) and a fixed workspace sized for n and m.
//! `predict` and `update` only write into the workspace; when every check has
//! passed, the candidate estimate is committed by swapping two boxed slices
//! (O(1), no copy). A failed step therefore leaves the filter untouched, and
//! steady-state operation performs no heap allocation.
//!
//! Numerics:
//! - predict: `x ← F x`, `P ← F P Fᵀ + Q` (only the upper triangle is
//!   computed, then mirrored: P is exactly symmetric after every step).
//! - update: `S = H P Hᵀ + R` is Cholesky-factored with a pivot test relative to
//!   each diagonal entry of S; `Kᵀ = S⁻¹ (H P)` is obtained by triangular solves
//!   (S is never inverted); the covariance uses the Joseph form
//!   `P ← (I − K H) P (I − K H)ᵀ + K R Kᵀ`, a sum of PSD terms.

use crate::error::{KalmanError, KalmanResult};
use crate::matrix::{
    CholeskyError, add_inplace, add_transpose_matmul_upper, all_finite, axpy, cholesky_in_place,
    cholesky_solve_rows, dot, matmul, matmul_transpose_upper, mirror_upper, symmetrize,
};
use crate::types::KalmanScalar;
use crate::validation::is_psd;

/// The public module named by the specification: `kalman_filters::linear`.
pub mod linear {
    pub use super::LinearKalmanFilter;
}

/// Linear Kalman filter for `x_{k+1} = F x_k + w`, `z_k = H x_k + v`,
/// `w ~ N(0, Q)`, `v ~ N(0, R)`. All matrices are dense and row-major.
#[derive(Debug, Clone)]
pub struct LinearKalmanFilter<T> {
    n: usize,
    m: usize,
    // Estimate.
    x: Box<[T]>, // n
    p: Box<[T]>, // n×n
    // Model; Q and R are stored exactly symmetric.
    f: Box<[T]>, // n×n
    q: Box<[T]>, // n×n
    h: Box<[T]>, // m×n
    r: Box<[T]>, // m×m
    ws: Workspace<T>,
}

/// Scratch buffers, overwritten by every step. `x` and `p` hold the candidate
/// estimate and trade places with the committed one on success.
#[derive(Debug, Clone)]
struct Workspace<T> {
    x: Box<[T]>,   // n    candidate state
    p: Box<[T]>,   // n×n  candidate covariance
    a: Box<[T]>,   // n×n  I − K H
    ap: Box<[T]>,  // n×n  F P (predict) / (I − K H) P (update)
    y: Box<[T]>,   // m    innovation z − H x
    s: Box<[T]>,   // m×m  S, then its Cholesky factor (lower triangle)
    kt: Box<[T]>,  // m×n  H P, then Kᵀ = S⁻¹ H P
    rkt: Box<[T]>, // m×n  R Kᵀ
}

impl<T: KalmanScalar> Workspace<T> {
    fn new(n: usize, m: usize, nn: usize, mn: usize, mm: usize) -> Self {
        let z = |len: usize| vec![T::zero(); len].into_boxed_slice();
        Self { x: z(n), p: z(nn), a: z(nn), ap: z(nn), y: z(m), s: z(mm), kt: z(mn), rkt: z(mn) }
    }
}

/// Pivot tolerance for factoring S, relative to each diagonal entry:
/// `64 · m · ε`, a small multiple of the Cholesky rounding error.
fn singular_tol<T: KalmanScalar>(m: usize) -> T {
    <T as num_traits::Float>::epsilon() * T::from_usize(64 * m)
}

/// Cheap O(n²) necessary conditions for a symmetric PSD matrix: every variance
/// is ≥ 0 and every correlation satisfies |p_ij| ≤ √(p_ii p_jj) up to rounding.
/// Catches the gross failures of finite precision (negative variances,
/// impossible correlations) without an O(n³) factorisation per step.
fn variances_consistent<T: KalmanScalar>(p: &[T], n: usize) -> bool {
    let slack = T::one() + <T as num_traits::Float>::epsilon() * T::from_usize(64 * n);
    let sd = |i: usize| p[i * n + i].sqrt();
    (0..n).all(|i| p[i * n + i] >= T::zero())
        && (0..n).all(|i| (0..i).all(|j| p[i * n + j].abs() <= sd(i) * sd(j) * slack))
}

fn diverged(what: &str) -> KalmanError {
    KalmanError::FilterDivergence(format!("{what} is not finite"))
}

/// `Ok(len)` if `slice` holds exactly `rows × cols` elements.
fn check_len<T>(slice: &[T], rows: usize, cols: usize, len: usize) -> KalmanResult<()> {
    if slice.len() == len {
        Ok(())
    } else {
        Err(KalmanError::DimensionMismatch { expected: (rows, cols), actual: (slice.len(), 1) })
    }
}

impl<T: KalmanScalar> LinearKalmanFilter<T> {
    /// The single validated constructor. Slices are row-major:
    /// `x0`: n, `p0`: n×n, `f`: n×n, `q`: n×n, `h`: m×n, `r`: m×m.
    ///
    /// Errors: `InvalidDimension` (zero or overflowing dimensions),
    /// `DimensionMismatch` (wrong slice length), `NonFiniteInput` (x0, F, H),
    /// `InvalidCovariance` (p0 not finite symmetric PSD),
    /// `InvalidNoiseCovariance` (q or r not finite symmetric PSD).
    /// Singular PSD matrices are accepted; all tolerances are relative to the
    /// matrix's own diagonal, so validity does not depend on units.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        state_dim: usize,
        measurement_dim: usize,
        x0: &[T],
        p0: &[T],
        f: &[T],
        q: &[T],
        h: &[T],
        r: &[T],
    ) -> KalmanResult<Self> {
        let (n, m) = (state_dim, measurement_dim);
        let bad_dims = KalmanError::InvalidDimension { state_dim: n, measurement_dim: m };
        if n == 0 || m == 0 {
            return Err(bad_dims);
        }
        let (Some(nn), Some(mn), Some(mm)) = (n.checked_mul(n), m.checked_mul(n), m.checked_mul(m))
        else {
            return Err(bad_dims);
        };
        check_len(x0, n, 1, n)?;
        check_len(p0, n, n, nn)?;
        check_len(f, n, n, nn)?;
        check_len(q, n, n, nn)?;
        check_len(h, m, n, mn)?;
        check_len(r, m, m, mm)?;
        if !(all_finite(x0) && all_finite(f) && all_finite(h)) {
            return Err(KalmanError::NonFiniteInput);
        }

        // Validation borrows workspace buffers as scratch: no extra allocation.
        let mut ws = Workspace::new(n, m, nn, mn, mm);
        if !is_psd(p0, n, &mut ws.p) {
            return Err(KalmanError::InvalidCovariance);
        }
        if !is_psd(q, n, &mut ws.p) || !is_psd(r, m, &mut ws.s) {
            return Err(KalmanError::InvalidNoiseCovariance);
        }
        let mut q: Box<[T]> = q.into();
        let mut r: Box<[T]> = r.into();
        symmetrize(&mut q, n);
        symmetrize(&mut r, m);

        Ok(Self { n, m, x: x0.into(), p: p0.into(), f: f.into(), q, h: h.into(), r, ws })
    }

    pub fn state_dim(&self) -> usize {
        self.n
    }

    pub fn measurement_dim(&self) -> usize {
        self.m
    }

    /// State estimate x (length n).
    pub fn state(&self) -> &[T] {
        &self.x
    }

    /// Covariance P (n×n, row-major). Exactly symmetric after any step.
    pub fn covariance(&self) -> &[T] {
        &self.p
    }

    /// State transition matrix F (n×n).
    pub fn transition(&self) -> &[T] {
        &self.f
    }

    /// Process noise covariance Q (n×n), as stored (symmetrized).
    pub fn process_noise(&self) -> &[T] {
        &self.q
    }

    /// Observation matrix H (m×n).
    pub fn observation(&self) -> &[T] {
        &self.h
    }

    /// Measurement noise covariance R (m×m), as stored (symmetrized).
    pub fn measurement_noise(&self) -> &[T] {
        &self.r
    }

    /// `x ← F x`, `P ← F P Fᵀ + Q`. On a non-finite result returns
    /// `FilterDivergence` and leaves the filter unchanged.
    pub fn predict(&mut self) -> KalmanResult<()> {
        let n = self.n;
        let ws = &mut self.ws;

        for (xi, f_row) in ws.x.iter_mut().zip(self.f.chunks_exact(n)) {
            *xi = dot(f_row, &self.x);
        }
        matmul(&self.f, &self.p, &mut ws.ap, n, n, n); // F P
        matmul_transpose_upper(&ws.ap, &self.f, &mut ws.p, n, n); // (F P) Fᵀ
        add_inplace(&mut ws.p, &self.q, n * n);
        mirror_upper(&mut ws.p, n);

        if !all_finite(&ws.x) {
            return Err(diverged("predicted state"));
        }
        if !all_finite(&ws.p) {
            return Err(diverged("predicted covariance"));
        }
        if !variances_consistent(&ws.p, n) {
            return Err(KalmanError::FilterDivergence(
                "predicted covariance lost positive semi-definiteness to rounding".into(),
            ));
        }
        self.commit();
        Ok(())
    }

    /// Incorporate measurement `z` (length m).
    ///
    /// Errors, all leaving the filter unchanged: `DimensionMismatch` (wrong
    /// length), `NonFiniteInput` (NaN/∞ in z), `SingularMatrix` (S not
    /// numerically positive definite relative to its own diagonal),
    /// `FilterDivergence` (non-finite intermediate or result).
    pub fn update(&mut self, z: &[T]) -> KalmanResult<()> {
        let (n, m) = (self.n, self.m);
        if z.len() != m {
            return Err(KalmanError::DimensionMismatch { expected: (m, 1), actual: (z.len(), 1) });
        }
        if !all_finite(z) {
            return Err(KalmanError::NonFiniteInput);
        }
        let ws = &mut self.ws;

        // y = z − H x
        for ((yi, &zi), h_row) in ws.y.iter_mut().zip(z).zip(self.h.chunks_exact(n)) {
            *yi = zi - dot(h_row, &self.x);
        }
        if !all_finite(&ws.y) {
            return Err(diverged("innovation"));
        }

        // H P into kt; S = (H P) Hᵀ + R, exactly symmetric.
        matmul(&self.h, &self.p, &mut ws.kt, m, n, n);
        matmul_transpose_upper(&ws.kt, &self.h, &mut ws.s, m, n);
        add_inplace(&mut ws.s, &self.r, m * m);
        mirror_upper(&mut ws.s, m);

        // S = L Lᵀ, then Kᵀ = S⁻¹ (H P) in place: K = P Hᵀ S⁻¹ without inverting S.
        cholesky_in_place(&mut ws.s, m, singular_tol::<T>(m)).map_err(|e| match e {
            CholeskyError::Singular => KalmanError::SingularMatrix,
            CholeskyError::NonFinite => diverged("innovation covariance"),
        })?;
        cholesky_solve_rows(&ws.s, &mut ws.kt, m, n);

        // x⁺ = x + K y  (sum over rows of Kᵀ)
        ws.x.copy_from_slice(&self.x);
        for (&yj, kt_row) in ws.y.iter().zip(ws.kt.chunks_exact(n)) {
            axpy(yj, kt_row, &mut ws.x);
        }

        // A = I − K H  (subtract the outer products of Kᵀ rows and H rows)
        ws.a.fill(T::zero());
        for (i, row) in ws.a.chunks_exact_mut(n).enumerate() {
            row[i] = T::one();
        }
        for (kt_row, h_row) in ws.kt.chunks_exact(n).zip(self.h.chunks_exact(n)) {
            for (a_row, &k_ij) in ws.a.chunks_exact_mut(n).zip(kt_row) {
                axpy(-k_ij, h_row, a_row);
            }
        }

        // Joseph form: P⁺ = A P Aᵀ + K R Kᵀ  (upper triangle, then mirror)
        matmul(&ws.a, &self.p, &mut ws.ap, n, n, n);
        matmul_transpose_upper(&ws.ap, &ws.a, &mut ws.p, n, n);
        matmul(&self.r, &ws.kt, &mut ws.rkt, m, m, n);
        add_transpose_matmul_upper(&ws.kt, &ws.rkt, &mut ws.p, n, m);
        mirror_upper(&mut ws.p, n);

        if !all_finite(&ws.x) {
            return Err(diverged("updated state"));
        }
        if !all_finite(&ws.p) {
            return Err(diverged("updated covariance"));
        }
        // Rounding in the Joseph sum grows with ‖I − K H‖²; when S is so
        // ill-conditioned that it swamps the posterior, report S as singular
        // rather than commit an invalid covariance.
        if !variances_consistent(&ws.p, n) {
            return Err(KalmanError::SingularMatrix);
        }
        self.commit();
        Ok(())
    }

    /// Make the candidate estimate in the workspace current (pointer swap).
    fn commit(&mut self) {
        core::mem::swap(&mut self.x, &mut self.ws.x);
        core::mem::swap(&mut self.p, &mut self.ws.p);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cv_filter() -> LinearKalmanFilter<f64> {
        LinearKalmanFilter::new(
            2,
            1,
            &[0.0, 1.0],
            &[1.0, 0.0, 0.0, 1.0],
            &[1.0, 0.1, 0.0, 1.0],
            &[0.01, 0.0, 0.0, 0.01],
            &[1.0, 0.0],
            &[0.1],
        )
        .unwrap()
    }

    #[test]
    fn rejected_steps_leave_filter_unchanged() {
        let mut kf = cv_filter();
        kf.predict().unwrap();
        let before = (kf.state().to_vec(), kf.covariance().to_vec());
        assert!(matches!(kf.update(&[f64::NAN]), Err(KalmanError::NonFiniteInput)));
        assert!(matches!(kf.update(&[1.0, 2.0]), Err(KalmanError::DimensionMismatch { .. })));
        assert_eq!((kf.state().to_vec(), kf.covariance().to_vec()), before);
    }

    #[test]
    fn covariance_is_exactly_symmetric_after_each_step() {
        let mut kf = cv_filter();
        for k in 0..100 {
            kf.predict().unwrap();
            kf.update(&[k as f64 * 0.1]).unwrap();
            let p = kf.covariance();
            assert_eq!(p[1], p[2]);
        }
    }

    #[test]
    fn scalar_update_matches_closed_form() {
        let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[0.25]).unwrap();
        kf.update(&[2.0]).unwrap();
        assert!((kf.state()[0] - 1.6).abs() < 1e-15);
        assert!((kf.covariance()[0] - 0.2).abs() < 1e-15);
    }

    #[test]
    fn invalid_dimensions_do_not_panic() {
        let e = LinearKalmanFilter::<f32>::new(0, 1, &[], &[], &[], &[], &[], &[1.0]);
        assert!(matches!(e, Err(KalmanError::InvalidDimension { .. })));
        let e = LinearKalmanFilter::<f32>::new(usize::MAX, 1, &[], &[], &[], &[], &[], &[1.0]);
        assert!(matches!(e, Err(KalmanError::InvalidDimension { .. })));
    }
}
