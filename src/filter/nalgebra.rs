//! The nalgebra-backed implementation of the linear Kalman filter.
//!
//! The filter owns its estimate, model and workspace as run-time-sized nalgebra
//! values. Everything is allocated once in [`LinearKalmanFilter::try_new`];
//! `predict` and `update` only write into the workspace and swap the candidate
//! estimate in once every check has passed, so a failed step leaves the filter
//! unchanged and steady-state operation performs no heap allocation.
//!
//! `P`, `Q` and `R` are kept exactly symmetric. nalgebra stores matrices in
//! column-major order, and an exactly symmetric matrix has the same element
//! sequence in row-major order, which is why
//! [`LinearKalmanFilter::covariance`] can hand out nalgebra's buffer as a
//! row-major slice.
//!
//! Numerics:
//! - predict: `x ← F x`, `P ← F P Fᵀ + Q`.
//! - update: `S = H P Hᵀ + R` is Cholesky-factored with a pivot test relative to
//!   each pre-factorisation diagonal entry of `S`; `Kᵀ = S⁻¹ (H P)` comes from
//!   triangular solves (`S` is never inverted); the covariance uses the Joseph
//!   form `P ← (I − K H) P (I − K H)ᵀ + K R Kᵀ`.
//! - every candidate covariance is checked with
//!   [`psd_to_working_precision`](crate::validation::psd_to_working_precision).
//!
//! The scalar must satisfy nalgebra's [`RealField`] as well as
//! [`KalmanScalar`]. Both `f32` and `f64` do; a third-party `KalmanScalar` that
//! is not a `RealField` cannot use this feature. The default engine has no such
//! requirement.

use core::cmp::Ordering::Greater;

use crate::error::{KalmanError, KalmanResult};
use crate::matrix::{all_finite, rel_tol};
use crate::nalgebra::{Cholesky, DMatrix, DVector, RealField};
use crate::types::KalmanScalar;
use crate::validation::{is_psd, psd_to_working_precision};

use super::{check_dims, check_len, diverged};

/// Linear Kalman filter for `x_{k+1} = F x_k + w`, `z_k = H x_k + v`,
/// `w ~ N(0, Q)`, `v ~ N(0, R)`. All matrices are dense and row-major.
///
/// With the `nalgebra` feature the scalar must implement nalgebra's
/// [`RealField`] as well as [`KalmanScalar`]; `f32` and `f64` do.
#[derive(Debug, Clone)]
pub struct LinearKalmanFilter<T: KalmanScalar + RealField> {
    n: usize,
    m: usize,
    // Estimate.
    x: DVector<T>, // n
    p: DMatrix<T>, // n×n, exactly symmetric
    // Model; Q and R are stored exactly symmetric.
    f: DMatrix<T>, // n×n
    q: DMatrix<T>, // n×n
    h: DMatrix<T>, // m×n
    r: DMatrix<T>, // m×m
    ws: Workspace<T>,
}

/// Scratch buffers, overwritten by every step. `x` and `p` hold the candidate
/// estimate and trade places with the committed one on success.
#[derive(Debug, Clone)]
struct Workspace<T: KalmanScalar + RealField> {
    x: DVector<T>,      // n    candidate state
    p: DMatrix<T>,      // n×n  candidate covariance
    a: DMatrix<T>,      // n×n  I − K H
    kt: DMatrix<T>,     // m×n  H P, then Kᵀ = S⁻¹ H P
    y: DVector<T>,      // m    innovation z − H x
    s: DMatrix<T>,      // m×m  S, then its Cholesky factor
    s_diag: DVector<T>, // m    diagonal of S before factorisation
    work_n: DVector<T>, // n    quadratic-form workspace
    work_m: DVector<T>, // m    quadratic-form workspace
    row_n: DVector<T>,  // n    one row of H, for `kt = H P`
    probe: DMatrix<T>,  // n×n  post-step PSD check
}

impl<T: KalmanScalar + RealField> Workspace<T> {
    /// `n·n`, `m·n` and `m·m` are known not to overflow: they are the lengths of
    /// validated input slices.
    fn new(n: usize, m: usize) -> Self {
        Self {
            x: DVector::zeros(n),
            p: DMatrix::zeros(n, n),
            a: DMatrix::zeros(n, n),
            kt: DMatrix::zeros(m, n),
            y: DVector::zeros(m),
            s: DMatrix::zeros(m, m),
            s_diag: DVector::zeros(m),
            work_n: DVector::zeros(n),
            work_m: DVector::zeros(m),
            row_n: DVector::zeros(n),
            probe: DMatrix::zeros(n, n),
        }
    }
}

impl<T: KalmanScalar + RealField> LinearKalmanFilter<T> {
    /// The one validated constructor; every other construction path ends here.
    /// Slices are row-major: `x0`: n, `p0`: n×n, `f`: n×n, `q`: n×n, `h`: m×n,
    /// `r`: m×m.
    ///
    /// Errors, checked in this order (one cause, one variant):
    /// `ZeroDimension` (n or m is 0), `DimensionMismatch` (wrong slice length,
    /// including an overflowing `rows × cols`), `NonFiniteInput` (NaN/±∞ in
    /// any input), `InvalidCovariance` (p0 not symmetric PSD),
    /// `InvalidNoiseCovariance` (q or r not symmetric PSD).
    /// Singular PSD matrices are accepted; all tolerances are relative to the
    /// matrix's own diagonal, so validity does not depend on units.
    #[allow(clippy::too_many_arguments)]
    pub fn try_new(
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
        check_dims(n, m)?;
        let inputs = [(x0, n, 1), (p0, n, n), (f, n, n), (q, n, n), (h, m, n), (r, m, m)];
        for (data, rows, cols) in inputs {
            check_len(data, rows, cols)?;
        }
        if !inputs.iter().all(|(data, _, _)| all_finite(data)) {
            return Err(KalmanError::NonFiniteInput);
        }

        // Validation borrows workspace buffers as scratch: no extra allocation.
        // `probe` is n×n (p0 and q), `s` is m×m (r).
        let mut ws = Workspace::new(n, m);
        if !is_psd(p0, n, ws.probe.as_mut_slice()) {
            return Err(KalmanError::InvalidCovariance);
        }
        if !is_psd(q, n, ws.probe.as_mut_slice()) || !is_psd(r, m, ws.s.as_mut_slice()) {
            return Err(KalmanError::InvalidNoiseCovariance);
        }

        // Symmetric within tolerance; store exactly symmetric (no arithmetic).
        let mut p = DMatrix::from_row_slice(n, n, p0);
        let mut q = DMatrix::from_row_slice(n, n, q);
        let mut r = DMatrix::from_row_slice(m, m, r);
        p.fill_lower_triangle_with_upper_triangle();
        q.fill_lower_triangle_with_upper_triangle();
        r.fill_lower_triangle_with_upper_triangle();

        Ok(Self {
            n,
            m,
            x: DVector::from_column_slice(x0),
            p,
            f: DMatrix::from_row_slice(n, n, f),
            q,
            h: DMatrix::from_row_slice(m, n, h),
            r,
            ws,
        })
    }

    pub fn state_dim(&self) -> usize {
        self.n
    }

    pub fn measurement_dim(&self) -> usize {
        self.m
    }

    /// State estimate x (length n).
    pub fn state(&self) -> &[T] {
        self.x.as_slice()
    }

    /// Covariance P (n×n, row-major). Exactly symmetric after any step.
    ///
    /// nalgebra stores matrices in column-major order; P is kept exactly
    /// symmetric, so its column-major buffer is also its row-major layout.
    pub fn covariance(&self) -> &[T] {
        self.p.as_slice()
    }

    /// State transition matrix F (n×n), row-major. Built on demand: this is a
    /// cold path used only by the information-form conversion.
    pub(crate) fn transition(&self) -> impl AsRef<[T]> + '_ {
        row_major(&self.f)
    }

    /// Process noise covariance Q (n×n, row-major, exactly symmetric).
    pub(crate) fn process_noise(&self) -> impl AsRef<[T]> + '_ {
        row_major(&self.q)
    }

    /// Observation matrix H (m×n), row-major. Built on demand.
    pub(crate) fn observation(&self) -> impl AsRef<[T]> + '_ {
        row_major(&self.h)
    }

    /// Measurement noise covariance R (m×m, row-major, exactly symmetric).
    pub(crate) fn measurement_noise(&self) -> impl AsRef<[T]> + '_ {
        row_major(&self.r)
    }

    /// `x ← F x`, `P ← F P Fᵀ + Q`. If the result is non-finite or not PSD to
    /// working precision, returns `FilterDivergence` and leaves the filter
    /// unchanged.
    pub fn predict(&mut self) -> KalmanResult<()> {
        let ws = &mut self.ws;
        ws.x.gemv(T::one(), &self.f, &self.x, T::zero());
        ws.p.copy_from(&self.q);
        ws.p.quadform_tr_with_workspace(&mut ws.work_n, T::one(), &self.f, &self.p, T::one());
        self.commit(|| {
            KalmanError::FilterDivergence(
                "predicted covariance lost positive semi-definiteness to rounding".into(),
            )
        })
    }

    /// Incorporate measurement `z` (length m).
    ///
    /// Errors, all leaving the filter unchanged: `DimensionMismatch` (wrong
    /// length), `NonFiniteInput` (NaN/∞ in z), `SingularMatrix` (S not
    /// numerically positive definite relative to its own diagonal, or so
    /// ill-conditioned that the Joseph-form P⁺ is not PSD to working precision),
    /// `FilterDivergence` (non-finite intermediate or result).
    pub fn update(&mut self, z: &[T]) -> KalmanResult<()> {
        let m = self.m;
        if z.len() != m {
            return Err(KalmanError::DimensionMismatch { expected: (m, 1), actual: (z.len(), 1) });
        }
        if !all_finite(z) {
            return Err(KalmanError::NonFiniteInput);
        }
        let ws = &mut self.ws;

        // y = z − H x
        ws.y.copy_from_slice(z);
        ws.y.gemv(-T::one(), &self.h, &self.x, T::one());
        // A non-finite y makes x⁺ = x + K y non-finite: `commit` catches it.

        // H P into kt, one row at a time: row i is (P h_iᵀ)ᵀ because P is
        // symmetric. `gemm` would take nalgebra's matrixmultiply path for large
        // dimensions, which allocates packing buffers; `gemv` does not.
        for i in 0..m {
            for (dst, &src) in ws.row_n.iter_mut().zip(self.h.row(i).iter()) {
                *dst = src;
            }
            ws.work_n.gemv(T::one(), &self.p, &ws.row_n, T::zero());
            for (dst, &src) in ws.kt.row_mut(i).iter_mut().zip(ws.work_n.iter()) {
                *dst = src;
            }
        }
        // S = H P Hᵀ + R
        ws.s.copy_from(&self.r);
        ws.s.quadform_tr_with_workspace(&mut ws.work_m, T::one(), &self.h, &self.p, T::one());

        // Capture S's diagonal before factoring: the pivot test is relative to it.
        for (d, &s_jj) in ws.s_diag.iter_mut().zip(ws.s.as_slice().iter().step_by(m + 1)) {
            *d = s_jj;
        }

        // A non-finite diagonal is reported like the default engine's
        // in-place factorisation does; a non-finite factor becomes a NaN
        // pivot, which the relative test below rejects as singular.
        if !all_finite(ws.s_diag.as_slice()) {
            return Err(diverged("innovation covariance"));
        }

        // S = L Lᵀ in place. `new_unchecked` computes the factor without
        // deciding definiteness; the pivot test below is the decision, and it
        // compares each L_jj² with rel_tol·S_jj (the spec's relative rule).
        let tol = rel_tol::<T>(m);
        let chol = Cholesky::new_unchecked(core::mem::take(&mut ws.s));
        let singular = chol
            .l_dirty()
            .as_slice()
            .iter()
            .step_by(m + 1)
            .zip(ws.s_diag.iter())
            .any(|(&l, &s_jj)| (l * l).partial_cmp(&(tol * s_jj)) != Some(Greater));
        if singular {
            ws.s = chol.unpack_dirty();
            return Err(KalmanError::SingularMatrix);
        }
        // Kᵀ = S⁻¹ (H P) in place: K = P Hᵀ S⁻¹ without inverting S.
        chol.solve_mut(&mut ws.kt);
        ws.s = chol.unpack_dirty();

        // x⁺ = x + K y  (sum over the rows of Kᵀ)
        ws.x.copy_from(&self.x);
        ws.x.gemv_tr(T::one(), &ws.kt, &ws.y, T::one());

        // A = I − K H
        ws.a.fill_with_identity();
        ws.a.gemm_tr(-T::one(), &ws.kt, &self.h, T::one());

        // Joseph form: P⁺ = A P Aᵀ + K R Kᵀ, a sum of PSD terms.
        ws.p.quadform_tr_with_workspace(&mut ws.work_n, T::one(), &ws.a, &self.p, T::zero());
        ws.p.quadform_with_workspace(&mut ws.work_m, T::one(), &self.r, &ws.kt, T::one());

        // When S is so ill-conditioned that rounding in the Joseph sum swamps
        // the posterior, report S as singular rather than commit an invalid
        // covariance (the default engine's mapping).
        self.commit(|| KalmanError::SingularMatrix)
    }

    /// Symmetrises the candidate P, then swaps the candidate estimate in if it
    /// is finite and P is PSD to working precision (SPEC B11/B21). `ws.probe` is
    /// free by now and serves as the check's scratch. `on_inconsistent` is a
    /// closure so no error value is built (or allocated) on success.
    fn commit(&mut self, on_inconsistent: impl FnOnce() -> KalmanError) -> KalmanResult<()> {
        let n = self.n;
        let ws = &mut self.ws;
        // P is exactly symmetric from here on. nalgebra's column-major buffer is
        // therefore also its row-major layout, which is what the PSD check (a
        // row-major kernel) needs.
        ws.p.fill_lower_triangle_with_upper_triangle();
        if !all_finite(ws.x.as_slice()) {
            return Err(diverged("candidate state"));
        }
        if !all_finite(ws.p.as_slice()) {
            return Err(diverged("candidate covariance"));
        }
        if !psd_to_working_precision(ws.p.as_slice(), n, ws.probe.as_mut_slice()) {
            return Err(on_inconsistent());
        }
        core::mem::swap(&mut self.x, &mut ws.x);
        core::mem::swap(&mut self.p, &mut ws.p);
        Ok(())
    }
}

/// The matrix as a row-major `Vec`. Cold path, used only by the crate-private
/// model accessors.
fn row_major<T: KalmanScalar + RealField>(a: &DMatrix<T>) -> Vec<T> {
    let (rows, cols) = a.shape();
    let mut out = Vec::with_capacity(rows * cols);
    for i in 0..rows {
        for j in 0..cols {
            out.push(a[(i, j)]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cv_filter() -> LinearKalmanFilter<f64> {
        LinearKalmanFilter::try_new(
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
    fn f32_and_f64_agree_on_the_baseline_model() {
        let mut kf64 = cv_filter();
        let mut kf32 = LinearKalmanFilter::<f32>::try_new(
            2,
            1,
            &[0.0, 1.0],
            &[1.0, 0.0, 0.0, 1.0],
            &[1.0, 0.1, 0.0, 1.0],
            &[0.01, 0.0, 0.0, 0.01],
            &[1.0, 0.0],
            &[0.1],
        )
        .unwrap();
        for _ in 0..10 {
            kf64.predict().unwrap();
            kf64.update(&[1.0]).unwrap();
            kf32.predict().unwrap();
            kf32.update(&[1.0]).unwrap();
        }
        for (a, b) in kf64.state().iter().zip(kf32.state()) {
            assert!((a - f64::from(*b)).abs() < 1e-4);
        }
    }

    #[test]
    fn asymmetric_input_is_rejected_before_mirroring() {
        let e = LinearKalmanFilter::<f64>::try_new(
            2,
            1,
            &[0.0, 0.0],
            &[1.0, 0.5, 0.0, 1.0],
            &[1.0, 0.0, 0.0, 1.0],
            &[0.0; 4],
            &[1.0, 0.0],
            &[1.0],
        );
        assert!(matches!(e, Err(KalmanError::InvalidCovariance)));
    }
}
