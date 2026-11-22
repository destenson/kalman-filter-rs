//! The linear Kalman filter, written directly against nalgebra.
//!
//! One type, [`LinearKalmanFilter<T, N, M>`], serves every dimension kind: `N` and `M` are
//! nalgebra dimensions, so `Dyn` (the default, sized at run time) and `Const<K>` (sized at
//! compile time, stack storage) run the same code. All matrices are nalgebra `OMatrix`
//! values; every temporary lives in a preallocated [`Workspace`], so `predict` and `update`
//! never allocate.
//!
//! Numerics:
//! - predict: `x ← F x`, `P ← F P Fᵀ + Q`
//! - update: `S = H P Hᵀ + R` is Cholesky-factored (never inverted) and declared singular when
//!   a pivot is small *relative to its own diagonal entry*; `Kᵀ = S⁻¹ H P` by two triangular
//!   solves; `P ← (I − K H) P (I − K H)ᵀ + K R Kᵀ` (Joseph form).
//! - Results are committed only if finite; otherwise the filter is left unchanged.
//!
//! ```
//! use kalman_filters::linear::LinearKalmanFilter;
//! // 1-D random walk observed directly (row-major slices).
//! let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.01], &[1.0], &[0.1])?;
//! kf.predict()?;
//! kf.update(&[0.5])?;
//! assert!(kf.state()[0] > 0.0);
//! # Ok::<(), kalman_filters::KalmanError>(())
//! ```
//!
//! Compile-time dimensions use the same constructor:
//!
//! ```
//! use kalman_filters::linear::LinearKalmanFilter;
//! use nalgebra::{U1, U2};
//! let kf = LinearKalmanFilter::<f32, U2, U1>::new(
//!     2, 1, &[0.0, 1.0], &[1.0, 0.0, 0.0, 1.0], &[1.0, 0.1, 0.0, 1.0],
//!     &[0.01, 0.0, 0.0, 0.01], &[1.0, 0.0], &[0.1],
//! )?;
//! assert_eq!(kf.state(), &[0.0, 1.0]);
//! # Ok::<(), kalman_filters::KalmanError>(())
//! ```

use core::cmp::Ordering::Greater;
use core::mem::swap;

use nalgebra::allocator::Allocator;
use nalgebra::{Cholesky, DefaultAllocator, Dim, Dyn, OMatrix, OVector, RealField};

use crate::error::{KalmanError, KalmanResult};

/// The allocator capabilities a filter with state dimension `N` and measurement dimension `M`
/// needs. It only names the five nalgebra bounds once; it is implemented for
/// `DefaultAllocator` whenever they hold, which is always for `Dyn` and `Const<K>`.
pub trait FilterAllocator<N: Dim, M: Dim>:
    Allocator<N, N> + Allocator<N> + Allocator<M, N> + Allocator<M, M> + Allocator<M>
{
}

impl<N: Dim, M: Dim> FilterAllocator<N, M> for DefaultAllocator where
    DefaultAllocator: Allocator<N, N> + Allocator<N> + Allocator<M, N> + Allocator<M, M> + Allocator<M>
{
}

/// Relative tolerance used by every numerical test in this module: `100·ε` of `T`.
///
/// Pivots of `S`, asymmetry and negative eigenvalues are compared against this times the
/// scale of the matrix being tested, never against an absolute threshold.
fn rel_tol<T: RealField + Copy>() -> T {
    T::default_epsilon() * nalgebra::convert::<f64, T>(100.0)
}

/// Linear Kalman filter over scalar `T` (`f32` or `f64`), state dimension `N`, measurement
/// dimension `M`.
///
/// Invariants (established by [`new`](Self::new), preserved by `predict`/`update`):
/// `P`, `Q` and `R` are exactly symmetric and PSD within [`rel_tol`]; `x` and `P` are finite.
#[derive(Clone, Debug)]
pub struct LinearKalmanFilter<T: RealField + Copy, N: Dim = Dyn, M: Dim = Dyn>
where
    DefaultAllocator: FilterAllocator<N, M>,
{
    x: OVector<T, N>,
    p: OMatrix<T, N, N>,
    f: OMatrix<T, N, N>,
    q: OMatrix<T, N, N>,
    h: OMatrix<T, M, N>,
    r: OMatrix<T, M, M>,
    ws: Workspace<T, N, M>,
}

/// Scratch storage, sized once at construction. Candidate results are built here and swapped
/// into the filter only once they are known to be finite.
#[derive(Clone, Debug)]
struct Workspace<T: RealField + Copy, N: Dim, M: Dim>
where
    DefaultAllocator: FilterAllocator<N, M>,
{
    x: OVector<T, N>,
    p: OMatrix<T, N, N>,
    /// `I − K H`.
    a: OMatrix<T, N, N>,
    /// `Kᵀ` (computed in place from `H P`).
    kt: OMatrix<T, M, N>,
    /// Innovation `y = z − H x`.
    y: OVector<T, M>,
    /// Diagonal of `S` before factoring, for the relative pivot test.
    s_diag: OVector<T, M>,
    /// Storage for `S` and its Cholesky factor. `Cholesky` takes its matrix by value, so the
    /// buffer is moved out and back each update; it is `None` only during that move.
    s: Option<OMatrix<T, M, M>>,
    work_n: OVector<T, N>,
    work_m: OVector<T, M>,
}

impl<T: RealField + Copy, N: Dim, M: Dim> LinearKalmanFilter<T, N, M>
where
    DefaultAllocator: FilterAllocator<N, M>,
{
    /// Builds a validated filter. All matrices are dense row-major slices:
    /// `x0`: n, `p0`: n×n, `f`: n×n, `q`: n×n, `h`: m×n, `r`: m×m.
    ///
    /// Errors: `ZeroDimension`, `DimensionMismatch` (wrong length, overflowing `n·n`/`m·n`/`m·m`,
    /// or `n`/`m` differing from a compile-time `N`/`M`), `NonFiniteInput`,
    /// `InvalidCovariance` (`p0` not symmetric PSD), `InvalidNoiseCovariance` (`q` or `r` not
    /// symmetric PSD). Singular PSD matrices are accepted.
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
        let n: N = dim(state_dim)?;
        let m: M = dim(measurement_dim)?;
        let (nu, mu) = (state_dim, measurement_dim);
        for (data, rows, cols) in [(x0, nu, 1), (p0, nu, nu), (f, nu, nu), (q, nu, nu), (h, mu, nu), (r, mu, mu)] {
            check_len(data, rows, cols)?;
        }
        if [x0, p0, f, q, h, r].iter().any(|s| s.iter().any(|v| !v.is_finite())) {
            return Err(KalmanError::NonFiniteInput);
        }

        let p = symmetric_psd(n, p0).ok_or(KalmanError::InvalidCovariance)?;
        let q = symmetric_psd(n, q).ok_or(KalmanError::InvalidNoiseCovariance)?;
        let r = symmetric_psd(m, r).ok_or(KalmanError::InvalidNoiseCovariance)?;

        Ok(Self {
            x: OVector::from_row_slice_generic(n, nalgebra::U1, x0),
            p,
            f: OMatrix::from_row_slice_generic(n, n, f),
            q,
            h: OMatrix::from_row_slice_generic(m, n, h),
            r,
            ws: Workspace {
                x: OVector::zeros_generic(n, nalgebra::U1),
                p: OMatrix::zeros_generic(n, n),
                a: OMatrix::zeros_generic(n, n),
                kt: OMatrix::zeros_generic(m, n),
                y: OVector::zeros_generic(m, nalgebra::U1),
                s_diag: OVector::zeros_generic(m, nalgebra::U1),
                s: Some(OMatrix::zeros_generic(m, m)),
                work_n: OVector::zeros_generic(n, nalgebra::U1),
                work_m: OVector::zeros_generic(m, nalgebra::U1),
            },
        })
    }

    /// State dimension n.
    pub fn state_dim(&self) -> usize {
        self.x.nrows()
    }

    /// Measurement dimension m.
    pub fn measurement_dim(&self) -> usize {
        self.h.nrows()
    }

    /// State estimate, length n.
    pub fn state(&self) -> &[T] {
        self.x.as_slice()
    }

    /// Covariance, n×n row-major. (`P` is kept exactly symmetric, so nalgebra's column-major
    /// storage is also its row-major layout.)
    pub fn covariance(&self) -> &[T] {
        self.p.as_slice()
    }

    /// Model matrices `(F, Q, H, R)` (used to convert to the information form).
    #[allow(clippy::type_complexity)]
    pub fn model(&self) -> (&OMatrix<T, N, N>, &OMatrix<T, N, N>, &OMatrix<T, M, N>, &OMatrix<T, M, M>) {
        (&self.f, &self.q, &self.h, &self.r)
    }

    /// Time update: `x ← F x`, `P ← F P Fᵀ + Q`.
    ///
    /// Returns `FilterDivergence` and leaves the filter unchanged if the result is not finite.
    pub fn predict(&mut self) -> KalmanResult<()> {
        let ws = &mut self.ws;
        ws.x.gemv(T::one(), &self.f, &self.x, T::zero());
        ws.p.copy_from(&self.q);
        ws.p.quadform_tr_with_workspace(&mut ws.work_n, T::one(), &self.f, &self.p, T::one());
        self.commit()
    }

    /// Measurement update with `z` (length m).
    ///
    /// Errors, all leaving the filter unchanged: `DimensionMismatch` (wrong length),
    /// `NonFiniteInput` (NaN/∞ in `z`), `SingularMatrix` (`S` not numerically positive
    /// definite), `FilterDivergence` (non-finite result).
    pub fn update(&mut self, z: &[T]) -> KalmanResult<()> {
        let m = self.measurement_dim();
        if z.len() != m {
            return Err(KalmanError::DimensionMismatch { expected: (m, 1), actual: (z.len(), 1) });
        }
        if z.iter().any(|v| !v.is_finite()) {
            return Err(KalmanError::NonFiniteInput);
        }
        let ws = &mut self.ws;

        // y = z − H x
        ws.y.copy_from_slice(z);
        ws.y.gemv(-T::one(), &self.h, &self.x, T::one());

        // S = H P Hᵀ + R, factored in place.
        let mut s = ws.s.take().unwrap_or_else(|| OMatrix::zeros_generic(self.r.shape_generic().0, self.r.shape_generic().1));
        s.copy_from(&self.r);
        s.quadform_tr_with_workspace(&mut ws.work_m, T::one(), &self.h, &self.p, T::one());
        for (d, &v) in ws.s_diag.iter_mut().zip(diagonal(&s, m)) {
            *d = v;
        }
        let chol = Cholesky::new_unchecked(s);
        // Pivot k of the factorization is L_kk². It must be a non-negligible fraction of S_kk;
        // this test is invariant to rescaling any measurement component (B10, B21, B22).
        let tol = rel_tol::<T>();
        let singular = diagonal(chol.l_dirty(), m)
            .zip(ws.s_diag.iter())
            .any(|(&l, &skk)| (l * l).partial_cmp(&(tol * skk)) != Some(Greater));
        if singular {
            ws.s = Some(chol.unpack_dirty());
            return Err(KalmanError::SingularMatrix);
        }

        // Kᵀ = S⁻¹ H P  (P symmetric, so this is (P Hᵀ S⁻¹)ᵀ)
        ws.kt.gemm(T::one(), &self.h, &self.p, T::zero());
        chol.solve_mut(&mut ws.kt);
        ws.s = Some(chol.unpack_dirty());

        // x⁺ = x + K y
        ws.x.copy_from(&self.x);
        ws.x.gemv_tr(T::one(), &ws.kt, &ws.y, T::one());

        // Joseph form: P⁺ = (I − K H) P (I − K H)ᵀ + K R Kᵀ
        ws.a.fill_with_identity();
        ws.a.gemm_tr(-T::one(), &ws.kt, &self.h, T::one());
        ws.p.quadform_tr_with_workspace(&mut ws.work_n, T::one(), &ws.a, &self.p, T::zero());
        ws.p.quadform_with_workspace(&mut ws.work_m, T::one(), &self.r, &ws.kt, T::one());
        self.commit()
    }

    /// Symmetrizes the candidate in `ws`, then swaps it in if finite.
    fn commit(&mut self) -> KalmanResult<()> {
        let ws = &mut self.ws;
        ws.p.fill_upper_triangle_with_lower_triangle();
        if !(all_finite(ws.x.as_slice()) && all_finite(ws.p.as_slice())) {
            return Err(KalmanError::FilterDivergence("non-finite estimate".into()));
        }
        swap(&mut self.x, &mut ws.x);
        swap(&mut self.p, &mut ws.p);
        Ok(())
    }
}

/// Converts a run-time dimension to `D`, rejecting zero and mismatches with a static `D`.
fn dim<D: Dim>(value: usize) -> KalmanResult<D> {
    if value == 0 {
        return Err(KalmanError::ZeroDimension);
    }
    match D::try_to_usize() {
        Some(fixed) if fixed != value => {
            Err(KalmanError::DimensionMismatch { expected: (fixed, 1), actual: (value, 1) })
        }
        _ => Ok(D::from_usize(value)),
    }
}

/// `data.len() == rows·cols`, with the product overflow-checked.
fn check_len<T>(data: &[T], rows: usize, cols: usize) -> KalmanResult<()> {
    match rows.checked_mul(cols) {
        Some(len) if len == data.len() => Ok(()),
        _ => Err(KalmanError::DimensionMismatch { expected: (rows, cols), actual: (data.len(), 1) }),
    }
}

fn all_finite<T: RealField + Copy>(data: &[T]) -> bool {
    data.iter().all(|v| v.is_finite())
}

/// Diagonal of a square, contiguous column-major matrix of size `m`.
fn diagonal<T, D: Dim>(a: &OMatrix<T, D, D>, m: usize) -> impl Iterator<Item = &T>
where
    T: RealField + Copy,
    DefaultAllocator: Allocator<D, D>,
{
    a.as_slice().iter().step_by(m + 1)
}

/// Parses a row-major `d×d` matrix and returns its symmetric part if it is symmetric PSD
/// within [`rel_tol`] of its own scale; `None` otherwise.
///
/// PSD test: a nonzero PSD matrix has positive trace, and `A + τ·tr(A)·I` is positive
/// definite iff `λ_min(A) > −τ·tr(A)`; nalgebra's Cholesky decides the latter.
fn symmetric_psd<T, D>(d: D, row_major: &[T]) -> Option<OMatrix<T, D, D>>
where
    T: RealField + Copy,
    D: Dim,
    DefaultAllocator: Allocator<D, D>,
{
    let mut a = OMatrix::from_row_slice_generic(d, d, row_major);
    let tol = rel_tol::<T>();
    let scale = a.amax();
    if (&a - a.transpose()).amax() > tol * scale {
        return None;
    }
    // Exact symmetry without arithmetic: symmetric input is returned bit-for-bit.
    a.fill_upper_triangle_with_lower_triangle();
    if scale == T::zero() {
        return Some(a);
    }
    let trace = a.trace();
    if trace.partial_cmp(&T::zero()) != Some(Greater) {
        return None;
    }
    let shifted = &a + OMatrix::identity_generic(d, d) * (tol * trace);
    Cholesky::new(shifted).map(|_| a)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{U1, U2};

    type Kf = LinearKalmanFilter<f64>;

    fn cv() -> Kf {
        Kf::new(2, 1, &[0.0, 1.0], &[1.0, 0.0, 0.0, 1.0], &[1.0, 0.1, 0.0, 1.0], &[0.01, 0.0, 0.0, 0.01], &[1.0, 0.0], &[0.1])
            .unwrap()
    }

    #[test]
    fn static_and_dynamic_dimensions_agree() {
        let mut dynamic = cv();
        let mut fixed = LinearKalmanFilter::<f64, U2, U1>::new(
            2, 1, &[0.0, 1.0], &[1.0, 0.0, 0.0, 1.0], &[1.0, 0.1, 0.0, 1.0], &[0.01, 0.0, 0.0, 0.01], &[1.0, 0.0], &[0.1],
        )
        .unwrap();
        for k in 0..20 {
            let z = [0.1 * k as f64];
            dynamic.predict().unwrap();
            fixed.predict().unwrap();
            dynamic.update(&z).unwrap();
            fixed.update(&z).unwrap();
        }
        assert_eq!(dynamic.state(), fixed.state());
        assert_eq!(dynamic.covariance(), fixed.covariance());
    }

    #[test]
    fn static_dimension_mismatch_is_an_error() {
        let r = LinearKalmanFilter::<f64, U2, U1>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[1.0]);
        assert!(matches!(r, Err(KalmanError::DimensionMismatch { .. })));
    }

    #[test]
    fn joseph_form_keeps_psd_where_simple_form_fails() {
        // Model B from SPEC.md in f32: the simple form (I−KH)P goes indefinite at step 0.
        let d = 1e-2f32;
        let mut kf = LinearKalmanFilter::<f32>::new(
            2, 2, &[0.0, 0.0], &[1.0, 0.0, 0.0, 1.0], &[1.0, 0.0, 0.0, 1.0], &[0.0; 4],
            &[1.0, 1.0, 1.0, 1.0 + d], &[d * d, 0.0, 0.0, d * d],
        )
        .unwrap();
        for _ in 0..1000 {
            kf.predict().unwrap();
            kf.update(&[0.05, 0.03]).unwrap();
            let p = nalgebra::DMatrix::from_iterator(2, 2, kf.covariance().iter().map(|&v| v as f64));
            let eig = p.symmetric_eigenvalues();
            assert!(eig.min() >= -1e-5 * eig.max(), "{eig}");
        }
    }

    #[test]
    fn filter_recovers_after_rejected_steps() {
        let mut kf = Kf::new(1, 1, &[1.0], &[1e200], &[1e200], &[0.0], &[1.0], &[1.0]).unwrap();
        assert!(kf.predict().is_err());
        assert_eq!(kf.covariance(), &[1e200]);
        // Scratch may hold non-finite values now; a valid update must still be exact.
        kf.update(&[1.0]).unwrap();
        assert!(kf.covariance()[0] <= 1.0 && kf.covariance()[0] > 0.99);
    }

    #[test]
    fn singular_update_then_valid_update_works() {
        let i2 = [1.0, 0.0, 0.0, 1.0];
        let mut kf = Kf::new(2, 2, &[0.0, 0.0], &[0.0; 4], &i2, &[1.0, 0.0, 0.0, 1.0], &i2, &[1.0, 1.0, 1.0, 1.0]).unwrap();
        assert!(matches!(kf.update(&[1.0, 2.0]), Err(KalmanError::SingularMatrix)));
        kf.predict().unwrap();
        kf.update(&[1.0, 2.0]).unwrap();
    }

    #[test]
    fn psd_validation_is_relative() {
        let d = Dyn(2);
        assert!(symmetric_psd(d, &[1e-30, 0.0, 0.0, 1e-30]).is_some());
        assert!(symmetric_psd(d, &[1e30, 1e30, 1e30, 1e30]).is_some());
        assert!(symmetric_psd(d, &[0.0; 4]).is_some());
        assert!(symmetric_psd(d, &[1e-30, 2e-30, 2e-30, 1e-30]).is_none());
        assert!(symmetric_psd(d, &[0.0, 1.0, 1.0, 0.0]).is_none());
        assert!(symmetric_psd(d, &[-1.0, 0.0, 0.0, 1.0]).is_none());
    }
}
