//! Compatibility shim for the modules not yet rebuilt on [`crate::linear`] (EKF, UKF, CKF,
//! information filters). It keeps their row-major `Vec<T>` call sites compiling by
//! delegating to nalgebra. New code should use nalgebra types directly; this module goes away
//! once those filters are rebuilt.
//!
//! `KalmanScalar` is `num_traits::Float`, not `nalgebra::RealField` (making it both would make
//! every `x.sqrt()` in the nonlinear filters ambiguous), so values round-trip through `f64`.
//! That is exact for `f32` and `f64`. Every function returns an error instead of panicking on
//! wrong slice lengths.

use nalgebra::DMatrix;

use crate::error::{KalmanError, KalmanResult};
use crate::types::KalmanScalar;

fn load<T: KalmanScalar>(a: &[T], rows: usize, cols: usize) -> KalmanResult<DMatrix<f64>> {
    match rows.checked_mul(cols) {
        Some(len) if len == a.len() => Ok(DMatrix::from_row_iterator(rows, cols, a.iter().map(KalmanScalar::to_f64))),
        _ => Err(KalmanError::DimensionMismatch { expected: (rows, cols), actual: (a.len(), 1) }),
    }
}

fn store<T: KalmanScalar>(m: &DMatrix<f64>, out: &mut [T]) -> KalmanResult<()> {
    if out.len() != m.len() {
        return Err(KalmanError::DimensionMismatch { expected: m.shape(), actual: (out.len(), 1) });
    }
    // Row-major output from column-major storage.
    for (o, v) in out.iter_mut().zip(m.transpose().iter()) {
        *o = T::from_f64(*v);
    }
    Ok(())
}

/// Inverse of a row-major `n×n` matrix (LU with partial pivoting).
/// `SingularMatrix` if nalgebra finds it singular or the inverse is not finite.
pub fn invert_matrix<T: KalmanScalar>(a: &[T], n: usize) -> KalmanResult<Vec<T>> {
    let inv = load(a, n, n)?.try_inverse().ok_or(KalmanError::SingularMatrix)?;
    if inv.iter().any(|v| !v.is_finite()) {
        return Err(KalmanError::SingularMatrix);
    }
    let mut out = vec![T::zero(); n * n];
    store(&inv, &mut out)?;
    Ok(out)
}

/// Row-major slice operations used by `extended.rs`. Length errors leave outputs untouched.
pub struct DefaultBackend;

impl DefaultBackend {
    /// `y = A x`, A is `rows×cols`.
    pub fn matvec<T: KalmanScalar>(a: &[T], x: &[T], y: &mut [T], rows: usize, cols: usize) {
        if let (Ok(a), Ok(x)) = (load(a, rows, cols), load(x, cols, 1)) {
            let _ = store(&(a * x), y);
        }
    }

    /// `C = A B`, A is `m×k`, B is `k×n`.
    pub fn matmul<T: KalmanScalar>(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        if let (Ok(a), Ok(b)) = (load(a, m, k), load(b, k, n)) {
            let _ = store(&(a * b), c);
        }
    }

    /// `C = A Bᵀ`, A is `m×k`, B is `n×k`.
    pub fn matmul_transpose<T: KalmanScalar>(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        if let (Ok(a), Ok(b)) = (load(a, m, k), load(b, n, k)) {
            let _ = store(&(a * b.transpose()), c);
        }
    }

    /// `a += b` over the first `size` elements.
    pub fn add_inplace<T: KalmanScalar>(a: &mut [T], b: &[T], size: usize) {
        for (x, y) in a.iter_mut().zip(b).take(size) {
            *x = *x + *y;
        }
    }

    /// `A ← (A + Aᵀ)/2` for a row-major `n×n` matrix.
    pub fn symmetrize<T: KalmanScalar>(a: &mut [T], n: usize) {
        if let Ok(m) = load(a, n, n) {
            let _ = store(&((&m + m.transpose()) * 0.5), a);
        }
    }

    /// Inverse of a row-major `n×n` matrix into `a_inv`.
    pub fn invert<T: KalmanScalar>(a: &[T], a_inv: &mut [T], n: usize) -> KalmanResult<()> {
        let inv = invert_matrix(a, n)?;
        if a_inv.len() != inv.len() {
            return Err(KalmanError::DimensionMismatch { expected: (n, n), actual: (a_inv.len(), 1) });
        }
        a_inv.copy_from_slice(&inv);
        Ok(())
    }
}
