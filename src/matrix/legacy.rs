//! Transitional slice backend for the nonlinear filters (EKF/UKF/CKF/IF).
//!
//! These are the pre-rewrite operations, trimmed to what the out-of-scope filters
//! still call, and made crate-private so they are no longer public API. The linear
//! filter does not use them. They keep the old behaviour on purpose: `invert` uses an
//! *absolute* pivot threshold and nothing checks slice lengths. Delete this file once
//! the nonlinear filters are rebuilt on `matrix::{mul, cholesky, ...}`.

use crate::{KalmanError, KalmanResult, KalmanScalar};

/// The only backend.
pub(crate) type DefaultBackend = NaiveBackend;

/// Hand-rolled row-major loops.
pub(crate) struct NaiveBackend;

/// Products and element-wise operations on row-major slices.
pub(crate) trait MatrixOps<T: KalmanScalar> {
    /// y = A x (A: rows × cols)
    fn matvec(a: &[T], x: &[T], y: &mut [T], rows: usize, cols: usize);
    /// C = A B (A: m × k, B: k × n)
    fn matmul(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize);
    /// C = A Bᵀ (A: m × k, B: n × k)
    fn matmul_transpose(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize);
    /// A += B
    fn add_inplace(a: &mut [T], b: &[T], size: usize);
    /// A = (A + Aᵀ) / 2
    fn symmetrize(a: &mut [T], n: usize);
}

/// Inversion.
pub(crate) trait MatrixDecomposition<T: KalmanScalar> {
    /// Gauss–Jordan inverse with partial pivoting.
    fn invert(a: &[T], a_inv: &mut [T], n: usize) -> KalmanResult<()>;
}

impl<T: KalmanScalar> MatrixOps<T> for NaiveBackend {
    fn matvec(a: &[T], x: &[T], y: &mut [T], rows: usize, cols: usize) {
        super::mul_vec(a, x, y, rows, cols);
    }

    fn matmul(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        super::mul(a, b, c, m, k, n);
    }

    fn matmul_transpose(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        super::mul_bt(a, b, c, m, k, n);
    }

    fn add_inplace(a: &mut [T], b: &[T], size: usize) {
        for (ai, &bi) in a[..size].iter_mut().zip(&b[..size]) {
            *ai = *ai + bi;
        }
    }

    fn symmetrize(a: &mut [T], n: usize) {
        super::symmetrize(a, n);
    }
}

impl<T: KalmanScalar> MatrixDecomposition<T> for NaiveBackend {
    fn invert(a: &[T], a_inv: &mut [T], n: usize) -> KalmanResult<()> {
        let w = 2 * n;
        let mut aug = vec![T::zero(); n * w];
        for i in 0..n {
            aug[i * w..i * w + n].copy_from_slice(&a[i * n..i * n + n]);
            aug[i * w + n + i] = T::one();
        }
        for i in 0..n {
            let pivot_row = (i..n)
                .max_by(|&p, &q| {
                    let (vp, vq) = (aug[p * w + i].abs(), aug[q * w + i].abs());
                    vp.partial_cmp(&vq).unwrap_or(core::cmp::Ordering::Equal)
                })
                .unwrap_or(i);
            if aug[pivot_row * w + i].abs() < <T as KalmanScalar>::epsilon() {
                return Err(KalmanError::SingularMatrix);
            }
            for j in 0..w {
                aug.swap(i * w + j, pivot_row * w + j);
            }
            let pivot = aug[i * w + i];
            for j in 0..w {
                aug[i * w + j] = aug[i * w + j] / pivot;
            }
            for r in (0..n).filter(|&r| r != i) {
                let factor = aug[r * w + i];
                for j in 0..w {
                    aug[r * w + j] = aug[r * w + j] - factor * aug[i * w + j];
                }
            }
        }
        for i in 0..n {
            a_inv[i * n..i * n + n].copy_from_slice(&aug[i * w + n..i * w + w]);
        }
        Ok(())
    }
}

/// Allocating inverse, the former `KalmanFilter::invert_matrix`, for the nonlinear
/// filters. Returns `DimensionMismatch` instead of panicking on a short slice.
pub(crate) fn invert<T: KalmanScalar>(a: &[T], n: usize) -> KalmanResult<Vec<T>> {
    if n.checked_mul(n) != Some(a.len()) {
        return Err(KalmanError::DimensionMismatch { expected: (n, n), actual: (a.len(), 1) });
    }
    let mut out = vec![T::zero(); a.len()];
    NaiveBackend::invert(a, &mut out, n)?;
    Ok(out)
}
