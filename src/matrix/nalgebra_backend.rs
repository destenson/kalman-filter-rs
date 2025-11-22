// nalgebra-based matrix operations
//
// This implementation uses nalgebra's optimized matrix operations.
// nalgebra uses matrixmultiply internally, providing good performance
// without C dependencies (pure Rust). Optional BLAS acceleration available.

use super::{MatrixDecomposition, MatrixOps};
use crate::{KalmanError, KalmanResult, KalmanScalar};
use nalgebra as na;
#[cfg(feature = "nalgebra")]
use nalgebra::{RealField, SMatrix, SVector};

/// nalgebra matrix backend
pub struct NalgebraBackend;

impl<T: KalmanScalar + na::RealField> MatrixOps<T> for NalgebraBackend {
    fn matvec(a: &[T], x: &[T], y: &mut [T], rows: usize, cols: usize) {
        // Convert from row-major to nalgebra's column-major
        let a_mat = na::DMatrix::from_iterator(rows, cols, a.iter().cloned()).transpose();
        let x_vec = na::DVector::from_column_slice(x);
        let y_vec = a_mat * x_vec;

        y.copy_from_slice(y_vec.as_slice());
    }

    fn matmul(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        // Convert from row-major to nalgebra's column-major
        let a_mat = na::DMatrix::from_iterator(m, k, a.iter().cloned()).transpose();
        let b_mat = na::DMatrix::from_iterator(k, n, b.iter().cloned()).transpose();
        let c_mat = a_mat * b_mat;

        // Convert back to row-major
        for i in 0..m {
            for j in 0..n {
                c[i * n + j] = c_mat[(i, j)];
            }
        }
    }

    fn matmul_transpose(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        // Convert from row-major to nalgebra's column-major
        let a_mat = na::DMatrix::from_iterator(m, k, a.iter().cloned()).transpose();
        let b_mat = na::DMatrix::from_iterator(n, k, b.iter().cloned()).transpose();
        let c_mat = a_mat * b_mat.transpose();

        // Convert back to row-major
        for i in 0..m {
            for j in 0..n {
                c[i * n + j] = c_mat[(i, j)];
            }
        }
    }

    fn transpose(a: &[T], b: &mut [T], rows: usize, cols: usize) {
        // Simple transpose without nalgebra (faster for this operation)
        for i in 0..rows {
            for j in 0..cols {
                b[j * rows + i] = a[i * cols + j];
            }
        }
    }

    fn add_inplace(a: &mut [T], b: &[T], size: usize) {
        for i in 0..size {
            a[i] += b[i];
        }
    }

    fn scale_inplace(a: &mut [T], alpha: T, size: usize) {
        for i in 0..size {
            a[i] *= alpha;
        }
    }

    fn symmetrize(a: &mut [T], n: usize) {
        let a_mat = na::DMatrix::from_row_slice(n, n, a);
        let half: T = na::convert(0.5);
        let sym = (a_mat.clone() + a_mat.transpose()) * half;

        a.copy_from_slice(sym.as_slice());
    }
}

impl<T: KalmanScalar + na::RealField> MatrixDecomposition<T> for NalgebraBackend {
    fn cholesky(p: &[T], l: &mut [T], n: usize) -> KalmanResult<()> {
        // Convert from row-major to nalgebra's column-major
        let p_mat = na::DMatrix::from_iterator(n, n, p.iter().cloned()).transpose();

        match p_mat.cholesky() {
            Some(chol) => {
                let l_mat = chol.l();
                // Convert back to row-major
                for i in 0..n {
                    for j in 0..n {
                        l[i * n + j] = l_mat[(i, j)];
                    }
                }
                Ok(())
            }
            None => Err(KalmanError::SingularMatrix),
        }
    }

    fn invert(a: &[T], a_inv: &mut [T], n: usize) -> KalmanResult<()> {
        // Convert from row-major to nalgebra's column-major
        let a_mat = na::DMatrix::from_iterator(n, n, a.iter().cloned()).transpose();

        match a_mat.try_inverse() {
            Some(inv) => {
                // Convert back to row-major
                for i in 0..n {
                    for j in 0..n {
                        a_inv[i * n + j] = inv[(i, j)];
                    }
                }
                Ok(())
            }
            None => Err(KalmanError::SingularMatrix),
        }
    }

    fn is_positive_definite(a: &[T], n: usize) -> bool {
        // Convert from row-major to nalgebra's column-major
        let a_mat = na::DMatrix::from_iterator(n, n, a.iter().cloned()).transpose();
        a_mat.cholesky().is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_matvec() {
        let a = vec![1.0_f64, 2.0, 3.0, 4.0];
        let x = vec![5.0, 7.0];
        let mut y = vec![0.0; 2];

        NalgebraBackend::matvec(&a, &x, &mut y, 2, 2);

        assert_relative_eq!(y[0], 19.0, epsilon = 1e-10);
        assert_relative_eq!(y[1], 43.0, epsilon = 1e-10);
    }

    #[test]
    fn test_matmul() {
        let a = vec![1.0_f64, 2.0, 3.0, 4.0];
        let b = vec![5.0, 6.0, 7.0, 8.0];
        let mut c = vec![0.0; 4];

        NalgebraBackend::matmul(&a, &b, &mut c, 2, 2, 2);

        assert_relative_eq!(c[0], 19.0, epsilon = 1e-10);
        assert_relative_eq!(c[1], 22.0, epsilon = 1e-10);
        assert_relative_eq!(c[2], 43.0, epsilon = 1e-10);
        assert_relative_eq!(c[3], 50.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cholesky() {
        let p = vec![
            4.0_f64, 12.0, -16.0,
            12.0, 37.0, -43.0,
            -16.0, -43.0, 98.0
        ];
        let mut l = vec![0.0; 9];

        let result = NalgebraBackend::cholesky(&p, &mut l, 3);
        assert!(result.is_ok());

        assert_relative_eq!(l[0], 2.0, epsilon = 1e-10);
        assert_relative_eq!(l[3], 6.0, epsilon = 1e-10);
        assert_relative_eq!(l[4], 1.0, epsilon = 1e-10);
        assert_relative_eq!(l[6], -8.0, epsilon = 1e-10);
        assert_relative_eq!(l[7], 5.0, epsilon = 1e-10);
        assert_relative_eq!(l[8], 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_invert() {
        let a = vec![4.0_f64, 7.0, 2.0, 6.0];
        let mut a_inv = vec![0.0; 4];

        let result = NalgebraBackend::invert(&a, &mut a_inv, 2);
        assert!(result.is_ok());

        assert_relative_eq!(a_inv[0], 0.6, epsilon = 1e-10);
        assert_relative_eq!(a_inv[1], -0.7, epsilon = 1e-10);
        assert_relative_eq!(a_inv[2], -0.2, epsilon = 1e-10);
        assert_relative_eq!(a_inv[3], 0.4, epsilon = 1e-10);
    }
}
