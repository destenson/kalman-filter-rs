// Naive hand-rolled matrix operations
//
// This implementation uses simple triple-nested loops for matrix operations.
// It's portable (no_std compatible) but not optimized for performance.
// Use nalgebra or BLAS backends for better performance.

use super::{MatrixDecomposition, MatrixOps};
use crate::{KalmanError, KalmanResult, KalmanScalar};

/// Naive matrix backend using hand-rolled loops
pub struct NaiveBackend;

impl<T: KalmanScalar> MatrixOps<T> for NaiveBackend {
    fn matvec(a: &[T], x: &[T], y: &mut [T], rows: usize, cols: usize) {
        for i in 0..rows {
            y[i] = T::zero();
            for j in 0..cols {
                y[i] = y[i] + a[i * cols + j] * x[j];
            }
        }
    }

    fn matmul(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        // C = A * B
        // A is m × k, B is k × n, C is m × n
        for i in 0..m {
            for j in 0..n {
                c[i * n + j] = T::zero();
                for p in 0..k {
                    c[i * n + j] = c[i * n + j] + a[i * k + p] * b[p * n + j];
                }
            }
        }
    }

    fn matmul_transpose(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize) {
        // C = A * B^T
        // A is m × k, B is n × k (stored row-major), C is m × n
        for i in 0..m {
            for j in 0..n {
                c[i * n + j] = T::zero();
                for p in 0..k {
                    c[i * n + j] = c[i * n + j] + a[i * k + p] * b[j * k + p];
                }
            }
        }
    }

    fn transpose(a: &[T], b: &mut [T], rows: usize, cols: usize) {
        for i in 0..rows {
            for j in 0..cols {
                b[j * rows + i] = a[i * cols + j];
            }
        }
    }

    fn add_inplace(a: &mut [T], b: &[T], size: usize) {
        for i in 0..size {
            a[i] = a[i] + b[i];
        }
    }

    fn scale_inplace(a: &mut [T], alpha: T, size: usize) {
        for i in 0..size {
            a[i] = a[i] * alpha;
        }
    }

    fn symmetrize(a: &mut [T], n: usize) {
        let half = T::from_f64(0.5);
        for i in 0..n {
            for j in 0..i {
                let avg = (a[i * n + j] + a[j * n + i]) * half;
                a[i * n + j] = avg;
                a[j * n + i] = avg;
            }
        }
    }
}

impl<T: KalmanScalar> MatrixDecomposition<T> for NaiveBackend {
    fn cholesky(p: &[T], l: &mut [T], n: usize) -> KalmanResult<()> {
        // Cholesky-Banachiewicz algorithm
        // P = L * L^T where L is lower triangular

        // Initialize L to zero
        for i in 0..n * n {
            l[i] = T::zero();
        }

        for i in 0..n {
            for j in 0..=i {
                let mut sum = p[i * n + j];

                for k in 0..j {
                    sum = sum - l[i * n + k] * l[j * n + k];
                }

                if i == j {
                    // Diagonal element
                    if sum <= T::zero() {
                        return Err(KalmanError::SingularMatrix);
                    }
                    l[i * n + j] = sum.sqrt();
                } else {
                    // Off-diagonal element
                    let diag = l[j * n + j];
                    if diag.abs() < KalmanScalar::epsilon() {
                        return Err(KalmanError::SingularMatrix);
                    }
                    l[i * n + j] = sum / diag;
                }
            }
        }

        Ok(())
    }

    fn invert(a: &[T], a_inv: &mut [T], n: usize) -> KalmanResult<()> {
        // Gauss-Jordan elimination
        // Create augmented matrix [A | I] and transform to [I | A^-1]

        let mut aug = vec![T::zero(); n * 2 * n];

        // Initialize augmented matrix [A | I]
        for i in 0..n {
            for j in 0..n {
                aug[i * (2 * n) + j] = a[i * n + j];
            }
            aug[i * (2 * n) + n + i] = T::one();
        }

        // Forward elimination with partial pivoting
        for i in 0..n {
            // Find pivot
            let mut pivot_row = i;
            let mut pivot_val = aug[i * (2 * n) + i].abs();

            for j in i + 1..n {
                let val = aug[j * (2 * n) + i].abs();
                if val > pivot_val {
                    pivot_row = j;
                    pivot_val = val;
                }
            }

            if pivot_val < KalmanScalar::epsilon() {
                return Err(KalmanError::SingularMatrix);
            }

            // Swap rows if needed
            if pivot_row != i {
                for j in 0..(2 * n) {
                    let tmp = aug[i * (2 * n) + j];
                    aug[i * (2 * n) + j] = aug[pivot_row * (2 * n) + j];
                    aug[pivot_row * (2 * n) + j] = tmp;
                }
            }

            // Scale pivot row
            let pivot = aug[i * (2 * n) + i];
            for j in 0..(2 * n) {
                aug[i * (2 * n) + j] = aug[i * (2 * n) + j] / pivot;
            }

            // Eliminate column
            for j in 0..n {
                if j != i {
                    let factor = aug[j * (2 * n) + i];
                    for k in 0..(2 * n) {
                        aug[j * (2 * n) + k] = aug[j * (2 * n) + k] - factor * aug[i * (2 * n) + k];
                    }
                }
            }
        }

        // Extract inverse from right half of augmented matrix
        for i in 0..n {
            for j in 0..n {
                a_inv[i * n + j] = aug[i * (2 * n) + n + j];
            }
        }

        Ok(())
    }

    fn is_positive_definite(a: &[T], n: usize) -> bool {
        // Check all leading principal minors are positive
        // Using Sylvester's criterion

        // For efficiency, we'll just try Cholesky and see if it succeeds
        let mut l = vec![T::zero(); n * n];
        Self::cholesky(a, &mut l, n).is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_matvec() {
        // [1 2]   [5]   [19]
        // [3 4] * [7] = [43]
        let a = vec![1.0_f64, 2.0, 3.0, 4.0];
        let x = vec![5.0, 7.0];
        let mut y = vec![0.0; 2];

        NaiveBackend::matvec(&a, &x, &mut y, 2, 2);

        assert_relative_eq!(y[0], 19.0, epsilon = 1e-10);
        assert_relative_eq!(y[1], 43.0, epsilon = 1e-10);
    }

    #[test]
    fn test_matmul() {
        // [1 2]   [5 6]   [19 22]
        // [3 4] * [7 8] = [43 50]
        let a = vec![1.0_f64, 2.0, 3.0, 4.0];
        let b = vec![5.0, 6.0, 7.0, 8.0];
        let mut c = vec![0.0; 4];

        NaiveBackend::matmul(&a, &b, &mut c, 2, 2, 2);

        assert_relative_eq!(c[0], 19.0, epsilon = 1e-10);
        assert_relative_eq!(c[1], 22.0, epsilon = 1e-10);
        assert_relative_eq!(c[2], 43.0, epsilon = 1e-10);
        assert_relative_eq!(c[3], 50.0, epsilon = 1e-10);
    }

    #[test]
    fn test_matmul_transpose() {
        // [1 2]   [5 7]^T   [19 22]
        // [3 4] * [6 8]   = [43 50]
        let a = vec![1.0_f64, 2.0, 3.0, 4.0];
        let b = vec![5.0, 7.0, 6.0, 8.0]; // stored as rows
        let mut c = vec![0.0; 4];

        NaiveBackend::matmul_transpose(&a, &b, &mut c, 2, 2, 2);

        assert_relative_eq!(c[0], 19.0, epsilon = 1e-10);
        assert_relative_eq!(c[1], 22.0, epsilon = 1e-10);
        assert_relative_eq!(c[2], 43.0, epsilon = 1e-10);
        assert_relative_eq!(c[3], 50.0, epsilon = 1e-10);
    }

    #[test]
    fn test_transpose() {
        // [1 2 3]^T   [1 4]
        // [4 5 6]   = [2 5]
        //             [3 6]
        let a = vec![1.0_f64, 2.0, 3.0, 4.0, 5.0, 6.0];
        let mut b = vec![0.0; 6];

        NaiveBackend::transpose(&a, &mut b, 2, 3);

        assert_relative_eq!(b[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(b[1], 4.0, epsilon = 1e-10);
        assert_relative_eq!(b[2], 2.0, epsilon = 1e-10);
        assert_relative_eq!(b[3], 5.0, epsilon = 1e-10);
        assert_relative_eq!(b[4], 3.0, epsilon = 1e-10);
        assert_relative_eq!(b[5], 6.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cholesky() {
        // Positive definite matrix:
        // [4  12 -16]
        // [12 37 -43]
        // [-16 -43 98]
        let p = vec![
            4.0_f64, 12.0, -16.0,
            12.0, 37.0, -43.0,
            -16.0, -43.0, 98.0
        ];
        let mut l = vec![0.0; 9];

        let result = NaiveBackend::cholesky(&p, &mut l, 3);
        assert!(result.is_ok());

        // Expected L:
        // [2  0  0]
        // [6  1  0]
        // [-8 5  3]
        assert_relative_eq!(l[0], 2.0, epsilon = 1e-10);
        assert_relative_eq!(l[3], 6.0, epsilon = 1e-10);
        assert_relative_eq!(l[4], 1.0, epsilon = 1e-10);
        assert_relative_eq!(l[6], -8.0, epsilon = 1e-10);
        assert_relative_eq!(l[7], 5.0, epsilon = 1e-10);
        assert_relative_eq!(l[8], 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_invert() {
        // [4 7]^-1 = [0.6 -0.7]
        // [2 6]      [-0.2 0.4]
        let a = vec![4.0_f64, 7.0, 2.0, 6.0];
        let mut a_inv = vec![0.0; 4];

        let result = NaiveBackend::invert(&a, &mut a_inv, 2);
        assert!(result.is_ok());

        assert_relative_eq!(a_inv[0], 0.6, epsilon = 1e-10);
        assert_relative_eq!(a_inv[1], -0.7, epsilon = 1e-10);
        assert_relative_eq!(a_inv[2], -0.2, epsilon = 1e-10);
        assert_relative_eq!(a_inv[3], 0.4, epsilon = 1e-10);
    }

    #[test]
    fn test_symmetrize() {
        // Non-symmetric matrix
        let mut a = vec![1.0_f64, 2.0, 3.0, 4.0];
        NaiveBackend::symmetrize(&mut a, 2);

        // Should become symmetric
        assert_relative_eq!(a[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(a[1], 2.5, epsilon = 1e-10);
        assert_relative_eq!(a[2], 2.5, epsilon = 1e-10);
        assert_relative_eq!(a[3], 4.0, epsilon = 1e-10);
    }
}
