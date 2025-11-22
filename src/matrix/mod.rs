// Matrix operations module
//
// This module provides optimized matrix operations for Kalman filtering.
// It supports multiple backends via feature flags:
// - `naive`: Hand-rolled pure Rust (no dependencies, no_std compatible)
// - `nalgebra`: High-performance pure Rust using nalgebra (default)
// - `blas`: Maximum performance using ndarray+BLAS (future)
//
// # Performance
//
// The nalgebra backend provides 9-13x speedup over naive implementation for
// matrix sizes typical in Kalman filtering (50x50 to 100x100). For very large
// matrices (100x100+), the speedup can exceed 13x.
//
// # Example
//
// ```
// use kalman_filters::matrix::{MatrixOps, DefaultBackend};
//
// // Matrix multiplication: C = A * B
// let a = vec![1.0, 2.0, 3.0, 4.0]; // 2x2 matrix in row-major order
// let b = vec![5.0, 6.0, 7.0, 8.0];
// let mut c = vec![0.0; 4];
//
// DefaultBackend::matmul(&a, &b, &mut c, 2, 2, 2);
// assert_eq!(c, vec![19.0, 22.0, 43.0, 50.0]);
// ```

use crate::{KalmanError, KalmanResult, KalmanScalar};

#[cfg(feature = "naive-matrix")]
mod naive;
#[cfg(feature = "naive-matrix")]
pub use naive::NaiveBackend as DefaultBackend;

#[cfg(all(feature = "nalgebra", not(feature = "naive-matrix")))]
mod nalgebra_backend;
#[cfg(all(feature = "nalgebra", not(feature = "naive-matrix")))]
pub use nalgebra_backend::NalgebraBackend as DefaultBackend;

#[cfg(feature = "blas-backend")]
mod ndarray_backend;

#[cfg(test)]
mod tests;

/// Core matrix operations trait
///
/// All matrix operations use row-major flat storage: `Vec<T>` where
/// element at (i,j) is stored at index `i * cols + j`.
pub trait MatrixOps<T: KalmanScalar> {
    /// Matrix-vector multiplication: y = A * x
    ///
    /// # Arguments
    /// * `a` - Matrix A (rows × cols) in row-major order
    /// * `x` - Vector x (cols elements)
    /// * `y` - Output vector y (rows elements)
    /// * `rows` - Number of rows in A
    /// * `cols` - Number of columns in A
    fn matvec(a: &[T], x: &[T], y: &mut [T], rows: usize, cols: usize);

    /// Matrix-matrix multiplication: C = A * B
    ///
    /// # Arguments
    /// * `a` - Matrix A (m × k) in row-major order
    /// * `b` - Matrix B (k × n) in row-major order
    /// * `c` - Output matrix C (m × n) in row-major order
    /// * `m` - Number of rows in A
    /// * `k` - Number of columns in A (= rows in B)
    /// * `n` - Number of columns in B
    fn matmul(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize);

    /// Matrix-matrix multiplication with B transposed: C = A * B^T
    ///
    /// This is more efficient than transposing B first, then multiplying.
    ///
    /// # Arguments
    /// * `a` - Matrix A (m × k) in row-major order
    /// * `b` - Matrix B (n × k) in row-major order (will be transposed)
    /// * `c` - Output matrix C (m × n) in row-major order
    /// * `m` - Number of rows in A
    /// * `k` - Number of columns in A (= columns in B)
    /// * `n` - Number of rows in B
    fn matmul_transpose(a: &[T], b: &[T], c: &mut [T], m: usize, k: usize, n: usize);

    /// Matrix transpose: B = A^T
    ///
    /// # Arguments
    /// * `a` - Matrix A (rows × cols) in row-major order
    /// * `b` - Output matrix B (cols × rows) in row-major order
    /// * `rows` - Number of rows in A
    /// * `cols` - Number of columns in A
    fn transpose(a: &[T], b: &mut [T], rows: usize, cols: usize);

    /// In-place matrix addition: A = A + B
    ///
    /// # Arguments
    /// * `a` - Matrix A (will be modified)
    /// * `b` - Matrix B
    /// * `size` - Total number of elements
    fn add_inplace(a: &mut [T], b: &[T], size: usize);

    /// In-place matrix scaling: A = alpha * A
    ///
    /// # Arguments
    /// * `a` - Matrix A (will be modified)
    /// * `alpha` - Scalar multiplier
    /// * `size` - Total number of elements
    fn scale_inplace(a: &mut [T], alpha: T, size: usize);

    /// Symmetrize matrix: A = (A + A^T) / 2
    ///
    /// Ensures numerical symmetry for covariance matrices.
    ///
    /// # Arguments
    /// * `a` - Square matrix (will be modified)
    /// * `n` - Dimension of square matrix
    fn symmetrize(a: &mut [T], n: usize);
}

/// Matrix decomposition and linear algebra operations
pub trait MatrixDecomposition<T: KalmanScalar> {
    /// Cholesky decomposition: L where P = L * L^T
    ///
    /// Returns lower triangular matrix L for positive definite matrix P.
    ///
    /// # Arguments
    /// * `p` - Positive definite symmetric matrix (n × n)
    /// * `l` - Output lower triangular matrix (n × n)
    /// * `n` - Dimension of square matrix
    ///
    /// # Errors
    /// Returns `KalmanError::SingularMatrix` if P is not positive definite
    fn cholesky(p: &[T], l: &mut [T], n: usize) -> KalmanResult<()>;

    /// Matrix inversion using Gauss-Jordan elimination
    ///
    /// # Arguments
    /// * `a` - Matrix to invert (n × n)
    /// * `a_inv` - Output inverted matrix (n × n)
    /// * `n` - Dimension of square matrix
    ///
    /// # Errors
    /// Returns `KalmanError::SingularMatrix` if matrix is singular
    fn invert(a: &[T], a_inv: &mut [T], n: usize) -> KalmanResult<()>;

    /// Check if matrix is positive definite
    ///
    /// # Arguments
    /// * `a` - Symmetric matrix (n × n)
    /// * `n` - Dimension of square matrix
    fn is_positive_definite(a: &[T], n: usize) -> bool;
}

/// Combined trait for all matrix operations
pub trait MatrixBackend<T: KalmanScalar>: MatrixOps<T> + MatrixDecomposition<T> {}

// Blanket implementation
impl<T: KalmanScalar, B: MatrixOps<T> + MatrixDecomposition<T>> MatrixBackend<T> for B {}
