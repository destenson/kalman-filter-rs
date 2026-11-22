//! Dense row-major kernels on caller-provided slices.
//!
//! Every matrix is a flat slice with element `(i, j)` at `i * cols + j`. Kernels
//! write into caller-owned output slices and never allocate (except
//! [`invert_matrix`], which exists only for the nonlinear filters).
//!
//! Kernels are crate-private: callers are responsible for passing slices of the
//! documented lengths. They iterate with `chunks_exact`/`zip` rather than
//! indexing, so a wrong length truncates instead of panicking, and the inner
//! loops are free of bounds checks.

use crate::error::{KalmanError, KalmanResult};
use num_traits::Float;

/// Rows of a row-major matrix with `cols` columns. `cols == 0` yields no rows
/// instead of panicking.
#[inline]
fn rows<T>(a: &[T], cols: usize) -> core::slice::ChunksExact<'_, T> {
    a.chunks_exact(cols.max(1))
}

#[inline]
fn rows_mut<T>(a: &mut [T], cols: usize) -> core::slice::ChunksExactMut<'_, T> {
    a.chunks_exact_mut(cols.max(1))
}

#[inline]
pub(crate) fn dot<T: Float>(a: &[T], b: &[T]) -> T {
    a.iter().zip(b).fold(T::zero(), |acc, (&x, &y)| acc + x * y)
}

/// `y += alpha * x`
#[inline]
pub(crate) fn axpy<T: Float>(alpha: T, x: &[T], y: &mut [T]) {
    for (yi, &xi) in y.iter_mut().zip(x) {
        *yi = *yi + alpha * xi;
    }
}

pub(crate) fn all_finite<T: Float>(a: &[T]) -> bool {
    a.iter().all(|v| v.is_finite())
}

/// `y = A x`, A is `rows × cols`.
pub(crate) fn matvec<T: Float>(a: &[T], x: &[T], y: &mut [T], _rows: usize, cols: usize) {
    for (yi, row) in y.iter_mut().zip(rows(a, cols)) {
        *yi = dot(row, x);
    }
}

/// `C = A B`, A is `m × k`, B is `k × n`. Row-axpy order: streams rows of B.
pub(crate) fn matmul<T: Float>(a: &[T], b: &[T], c: &mut [T], _m: usize, k: usize, n: usize) {
    for (c_row, a_row) in rows_mut(c, n).zip(rows(a, k)) {
        c_row.fill(T::zero());
        for (&aik, b_row) in a_row.iter().zip(rows(b, n)) {
            axpy(aik, b_row, c_row);
        }
    }
}

/// `C = A Bᵀ`, A is `m × k`, B is `n × k`. Every entry is a dot of two rows.
pub(crate) fn matmul_transpose<T: Float>(a: &[T], b: &[T], c: &mut [T], _m: usize, k: usize, n: usize) {
    for (c_row, a_row) in rows_mut(c, n).zip(rows(a, k)) {
        for (cij, b_row) in c_row.iter_mut().zip(rows(b, k)) {
            *cij = dot(a_row, b_row);
        }
    }
}

/// Upper triangle (j ≥ i) of `C = A Bᵀ`, A and B are `n × k`, C is `n × n`.
/// The strict lower triangle is left untouched; finish with [`mirror_upper`].
pub(crate) fn matmul_transpose_upper<T: Float>(a: &[T], b: &[T], c: &mut [T], n: usize, k: usize) {
    for (i, (c_row, a_row)) in rows_mut(c, n).zip(rows(a, k)).enumerate() {
        for (cij, b_row) in c_row[i..].iter_mut().zip(rows(b, k).skip(i)) {
            *cij = dot(a_row, b_row);
        }
    }
}

/// Upper triangle of `C += Aᵀ B`, A and B are `k × n`, C is `n × n`
/// (a sum of k outer products of matching rows).
pub(crate) fn add_transpose_matmul_upper<T: Float>(a: &[T], b: &[T], c: &mut [T], n: usize, k: usize) {
    let _ = k;
    for (a_row, b_row) in rows(a, n).zip(rows(b, n)) {
        for (i, (c_row, &a_ji)) in rows_mut(c, n).zip(a_row).enumerate() {
            axpy(a_ji, &b_row[i..], &mut c_row[i..]);
        }
    }
}

/// Copy the upper triangle onto the lower one: the result is exactly symmetric.
pub(crate) fn mirror_upper<T: Copy>(c: &mut [T], n: usize) {
    for i in 1..n {
        let (head, tail) = c.split_at_mut(i * n);
        for (j, cij) in tail[..i].iter_mut().enumerate() {
            *cij = head[j * n + i];
        }
    }
}

/// `A += B` over the first `size` elements.
pub(crate) fn add_inplace<T: Float>(a: &mut [T], b: &[T], size: usize) {
    for (ai, &bi) in a.iter_mut().zip(b).take(size) {
        *ai = *ai + bi;
    }
}

/// `A = (A + Aᵀ) / 2`
pub(crate) fn symmetrize<T: Float>(a: &mut [T], n: usize) {
    let half = T::from(0.5).unwrap_or_else(T::zero);
    for i in 1..n {
        let (head, tail) = a.split_at_mut(i * n);
        for (j, aij) in tail[..i].iter_mut().enumerate() {
            let aji = &mut head[j * n + i];
            let avg = (*aij + *aji) * half;
            *aij = avg;
            *aji = avg;
        }
    }
}

/// Why a Cholesky factorisation stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CholeskyError {
    /// A diagonal entry is NaN or infinite.
    NonFinite,
    /// A pivot is not above `rel_tol` times its own diagonal entry.
    Singular,
}

/// In-place Cholesky `S = L Lᵀ` of a symmetric `m × m` matrix; `L` overwrites
/// the lower triangle (the strict upper triangle is left untouched).
///
/// A pivot `d_j` is rejected unless `d_j > rel_tol · S_jj`. The test is relative
/// to the entry's own diagonal, so it is invariant under any diagonal scaling
/// `S → D S D`: no absolute magnitude threshold exists.
pub(crate) fn cholesky_in_place<T: Float>(s: &mut [T], m: usize, rel_tol: T) -> Result<(), CholeskyError> {
    for j in 0..m {
        let (row_j, below) = s[j * m..].split_at_mut(m);
        let s_jj = row_j[j];
        if !s_jj.is_finite() {
            return Err(CholeskyError::NonFinite);
        }
        // Row j of L, columns < j, is already final.
        let d = s_jj - dot(&row_j[..j], &row_j[..j]);
        if d.is_nan() || d <= rel_tol * s_jj {
            return Err(CholeskyError::Singular);
        }
        let l_jj = d.sqrt();
        row_j[j] = l_jj;
        // Column j below the diagonal: L_ij = (S_ij - L_i,<j · L_j,<j) / L_jj.
        let l_j = &row_j[..j];
        for row_i in rows_mut(below, m) {
            row_i[j] = (row_i[j] - dot(&row_i[..j], l_j)) / l_jj;
        }
    }
    Ok(())
}

/// Solve `L Lᵀ X = B` in place for an `m × n` right-hand side stored row-major
/// in `x`; `l` holds the factor from [`cholesky_in_place`]. Each step is an
/// axpy over a contiguous row of length `n`.
pub(crate) fn cholesky_solve_rows<T: Float>(l: &[T], x: &mut [T], m: usize, n: usize) {
    // Forward: L Y = B.
    for (j, l_row) in rows(l, m).enumerate().take(m) {
        let (done, rest) = x.split_at_mut(j * n);
        let row_j = &mut rest[..n];
        for (&l_jk, row_k) in l_row[..j].iter().zip(rows(done, n)) {
            axpy(-l_jk, row_k, row_j);
        }
        let l_jj = l_row[j];
        row_j.iter_mut().for_each(|v| *v = *v / l_jj);
    }
    // Backward: Lᵀ X = Y. Column j of L below the diagonal is L[k][j], k > j.
    for j in (0..m).rev() {
        let (head, done) = x.split_at_mut((j + 1) * n);
        let row_j = &mut head[j * n..];
        for (k, row_k) in rows(done, n).enumerate() {
            axpy(-l[(j + 1 + k) * m + j], row_k, row_j);
        }
        let l_jj = l[j * m + j];
        row_j.iter_mut().for_each(|v| *v = *v / l_jj);
    }
}

/// General inverse by Gauss–Jordan elimination with partial pivoting.
///
/// Allocates. Not used by the linear filter (which solves with S instead of
/// inverting it); kept for the nonlinear and information filters until they
/// are rebuilt. A pivot is singular when it is at most `n · ε · max|A|`.
pub fn invert_matrix<T: Float>(a: &[T], n: usize) -> KalmanResult<Vec<T>> {
    let nn = n.checked_mul(n).filter(|&nn| nn == a.len() && n > 0).ok_or(
        KalmanError::DimensionMismatch { expected: (n, n), actual: (a.len(), 1) },
    )?;
    let scale = a.iter().fold(T::zero(), |acc, v| acc.max(v.abs()));
    if !scale.is_finite() || scale == T::zero() {
        return Err(KalmanError::SingularMatrix);
    }
    let tol = T::epsilon() * T::from(n).unwrap_or_else(T::one) * scale;
    let mut m = a.to_vec();
    let mut inv = vec![T::zero(); nn];
    for i in 0..n {
        inv[i * n + i] = T::one();
    }
    for col in 0..n {
        let mut piv = col;
        for r in col + 1..n {
            if m[r * n + col].abs() > m[piv * n + col].abs() {
                piv = r;
            }
        }
        let p = m[piv * n + col];
        if p.is_nan() || p.abs() <= tol {
            return Err(KalmanError::SingularMatrix);
        }
        if piv != col {
            for k in 0..n {
                m.swap(col * n + k, piv * n + k);
                inv.swap(col * n + k, piv * n + k);
            }
        }
        for k in 0..n {
            m[col * n + k] = m[col * n + k] / p;
            inv[col * n + k] = inv[col * n + k] / p;
        }
        for r in 0..n {
            let f = m[r * n + col];
            if r != col && f != T::zero() {
                for k in 0..n {
                    m[r * n + k] = m[r * n + k] - f * m[col * n + k];
                    inv[r * n + k] = inv[r * n + k] - f * inv[col * n + k];
                }
            }
        }
    }
    Ok(inv)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn products() {
        let a = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]; // 2×3
        let b = [1.0, 0.0, 0.0, 1.0, 1.0, 1.0]; // 3×2
        let mut c = [0.0; 4];
        matmul(&a, &b, &mut c, 2, 3, 2);
        assert_eq!(c, [4.0, 5.0, 10.0, 11.0]);
        matmul_transpose(&a, &a, &mut c, 2, 3, 2);
        assert_eq!(c, [14.0, 32.0, 32.0, 77.0]);
        let mut u = [0.0; 4];
        matmul_transpose_upper(&a, &a, &mut u, 2, 3);
        mirror_upper(&mut u, 2);
        assert_eq!(u, c);
        // Aᵀ A for A = 2×2 [[1,2],[3,4]] = [[10,14],[14,20]]
        let mut s = [0.0; 4];
        add_transpose_matmul_upper(&[1.0, 2.0, 3.0, 4.0], &[1.0, 2.0, 3.0, 4.0], &mut s, 2, 2);
        mirror_upper(&mut s, 2);
        assert_eq!(s, [10.0, 14.0, 14.0, 20.0]);
    }

    #[test]
    fn cholesky_solves_and_detects_singularity_relative_to_scale() {
        for scale in [1e-200, 1.0, 1e200] {
            let mut s = [4.0 * scale, 2.0 * scale, 2.0 * scale, 3.0 * scale];
            cholesky_in_place(&mut s, 2, 1e-12).unwrap();
            let mut x = [6.0 * scale, 5.0 * scale]; // S [1,1]ᵀ
            cholesky_solve_rows(&s, &mut x, 2, 1);
            assert!((x[0] - 1.0).abs() < 1e-12 && (x[1] - 1.0).abs() < 1e-12, "{x:?}");
            let mut sing = [scale, scale, scale, scale];
            assert_eq!(cholesky_in_place(&mut sing, 2, 1e-12), Err(CholeskyError::Singular));
        }
    }

    #[test]
    fn inverse_round_trip() {
        let a = [4.0, 7.0, 2.0, 6.0];
        let inv = invert_matrix(&a, 2).unwrap();
        let mut id = [0.0; 4];
        matmul(&a, &inv, &mut id, 2, 2, 2);
        for (v, e) in id.iter().zip([1.0, 0.0, 0.0, 1.0]) {
            assert!((v - e).abs() < 1e-12);
        }
        assert!(matches!(invert_matrix(&[1.0, 2.0, 2.0, 4.0], 2), Err(KalmanError::SingularMatrix)));
        assert!(invert_matrix(&[1.0, 2.0], 2).is_err());
    }
}
