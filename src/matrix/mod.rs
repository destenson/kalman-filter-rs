//! Dense row-major linear-algebra kernels.
//!
//! Every kernel is a pure function in destination-passing style: it reads its
//! inputs, writes only to the explicit `out` argument, and keeps no state. Dimensions
//! are passed explicitly; callers guarantee the slice lengths (the linear filter's
//! `Model` validates them once at construction). Nothing here allocates.
//!
//! Element (i, j) of an r×c matrix is stored at index `i * c + j`.

use crate::{KalmanError, KalmanResult};
use num_traits::Float;

mod legacy;
pub(crate) use legacy::{DefaultBackend, MatrixDecomposition, MatrixOps, invert};

/// out = A·B, where A is r×k and B is k×c.
pub(crate) fn mul<T: Float>(a: &[T], b: &[T], out: &mut [T], r: usize, k: usize, c: usize) {
    for i in 0..r {
        for j in 0..c {
            out[i * c + j] = (0..k).fold(T::zero(), |acc, p| acc + a[i * k + p] * b[p * c + j]);
        }
    }
}

/// out = A·Bᵀ, where A is r×k and B is c×k.
pub(crate) fn mul_bt<T: Float>(a: &[T], b: &[T], out: &mut [T], r: usize, k: usize, c: usize) {
    for i in 0..r {
        for j in 0..c {
            out[i * c + j] = dot(&a[i * k..(i + 1) * k], &b[j * k..(j + 1) * k]);
        }
    }
}

/// out += A·Bᵀ, where A is r×k and B is c×k.
pub(crate) fn add_mul_bt<T: Float>(a: &[T], b: &[T], out: &mut [T], r: usize, k: usize, c: usize) {
    for i in 0..r {
        for j in 0..c {
            out[i * c + j] = out[i * c + j] + dot(&a[i * k..(i + 1) * k], &b[j * k..(j + 1) * k]);
        }
    }
}

/// out = A·x, where A is r×c.
pub(crate) fn mul_vec<T: Float>(a: &[T], x: &[T], out: &mut [T], r: usize, c: usize) {
    for i in 0..r {
        out[i] = dot(&a[i * c..(i + 1) * c], &x[..c]);
    }
}

fn dot<T: Float>(a: &[T], b: &[T]) -> T {
    a.iter().zip(b).fold(T::zero(), |acc, (&x, &y)| acc + x * y)
}

/// A ← (A + Aᵀ)/2 for an n×n matrix. Used on a freshly computed output buffer.
pub(crate) fn symmetrize<T: Float>(a: &mut [T], n: usize) {
    let half = T::one() / (T::one() + T::one());
    for i in 0..n {
        for j in 0..i {
            let avg = (a[i * n + j] + a[j * n + i]) * half;
            a[i * n + j] = avg;
            a[j * n + i] = avg;
        }
    }
}

/// Cholesky factor of a symmetric matrix: S = L·Lᵀ, L lower triangular, written to
/// `l` (the strict upper triangle is zeroed). Only the lower triangle of `s` is read.
///
/// Singularity is judged relative to `S` itself: pivot k is rejected unless
/// `d_k > rel_tol · S_kk`. Scaling row/column k of `S` by c scales both sides by c², so
/// the test is invariant to the units of each component, at every magnitude.
pub(crate) fn cholesky<T: Float>(s: &[T], l: &mut [T], n: usize, rel_tol: T) -> KalmanResult<()> {
    l[..n * n].fill(T::zero());
    for j in 0..n {
        let d = s[j * n + j] - dot(&l[j * n..j * n + j], &l[j * n..j * n + j]);
        // Written so that NaN also counts as singular.
        if !(d > rel_tol * s[j * n + j]) {
            return Err(KalmanError::SingularMatrix);
        }
        let ljj = d.sqrt();
        l[j * n + j] = ljj;
        for i in j + 1..n {
            let acc = s[i * n + j] - dot(&l[i * n..i * n + j], &l[j * n..j * n + j]);
            l[i * n + j] = acc / ljj;
        }
    }
    Ok(())
}

/// Solves L·Lᵀ·x = b in place (`b` becomes x), with `l` from [`cholesky`].
pub(crate) fn cholesky_solve<T: Float>(l: &[T], n: usize, b: &mut [T]) {
    for i in 0..n {
        let acc = b[i] - dot(&l[i * n..i * n + i], &b[..i]);
        b[i] = acc / l[i * n + i];
    }
    for i in (0..n).rev() {
        let acc = (i + 1..n).fold(b[i], |acc, k| acc - l[k * n + i] * b[k]);
        b[i] = acc / l[i * n + i];
    }
}

/// Whether an n×n matrix is symmetric positive semi-definite, within a tolerance
/// relative to its largest entry. `work` (n×n) is scratch.
///
/// Uses symmetric diagonal pivoting (an LDLᵀ / pivoted-Cholesky elimination): take
/// the largest remaining diagonal as pivot; a pivot below `-tol` means indefinite;
/// once every remaining diagonal is within `tol` of zero, the rest of the Schur
/// complement must be ~0 too (a PSD matrix has |a_ij| ≤ √(a_ii·a_jj)). This accepts
/// singular PSD matrices (e.g. all zeros, or `[[1,1],[1,1]]`), which plain Cholesky
/// would reject.
pub(crate) fn is_symmetric_psd<T: Float>(a: &[T], n: usize, work: &mut [T]) -> bool {
    let scale = a[..n * n].iter().fold(T::zero(), |m, v| m.max(v.abs()));
    let tol = scale * relative_tolerance::<T>(n);
    let symmetric = (0..n).all(|i| (0..i).all(|j| (a[i * n + j] - a[j * n + i]).abs() <= tol));
    if !symmetric {
        return false;
    }
    let w = &mut work[..n * n];
    w.copy_from_slice(&a[..n * n]);
    for k in 0..n {
        let Some(p) = (k..n).max_by(|&i, &j| w[i * n + i].partial_cmp(&w[j * n + j]).unwrap_or(core::cmp::Ordering::Equal)) else {
            return true;
        };
        let pivot = w[p * n + p];
        if pivot < -tol {
            return false;
        }
        if pivot <= tol {
            return (k..n).all(|i| (k..n).all(|j| w[i * n + j].abs() <= tol));
        }
        swap_symmetric(w, n, k, p);
        for i in k + 1..n {
            let factor = w[i * n + k] / pivot;
            for j in k + 1..n {
                w[i * n + j] = w[i * n + j] - factor * w[k * n + j];
            }
        }
    }
    true
}

/// Swaps rows and columns i and j of an n×n matrix.
fn swap_symmetric<T>(w: &mut [T], n: usize, i: usize, j: usize) {
    if i == j {
        return;
    }
    for c in 0..n {
        w.swap(i * n + c, j * n + c);
    }
    for r in 0..n {
        w.swap(r * n + i, r * n + j);
    }
}

/// Relative tolerance for decisions on an n×n matrix: a small multiple of n·ε.
/// Dimensionless, so every threshold built from it scales with the data.
pub(crate) fn relative_tolerance<T: Float>(n: usize) -> T {
    let n = T::from(n).unwrap_or_else(T::max_value);
    T::from(32.0).unwrap_or_else(T::one) * n * T::epsilon()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn products() {
        let a = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]; // 2×3
        let b = [1.0, 0.0, 0.0, 1.0, 1.0, 1.0]; // 3×2
        let mut c = [0.0; 4];
        mul(&a, &b, &mut c, 2, 3, 2);
        assert_eq!(c, [4.0, 5.0, 10.0, 11.0]);
        mul_bt(&a, &a, &mut c, 2, 3, 2); // A·Aᵀ
        assert_eq!(c, [14.0, 32.0, 32.0, 77.0]);
        add_mul_bt(&a, &a, &mut c, 2, 3, 2);
        assert_eq!(c, [28.0, 64.0, 64.0, 154.0]);
        let mut y = [0.0; 2];
        mul_vec(&a, &[1.0, 1.0, 1.0], &mut y, 2, 3);
        assert_eq!(y, [6.0, 15.0]);
    }

    #[test]
    fn cholesky_solves_and_is_scale_invariant() {
        let s = [4.0, 2.0, 2.0, 3.0];
        let mut l = [0.0; 4];
        cholesky(&s, &mut l, 2, 1e-12).unwrap();
        assert_eq!(l, [2.0, 0.0, 1.0, 2f64.sqrt()]);
        let mut b = [8.0, 7.0]; // S·[1.25, 1.5]
        cholesky_solve(&l, 2, &mut b);
        assert!((b[0] - 1.25).abs() < 1e-14 && (b[1] - 1.5).abs() < 1e-14);
        // Same matrix with component 2 in tiny units: still not singular.
        let c = 1e-9;
        let s2 = [4.0, 2.0 * c, 2.0 * c, 3.0 * c * c];
        assert!(cholesky(&s2, &mut l, 2, 1e-12).is_ok());
        // Rank-1 at a huge scale: singular.
        let big = 1e12;
        let s3 = [big, 0.3 * big, 0.3 * big, 0.09 * big];
        assert!(matches!(cholesky(&s3, &mut l, 2, 1e-12), Err(KalmanError::SingularMatrix)));
    }

    #[test]
    fn psd_classification() {
        let mut w = [0.0; 9];
        let psd = |a: &[f64], n, w: &mut [f64]| is_symmetric_psd(a, n, w);
        assert!(psd(&[0.0; 4], 2, &mut w));
        assert!(psd(&[1.0, 1.0, 1.0, 1.0], 2, &mut w));
        assert!(psd(&[1e-30, 0.0, 0.0, 1e-30], 2, &mut w));
        assert!(psd(&[1.0, 0.0, 0.0, 1e-13], 2, &mut w));
        assert!(!psd(&[1.0, 0.5, 0.0, 1.0], 2, &mut w)); // asymmetric
        assert!(!psd(&[-1.0, 0.0, 0.0, 1.0], 2, &mut w)); // negative diagonal
        assert!(!psd(&[1.0, 2.0, 2.0, 1.0], 2, &mut w)); // indefinite
        assert!(!psd(&[0.0, 1.0, 1.0, 0.0], 2, &mut w)); // zero diagonal, indefinite
        // 3×3 rank-2 PSD: v vᵀ + u uᵀ.
        let (v, u) = ([1.0, 2.0, 3.0], [0.5, -1.0, 0.25]);
        let a: Vec<f64> = (0..9).map(|k| v[k / 3] * v[k % 3] + u[k / 3] * u[k % 3]).collect();
        assert!(psd(&a, 3, &mut w));
        assert!(psd(&a.iter().map(|x| x * 1e-20).collect::<Vec<_>>(), 3, &mut w));
    }
}
