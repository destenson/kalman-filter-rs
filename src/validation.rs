//! Construction-time checks on covariance matrices.
//!
//! The PSD test is invariant under any diagonal rescaling `A → D A D` (a change
//! of units per component): the matrix is first scaled to unit diagonal (a
//! correlation matrix), and every tolerance is relative to that unit scale.

use crate::matrix::{cholesky_in_place, rel_tol};
use crate::types::KalmanScalar;

/// Is the `n × n` row-major matrix `a` finite, symmetric and positive
/// semi-definite (singular allowed)? `work` must hold `n · n` elements; it is
/// overwritten.
///
/// 1. Entries finite; diagonal ≥ 0.
/// 2. Symmetry: `|a_ij − a_ji| ≤ tol · √(a_ii a_jj)`. A zero diagonal forces
///    its row and column to be exactly zero (otherwise A cannot be PSD).
/// 3. `C = D⁻¹ A D⁻¹` with `D = diag(√a_ii)` has unit diagonal; `C + tol·I`
///    must admit a Cholesky factorisation. This accepts every PSD matrix
///    (λ_min(C + tol·I) ≥ tol exceeds the rounding error of the factorisation)
///    and rejects any matrix with λ_min(C) < −tol.
pub(crate) fn is_psd<T: KalmanScalar>(a: &[T], n: usize, work: &mut [T]) -> bool {
    if a.len() != n * n || work.len() != n * n || !crate::matrix::all_finite(a) {
        return false;
    }
    let tol = rel_tol::<T>(n);
    let half = T::one() / (T::one() + T::one());
    let diag = |i: usize| a[i * n + i];
    if (0..n).any(|i| diag(i) < T::zero()) {
        return false;
    }
    for i in 0..n {
        for j in 0..=i {
            let (aij, aji) = (a[i * n + j], a[j * n + i]);
            let (di, dj) = (diag(i).sqrt(), diag(j).sqrt());
            if di == T::zero() || dj == T::zero() {
                // PSD forces the whole row/column of a zero variance to vanish.
                if aij != T::zero() || aji != T::zero() {
                    return false;
                }
                // The component is decoupled; give it a unit pivot.
                work[i * n + j] = if i == j { T::one() } else { T::zero() };
                continue;
            }
            // Divide stepwise so neither tiny nor huge variances under/overflow.
            let (cij, cji) = (aij / di / dj, aji / di / dj);
            let asym = (cij - cji).abs();
            if asym.is_nan() || asym > tol {
                return false;
            }
            work[i * n + j] = (cij + cji) * half + if i == j { tol } else { T::zero() };
        }
    }
    // Only the lower triangle (just written) is read by the factorisation.
    // Pivot test `d > 0`: the tolerance is already in the diagonal shift.
    cholesky_in_place(work, n, T::zero()).is_ok()
}

/// Acceptance test for a *computed* covariance (SPEC B11/B21): is the exactly
/// symmetric, finite `n × n` matrix `p` PSD up to `λ_min ≥ −τ · max_i p_ii`
/// (hence `≥ −τ · λ_max`), with `τ = ε^¾`? τ is 6e-6 (f32) / 2e-12 (f64), inside
/// the spec's 1e-5 / 1e-9, and far above the factorisation's own rounding.
/// Unlike [`is_psd`] this does not rescale per component: the criterion is
/// relative to λ_max by definition. `work` (n·n) is overwritten.
pub(crate) fn psd_to_working_precision<T: KalmanScalar>(p: &[T], n: usize, work: &mut [T]) -> bool {
    let max_var = (0..n).fold(T::zero(), |acc, i| acc.max(p[i * n + i]));
    // The `min_positive_value` floor keeps P = 0 (a valid covariance) factorable.
    let eps = <T as num_traits::Float>::epsilon();
    let eps = <T as num_traits::Float>::epsilon();
    let shift = eps.sqrt() * eps.sqrt().sqrt() * max_var + T::min_positive_value();
    work.copy_from_slice(p);
    for i in 0..n {
        work[i * n + i] = work[i * n + i] + shift;
    }
    cholesky_in_place(work, n, T::zero()).is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn psd(a: &[f64], n: usize) -> bool {
        is_psd(a, n, &mut vec![0.0; n * n])
    }

    #[test]
    fn accepts_psd_at_any_scale_including_singular() {
        for s in [1e-300, 1e-14, 1.0, 1e14, 1e300] {
            assert!(psd(&[s, 0.0, 0.0, s], 2));
            assert!(psd(&[s, s, s, s], 2), "rank-deficient at {s:e}");
            assert!(psd(&[s, 0.0, 0.0, 1e-13 * s], 2));
        }
        assert!(psd(&[0.0; 4], 2));
        assert!(psd(&[1.0, 0.0, 0.0, 0.0], 2));
    }

    #[test]
    fn rejects_asymmetric_indefinite_and_non_finite() {
        assert!(!psd(&[1.0, 0.5, 0.0, 1.0], 2));
        assert!(!psd(&[1.0, 2.0, 2.0, 1.0], 2));
        assert!(!psd(&[-1.0, 0.0, 0.0, 1.0], 2));
        assert!(!psd(&[0.0, 1.0, 1.0, 1.0], 2));
        assert!(!psd(&[f64::NAN, 0.0, 0.0, 1.0], 2));
        assert!(!psd(&[1e300, 1e300, 1e300, 1e-300], 2));
        assert!(!is_psd(&[1.0f32, 1.001, 1.001, 1.0], 2, &mut [0.0; 4]));
    }
}
