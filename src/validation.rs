//! Input validation for the linear filter's construction and update.
//!
//! Pure checks: each takes data and returns `Ok` or the `KalmanError` describing the
//! first problem. No logging, no repair (nothing is silently regularized).

use crate::error::{KalmanError, KalmanResult};
use crate::matrix;
use num_traits::Float;

/// Both dimensions must be non-zero.
pub(crate) fn dimensions(state_dim: usize, measurement_dim: usize) -> KalmanResult<()> {
    if state_dim == 0 {
        return Err(KalmanError::InvalidInput("state_dim must be at least 1"));
    }
    if measurement_dim == 0 {
        return Err(KalmanError::InvalidInput("measurement_dim must be at least 1"));
    }
    Ok(())
}

/// `data` must hold exactly `rows × cols` finite values. The product is
/// overflow-checked: a product that does not fit in `usize` cannot match any slice.
pub(crate) fn matrix<T: Float>(
    name: &'static str,
    data: &[T],
    rows: usize,
    cols: usize,
) -> KalmanResult<()> {
    if rows.checked_mul(cols) != Some(data.len()) {
        return Err(KalmanError::DimensionMismatch {
            expected: (rows, cols),
            actual: (data.len(), 1),
        });
    }
    finite(name, data)
}

/// Every value must be finite (no NaN, no ±∞).
pub(crate) fn finite<T: Float>(name: &'static str, data: &[T]) -> KalmanResult<()> {
    if data.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(KalmanError::InvalidInput(name))
    }
}

/// An n×n matrix (already length- and finiteness-checked) must be symmetric PSD,
/// within a tolerance relative to its own scale. Otherwise returns `error`.
pub(crate) fn covariance<T: Float>(data: &[T], n: usize, error: KalmanError) -> KalmanResult<()> {
    let mut work = vec![T::zero(); data.len()];
    if matrix::is_symmetric_psd(data, n, &mut work) {
        Ok(())
    } else {
        Err(error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_zero_dimensions_and_overflow() {
        assert!(dimensions(0, 1).is_err());
        assert!(dimensions(1, 0).is_err());
        assert!(dimensions(1, 1).is_ok());
        let r = matrix::<f64>("h", &[], usize::MAX / 2 + 1, 2);
        assert!(matches!(r, Err(KalmanError::DimensionMismatch { .. })));
    }

    #[test]
    fn rejects_non_finite_values() {
        assert!(matches!(matrix("q", &[1.0, f64::NAN], 1, 2), Err(KalmanError::InvalidInput("q"))));
        assert!(finite("z", &[f32::INFINITY]).is_err());
        assert!(finite("z", &[1.0f32]).is_ok());
    }

    #[test]
    fn covariance_check_is_scale_invariant() {
        for s in [1e-30, 1.0, 1e30] {
            assert!(covariance(&[s, 0.0, 0.0, s * 1e-13], 2, KalmanError::InvalidCovariance).is_ok());
            assert!(covariance(&[s, 2.0 * s, 2.0 * s, s], 2, KalmanError::InvalidCovariance).is_err());
        }
    }
}
