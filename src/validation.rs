// Input validation and numerical stability utilities
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use num_traits::Float;

/// Minimum acceptable condition number for matrix operations
const MIN_CONDITION_NUMBER: f64 = 1e-12;

/// Regularization factor for near-singular matrices
const REGULARIZATION_EPSILON: f64 = 1e-10;

/// Validate that a covariance matrix is suitable for Kalman filter operations
pub fn validate_covariance<T: KalmanScalar>(P: &[T], n: usize) -> KalmanResult<()> {
    if P.len() != n * n {
        return Err(KalmanError::DimensionMismatch {
            expected: (n, n),
            actual: (P.len(), 1),
        });
    }

    // Check diagonal elements are positive
    for i in 0..n {
        let diag = P[i * n + i];
        if diag < T::zero() {
            return Err(KalmanError::InvalidCovariance);
        }
        if diag < T::from(MIN_CONDITION_NUMBER).unwrap() {
            return Err(KalmanError::InvalidCovariance);
        }
    }

    // Check symmetry
    for i in 0..n {
        for j in i + 1..n {
            let diff = (P[i * n + j] - P[j * n + i]).abs();
            if diff > T::from(1e-10).unwrap() {
                return Err(KalmanError::InvalidCovariance);
            }
        }
    }

    Ok(())
}

/// Validate measurement noise matrix
pub fn validate_measurement_noise<T: KalmanScalar>(R: &[T], m: usize) -> KalmanResult<()> {
    if R.len() != m * m {
        return Err(KalmanError::DimensionMismatch {
            expected: (m, m),
            actual: (R.len(), 1),
        });
    }

    // Check diagonal elements are positive (measurement noise must be > 0)
    for i in 0..m {
        let diag = R[i * m + i];
        if diag <= T::zero() {
            return Err(KalmanError::InvalidNoiseCovariance);
        }
        if diag < T::from(MIN_CONDITION_NUMBER).unwrap() {
            return Err(KalmanError::InvalidNoiseCovariance);
        }
    }

    Ok(())
}

/// Regularize a nearly singular matrix by adding small values to diagonal
pub fn regularize_matrix<T: KalmanScalar>(matrix: &mut [T], n: usize) {
    let epsilon = T::from(REGULARIZATION_EPSILON).unwrap();
    for i in 0..n {
        matrix[i * n + i] = matrix[i * n + i] + epsilon;
    }
}

/// Check if a value is numerically stable (not NaN or Inf)
pub fn is_numerically_stable<T: KalmanScalar + Float>(value: T) -> bool {
    !value.is_nan() && !value.is_infinite()
}

/// Validate state vector for numerical stability
pub fn validate_state<T: KalmanScalar + Float>(state: &[T]) -> KalmanResult<()> {
    for (i, &val) in state.iter().enumerate() {
        if !is_numerically_stable(val) {
            return Err(KalmanError::FilterDivergence(format!(
                "State element {} is NaN or Inf",
                i
            )));
        }
    }
    Ok(())
}

/// Compute condition number estimate (ratio of largest to smallest diagonal)
/// This is a simplified estimate - full condition number requires eigenvalues
pub fn estimate_condition_number<T: KalmanScalar>(matrix: &[T], n: usize) -> T {
    let mut min_diag = matrix[0];
    let mut max_diag = matrix[0];

    for i in 1..n {
        let diag = matrix[i * n + i];
        if diag < min_diag {
            min_diag = diag;
        }
        if diag > max_diag {
            max_diag = diag;
        }
    }

    if min_diag > T::zero() {
        max_diag / min_diag
    } else {
        T::from(f64::INFINITY).unwrap_or(T::from(1e20).unwrap())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_covariance_valid() {
        let p = vec![1.0, 0.0, 0.0, 1.0];
        assert!(validate_covariance(&p, 2).is_ok());
    }

    #[test]
    fn test_validate_covariance_negative_diagonal() {
        let p = vec![-1.0, 0.0, 0.0, 1.0];
        assert!(validate_covariance(&p, 2).is_err());
    }

    #[test]
    fn test_validate_covariance_not_symmetric() {
        let p = vec![1.0, 2.0, 3.0, 1.0];
        assert!(validate_covariance(&p, 2).is_err());
    }

    #[test]
    fn test_regularize_matrix() {
        let mut p = vec![0.0, 0.0, 0.0, 0.0];
        regularize_matrix(&mut p, 2);
        assert!(p[0] > 0.0);
        assert!(p[3] > 0.0);
    }

    #[test]
    fn test_condition_number() {
        let p = vec![1.0, 0.0, 0.0, 100.0];
        let cond = estimate_condition_number(&p, 2);
        assert!((cond - 100.0).abs() < 1e-10);
    }
}
