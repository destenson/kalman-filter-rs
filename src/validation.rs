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
        if diag < T::from_f64(MIN_CONDITION_NUMBER) {
            return Err(KalmanError::InvalidCovariance);
        }
    }

    // Check symmetry
    for i in 0..n {
        for j in i + 1..n {
            let diff = (P[i * n + j] - P[j * n + i]).abs();
            if diff > T::from_f64(1e-10) {
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
        if diag < T::from_f64(MIN_CONDITION_NUMBER) {
            return Err(KalmanError::InvalidNoiseCovariance);
        }
    }

    Ok(())
}

/// Regularize a nearly singular matrix by adding small values to diagonal
pub fn regularize_matrix<T: KalmanScalar>(matrix: &mut [T], n: usize) {
    let epsilon = T::from_f64(REGULARIZATION_EPSILON);
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
        T::infinity()
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

    // =========================================================================
    // Additional tests for previously untested functions
    // =========================================================================

    #[test]
    fn test_validate_measurement_noise_valid() {
        let r = vec![1.0, 0.0, 0.0, 1.0];
        assert!(validate_measurement_noise(&r, 2).is_ok());
    }

    #[test]
    fn test_validate_measurement_noise_zero_diagonal() {
        // Measurement noise MUST be strictly positive
        let r = vec![0.0, 0.0, 0.0, 1.0];
        assert!(validate_measurement_noise(&r, 2).is_err());
    }

    #[test]
    fn test_validate_measurement_noise_negative_diagonal() {
        let r = vec![-1.0, 0.0, 0.0, 1.0];
        assert!(validate_measurement_noise(&r, 2).is_err());
    }

    #[test]
    fn test_validate_measurement_noise_near_zero() {
        // Very small values below MIN_CONDITION_NUMBER should fail
        let r = vec![1e-15, 0.0, 0.0, 1.0];
        assert!(validate_measurement_noise(&r, 2).is_err());
    }

    #[test]
    fn test_validate_measurement_noise_dimension_mismatch() {
        let r = vec![1.0]; // 1x1 but claiming 2x2
        assert!(validate_measurement_noise(&r, 2).is_err());
    }

    #[test]
    fn test_is_numerically_stable_valid() {
        assert!(is_numerically_stable(1.0_f64));
        assert!(is_numerically_stable(0.0_f64));
        assert!(is_numerically_stable(-1e100_f64));
        assert!(is_numerically_stable(1e-100_f64));
    }

    #[test]
    fn test_is_numerically_stable_nan() {
        assert!(!is_numerically_stable(f64::NAN));
    }

    #[test]
    fn test_is_numerically_stable_infinity() {
        assert!(!is_numerically_stable(f64::INFINITY));
        assert!(!is_numerically_stable(f64::NEG_INFINITY));
    }

    #[test]
    fn test_validate_state_valid() {
        let state = vec![1.0, 2.0, 3.0];
        assert!(validate_state(&state).is_ok());
    }

    #[test]
    fn test_validate_state_nan() {
        let state = vec![1.0, f64::NAN, 3.0];
        let result = validate_state(&state);
        assert!(result.is_err());
        // Check the error message mentions the correct index
        if let Err(KalmanError::FilterDivergence(msg)) = result {
            assert!(msg.contains("1"), "Error should mention element index 1");
        }
    }

    #[test]
    fn test_validate_state_infinity() {
        let state = vec![f64::INFINITY, 2.0];
        assert!(validate_state(&state).is_err());
    }

    #[test]
    fn test_validate_state_empty() {
        let state: Vec<f64> = vec![];
        assert!(validate_state(&state).is_ok()); // Empty is technically valid
    }

    #[test]
    fn test_validate_covariance_dimension_mismatch() {
        let p = vec![1.0, 0.0, 0.0]; // 3 elements but claiming 2x2
        assert!(validate_covariance(&p, 2).is_err());
    }

    #[test]
    fn test_validate_covariance_near_zero_diagonal() {
        // Very small positive values below threshold should fail
        let p = vec![1e-15, 0.0, 0.0, 1.0];
        assert!(validate_covariance(&p, 2).is_err());
    }

    #[test]
    fn test_validate_covariance_nearly_symmetric() {
        // Asymmetry within tolerance should pass
        let p = vec![1.0, 0.5 + 1e-12, 0.5, 1.0];
        assert!(validate_covariance(&p, 2).is_ok());
    }

    #[test]
    fn test_regularize_matrix_adds_to_diagonal_only() {
        let mut p = vec![1.0, 2.0, 3.0, 4.0];
        let orig_offdiag_01 = p[1];
        let orig_offdiag_10 = p[2];
        regularize_matrix(&mut p, 2);

        // Diagonal should increase
        assert!(p[0] > 1.0);
        assert!(p[3] > 4.0);
        // Off-diagonal should be unchanged
        assert_eq!(p[1], orig_offdiag_01);
        assert_eq!(p[2], orig_offdiag_10);
    }

    #[test]
    fn test_condition_number_identity() {
        let identity = vec![1.0, 0.0, 0.0, 1.0];
        let cond = estimate_condition_number(&identity, 2);
        assert!(
            (cond - 1.0).abs() < 1e-10,
            "Identity matrix has condition number 1"
        );
    }

    #[test]
    fn test_condition_number_ill_conditioned() {
        // Large condition number indicates ill-conditioning
        let p = vec![1.0, 0.0, 0.0, 1e-10];
        let cond = estimate_condition_number(&p, 2);
        assert!(cond > 1e9, "Should detect ill-conditioned matrix");
    }

    #[test]
    fn test_condition_number_zero_diagonal() {
        // Zero on diagonal should give infinity (or very large number)
        let p = vec![1.0, 0.0, 0.0, 0.0];
        let cond = estimate_condition_number(&p, 2);
        assert!(
            cond > 1e10,
            "Zero diagonal should give huge condition number"
        );
    }

    #[test]
    fn test_condition_number_3x3() {
        // Test with larger matrix
        let p = vec![10.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1];
        let cond = estimate_condition_number(&p, 3);
        assert!(
            (cond - 100.0).abs() < 1e-10,
            "Condition number should be 10/0.1 = 100"
        );
    }
}
