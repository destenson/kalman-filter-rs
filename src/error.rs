use std::fmt;

/// Result type for Kalman filter operations
pub type KalmanResult<T> = Result<T, KalmanError>;

/// Error types for Kalman filter operations
#[derive(Debug, Clone)]
pub enum KalmanError {
    /// Matrix dimension mismatch
    DimensionMismatch {
        expected: (usize, usize),
        actual: (usize, usize),
    },
    /// Singular matrix (not invertible)
    SingularMatrix,
    /// Invalid covariance matrix (not positive semi-definite)
    InvalidCovariance,
    /// Invalid noise covariance
    InvalidNoiseCovariance,
    /// Builder incomplete - missing required field
    BuilderIncomplete(String),
    /// Jacobian computation failed
    JacobianComputation(String),
    /// Filter divergence detected
    FilterDivergence(String),
}

impl fmt::Display for KalmanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            KalmanError::DimensionMismatch { expected, actual } => {
                write!(
                    f,
                    "Matrix dimension mismatch: expected {}x{}, got {}x{}",
                    expected.0, expected.1, actual.0, actual.1
                )
            }
            KalmanError::SingularMatrix => write!(f, "Matrix is singular and cannot be inverted"),
            KalmanError::InvalidCovariance => {
                write!(f, "Covariance matrix must be positive semi-definite")
            }
            KalmanError::InvalidNoiseCovariance => {
                write!(f, "Noise covariance matrix must be positive semi-definite")
            }
            KalmanError::BuilderIncomplete(field) => {
                write!(f, "Builder incomplete: missing required field '{}'", field)
            }
            KalmanError::JacobianComputation(msg) => {
                write!(f, "Jacobian computation failed: {}", msg)
            }
            KalmanError::FilterDivergence(msg) => {
                write!(f, "Filter divergence detected: {}", msg)
            }
        }
    }
}

impl std::error::Error for KalmanError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = KalmanError::DimensionMismatch {
            expected: (3, 3),
            actual: (2, 2),
        };
        assert_eq!(
            format!("{}", err),
            "Matrix dimension mismatch: expected 3x3, got 2x2"
        );

        let err = KalmanError::SingularMatrix;
        assert_eq!(
            format!("{}", err),
            "Matrix is singular and cannot be inverted"
        );

        let err = KalmanError::InvalidCovariance;
        assert_eq!(
            format!("{}", err),
            "Covariance matrix must be positive semi-definite"
        );

        let err = KalmanError::InvalidNoiseCovariance;
        assert_eq!(
            format!("{}", err),
            "Noise covariance matrix must be positive semi-definite"
        );

        let err = KalmanError::BuilderIncomplete("state".to_string());
        assert_eq!(
            format!("{}", err),
            "Builder incomplete: missing required field 'state'"
        );

        let err = KalmanError::JacobianComputation("failed to compute".to_string());
        assert_eq!(
            format!("{}", err),
            "Jacobian computation failed: failed to compute"
        );

        let err = KalmanError::FilterDivergence("NaN detected".to_string());
        assert_eq!(
            format!("{}", err),
            "Filter divergence detected: NaN detected"
        );
    }

    #[test]
    fn test_error_clone() {
        let err1 = KalmanError::DimensionMismatch {
            expected: (3, 3),
            actual: (2, 2),
        };
        let err2 = err1.clone();
        assert_eq!(format!("{}", err1), format!("{}", err2));
    }

    #[test]
    fn test_result_type() {
        let result: KalmanResult<i32> = Ok(42);
        assert_eq!(result.unwrap(), 42);

        let result: KalmanResult<i32> = Err(KalmanError::SingularMatrix);
        assert!(result.is_err());
    }
}
