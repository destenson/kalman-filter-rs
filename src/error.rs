
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

