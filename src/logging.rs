//! Logging utilities for the Kalman filter library
//!
//! This module provides utilities for efficient logging of numerical operations,
//! matrix formatting, and diagnostic information. All logging is done through
//! the `log` crate facade, allowing users to control logging output.

use crate::types::KalmanScalar;
use core::fmt;

/// Format a matrix for logging with lazy evaluation
///
/// Only computes the string representation if the log level is enabled
pub fn format_matrix<'a, T: KalmanScalar>(
    matrix: &'a [T],
    rows: usize,
    cols: usize,
    name: &'a str,
) -> impl fmt::Display + 'a {
    MatrixFormatter {
        matrix,
        rows,
        cols,
        name,
    }
}

struct MatrixFormatter<'a, T: KalmanScalar> {
    matrix: &'a [T],
    rows: usize,
    cols: usize,
    name: &'a str,
}

impl<T: KalmanScalar> fmt::Display for MatrixFormatter<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.matrix.is_empty() {
            return write!(f, "{}=[empty]", self.name);
        }

        write!(f, "{}=[", self.name)?;
        for i in 0..self.rows {
            if i > 0 {
                write!(f, "; ")?;
            }
            for j in 0..self.cols {
                if j > 0 {
                    write!(f, ", ")?;
                }
                let idx = i * self.cols + j;
                write!(f, "{:.4}", KalmanScalar::to_f64(&self.matrix[idx]))?;
            }
        }
        write!(f, "]")
    }
}

/// Format a state vector for logging
pub fn format_state<'a, T: KalmanScalar>(state: &'a [T], name: &'a str) -> impl fmt::Display + 'a {
    StateFormatter { state, name }
}

struct StateFormatter<'a, T: KalmanScalar> {
    state: &'a [T],
    name: &'a str,
}

impl<T: KalmanScalar> fmt::Display for StateFormatter<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}=[", self.name)?;
        for (i, val) in self.state.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "{:.4}", KalmanScalar::to_f64(val))?;
        }
        write!(f, "]")
    }
}

/// Calculate the Frobenius norm of a state vector
pub fn state_norm<T: KalmanScalar>(state: &[T]) -> f64 {
    state
        .iter()
        .map(|x| {
            let val = KalmanScalar::to_f64(x);
            val * val
        })
        .sum::<f64>()
        .sqrt()
}

/// Calculate the condition number of a matrix (ratio of largest to smallest singular value)
/// This is an approximation using the Frobenius norm for efficiency
pub fn matrix_condition_estimate<T: KalmanScalar>(matrix: &[T], size: usize) -> f64 {
    if matrix.is_empty() || size == 0 {
        return f64::INFINITY;
    }

    // Calculate Frobenius norm as an estimate
    let norm: f64 = matrix
        .iter()
        .map(|x| {
            let val = KalmanScalar::to_f64(x);
            val * val
        })
        .sum::<f64>()
        .sqrt();

    // Find minimum diagonal element as estimate of smallest eigenvalue
    let mut min_diag = f64::MAX;
    for i in 0..size {
        let idx = i * size + i;
        if idx < matrix.len() {
            let val = KalmanScalar::to_f64(&matrix[idx]).abs();
            if val < min_diag && val > 0.0f64 {
                min_diag = val;
            }
        }
    }

    if min_diag == 0.0 || min_diag == f64::MAX {
        return f64::INFINITY;
    }

    norm / min_diag
}

/// Check if a covariance matrix is near-singular
pub fn is_near_singular<T: KalmanScalar>(matrix: &[T], size: usize, epsilon: f64) -> bool {
    // Check diagonal elements for very small values
    for i in 0..size {
        let idx = i * size + i;
        if idx < matrix.len() {
            let val = KalmanScalar::to_f64(&matrix[idx]).abs();
            if val < epsilon {
                return true;
            }
        }
    }

    // Check condition number estimate
    let cond = matrix_condition_estimate(matrix, size);
    cond > 1.0 / epsilon
}

/// Calculate the determinant of a 2x2 or 3x3 matrix (for small matrices only)
pub fn small_determinant<T: KalmanScalar>(matrix: &[T], size: usize) -> Option<f64> {
    match size {
        1 => {
            if !matrix.is_empty() {
                Some(KalmanScalar::to_f64(&matrix[0]))
            } else {
                None
            }
        }
        2 => {
            if matrix.len() >= 4 {
                let a = KalmanScalar::to_f64(&matrix[0]);
                let b = KalmanScalar::to_f64(&matrix[1]);
                let c = KalmanScalar::to_f64(&matrix[2]);
                let d = KalmanScalar::to_f64(&matrix[3]);
                Some(a * d - b * c)
            } else {
                None
            }
        }
        3 => {
            if matrix.len() >= 9 {
                let a = KalmanScalar::to_f64(&matrix[0]);
                let b = KalmanScalar::to_f64(&matrix[1]);
                let c = KalmanScalar::to_f64(&matrix[2]);
                let d = KalmanScalar::to_f64(&matrix[3]);
                let e = KalmanScalar::to_f64(&matrix[4]);
                let f = KalmanScalar::to_f64(&matrix[5]);
                let g = KalmanScalar::to_f64(&matrix[6]);
                let h = KalmanScalar::to_f64(&matrix[7]);
                let i = KalmanScalar::to_f64(&matrix[8]);

                Some(a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g))
            } else {
                None
            }
        }
        _ => None, // Don't compute for larger matrices
    }
}

/// Format innovation/residual for logging
pub fn format_innovation<'a, T: KalmanScalar>(innovation: &'a [T]) -> impl fmt::Display + 'a {
    InnovationFormatter { innovation }
}

struct InnovationFormatter<'a, T: KalmanScalar> {
    innovation: &'a [T],
}

impl<T: KalmanScalar> fmt::Display for InnovationFormatter<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "innovation=[")?;
        for (i, val) in self.innovation.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "{:.4}", KalmanScalar::to_f64(val))?;
        }
        write!(f, "], norm={:.4}", state_norm(self.innovation))
    }
}

/// Log filter dimensions for initialization
pub fn log_filter_dimensions(state_dim: usize, measurement_dim: usize, control_dim: Option<usize>) {
    if let Some(control) = control_dim {
        log::info!(
            "Initializing Kalman filter: state_dim={}, measurement_dim={}, control_dim={}",
            state_dim,
            measurement_dim,
            control
        );
    } else {
        log::info!(
            "Initializing Kalman filter: state_dim={}, measurement_dim={}",
            state_dim,
            measurement_dim
        );
    }
}

/// Check and log numerical stability warnings
pub fn check_numerical_stability<T: KalmanScalar>(covariance: &[T], size: usize, context: &str) {
    const EPSILON: f64 = 1e-10;

    if is_near_singular(covariance, size, EPSILON) {
        log::warn!(
            "{}: Covariance matrix may be near-singular (condition number > {:.2e})",
            context,
            1.0 / EPSILON
        );

        if log::log_enabled!(log::Level::Debug) {
            if let Some(det) = small_determinant(covariance, size) {
                log::debug!("{}: Determinant = {:.6e}", context, det);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_matrix_formatter() {
        let matrix = vec![1.0, 2.0, 3.0, 4.0];
        let formatted = format!("{}", format_matrix(&matrix, 2, 2, "test"));
        assert!(formatted.contains("test=["));
        assert!(formatted.contains("1.0"));
    }

    #[test]
    fn test_state_norm() {
        let state = vec![3.0, 4.0];
        assert_eq!(state_norm(&state), 5.0);
    }

    #[test]
    fn test_determinant_2x2() {
        let matrix = vec![1.0, 2.0, 3.0, 4.0];
        assert_eq!(small_determinant(&matrix, 2), Some(-2.0));
    }

    #[test]
    fn test_near_singular_detection() {
        let singular = vec![1.0, 0.0, 0.0, 0.0];
        assert!(is_near_singular(&singular, 2, 1e-10));

        let regular = vec![1.0, 0.0, 0.0, 1.0];
        assert!(!is_near_singular(&regular, 2, 1e-10));
    }
}
