//! Builder pattern for Kalman filter construction

use crate::filter::KalmanFilter;
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use log::{debug, error, info, warn};

/// Builder for constructing Kalman filters
///
/// Provides a convenient way to construct Kalman filters with validation
///
/// # Example
/// ```
/// use kalman_filter::KalmanFilterBuilder;
///
/// let kf = KalmanFilterBuilder::<f64>::new(2, 1)  // 2 states, 1 measurement
///     .initial_state(vec![0.0, 0.0])
///     .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
///     .transition_matrix(vec![1.0, 1.0, 0.0, 1.0])
///     .process_noise(vec![0.01, 0.0, 0.0, 0.01])
///     .observation_matrix(vec![1.0, 0.0])
///     .measurement_noise(vec![0.1])
///     .build()
///     .unwrap();
/// ```
pub struct KalmanFilterBuilder<T>
where
    T: KalmanScalar,
{
    state_dim: usize,
    measurement_dim: usize,
    initial_state: Option<Vec<T>>,
    initial_covariance: Option<Vec<T>>,
    transition_matrix: Option<Vec<T>>,
    process_noise: Option<Vec<T>>,
    observation_matrix: Option<Vec<T>>,
    measurement_noise: Option<Vec<T>>,
}

impl<T> KalmanFilterBuilder<T>
where
    T: KalmanScalar,
{
    /// Create a new builder with specified dimensions
    pub fn new(state_dim: usize, measurement_dim: usize) -> Self {
        debug!(
            "Creating KalmanFilterBuilder: state_dim={}, measurement_dim={}",
            state_dim, measurement_dim
        );
        Self {
            state_dim,
            measurement_dim,
            initial_state: None,
            initial_covariance: None,
            transition_matrix: None,
            process_noise: None,
            observation_matrix: None,
            measurement_noise: None,
        }
    }

    /// Set the initial state vector
    pub fn initial_state(mut self, state: Vec<T>) -> Self {
        self.initial_state = Some(state);
        self
    }

    /// Set the initial state covariance matrix (row-major)
    pub fn initial_covariance(mut self, covariance: Vec<T>) -> Self {
        self.initial_covariance = Some(covariance);
        self
    }

    /// Set the state transition matrix (F) (row-major)
    pub fn transition_matrix(mut self, matrix: Vec<T>) -> Self {
        self.transition_matrix = Some(matrix);
        self
    }

    /// Set the process noise covariance (Q) (row-major)
    pub fn process_noise(mut self, noise: Vec<T>) -> Self {
        self.process_noise = Some(noise);
        self
    }

    /// Set the observation matrix (H) (row-major)
    pub fn observation_matrix(mut self, matrix: Vec<T>) -> Self {
        self.observation_matrix = Some(matrix);
        self
    }

    /// Set the measurement noise covariance (R) (row-major)
    pub fn measurement_noise(mut self, noise: Vec<T>) -> Self {
        self.measurement_noise = Some(noise);
        self
    }

    /// Build the Kalman filter
    pub fn build(self) -> KalmanResult<KalmanFilter<T>> {
        info!(
            "Building Kalman filter: state_dim={}, measurement_dim={}",
            self.state_dim, self.measurement_dim
        );
        let initial_state = self.initial_state.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing initial_state");
            KalmanError::BuilderIncomplete("initial_state".to_string())
        })?;

        let initial_covariance = self.initial_covariance.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing initial_covariance");
            KalmanError::BuilderIncomplete("initial_covariance".to_string())
        })?;

        let transition_matrix = self.transition_matrix.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing transition_matrix");
            KalmanError::BuilderIncomplete("transition_matrix".to_string())
        })?;

        let process_noise = self.process_noise.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing process_noise");
            KalmanError::BuilderIncomplete("process_noise".to_string())
        })?;

        let observation_matrix = self.observation_matrix.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing observation_matrix");
            KalmanError::BuilderIncomplete("observation_matrix".to_string())
        })?;

        let measurement_noise = self.measurement_noise.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing measurement_noise");
            KalmanError::BuilderIncomplete("measurement_noise".to_string())
        })?;

        // Validate covariance matrices are symmetric
        let n = self.state_dim;
        let m = self.measurement_dim;

        if !is_symmetric(&initial_covariance, n) {
            error!(
                "KalmanFilterBuilder: initial_covariance is not symmetric (size={}x{})",
                n, n
            );
            return Err(KalmanError::InvalidCovariance);
        }
        debug!("Initial covariance matrix validated as symmetric");
        if !is_symmetric(&process_noise, n) {
            error!(
                "KalmanFilterBuilder: process_noise is not symmetric (size={}x{})",
                n, n
            );
            return Err(KalmanError::InvalidNoiseCovariance);
        }
        debug!("Process noise matrix validated as symmetric");
        if !is_symmetric(&measurement_noise, m) {
            error!(
                "KalmanFilterBuilder: measurement_noise is not symmetric (size={}x{})",
                m, m
            );
            return Err(KalmanError::InvalidNoiseCovariance);
        }
        debug!("Measurement noise matrix validated as symmetric");

        KalmanFilter::initialize(
            self.state_dim,
            self.measurement_dim,
            initial_state,
            initial_covariance,
            transition_matrix,
            process_noise,
            observation_matrix,
            measurement_noise,
        )
    }
}

impl<T> Default for KalmanFilterBuilder<T>
where
    T: KalmanScalar,
{
    fn default() -> Self {
        Self::new(1, 1)
    }
}

fn is_symmetric<T>(matrix: &[T], size: usize) -> bool
where
    T: KalmanScalar,
{
    for i in 0..size {
        for j in i + 1..size {
            let diff = (matrix[i * size + j] - matrix[j * size + i]).abs();
            if diff > <T as KalmanScalar>::epsilon() {
                warn!(
                    "Matrix symmetry check failed at ({},{}) and ({},{}): diff={:.6e}",
                    i,
                    j,
                    j,
                    i,
                    KalmanScalar::to_f64(&diff)
                );
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_complete() {
        let result = KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
            .transition_matrix(vec![1.0, 1.0, 0.0, 1.0])
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .observation_matrix(vec![1.0, 0.0])
            .measurement_noise(vec![0.1])
            .build();

        assert!(result.is_ok());
    }

    #[test]
    fn test_builder_incomplete() {
        let result = KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .build();

        assert!(result.is_err());
        match result {
            Err(KalmanError::BuilderIncomplete(field)) => {
                assert_eq!(field, "initial_covariance");
            }
            _ => panic!("Expected BuilderIncomplete error"),
        }
    }

    #[test]
    fn test_builder_invalid_covariance() {
        let result = KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .initial_covariance(vec![1.0, 2.0, 0.0, 1.0]) // Not symmetric
            .transition_matrix(vec![1.0, 1.0, 0.0, 1.0])
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .observation_matrix(vec![1.0, 0.0])
            .measurement_noise(vec![0.1])
            .build();

        assert!(result.is_err());
        match result {
            Err(KalmanError::InvalidCovariance) => {}
            _ => panic!("Expected InvalidCovariance error"),
        }
    }
}
