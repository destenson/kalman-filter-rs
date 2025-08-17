//! Builder for Information Filter

use crate::information::InformationFilter;
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use log::{debug, error, info};

/// Builder for constructing Information Filters
///
/// # Example
/// ```no_run
/// use kalman_filter::builders::InformationFilterBuilder;
/// 
/// let if_filter = InformationFilterBuilder::new(2, 1)
///     .initial_information_matrix(vec![1.0, 0.0, 0.0, 1.0])
///     .initial_information_vector(vec![0.0, 0.0])
///     .state_transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
///     .process_noise(vec![0.01, 0.0, 0.0, 0.01])
///     .observation_matrix(vec![1.0, 0.0])
///     .measurement_noise(vec![0.1])
///     .build()
///     .unwrap();
/// ```
pub struct InformationFilterBuilder<T: KalmanScalar> {
    state_dim: usize,
    measurement_dim: usize,
    initial_information_matrix: Option<Vec<T>>,
    initial_information_vector: Option<Vec<T>>,
    state_transition_matrix: Option<Vec<T>>,
    process_noise: Option<Vec<T>>,
    observation_matrix: Option<Vec<T>>,
    measurement_noise: Option<Vec<T>>,
}

impl<T: KalmanScalar> InformationFilterBuilder<T> {
    /// Create a new builder with the given dimensions
    pub fn new(state_dim: usize, measurement_dim: usize) -> Self {
        debug!(
            "Creating InformationFilterBuilder: state_dim={}, measurement_dim={}",
            state_dim, measurement_dim
        );
        Self {
            state_dim,
            measurement_dim,
            initial_information_matrix: None,
            initial_information_vector: None,
            state_transition_matrix: None,
            process_noise: None,
            observation_matrix: None,
            measurement_noise: None,
        }
    }

    /// Create builder from initial state and covariance
    pub fn from_state_covariance(
        initial_state: Vec<T>,
        initial_covariance: Vec<T>,
        measurement_dim: usize,
    ) -> KalmanResult<Self> {
        let state_dim = initial_state.len();
        
        debug!(
            "Creating InformationFilterBuilder from state/covariance: state_dim={}, measurement_dim={}",
            state_dim, measurement_dim
        );

        // Convert to information form using matrix inversion
        use crate::filter::KalmanFilter;
        let Y = KalmanFilter::<T>::invert_matrix(&initial_covariance, state_dim)?;
        
        // y = Y * x
        let mut y = vec![T::zero(); state_dim];
        for i in 0..state_dim {
            for j in 0..state_dim {
                y[i] = y[i] + Y[i * state_dim + j] * initial_state[j];
            }
        }

        Ok(Self {
            state_dim,
            measurement_dim,
            initial_information_matrix: Some(Y),
            initial_information_vector: Some(y),
            state_transition_matrix: None,
            process_noise: None,
            observation_matrix: None,
            measurement_noise: None,
        })
    }

    /// Set the initial information matrix Y = P^-1 (row-major), and vector y = Y * x
    pub fn initial_information(mut self, matrix: Vec<T>, vector: Vec<T>) -> Self {
        self.initial_information_matrix = Some(matrix);
        self.initial_information_vector = Some(vector);
        self
    }

    /// Set the initial information matrix Y = P^-1 (row-major)
    pub fn initial_information_matrix(mut self, matrix: Vec<T>) -> Self {
        self.initial_information_matrix = Some(matrix);
        self
    }

    /// Set the initial information vector y = Y * x
    pub fn initial_information_vector(mut self, vector: Vec<T>) -> Self {
        self.initial_information_vector = Some(vector);
        self
    }

    /// Set the state transition matrix F (row-major)
    pub fn state_transition_matrix(mut self, matrix: Vec<T>) -> Self {
        self.state_transition_matrix = Some(matrix);
        self
    }

    /// Set the process noise covariance Q (row-major)
    pub fn process_noise(mut self, noise: Vec<T>) -> Self {
        self.process_noise = Some(noise);
        self
    }

    /// Set the observation matrix H (row-major)
    pub fn observation_matrix(mut self, matrix: Vec<T>) -> Self {
        self.observation_matrix = Some(matrix);
        self
    }

    /// Set the measurement noise covariance R (row-major)
    pub fn measurement_noise(mut self, noise: Vec<T>) -> Self {
        self.measurement_noise = Some(noise);
        self
    }

    /// Build the Information Filter
    pub fn build(self) -> KalmanResult<InformationFilter<T>> {
        info!(
            "Building Information Filter: state_dim={}, measurement_dim={}",
            self.state_dim, self.measurement_dim
        );

        // Extract required parameters
        let initial_Y = self.initial_information_matrix.ok_or_else(|| {
            error!("InformationFilterBuilder: missing initial_information_matrix");
            KalmanError::BuilderIncomplete("initial_information_matrix".to_string())
        })?;

        let initial_y = self.initial_information_vector.ok_or_else(|| {
            error!("InformationFilterBuilder: missing initial_information_vector");
            KalmanError::BuilderIncomplete("initial_information_vector".to_string())
        })?;

        let F = self.state_transition_matrix.ok_or_else(|| {
            error!("InformationFilterBuilder: missing state_transition_matrix");
            KalmanError::BuilderIncomplete("state_transition_matrix".to_string())
        })?;

        let Q = self.process_noise.ok_or_else(|| {
            error!("InformationFilterBuilder: missing process_noise");
            KalmanError::BuilderIncomplete("process_noise".to_string())
        })?;

        let H = self.observation_matrix.ok_or_else(|| {
            error!("InformationFilterBuilder: missing observation_matrix");
            KalmanError::BuilderIncomplete("observation_matrix".to_string())
        })?;

        let R = self.measurement_noise.ok_or_else(|| {
            error!("InformationFilterBuilder: missing measurement_noise");
            KalmanError::BuilderIncomplete("measurement_noise".to_string())
        })?;

        // Call the initialize method
        InformationFilter::initialize(
            self.state_dim,
            self.measurement_dim,
            initial_Y,
            initial_y,
            F,
            Q,
            H,
            R,
        )
    }
}
