//! Builder for Cubature Kalman Filter

use crate::scented::CubatureKalmanFilter;
use crate::types::{KalmanError, KalmanResult, KalmanScalar, NonlinearSystem};
use log::{debug, error, info};

/// Builder for constructing Cubature Kalman Filters
///
/// # Example
/// ```no_run
/// use kalman_filters::builders::CubatureKalmanFilterBuilder;
/// use kalman_filters::NonlinearSystem;
///
/// # struct MySystem;
/// # impl NonlinearSystem<f64> for MySystem {
/// #     fn state_dim(&self) -> usize { 2 }
/// #     fn measurement_dim(&self) -> usize { 1 }
/// #     fn state_transition(&self, x: &[f64], u: Option<&[f64]>, dt: f64) -> Vec<f64> { vec![0.0; 2] }
/// #     fn measurement(&self, x: &[f64]) -> Vec<f64> { vec![0.0] }
/// #     fn state_jacobian(&self, x: &[f64], u: Option<&[f64]>, dt: f64) -> Vec<f64> { vec![0.0; 4] }
/// #     fn measurement_jacobian(&self, x: &[f64]) -> Vec<f64> { vec![0.0; 2] }
/// # }
/// let system = MySystem;
/// let ckf = CubatureKalmanFilterBuilder::new(system)
///     .initial_state(vec![0.0, 0.0])
///     .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
///     .process_noise(vec![0.01, 0.0, 0.0, 0.01])
///     .measurement_noise(vec![0.1])
///     .dt(0.01)
///     .build()
///     .unwrap();
/// ```
pub struct CubatureKalmanFilterBuilder<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    system: S,
    initial_state: Option<Vec<T>>,
    initial_covariance: Option<Vec<T>>,
    process_noise: Option<Vec<T>>,
    measurement_noise: Option<Vec<T>>,
    dt: Option<T>,
    control: Option<Vec<T>>,
}

impl<T, S> CubatureKalmanFilterBuilder<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// Create a new builder with the nonlinear system
    pub fn new(system: S) -> Self {
        debug!(
            "Creating CubatureKalmanFilterBuilder: state_dim={}, measurement_dim={}",
            system.state_dim(),
            system.measurement_dim()
        );
        Self {
            system,
            initial_state: None,
            initial_covariance: None,
            process_noise: None,
            measurement_noise: None,
            dt: None,
            control: None,
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

    /// Set the process noise covariance (Q) (row-major)
    pub fn process_noise(mut self, noise: Vec<T>) -> Self {
        self.process_noise = Some(noise);
        self
    }

    /// Set the measurement noise covariance (R) (row-major)
    pub fn measurement_noise(mut self, noise: Vec<T>) -> Self {
        self.measurement_noise = Some(noise);
        self
    }

    /// Set the time step
    pub fn dt(mut self, dt: T) -> Self {
        self.dt = Some(dt);
        self
    }

    /// Set the control input
    pub fn control(mut self, control: Vec<T>) -> Self {
        self.control = Some(control);
        self
    }

    /// Build the Cubature Kalman Filter
    pub fn build(self) -> KalmanResult<CubatureKalmanFilter<T, S>> {
        let n = self.system.state_dim();
        let m = self.system.measurement_dim();

        debug!(
            "Building Cubature Kalman Filter: state_dim={}, measurement_dim={}",
            n, m
        );

        // Extract required parameters
        let initial_state = self.initial_state.ok_or_else(|| {
            error!("CubatureKalmanFilterBuilder: missing initial_state");
            KalmanError::BuilderIncomplete("initial_state".to_string())
        })?;

        let initial_covariance = self.initial_covariance.ok_or_else(|| {
            error!("CubatureKalmanFilterBuilder: missing initial_covariance");
            KalmanError::BuilderIncomplete("initial_covariance".to_string())
        })?;

        let process_noise = self.process_noise.ok_or_else(|| {
            error!("CubatureKalmanFilterBuilder: missing process_noise");
            KalmanError::BuilderIncomplete("process_noise".to_string())
        })?;

        let measurement_noise = self.measurement_noise.ok_or_else(|| {
            error!("CubatureKalmanFilterBuilder: missing measurement_noise");
            KalmanError::BuilderIncomplete("measurement_noise".to_string())
        })?;

        let dt = self.dt.ok_or_else(|| {
            error!("CubatureKalmanFilterBuilder: missing dt");
            KalmanError::BuilderIncomplete("dt".to_string())
        })?;

        // Call the initialize method
        CubatureKalmanFilter::initialize(
            self.system,
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise,
            dt,
            self.control,
        )
    }
}
