//! Builder for Ensemble Kalman Filter

use crate::ensemble::EnsembleKalmanFilter;
use crate::types::{KalmanError, KalmanResult, KalmanScalar, NonlinearSystem};
use log::{debug, error, info};

/// Builder for constructing Ensemble Kalman Filters
///
/// # Example
/// ```no_run
/// use kalman_filters::builders::EnsembleKalmanFilterBuilder;
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
/// let enkf = EnsembleKalmanFilterBuilder::new(system)
///     .initial_mean(vec![0.0, 0.0])
///     .initial_spread(vec![1.0, 1.0])
///     .ensemble_size(100)
///     .process_noise(vec![0.01, 0.0, 0.0, 0.01])
///     .measurement_noise(vec![0.1])
///     .dt(0.01)
///     .build()
///     .unwrap();
/// ```
pub struct EnsembleKalmanFilterBuilder<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    system: S,
    initial_mean: Option<Vec<T>>,
    initial_spread: Option<Vec<T>>,
    ensemble_size: Option<usize>,
    process_noise: Option<Vec<T>>,
    measurement_noise: Option<Vec<T>>,
    dt: Option<T>,
    inflation_factor: T,
    localization_radius: T,
    control: Option<Vec<T>>,
}

impl<T, S> EnsembleKalmanFilterBuilder<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// Create a new builder with the nonlinear system
    pub fn new(system: S) -> Self {
        debug!(
            "Creating EnsembleKalmanFilterBuilder: state_dim={}, measurement_dim={}",
            system.state_dim(),
            system.measurement_dim()
        );
        Self {
            system,
            initial_mean: None,
            initial_spread: None,
            ensemble_size: None,
            process_noise: None,
            measurement_noise: None,
            dt: None,
            inflation_factor: T::from(1.05).unwrap_or_else(|| T::from(1.05f64).unwrap()), // 5% default inflation
            localization_radius: T::zero(),
            control: None,
        }
    }

    /// Set the initial mean state vector
    pub fn initial_mean(mut self, mean: Vec<T>) -> Self {
        self.initial_mean = Some(mean);
        self
    }

    /// Set the initial spread (standard deviation) for each state component
    pub fn initial_spread(mut self, spread: Vec<T>) -> Self {
        self.initial_spread = Some(spread);
        self
    }

    /// Set the ensemble size (number of particles)
    pub fn ensemble_size(mut self, size: usize) -> Self {
        self.ensemble_size = Some(size);
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

    /// Set the multiplicative inflation factor (> 1.0)
    pub fn inflation_factor(mut self, factor: T) -> Self {
        self.inflation_factor = factor;
        self
    }

    /// Set the localization radius (0 = no localization)
    pub fn localization_radius(mut self, radius: T) -> Self {
        self.localization_radius = radius;
        self
    }

    /// Set the control input
    pub fn control(mut self, control: Vec<T>) -> Self {
        self.control = Some(control);
        self
    }

    /// Build the Ensemble Kalman Filter
    pub fn build(self) -> KalmanResult<EnsembleKalmanFilter<T, S>> {
        let n = self.system.state_dim();
        let m = self.system.measurement_dim();
        
        info!(
            "Building Ensemble Kalman Filter: state_dim={}, measurement_dim={}",
            n, m
        );

        // Extract required parameters
        let initial_mean = self.initial_mean.ok_or_else(|| {
            error!("EnsembleKalmanFilterBuilder: missing initial_mean");
            KalmanError::BuilderIncomplete("initial_mean".to_string())
        })?;

        let initial_spread = self.initial_spread.ok_or_else(|| {
            error!("EnsembleKalmanFilterBuilder: missing initial_spread");
            KalmanError::BuilderIncomplete("initial_spread".to_string())
        })?;

        let ensemble_size = self.ensemble_size.ok_or_else(|| {
            error!("EnsembleKalmanFilterBuilder: missing ensemble_size");
            KalmanError::BuilderIncomplete("ensemble_size".to_string())
        })?;

        let process_noise = self.process_noise.ok_or_else(|| {
            error!("EnsembleKalmanFilterBuilder: missing process_noise");
            KalmanError::BuilderIncomplete("process_noise".to_string())
        })?;

        let measurement_noise = self.measurement_noise.ok_or_else(|| {
            error!("EnsembleKalmanFilterBuilder: missing measurement_noise");
            KalmanError::BuilderIncomplete("measurement_noise".to_string())
        })?;

        let dt = self.dt.ok_or_else(|| {
            error!("EnsembleKalmanFilterBuilder: missing dt");
            KalmanError::BuilderIncomplete("dt".to_string())
        })?;

        // Call the initialize method
        EnsembleKalmanFilter::initialize(
            self.system,
            initial_mean,
            initial_spread,
            ensemble_size,
            process_noise,
            measurement_noise,
            dt,
            self.inflation_factor,
            self.localization_radius,
            self.control,
        )
    }
}
