//! Builder for Particle Filter

use crate::particle::{ParticleFilter, ResamplingStrategy};
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use log::{debug, error, info};

/// Builder for constructing Particle Filters
///
/// # Example
/// ```no_run
/// use kalman_filters::builders::ParticleFilterBuilder;
/// 
/// let pf = ParticleFilterBuilder::new(2, 100)
///     .initial_mean(vec![0.0, 0.0])
///     .initial_std(vec![1.0, 1.0])
///     .process_noise_std(vec![0.1, 0.1])
///     .measurement_noise_std(vec![0.5])
///     .dt(0.01)
///     .build()
///     .unwrap();
/// ```
pub struct ParticleFilterBuilder<T: KalmanScalar> {
    state_dim: usize,
    num_particles: usize,
    initial_mean: Option<Vec<T>>,
    initial_std: Option<Vec<T>>,
    process_noise_std: Option<Vec<T>>,
    measurement_noise_std: Option<Vec<T>>,
    dt: Option<T>,
    resampling_strategy: ResamplingStrategy,
    ess_threshold: Option<T>,
}

impl<T: KalmanScalar> ParticleFilterBuilder<T> {
    /// Create a new builder with the given state dimension and number of particles
    pub fn new(state_dim: usize, num_particles: usize) -> Self {
        debug!(
            "Creating ParticleFilterBuilder: state_dim={}, num_particles={}",
            state_dim, num_particles
        );
        Self {
            state_dim,
            num_particles,
            initial_mean: None,
            initial_std: None,
            process_noise_std: None,
            measurement_noise_std: None,
            dt: None,
            resampling_strategy: ResamplingStrategy::Systematic,
            ess_threshold: None,
        }
    }

    /// Set the initial mean state vector
    pub fn initial_mean(mut self, mean: Vec<T>) -> Self {
        self.initial_mean = Some(mean);
        self
    }

    /// Set the initial standard deviation for each state component
    pub fn initial_std(mut self, std: Vec<T>) -> Self {
        self.initial_std = Some(std);
        self
    }

    /// Set the process noise standard deviation for each state component
    pub fn process_noise_std(mut self, std: Vec<T>) -> Self {
        self.process_noise_std = Some(std);
        self
    }

    /// Set the measurement noise standard deviation for each measurement component
    pub fn measurement_noise_std(mut self, std: Vec<T>) -> Self {
        self.measurement_noise_std = Some(std);
        self
    }

    /// Set the time step
    pub fn dt(mut self, dt: T) -> Self {
        self.dt = Some(dt);
        self
    }

    /// Set the resampling strategy
    pub fn resampling_strategy(mut self, strategy: ResamplingStrategy) -> Self {
        self.resampling_strategy = strategy;
        self
    }

    /// Set the effective sample size threshold for resampling
    pub fn ess_threshold(mut self, threshold: T) -> Self {
        self.ess_threshold = Some(threshold);
        self
    }

    /// Build the Particle Filter
    pub fn build(self) -> KalmanResult<ParticleFilter<T>> {
        info!(
            "Building Particle Filter: state_dim={}, num_particles={}",
            self.state_dim, self.num_particles
        );

        // Extract required parameters
        let initial_mean = self.initial_mean.ok_or_else(|| {
            error!("ParticleFilterBuilder: missing initial_mean");
            KalmanError::BuilderIncomplete("initial_mean".to_string())
        })?;

        let initial_std = self.initial_std.ok_or_else(|| {
            error!("ParticleFilterBuilder: missing initial_std");
            KalmanError::BuilderIncomplete("initial_std".to_string())
        })?;

        let process_noise_std = self.process_noise_std.ok_or_else(|| {
            error!("ParticleFilterBuilder: missing process_noise_std");
            KalmanError::BuilderIncomplete("process_noise_std".to_string())
        })?;

        let measurement_noise_std = self.measurement_noise_std.ok_or_else(|| {
            error!("ParticleFilterBuilder: missing measurement_noise_std");
            KalmanError::BuilderIncomplete("measurement_noise_std".to_string())
        })?;

        let dt = self.dt.ok_or_else(|| {
            error!("ParticleFilterBuilder: missing dt");
            KalmanError::BuilderIncomplete("dt".to_string())
        })?;

        // Call the initialize method
        let mut pf = ParticleFilter::initialize(
            self.state_dim,
            self.num_particles,
            initial_mean,
            initial_std,
            process_noise_std,
            measurement_noise_std,
            dt,
        )?;

        // Set optional parameters
        pf.set_resampling_strategy(self.resampling_strategy);
        
        if let Some(threshold) = self.ess_threshold {
            pf.ess_threshold = threshold;
        }

        Ok(pf)
    }
}
