//! Core Particle Filter implementation
//!
//! Sequential Monte Carlo methods for state estimation in nonlinear,
//! non-Gaussian systems using weighted particle samples.
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use crate::logging::{format_state, log_filter_dimensions, state_norm};
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use log::{debug, error, info, trace, warn};
use num_traits::{One, Zero};
use rand::distributions::{Distribution, Uniform};
use rand::{thread_rng, Rng};
use rand_distr::Normal;
use std::f64::consts::PI;

/// A single particle representing a state hypothesis
#[derive(Debug, Clone)]
pub struct Particle<T: KalmanScalar> {
    /// State vector
    pub state: Vec<T>,
    /// Weight (probability)
    pub weight: T,
    /// Log weight for numerical stability
    pub log_weight: T,
}

/// Resampling strategy for particle filter
#[derive(Debug, Clone, Copy)]
pub enum ResamplingStrategy {
    /// Simple multinomial resampling
    Multinomial,
    /// Systematic resampling (lower variance)
    Systematic,
    /// Stratified resampling
    Stratified,
    /// Residual resampling
    Residual,
}

/// Particle Filter for Sequential Monte Carlo estimation
///
/// The particle filter represents the posterior distribution using
/// a set of weighted samples (particles), avoiding the Gaussian
/// assumption of traditional Kalman filters.
pub struct ParticleFilter<T>
where
    T: KalmanScalar,
{
    /// State dimension
    pub state_dim: usize,
    /// Number of particles
    pub num_particles: usize,
    /// Particle set
    pub particles: Vec<Particle<T>>,
    /// Process noise standard deviation
    pub process_noise_std: Vec<T>,
    /// Measurement noise standard deviation  
    pub measurement_noise_std: Vec<T>,
    /// Resampling strategy
    pub resampling_strategy: ResamplingStrategy,
    /// Effective sample size threshold for resampling
    pub ess_threshold: T,
    /// Time step
    pub dt: T,
}

impl<T> ParticleFilter<T>
where
    T: KalmanScalar,
{
    /// Create a new Particle Filter
    pub fn new(
        state_dim: usize,
        num_particles: usize,
        initial_mean: Vec<T>,
        initial_std: Vec<T>,
        process_noise_std: Vec<T>,
        measurement_noise_std: Vec<T>,
        dt: T,
    ) -> KalmanResult<Self> {
        log_filter_dimensions(state_dim, measurement_noise_std.len(), None);
        info!(
            "Particle Filter: Initializing with {} particles",
            num_particles
        );

        // Validate dimensions
        if initial_mean.len() != state_dim {
            error!(
                "Particle Filter: initial mean dimension mismatch: expected {}x1, got {}x1",
                state_dim,
                initial_mean.len()
            );
            return Err(KalmanError::DimensionMismatch {
                expected: (state_dim, 1),
                actual: (initial_mean.len(), 1),
            });
        }
        if initial_std.len() != state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (state_dim, 1),
                actual: (initial_std.len(), 1),
            });
        }
        if process_noise_std.len() != state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (state_dim, 1),
                actual: (process_noise_std.len(), 1),
            });
        }

        // Initialize particles from initial distribution
        let mut particles = Vec::with_capacity(num_particles);
        let mut rng = thread_rng();
        let initial_weight = T::one() / T::from(num_particles).unwrap();
        let initial_log_weight = initial_weight.ln();

        for _ in 0..num_particles {
            let mut state = vec![T::zero(); state_dim];
            for i in 0..state_dim {
                let mean_f64 = KalmanScalar::to_f64(&initial_mean[i]);
                let std_f64 = KalmanScalar::to_f64(&initial_std[i]);

                if std_f64 > 0.0f64 {
                    let normal = Normal::new(mean_f64, std_f64).unwrap();
                    state[i] = T::from(normal.sample(&mut rng)).unwrap();
                } else {
                    state[i] = initial_mean[i];
                }
            }

            particles.push(Particle {
                state,
                weight: initial_weight,
                log_weight: initial_log_weight,
            });
        }

        Ok(Self {
            state_dim,
            num_particles,
            particles,
            process_noise_std,
            measurement_noise_std,
            resampling_strategy: ResamplingStrategy::Systematic,
            ess_threshold: T::from(num_particles / 2).unwrap(),
            dt,
        })
    }

    /// Set resampling strategy
    pub fn set_resampling_strategy(&mut self, strategy: ResamplingStrategy) {
        self.resampling_strategy = strategy;
    }

    /// Predict step: propagate particles through motion model
    pub fn predict<F>(&mut self, motion_model: F)
    where
        F: Fn(&[T], T) -> Vec<T>,
    {
        #[cfg(feature = "prometheus-metrics")]
        let _timer = crate::metrics::MetricsTimer::start();

        let mean_state = self.mean();
        debug!(
            "Particle Filter predict: {} prior={:.4}",
            format_state(&mean_state, "mean_state"),
            state_norm(&mean_state)
        );

        let mut rng = thread_rng();

        for particle in &mut self.particles {
            // Apply motion model
            let mut new_state = motion_model(&particle.state, self.dt);

            // Add process noise
            for i in 0..self.state_dim {
                let noise_std = KalmanScalar::to_f64(&self.process_noise_std[i]);
                if noise_std > 0.0f64 {
                    let normal = Normal::new(0.0, noise_std).unwrap();
                    new_state[i] = new_state[i] + T::from(normal.sample(&mut rng)).unwrap();
                }
            }

            particle.state = new_state;
        }

        let new_mean_state = self.mean();
        debug!(
            "Particle Filter predict: {} posterior={:.4}",
            format_state(&new_mean_state, "mean_state"),
            state_norm(&new_mean_state)
        );

        #[cfg(feature = "prometheus-metrics")]
        {
            _timer.finish_predict("pf");
            crate::metrics::record_prediction("pf");
            crate::metrics::set_state_dimension("pf", self.state_dim);
        }
    }

    /// Update step: update particle weights based on measurement likelihood
    pub fn update<F>(&mut self, measurement: &[T], likelihood_fn: F) -> KalmanResult<()>
    where
        F: Fn(&[T], &[T]) -> T,
    {
        #[cfg(feature = "prometheus-metrics")]
        let _timer = crate::metrics::MetricsTimer::start();

        debug!(
            "Particle Filter update: {}",
            format_state(measurement, "measurement")
        );

        // Update weights based on measurement likelihood
        let mut max_log_weight = T::from(-1e308).unwrap();

        for particle in &mut self.particles {
            // Compute likelihood
            let likelihood = likelihood_fn(&particle.state, measurement);

            if likelihood <= T::zero() {
                particle.log_weight = T::from(-1e308).unwrap();
            } else {
                particle.log_weight = particle.log_weight + likelihood.ln();
            }

            if particle.log_weight > max_log_weight {
                max_log_weight = particle.log_weight;
            }
        }

        // Normalize weights using log-sum-exp trick for numerical stability
        let mut log_sum = T::zero();
        for particle in &self.particles {
            log_sum = log_sum + (particle.log_weight - max_log_weight).exp();
        }
        log_sum = max_log_weight + log_sum.ln();

        // Convert back to regular weights
        for particle in &mut self.particles {
            particle.weight = (particle.log_weight - log_sum).exp();
        }

        // Check for particle degeneracy and resample if needed
        let ess = self.effective_sample_size();
        debug!(
            "Particle Filter update: ESS={:.2}/{}, threshold={:.2}",
            KalmanScalar::to_f64(&ess),
            self.num_particles,
            KalmanScalar::to_f64(&self.ess_threshold)
        );

        if ess < self.ess_threshold {
            debug!("Particle Filter: Resampling triggered (ESS below threshold)");
            self.resample()?;
        }

        let final_mean_state = self.mean();
        debug!(
            "Particle Filter update: {} posterior={:.4}",
            format_state(&final_mean_state, "mean_state"),
            state_norm(&final_mean_state)
        );

        #[cfg(feature = "prometheus-metrics")]
        {
            _timer.finish_update("pf");
            crate::metrics::record_update("pf");
            crate::metrics::set_effective_particles("pf", KalmanScalar::to_f64(&ess));
        }

        Ok(())
    }

    /// Compute effective sample size (ESS)
    pub fn effective_sample_size(&self) -> T {
        let mut sum_squared = T::zero();
        for particle in &self.particles {
            sum_squared = sum_squared + particle.weight * particle.weight;
        }

        if sum_squared > T::zero() {
            T::one() / sum_squared
        } else {
            T::zero()
        }
    }

    /// Resample particles according to their weights
    pub fn resample(&mut self) -> KalmanResult<()> {
        trace!(
            "Particle Filter: Resampling using {:?} strategy",
            self.resampling_strategy
        );

        #[cfg(feature = "prometheus-metrics")]
        let strategy_str = match self.resampling_strategy {
            ResamplingStrategy::Multinomial => "multinomial",
            ResamplingStrategy::Systematic => "systematic",
            ResamplingStrategy::Stratified => "stratified",
            ResamplingStrategy::Residual => "residual",
        };

        let result = match self.resampling_strategy {
            ResamplingStrategy::Multinomial => self.multinomial_resample(),
            ResamplingStrategy::Systematic => self.systematic_resample(),
            ResamplingStrategy::Stratified => self.stratified_resample(),
            ResamplingStrategy::Residual => self.residual_resample(),
        };

        if result.is_ok() {
            debug!("Particle Filter: Resampling completed, weights reset to uniform");
            #[cfg(feature = "prometheus-metrics")]
            crate::metrics::record_resampling("pf", strategy_str);
        } else {
            error!("Particle Filter: Resampling failed");
        }

        result
    }

    /// Multinomial resampling
    fn multinomial_resample(&mut self) -> KalmanResult<()> {
        let mut rng = thread_rng();
        let mut new_particles = Vec::with_capacity(self.num_particles);

        // Build cumulative distribution
        let mut cumsum = vec![T::zero(); self.num_particles];
        cumsum[0] = self.particles[0].weight;
        for i in 1..self.num_particles {
            cumsum[i] = cumsum[i - 1] + self.particles[i].weight;
        }

        // Sample new particles
        let uniform_weight = T::one() / T::from(self.num_particles).unwrap();
        let log_weight = uniform_weight.ln();

        for _ in 0..self.num_particles {
            let u: f64 = rng.gen();
            let u_t = T::from(u).unwrap();

            // Binary search for particle
            let mut idx = 0;
            for (i, &cum_weight) in cumsum.iter().enumerate() {
                if u_t <= cum_weight {
                    idx = i;
                    break;
                }
            }

            new_particles.push(Particle {
                state: self.particles[idx].state.clone(),
                weight: uniform_weight,
                log_weight,
            });
        }

        self.particles = new_particles;
        Ok(())
    }

    /// Systematic resampling (lower variance)
    fn systematic_resample(&mut self) -> KalmanResult<()> {
        let mut rng = thread_rng();
        let mut new_particles = Vec::with_capacity(self.num_particles);

        // Build cumulative distribution
        let mut cumsum = vec![T::zero(); self.num_particles];
        cumsum[0] = self.particles[0].weight;
        for i in 1..self.num_particles {
            cumsum[i] = cumsum[i - 1] + self.particles[i].weight;
        }

        // Systematic resampling
        let step = T::one() / T::from(self.num_particles).unwrap();
        let u0: f64 = rng.gen::<f64>() / self.num_particles as f64;
        let uniform_weight = step;
        let log_weight = uniform_weight.ln();

        let mut j = 0;
        for i in 0..self.num_particles {
            let u = T::from(u0).unwrap() + step * T::from(i).unwrap();

            while j < self.num_particles - 1 && cumsum[j] < u {
                j += 1;
            }

            new_particles.push(Particle {
                state: self.particles[j].state.clone(),
                weight: uniform_weight,
                log_weight,
            });
        }

        self.particles = new_particles;
        Ok(())
    }

    /// Stratified resampling
    fn stratified_resample(&mut self) -> KalmanResult<()> {
        let mut rng = thread_rng();
        let mut new_particles = Vec::with_capacity(self.num_particles);

        // Build cumulative distribution
        let mut cumsum = vec![T::zero(); self.num_particles];
        cumsum[0] = self.particles[0].weight;
        for i in 1..self.num_particles {
            cumsum[i] = cumsum[i - 1] + self.particles[i].weight;
        }

        // Stratified resampling
        let step = T::one() / T::from(self.num_particles).unwrap();
        let uniform_weight = step;
        let log_weight = uniform_weight.ln();

        for i in 0..self.num_particles {
            let u: f64 = (i as f64 + rng.gen::<f64>()) / self.num_particles as f64;
            let u_t = T::from(u).unwrap();

            // Find particle
            let mut j = 0;
            while j < self.num_particles - 1 && cumsum[j] < u_t {
                j += 1;
            }

            new_particles.push(Particle {
                state: self.particles[j].state.clone(),
                weight: uniform_weight,
                log_weight,
            });
        }

        self.particles = new_particles;
        Ok(())
    }

    /// Residual resampling
    fn residual_resample(&mut self) -> KalmanResult<()> {
        let mut new_particles = Vec::with_capacity(self.num_particles);

        // Deterministic part
        let n_f64 = self.num_particles as f64;
        let mut residuals = Vec::with_capacity(self.num_particles);
        let mut residual_sum = 0.0;

        for particle in &self.particles {
            let expected = KalmanScalar::to_f64(&particle.weight) * n_f64;
            let deterministic = expected.floor() as usize;
            let residual = expected - deterministic as f64;

            // Add deterministic copies
            for _ in 0..deterministic {
                new_particles.push(particle.clone());
            }

            residuals.push(residual);
            residual_sum += residual;
        }

        // Stochastic part for remaining particles
        let remaining = self.num_particles - new_particles.len();
        if remaining > 0 {
            // Normalize residuals
            for r in &mut residuals {
                *r /= residual_sum;
            }

            // Sample from residuals
            let mut rng = thread_rng();
            for _ in 0..remaining {
                let u: f64 = rng.gen();
                let mut cumsum = 0.0;

                for (i, &r) in residuals.iter().enumerate() {
                    cumsum += r;
                    if u <= cumsum {
                        new_particles.push(self.particles[i].clone());
                        break;
                    }
                }
            }
        }

        // Reset weights
        let uniform_weight = T::one() / T::from(self.num_particles).unwrap();
        let log_weight = uniform_weight.ln();

        for particle in &mut new_particles {
            particle.weight = uniform_weight;
            particle.log_weight = log_weight;
        }

        self.particles = new_particles;
        Ok(())
    }

    /// Get weighted mean estimate
    pub fn mean(&self) -> Vec<T> {
        let mut mean = vec![T::zero(); self.state_dim];

        for particle in &self.particles {
            for i in 0..self.state_dim {
                mean[i] = mean[i] + particle.weight * particle.state[i];
            }
        }

        mean
    }

    /// Get weighted covariance estimate
    pub fn covariance(&self) -> Vec<T> {
        let mean = self.mean();
        let mut cov = vec![T::zero(); self.state_dim * self.state_dim];

        for particle in &self.particles {
            for i in 0..self.state_dim {
                for j in 0..self.state_dim {
                    let di = particle.state[i] - mean[i];
                    let dj = particle.state[j] - mean[j];
                    cov[i * self.state_dim + j] =
                        cov[i * self.state_dim + j] + particle.weight * di * dj;
                }
            }
        }

        cov
    }

    /// Get particle with maximum weight
    pub fn best_particle(&self) -> &Particle<T> {
        self.particles
            .iter()
            .max_by(|a, b| a.weight.partial_cmp(&b.weight).unwrap())
            .unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_particle_filter_basic() {
        let state_dim = 2;
        let num_particles = 100;
        let initial_mean = vec![0.0, 0.0];
        let initial_std = vec![1.0, 1.0];
        let process_noise_std = vec![0.1, 0.1];
        let measurement_noise_std = vec![0.5];

        let mut pf = ParticleFilter::new(
            state_dim,
            num_particles,
            initial_mean,
            initial_std,
            process_noise_std,
            measurement_noise_std,
            0.1,
        )
        .unwrap();

        // Simple motion model: constant velocity
        let motion_model =
            |state: &[f64], dt: f64| -> Vec<f64> { vec![state[0] + state[1] * dt, state[1]] };

        // Simple likelihood: Gaussian around measurement
        let likelihood = |state: &[f64], measurement: &[f64]| -> f64 {
            let diff = state[0] - measurement[0];
            let sigma = 0.5;
            (-0.5 * diff * diff / (sigma * sigma)).exp()
        };

        // Predict and update
        pf.predict(motion_model);
        pf.update(&[1.0], likelihood).unwrap();

        // Check ESS
        let ess = pf.effective_sample_size();
        assert!(ess > 0.0);

        // Check mean is reasonable
        let mean = pf.mean();
        assert!(mean[0].abs() < 5.0);
    }

    #[test]
    fn test_resampling_strategies() {
        let state_dim = 1;
        let num_particles = 50;
        let initial_mean = vec![0.0];
        let initial_std = vec![1.0];
        let process_noise_std = vec![0.1];
        let measurement_noise_std = vec![0.5];

        // Test each resampling strategy
        for strategy in [
            ResamplingStrategy::Multinomial,
            ResamplingStrategy::Systematic,
            ResamplingStrategy::Stratified,
            ResamplingStrategy::Residual,
        ] {
            let mut pf = ParticleFilter::new(
                state_dim,
                num_particles,
                initial_mean.clone(),
                initial_std.clone(),
                process_noise_std.clone(),
                measurement_noise_std.clone(),
                0.1,
            )
            .unwrap();

            pf.set_resampling_strategy(strategy);

            // Force resampling by setting skewed weights
            for (i, particle) in pf.particles.iter_mut().enumerate() {
                particle.weight = if i == 0 {
                    0.9
                } else {
                    0.1 / (num_particles - 1) as f64
                };
            }

            pf.resample().unwrap();

            // Check weights are uniform after resampling
            let expected_weight = 1.0 / num_particles as f64;
            for particle in &pf.particles {
                assert!((particle.weight - expected_weight).abs() < 1e-10);
            }
        }
    }
}
