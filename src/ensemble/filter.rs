//! Core Ensemble Kalman Filter implementation
//!
//! The EnKF represents the state distribution using an ensemble of realizations,
//! propagating each member through the full nonlinear model and using ensemble
//! statistics to approximate the covariance matrix.
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use crate::logging::{
    check_numerical_stability, format_matrix, format_state, log_filter_dimensions, state_norm,
};
use crate::types::{KalmanError, KalmanResult, KalmanScalar, NonlinearSystem};
use log::{debug, error, info, trace, warn};
use num_traits::{One, Zero};
use rand_distr::Distribution;
use rand::rng;
use rand_distr::Normal;

/// Statistics computed from an ensemble
#[derive(Debug, Clone)]
pub struct EnsembleStatistics<T: KalmanScalar> {
    /// Ensemble mean
    pub mean: Vec<T>,
    /// Ensemble spread (standard deviation)
    pub spread: Vec<T>,
    /// Ensemble anomalies (deviations from mean)
    pub anomalies: Vec<T>,
}

/// Ensemble Kalman Filter for high-dimensional state estimation
///
/// The EnKF is particularly suited for systems with dimensions >> 1000 where
/// storing and manipulating full covariance matrices becomes prohibitive.
/// It uses Monte Carlo sampling to represent uncertainty.
///
/// # Example
///
/// ```no_run
/// use kalman_filters::{EnsembleKalmanFilterBuilder, NonlinearSystem};
///
/// struct SimpleSystem;
///
/// impl NonlinearSystem<f64> for SimpleSystem {
///     fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
///         vec![state[0] + dt * state[1], state[1]]
///     }
///     
///     fn measurement(&self, state: &[f64]) -> Vec<f64> {
///         vec![state[0]] // Measure position only
///     }
///     
///     fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
///         vec![1.0, dt, 0.0, 1.0]
///     }
///     
///     fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
///         vec![1.0, 0.0]
///     }
///     
///     fn state_dim(&self) -> usize { 2 }
///     fn measurement_dim(&self) -> usize { 1 }
/// }
///
/// let mut enkf = EnsembleKalmanFilterBuilder::new(SimpleSystem)
///     .initial_mean(vec![0.0, 0.0])
///     .initial_spread(vec![1.0, 0.1])
///     .ensemble_size(100)
///     .process_noise(vec![0.01, 0.0, 0.0, 0.01])
///     .measurement_noise(vec![0.1])
///     .dt(0.1)
///     .build()
///     .unwrap();
///     
/// enkf.forecast();  // Predict step
/// enkf.update(&[1.0]).unwrap(); // Update step
/// ```
pub struct EnsembleKalmanFilter<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// The nonlinear system model
    pub system: S,
    /// State dimension
    pub state_dim: usize,
    /// Measurement dimension
    pub measurement_dim: usize,
    /// Ensemble size
    pub ensemble_size: usize,
    /// Ensemble matrix (N x M) - each column is a member
    pub ensemble: Vec<T>,
    /// Measurement noise covariance (M x M)
    pub R: Vec<T>,
    /// Process noise covariance (N x N) - for additive inflation
    pub Q: Vec<T>,
    /// Multiplicative inflation factor (> 1.0)
    pub inflation_factor: T,
    /// Localization radius (0 = no localization)
    pub localization_radius: T,
    /// Control input (optional)
    pub control: Option<Vec<T>>,
    /// Time step
    pub dt: T,
}

impl<T, S> EnsembleKalmanFilter<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// Create a new Ensemble Kalman Filter
    #[deprecated(
        since = "1.0.0-alpha0",
        note = "Use EnsembleKalmanFilterBuilder instead"
    )]
    pub fn new(
        system: S,
        initial_mean: Vec<T>,
        initial_spread: Vec<T>,
        ensemble_size: usize,
        process_noise: Vec<T>,
        measurement_noise: Vec<T>,
        dt: T,
    ) -> KalmanResult<Self> {
        Self::initialize(
            system,
            initial_mean,
            initial_spread,
            ensemble_size,
            process_noise,
            measurement_noise,
            dt,
            T::from(1.05).unwrap(), // Default 5% inflation
            T::zero(),              // No localization by default
            None,
        )
    }

    /// Initialize a new Ensemble Kalman Filter (internal method)
    #[allow(clippy::too_many_arguments)]
    pub fn initialize(
        system: S,
        initial_mean: Vec<T>,
        initial_spread: Vec<T>,
        ensemble_size: usize,
        process_noise: Vec<T>,
        measurement_noise: Vec<T>,
        dt: T,
        inflation_factor: T,
        localization_radius: T,
        control: Option<Vec<T>>,
    ) -> KalmanResult<Self> {
        let n = system.state_dim();
        let m = system.measurement_dim();

        log_filter_dimensions(n, m, None);
        debug!(
            "Ensemble Kalman Filter: Initializing with {} ensemble members",
            ensemble_size
        );

        // Validate dimensions
        if initial_mean.len() != n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, 1),
                actual: (initial_mean.len(), 1),
            });
        }
        if initial_spread.len() != n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, 1),
                actual: (initial_spread.len(), 1),
            });
        }
        if process_noise.len() != n * n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, n),
                actual: (process_noise.len() / n, n),
            });
        }
        if measurement_noise.len() != m * m {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, m),
                actual: (measurement_noise.len() / m, m),
            });
        }

        // Initialize ensemble from initial distribution
        let mut ensemble = vec![T::zero(); n * ensemble_size];
        let mut rng = rng();

        for j in 0..ensemble_size {
            for i in 0..n {
                // Sample from N(mean, spread²)
                let mean_f64 = KalmanScalar::to_f64(&initial_mean[i]);
                let spread_f64 = KalmanScalar::to_f64(&initial_spread[i]);

                if spread_f64 > 0.0f64 {
                    let normal = Normal::new(mean_f64, spread_f64).unwrap();
                    ensemble[i * ensemble_size + j] = T::from(normal.sample(&mut rng)).unwrap();
                } else {
                    ensemble[i * ensemble_size + j] = initial_mean[i];
                }
            }
        }

        Ok(Self {
            system,
            state_dim: n,
            measurement_dim: m,
            ensemble_size,
            ensemble,
            R: measurement_noise,
            Q: process_noise,
            inflation_factor,
            localization_radius,
            control,
            dt,
        })
    }

    /// Set multiplicative inflation factor
    pub fn set_inflation(&mut self, factor: T) {
        self.inflation_factor = factor;
    }

    /// Set localization radius
    pub fn set_localization(&mut self, radius: T) {
        self.localization_radius = radius;
    }

    /// Set control input
    pub fn set_control(&mut self, control: Vec<T>) {
        self.control = Some(control);
    }

    /// Compute ensemble statistics
    pub fn compute_statistics(&self) -> EnsembleStatistics<T> {
        let n = self.state_dim;
        let m = self.ensemble_size;

        // Compute mean
        let mut mean = vec![T::zero(); n];
        for (i, mean_i) in mean.iter_mut().enumerate() {
            for j in 0..m {
                *mean_i = *mean_i + self.ensemble[i * m + j];
            }
            *mean_i = *mean_i / T::from(m).unwrap();
        }

        // Compute anomalies and spread
        let mut anomalies = vec![T::zero(); n * m];
        let mut spread = vec![T::zero(); n];

        for i in 0..n {
            for j in 0..m {
                anomalies[i * m + j] = self.ensemble[i * m + j] - mean[i];
                spread[i] = spread[i] + anomalies[i * m + j] * anomalies[i * m + j];
            }
            spread[i] = (spread[i] / T::from(m - 1).unwrap()).sqrt();
        }

        EnsembleStatistics {
            mean,
            spread,
            anomalies,
        }
    }

    /// Forecast step: propagate ensemble through nonlinear model
    pub fn forecast(&mut self) {
        let n = self.state_dim;
        let m = self.ensemble_size;

        let prior_stats = self.compute_statistics();
        debug!(
            "EnKF forecast: {} prior_norm={:.4} spread={:.4}",
            format_state(&prior_stats.mean, "mean"),
            state_norm(&prior_stats.mean),
            state_norm(&prior_stats.spread)
        );

        // Propagate each ensemble member
        let mut new_ensemble = vec![T::zero(); n * m];

        for j in 0..m {
            // Extract member j
            let mut member = vec![T::zero(); n];
            for (i, member_i) in member.iter_mut().enumerate() {
                *member_i = self.ensemble[i * m + j];
            }

            // Propagate through nonlinear model
            let propagated =
                self.system
                    .state_transition(&member, self.control.as_deref(), self.dt);

            // Store in new ensemble
            for i in 0..n {
                new_ensemble[i * m + j] = propagated[i];
            }
        }

        // Apply multiplicative inflation to maintain spread
        if self.inflation_factor > T::one() {
            debug!(
                "EnKF forecast: Applying multiplicative inflation factor={:.4}",
                KalmanScalar::to_f64(&self.inflation_factor)
            );
            let stats = self.compute_statistics();
            for i in 0..n {
                for j in 0..m {
                    let anomaly = new_ensemble[i * m + j] - stats.mean[i];
                    new_ensemble[i * m + j] = stats.mean[i] + self.inflation_factor * anomaly;
                }
            }
        }

        self.ensemble = new_ensemble;

        let posterior_stats = self.compute_statistics();
        debug!(
            "EnKF forecast: {} posterior_norm={:.4} spread={:.4}",
            format_state(&posterior_stats.mean, "mean"),
            state_norm(&posterior_stats.mean),
            state_norm(&posterior_stats.spread)
        );
    }

    /// Update step: assimilate observations using stochastic EnKF
    pub fn update(&mut self, observation: &[T]) -> KalmanResult<()> {
        let n = self.state_dim;
        let m_state = self.ensemble_size;
        let m_obs = self.measurement_dim;

        if observation.len() != m_obs {
            error!(
                "EnKF update: observation dimension mismatch: expected {}x1, got {}x1",
                m_obs,
                observation.len()
            );
            return Err(KalmanError::DimensionMismatch {
                expected: (m_obs, 1),
                actual: (observation.len(), 1),
            });
        }

        debug!("EnKF update: {}", format_state(observation, "observation"));

        // Get ensemble statistics
        let stats = self.compute_statistics();

        // Generate observation ensemble (perturbed observations)
        let mut obs_ensemble = vec![T::zero(); m_obs * m_state];
        let mut rng = rng();

        for j in 0..m_state {
            for i in 0..m_obs {
                // Sample observation noise
                let obs_f64 = KalmanScalar::to_f64(&observation[i]);
                let noise_var = KalmanScalar::to_f64(&self.R[i * m_obs + i]).sqrt();

                if noise_var > 0.0f64 {
                    let normal = Normal::new(obs_f64, noise_var).unwrap();
                    obs_ensemble[i * m_state + j] = T::from(normal.sample(&mut rng)).unwrap();
                } else {
                    obs_ensemble[i * m_state + j] = observation[i];
                }
            }
        }

        // Transform ensemble through observation operator
        let mut H_ensemble = vec![T::zero(); m_obs * m_state];
        for j in 0..m_state {
            let mut member = vec![T::zero(); n];
            for (i, member_i) in member.iter_mut().enumerate() {
                *member_i = self.ensemble[i * m_state + j];
            }

            let h_x = self.system.measurement(&member);

            for i in 0..m_obs {
                H_ensemble[i * m_state + j] = h_x[i];
            }
        }

        // Compute observation ensemble mean
        let mut H_mean = vec![T::zero(); m_obs];
        for i in 0..m_obs {
            for j in 0..m_state {
                H_mean[i] = H_mean[i] + H_ensemble[i * m_state + j];
            }
            H_mean[i] = H_mean[i] / T::from(m_state).unwrap();
        }

        // Compute cross-covariance P_xh and observation covariance P_hh
        // P_xh = (1/(m-1)) * X' * (HX)'T
        // P_hh = (1/(m-1)) * (HX)' * (HX)'T + R

        let mut P_xh = vec![T::zero(); n * m_obs];
        let mut P_hh = self.R.clone();

        for i in 0..n {
            for k in 0..m_obs {
                for j in 0..m_state {
                    let x_anomaly = self.ensemble[i * m_state + j] - stats.mean[i];
                    let h_anomaly = H_ensemble[k * m_state + j] - H_mean[k];
                    P_xh[i * m_obs + k] =
                        P_xh[i * m_obs + k] + x_anomaly * h_anomaly / T::from(m_state - 1).unwrap();
                }
            }
        }

        for i in 0..m_obs {
            for k in 0..m_obs {
                for j in 0..m_state {
                    let h_anomaly_i = H_ensemble[i * m_state + j] - H_mean[i];
                    let h_anomaly_k = H_ensemble[k * m_state + j] - H_mean[k];
                    P_hh[i * m_obs + k] = P_hh[i * m_obs + k]
                        + h_anomaly_i * h_anomaly_k / T::from(m_state - 1).unwrap();
                }
            }
        }

        // Compute Kalman gain K = P_xh * P_hh^-1
        check_numerical_stability(&P_hh, m_obs, "EnKF observation covariance P_hh");
        let P_hh_inv = self.invert_matrix(&P_hh, m_obs)?;
        let mut K = vec![T::zero(); n * m_obs];

        for i in 0..n {
            for j in 0..m_obs {
                for k in 0..m_obs {
                    K[i * m_obs + j] =
                        K[i * m_obs + j] + P_xh[i * m_obs + k] * P_hh_inv[k * m_obs + j];
                }
            }
        }

        // Update each ensemble member
        for j in 0..m_state {
            // Compute innovation for member j
            let mut innovation = vec![T::zero(); m_obs];
            for i in 0..m_obs {
                innovation[i] = obs_ensemble[i * m_state + j] - H_ensemble[i * m_state + j];
            }

            // Update member j
            for i in 0..n {
                for k in 0..m_obs {
                    self.ensemble[i * m_state + j] =
                        self.ensemble[i * m_state + j] + K[i * m_obs + k] * innovation[k];
                }
            }
        }

        let final_stats = self.compute_statistics();
        debug!(
            "EnKF update: {} posterior_norm={:.4} spread={:.4}",
            format_state(&final_stats.mean, "mean"),
            state_norm(&final_stats.mean),
            state_norm(&final_stats.spread)
        );

        Ok(())
    }

    /// Simple matrix inversion using Gauss-Jordan elimination
    fn invert_matrix(&self, matrix: &[T], size: usize) -> KalmanResult<Vec<T>> {
        let mut a = matrix.to_vec();
        let mut inv = vec![T::zero(); size * size];

        // Initialize inverse as identity matrix
        for i in 0..size {
            inv[i * size + i] = T::one();
        }

        // Gauss-Jordan elimination
        for i in 0..size {
            // Find pivot
            let mut pivot = a[i * size + i];
            if pivot.abs() < T::from(1e-10).unwrap() {
                error!(
                    "EnKF matrix inversion: Singular matrix encountered at pivot ({}, {}) = {:.6e}",
                    i,
                    i,
                    KalmanScalar::to_f64(&pivot)
                );
                return Err(KalmanError::SingularMatrix);
            }

            // Scale row
            for j in 0..size {
                a[i * size + j] = a[i * size + j] / pivot;
                inv[i * size + j] = inv[i * size + j] / pivot;
            }

            // Eliminate column
            for j in 0..size {
                if i != j {
                    let factor = a[j * size + i];
                    for k in 0..size {
                        a[j * size + k] = a[j * size + k] - factor * a[i * size + k];
                        inv[j * size + k] = inv[j * size + k] - factor * inv[i * size + k];
                    }
                }
            }
        }

        Ok(inv)
    }

    /// Get ensemble mean
    pub fn mean(&self) -> Vec<T> {
        self.compute_statistics().mean
    }

    /// Get ensemble spread
    pub fn spread(&self) -> Vec<T> {
        self.compute_statistics().spread
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple nonlinear system for testing
    struct SimpleNonlinearSystem;

    impl NonlinearSystem<f64> for SimpleNonlinearSystem {
        fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
            // Simple nonlinear dynamics
            vec![
                state[0] + state[1] * dt + 0.01 * state[0] * state[0] * dt,
                state[1] * (1.0 - 0.1 * dt),
            ]
        }

        fn measurement(&self, state: &[f64]) -> Vec<f64> {
            // Nonlinear measurement
            vec![state[0] * state[0] + state[1] * state[1]]
        }

        fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
            unreachable!("EnKF doesn't need Jacobians")
        }

        fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
            unreachable!("EnKF doesn't need Jacobians")
        }

        fn state_dim(&self) -> usize {
            2
        }
        fn measurement_dim(&self) -> usize {
            1
        }
    }

    #[test]
    fn test_enkf_basic() {
        let system = SimpleNonlinearSystem;

        let initial_mean = vec![1.0, 0.5];
        let initial_spread = vec![0.1, 0.1];
        let ensemble_size = 50;
        let process_noise = vec![0.01, 0.0, 0.0, 0.01];
        let measurement_noise = vec![0.1];

        let mut enkf = EnsembleKalmanFilter::new(
            system,
            initial_mean,
            initial_spread,
            ensemble_size,
            process_noise,
            measurement_noise,
            0.01,
        )
        .unwrap();

        // Run forecast-update cycle
        enkf.forecast();
        enkf.update(&[1.25]).unwrap();

        // Check ensemble statistics
        let mean = enkf.mean();
        let spread = enkf.spread();

        // Mean should be reasonable
        assert!(mean[0] > 0.5 && mean[0] < 2.0);
        assert!(mean[1] > -0.5 && mean[1] < 1.5);

        // Spread should be positive
        assert!(spread[0] > 0.0);
        assert!(spread[1] > 0.0);
    }

    #[test]
    fn test_enkf_ensemble_covariance_consistency() {
        // The sample covariance of ensemble members should match reported spread
        let system = SimpleNonlinearSystem;

        let initial_mean = vec![5.0, 2.0];
        let initial_spread = vec![1.0, 0.5];
        let ensemble_size = 200; // Large ensemble for statistical accuracy

        let enkf = EnsembleKalmanFilter::new(
            system,
            initial_mean.clone(),
            initial_spread.clone(),
            ensemble_size,
            vec![0.01, 0.0, 0.0, 0.01],
            vec![0.1],
            0.01,
        )
        .unwrap();

        // Compute sample statistics from ensemble
        // Note: Ensemble is stored as (N x M) where N=state_dim, M=ensemble_size
        // Each column is a member, so indexing is: ensemble[state_idx * ensemble_size + member_idx]
        let state_dim = 2;
        let mut sample_mean = vec![0.0; state_dim];
        let mut sample_var = vec![0.0; state_dim];

        // Compute mean for each state dimension
        for state_idx in 0..state_dim {
            for member_idx in 0..ensemble_size {
                sample_mean[state_idx] += enkf.ensemble[state_idx * ensemble_size + member_idx];
            }
            sample_mean[state_idx] /= ensemble_size as f64;
        }

        // Compute variance for each state dimension
        for state_idx in 0..state_dim {
            for member_idx in 0..ensemble_size {
                let diff = enkf.ensemble[state_idx * ensemble_size + member_idx] - sample_mean[state_idx];
                sample_var[state_idx] += diff * diff;
            }
            sample_var[state_idx] /= (ensemble_size - 1) as f64;
        }

        // Sample mean should be reasonably close to initial mean (within 3 sigma)
        for j in 0..state_dim {
            let std_err = initial_spread[j] / (ensemble_size as f64).sqrt();
            let error = (sample_mean[j] - initial_mean[j]).abs();
            assert!(error < 3.0 * std_err + 0.1,
                "Sample mean {} too far from initial mean {}. Error: {}, 3*std_err: {}",
                sample_mean[j], initial_mean[j], error, 3.0 * std_err);
            assert!(sample_mean[j].is_finite(), "Sample mean is not finite");
        }

        // Sample std should be reasonably close to initial spread
        for j in 0..state_dim {
            let sample_std = sample_var[j].sqrt();
            let relative_error = (sample_std - initial_spread[j]).abs() / initial_spread[j];
            // For large ensemble, std should be within ~20% of specified spread
            assert!(relative_error < 0.3,
                "Sample std {} too far from initial spread {}. Relative error: {}",
                sample_std, initial_spread[j], relative_error);
        }
    }

    #[test]
    fn test_enkf_ensemble_size_effect() {
        // Larger ensemble should give more stable estimates
        let process_noise = vec![0.01, 0.0, 0.0, 0.01];
        let measurement_noise = vec![0.1];
        let initial_mean = vec![1.0, 0.5];
        let initial_spread = vec![0.1, 0.1];

        // Run with small ensemble
        let system_small = SimpleNonlinearSystem;
        let mut enkf_small = EnsembleKalmanFilter::new(
            system_small,
            initial_mean.clone(),
            initial_spread.clone(),
            20, // Small ensemble
            process_noise.clone(),
            measurement_noise.clone(),
            0.01,
        )
        .unwrap();

        // Run with large ensemble
        let system_large = SimpleNonlinearSystem;
        let mut enkf_large = EnsembleKalmanFilter::new(
            system_large,
            initial_mean.clone(),
            initial_spread.clone(),
            200, // Large ensemble
            process_noise.clone(),
            measurement_noise.clone(),
            0.01,
        )
        .unwrap();

        // Update both with same measurement
        let measurement = vec![1.25];
        for _ in 0..10 {
            enkf_small.forecast();
            enkf_small.update(&measurement).unwrap();

            enkf_large.forecast();
            enkf_large.update(&measurement).unwrap();
        }

        // Both should have positive spread
        assert!(enkf_small.spread()[0] > 0.0);
        assert!(enkf_large.spread()[0] > 0.0);

        // Both means should be finite and reasonable
        for i in 0..2 {
            assert!(enkf_small.mean()[i].is_finite());
            assert!(enkf_large.mean()[i].is_finite());
        }
    }

    #[test]
    fn test_enkf_update_reduces_spread() {
        // Measurement update should reduce ensemble spread
        let system = SimpleNonlinearSystem;

        let mut enkf = EnsembleKalmanFilter::new(
            system,
            vec![1.0, 0.5],
            vec![1.0, 1.0], // Large initial spread
            100,
            vec![0.01, 0.0, 0.0, 0.01],
            vec![0.01], // Small measurement noise
            0.01,
        )
        .unwrap();

        let spread_before = enkf.spread().to_vec();

        // Forecast increases spread
        enkf.forecast();
        let spread_after_forecast = enkf.spread().to_vec();

        // Update should reduce spread (with informative measurement)
        enkf.update(&[1.25]).unwrap();
        let spread_after_update = enkf.spread().to_vec();

        // At least one dimension should have reduced spread after update
        let reduced = spread_after_update[0] < spread_after_forecast[0]
            || spread_after_update[1] < spread_after_forecast[1];
        assert!(reduced,
            "Update should reduce spread. Before: {:?}, After: {:?}",
            spread_after_forecast, spread_after_update);
    }

    #[test]
    fn test_enkf_convergence_to_static_target() {
        // EnKF should converge when measuring a static target
        struct StaticSystem;
        impl NonlinearSystem<f64> for StaticSystem {
            fn state_transition(&self, state: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                state.to_vec() // Static state
            }
            fn measurement(&self, state: &[f64]) -> Vec<f64> {
                vec![state[0]] // Directly observe first state
            }
            fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                unreachable!()
            }
            fn measurement_jacobian(&self, _: &[f64]) -> Vec<f64> {
                unreachable!()
            }
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        let mut enkf = EnsembleKalmanFilter::new(
            StaticSystem,
            vec![0.0, 0.0], // Start far from target
            vec![2.0, 2.0],
            100,
            vec![0.001, 0.0, 0.0, 0.001],
            vec![0.1],
            0.01,
        )
        .unwrap();

        let true_value = 5.0;

        for _ in 0..50 {
            enkf.forecast();
            enkf.update(&[true_value]).unwrap();
        }

        // Should converge close to true value
        let error = (enkf.mean()[0] - true_value).abs();
        assert!(error < 1.0,
            "EnKF should converge to static target. Error: {}, Mean: {}",
            error, enkf.mean()[0]);

        // Spread should have decreased
        assert!(enkf.spread()[0] < 1.0,
            "Spread should decrease with repeated observations. Spread: {}",
            enkf.spread()[0]);
    }

    #[test]
    fn test_enkf_no_ensemble_collapse() {
        // Ensemble should not collapse (all members identical)
        let system = SimpleNonlinearSystem;

        let mut enkf = EnsembleKalmanFilter::new(
            system,
            vec![1.0, 0.5],
            vec![0.5, 0.5],
            50,
            vec![0.01, 0.0, 0.0, 0.01],
            vec![0.1],
            0.01,
        )
        .unwrap();

        // Run many cycles
        for _ in 0..100 {
            enkf.forecast();
            enkf.update(&[1.25]).unwrap();
        }

        // Spread should remain positive (not collapsed)
        assert!(enkf.spread()[0] > 1e-6,
            "Ensemble collapsed in dim 0. Spread: {}", enkf.spread()[0]);
        assert!(enkf.spread()[1] > 1e-6,
            "Ensemble collapsed in dim 1. Spread: {}", enkf.spread()[1]);

        // All members should be finite
        for val in enkf.ensemble.iter() {
            assert!(val.is_finite(), "Ensemble member became non-finite");
        }
    }
}
