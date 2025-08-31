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
use rand::distributions::Distribution;
use rand::thread_rng;
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
    #[deprecated(since = "1.0.0-alpha0", note = "Use EnsembleKalmanFilterBuilder instead")]
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
        info!(
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
        let mut rng = thread_rng();

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
        for i in 0..n {
            for j in 0..m {
                mean[i] = mean[i] + self.ensemble[i * m + j];
            }
            mean[i] = mean[i] / T::from(m).unwrap();
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
            for i in 0..n {
                member[i] = self.ensemble[i * m + j];
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
        let mut rng = thread_rng();

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
            for i in 0..n {
                member[i] = self.ensemble[i * m_state + j];
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
    fn test_enkf_ensemble_size() {
        let system = SimpleNonlinearSystem;

        let initial_mean = vec![1.0, 0.5];
        let initial_spread = vec![0.1, 0.1];
        let ensemble_size = 100;
        let process_noise = vec![0.01, 0.0, 0.0, 0.01];
        let measurement_noise = vec![0.1];

        let enkf = EnsembleKalmanFilter::new(
            system,
            initial_mean,
            initial_spread,
            ensemble_size,
            process_noise,
            measurement_noise,
            0.01,
        )
        .unwrap();

        assert_eq!(enkf.ensemble_size, 100);
        assert_eq!(enkf.ensemble.len(), 2 * 100); // state_dim * ensemble_size
    }
}
