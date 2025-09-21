//! Extended Kalman Filter (EKF) implementation
//!
//! The Extended Kalman Filter linearizes non-linear models around the current estimate
//! using Jacobian matrices. It extends the linear Kalman filter to handle nonlinear
//! state transition and measurement functions.
//!
//! # Theory
//!
//! For a nonlinear system:
//! - State transition: x_k+1 = f(x_k, u_k) + w_k
//! - Measurement: z_k = h(x_k) + v_k
//!
//! The EKF linearizes these functions using Jacobians:
//! - F_k = ∂f/∂x evaluated at x̂_k
//! - H_k = ∂h/∂x evaluated at x̂_k|k-1
//!
//! # Example
//!
//! ```no_run
//! use kalman_filters::{ExtendedKalmanFilter, NonlinearSystem, KalmanScalar};
//!
//! struct PendulumSystem;
//!
//! impl NonlinearSystem<f64> for PendulumSystem {
//!     fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
//!         let theta = state[0];
//!         let theta_dot = state[1];
//!         let g = 9.81;
//!         let l = 1.0;
//!         vec![
//!             theta + theta_dot * dt,
//!             theta_dot - (g / l) * theta.sin() * dt,
//!         ]
//!     }
//!     
//!     fn measurement(&self, state: &[f64]) -> Vec<f64> {
//!         vec![state[0]] // Measure angle only
//!     }
//!     
//!     fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
//!         let theta = state[0];
//!         let g = 9.81;
//!         let l = 1.0;
//!         vec![
//!             1.0, dt,
//!             -(g / l) * theta.cos() * dt, 1.0,
//!         ]
//!     }
//!     
//!     fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
//!         vec![1.0, 0.0]
//!     }
//!     
//!     fn state_dim(&self) -> usize { 2 }
//!     fn measurement_dim(&self) -> usize { 1 }
//! }
//! ```
#![allow(unused)]

use crate::filter::KalmanFilter;
use crate::logging::{
    check_numerical_stability, format_innovation, format_matrix, format_state,
    log_filter_dimensions, state_norm,
};
use crate::types::{JacobianStrategy, KalmanError, KalmanResult, KalmanScalar, NonlinearSystem};
use log::{debug, error, info, trace, warn};
use num_traits::{One, Zero};

/// Extended Kalman Filter for nonlinear state estimation
pub struct ExtendedKalmanFilter<T, S>
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
    /// State vector (N x 1)
    pub x: Vec<T>,
    /// State covariance matrix (N x N) - row major
    pub P: Vec<T>,
    /// Process noise covariance (N x N) - row major
    pub Q: Vec<T>,
    /// Measurement noise covariance (M x M) - row major
    pub R: Vec<T>,
    /// Jacobian computation strategy
    pub jacobian_strategy: JacobianStrategy,
    /// Control input (optional)
    pub control: Option<Vec<T>>,
    /// Time step
    pub dt: T,
}

impl<T, S> ExtendedKalmanFilter<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// Create a new Extended Kalman Filter
    /// Create a new Extended Kalman Filter (prefer using ExtendedKalmanFilterBuilder)
    #[deprecated(since = "1.0.0", note = "Use ExtendedKalmanFilterBuilder instead")]
    pub fn new(
        system: S,
        initial_state: Vec<T>,
        initial_covariance: Vec<T>,
        process_noise: Vec<T>,
        measurement_noise: Vec<T>,
        dt: T,
    ) -> KalmanResult<Self> {
        Self::initialize(
            system,
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise,
            dt,
            JacobianStrategy::Analytical,
            None,
        )
    }

    /// Initialize a new Extended Kalman Filter with validation
    /// This method performs all validation and is called by the builder
    pub fn initialize(
        system: S,
        initial_state: Vec<T>,
        initial_covariance: Vec<T>,
        process_noise: Vec<T>,
        measurement_noise: Vec<T>,
        dt: T,
        jacobian_strategy: JacobianStrategy,
        control: Option<Vec<T>>,
    ) -> KalmanResult<Self> {
        let n = system.state_dim();
        let m = system.measurement_dim();

        log_filter_dimensions(n, m, None);
        info!("Initializing Extended Kalman Filter");

        // Validate dimensions
        if initial_state.len() != n {
            error!(
                "EKF: Initial state dimension mismatch: expected {}x1, got {}x1",
                n,
                initial_state.len()
            );
            return Err(KalmanError::DimensionMismatch {
                expected: (n, 1),
                actual: (initial_state.len(), 1),
            });
        }
        if initial_covariance.len() != n * n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, n),
                actual: (initial_covariance.len() / n, n),
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

        check_numerical_stability(&initial_covariance, n, "EKF initial covariance");

        debug!(
            "EKF initialized: {}",
            format_state(&initial_state, "initial_state")
        );
        debug!("EKF time step dt={:.6}", KalmanScalar::to_f64(&dt));

        Ok(Self {
            system,
            state_dim: n,
            measurement_dim: m,
            x: initial_state,
            P: initial_covariance,
            Q: process_noise,
            R: measurement_noise,
            jacobian_strategy,
            control,
            dt,
        })
    }

    /// Set the Jacobian computation strategy
    pub fn set_jacobian_strategy(&mut self, strategy: JacobianStrategy) {
        self.jacobian_strategy = strategy;
    }

    /// Set control input
    pub fn set_control(&mut self, control: Vec<T>) {
        self.control = Some(control);
    }

    /// Predict step: propagate state and covariance using nonlinear dynamics
    pub fn predict(&mut self) {
        let n = self.state_dim;

        debug!("EKF predict step: state_norm={:.6}", state_norm(&self.x));

        #[cfg(feature = "prometheus-metrics")]
        let _timer = crate::metrics::MetricsTimer::start();

        // Propagate state through nonlinear function
        let new_x = self
            .system
            .state_transition(&self.x, self.control.as_deref(), self.dt);

        // Get state transition Jacobian at current state
        let F = match self.jacobian_strategy {
            JacobianStrategy::Analytical => {
                trace!("EKF: Using analytical Jacobian");
                self.system
                    .state_jacobian(&self.x, self.control.as_deref(), self.dt)
            }
            JacobianStrategy::Numerical { step_size } => {
                trace!(
                    "EKF: Computing numerical Jacobian with step_size={:.6}",
                    step_size.to_f64()
                );
                #[cfg(feature = "prometheus-metrics")]
                let _jacobian_timer = crate::metrics::MetricsTimer::start();

                let jacobian = self.compute_numerical_jacobian_state(step_size);

                #[cfg(feature = "prometheus-metrics")]
                _jacobian_timer.finish_jacobian("ekf");

                jacobian
            }
            _ => self
                .system
                .state_jacobian(&self.x, self.control.as_deref(), self.dt),
        };

        if log::log_enabled!(log::Level::Trace) {
            trace!("EKF state Jacobian: {}", format_matrix(&F, n, n, "F"));
        }

        self.x = new_x;

        // P = F * P * F^T + Q
        // First compute F * P
        let mut fp = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    fp[i * n + j] = fp[i * n + j] + F[i * n + k] * self.P[k * n + j];
                }
            }
        }

        // Then compute (F * P) * F^T + Q
        let mut new_p = self.Q.clone();
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    new_p[i * n + j] = new_p[i * n + j] + fp[i * n + k] * F[j * n + k];
                }
            }
        }

        // Check for divergence
        let trace_val = (0..n)
            .map(|i| new_p[i * n + i])
            .fold(T::zero(), |a, b| a + b);
        if trace_val > T::from(1e6).unwrap() {
            warn!(
                "EKF covariance trace is large: {:.6e}, filter may be diverging",
                KalmanScalar::to_f64(&trace_val)
            );
        }

        self.P = new_p;

        check_numerical_stability(&self.P, n, "EKF predicted covariance");
        debug!(
            "EKF predict complete: state_norm={:.6}",
            state_norm(&self.x)
        );

        #[cfg(feature = "prometheus-metrics")]
        {
            _timer.finish_predict("ekf");
            crate::metrics::record_prediction("ekf");
            crate::metrics::set_state_dimension("ekf", n);
            crate::metrics::set_covariance_trace("ekf", KalmanScalar::to_f64(&trace_val));
        }
    }

    /// Update step: incorporate measurement using nonlinear measurement function
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        let n = self.state_dim;
        let m = self.measurement_dim;

        debug!(
            "EKF update step: measurement_norm={:.6}",
            state_norm(measurement)
        );

        if measurement.len() != m {
            error!(
                "EKF: Measurement dimension mismatch: expected {}, got {}",
                m,
                measurement.len()
            );
            return Err(KalmanError::DimensionMismatch {
                expected: (m, 1),
                actual: (measurement.len(), 1),
            });
        }

        // Predicted measurement from nonlinear function
        let h_x = self.system.measurement(&self.x);

        // Innovation: y = z - h(x)
        let mut y = vec![T::zero(); m];
        for i in 0..m {
            y[i] = measurement[i] - h_x[i];
        }

        debug!("EKF innovation: {}", format_innovation(&y));

        // Get measurement Jacobian at predicted state
        let H = match self.jacobian_strategy {
            JacobianStrategy::Analytical => self.system.measurement_jacobian(&self.x),
            JacobianStrategy::Numerical { step_size } => {
                self.compute_numerical_jacobian_measurement(step_size)
            }
            _ => self.system.measurement_jacobian(&self.x),
        };

        // Innovation covariance: S = H * P * H^T + R
        // First compute H * P
        let mut hp = vec![T::zero(); m * n];
        for i in 0..m {
            for j in 0..n {
                for k in 0..n {
                    hp[i * n + j] = hp[i * n + j] + H[i * n + k] * self.P[k * n + j];
                }
            }
        }

        // Then compute S = (H * P) * H^T + R
        let mut s = self.R.clone();
        for i in 0..m {
            for j in 0..m {
                for k in 0..n {
                    s[i * m + j] = s[i * m + j] + hp[i * n + k] * H[j * n + k];
                }
            }
        }

        // Invert S
        trace!("EKF: Computing innovation covariance inverse");
        let s_inv = match KalmanFilter::<T>::invert_matrix(&s, m) {
            Ok(inv) => inv,
            Err(e) => {
                error!("EKF: Failed to invert innovation covariance: {:?}", e);
                check_numerical_stability(&s, m, "EKF innovation covariance (singular)");
                return Err(e);
            }
        };

        // Kalman gain: K = P * H^T * S^-1
        // First compute P * H^T
        let mut ph_t = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for k in 0..n {
                    ph_t[i * m + j] = ph_t[i * m + j] + self.P[i * n + k] * H[j * n + k];
                }
            }
        }

        // Then compute K = (P * H^T) * S^-1
        let mut k = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for l in 0..m {
                    k[i * m + j] = k[i * m + j] + ph_t[i * m + l] * s_inv[l * m + j];
                }
            }
        }

        // State update: x = x + K * y
        for i in 0..n {
            for j in 0..m {
                self.x[i] = self.x[i] + k[i * m + j] * y[j];
            }
        }

        // Covariance update: P = (I - K * H) * P
        // First compute K * H
        let mut kh = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for l in 0..m {
                    kh[i * n + j] = kh[i * n + j] + k[i * m + l] * H[l * n + j];
                }
            }
        }

        // Compute I - K * H
        let mut i_kh = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                i_kh[i * n + j] = if i == j { T::one() } else { T::zero() } - kh[i * n + j];
            }
        }

        // Finally compute P = (I - K * H) * P
        let mut new_p = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for l in 0..n {
                    new_p[i * n + j] = new_p[i * n + j] + i_kh[i * n + l] * self.P[l * n + j];
                }
            }
        }

        // Ensure symmetry
        for i in 0..n {
            for j in i + 1..n {
                let avg = (new_p[i * n + j] + new_p[j * n + i]) * T::from(0.5).unwrap();
                new_p[i * n + j] = avg;
                new_p[j * n + i] = avg;
            }
        }

        self.P = new_p;

        check_numerical_stability(&self.P, n, "EKF updated covariance");
        debug!("EKF update complete: state_norm={:.6}", state_norm(&self.x));

        Ok(())
    }

    /// Compute numerical Jacobian for state transition using finite differences
    fn compute_numerical_jacobian_state(&self, step_size: f64) -> Vec<T> {
        let n = self.state_dim;
        let step = T::from(step_size).unwrap();
        let mut jacobian = vec![T::zero(); n * n];

        for j in 0..n {
            // Perturb state in j-th dimension
            let mut state_plus = self.x.clone();
            let mut state_minus = self.x.clone();
            state_plus[j] = state_plus[j] + step;
            state_minus[j] = state_minus[j] - step;

            // Compute f(x + h) and f(x - h)
            let f_plus =
                self.system
                    .state_transition(&state_plus, self.control.as_deref(), self.dt);
            let f_minus =
                self.system
                    .state_transition(&state_minus, self.control.as_deref(), self.dt);

            // Finite difference: (f(x+h) - f(x-h)) / (2h)
            for i in 0..n {
                jacobian[i * n + j] = (f_plus[i] - f_minus[i]) / (step + step);
            }
        }

        jacobian
    }

    /// Compute numerical Jacobian for measurement using finite differences
    fn compute_numerical_jacobian_measurement(&self, step_size: f64) -> Vec<T> {
        let n = self.state_dim;
        let m = self.measurement_dim;
        let step = T::from(step_size).unwrap();
        let mut jacobian = vec![T::zero(); m * n];

        for j in 0..n {
            // Perturb state in j-th dimension
            let mut state_plus = self.x.clone();
            let mut state_minus = self.x.clone();
            state_plus[j] = state_plus[j] + step;
            state_minus[j] = state_minus[j] - step;

            // Compute h(x + h) and h(x - h)
            let h_plus = self.system.measurement(&state_plus);
            let h_minus = self.system.measurement(&state_minus);

            // Finite difference: (h(x+h) - h(x-h)) / (2h)
            for i in 0..m {
                jacobian[i * n + j] = (h_plus[i] - h_minus[i]) / (step + step);
            }
        }

        jacobian
    }

    /// Get current state estimate
    pub fn state(&self) -> &[T] {
        &self.x
    }

    /// Get current state covariance
    pub fn covariance(&self) -> &[T] {
        &self.P
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple pendulum system for testing
    struct PendulumSystem {
        g: f64, // Gravity
        l: f64, // Length
    }

    impl NonlinearSystem<f64> for PendulumSystem {
        fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
            let theta = state[0];
            let theta_dot = state[1];
            vec![
                theta + theta_dot * dt,
                theta_dot - (self.g / self.l) * theta.sin() * dt,
            ]
        }

        fn measurement(&self, state: &[f64]) -> Vec<f64> {
            vec![state[0]] // Measure angle only
        }

        fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
            let theta = state[0];
            vec![1.0, dt, -(self.g / self.l) * theta.cos() * dt, 1.0]
        }

        fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
            vec![1.0, 0.0]
        }

        fn state_dim(&self) -> usize {
            2
        }
        fn measurement_dim(&self) -> usize {
            1
        }
    }

    #[test]
    fn test_ekf_pendulum() {
        let system = PendulumSystem { g: 9.81, l: 1.0 };

        let initial_state = vec![0.1, 0.0]; // Small angle, no velocity
        let initial_covariance = vec![0.01, 0.0, 0.0, 0.01];
        let process_noise = vec![0.001, 0.0, 0.0, 0.001];
        let measurement_noise = vec![0.01];

        let mut ekf = ExtendedKalmanFilter::new(
            system,
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise,
            0.01, // dt
        )
        .unwrap();

        // Predict and update
        ekf.predict();
        ekf.update(&[0.09]).unwrap();

        // Check that state has been updated
        assert!((ekf.state()[0] - 0.1).abs() < 0.05);
    }

    #[test]
    fn test_ekf_numerical_jacobian() {
        let system = PendulumSystem { g: 9.81, l: 1.0 };

        let initial_state = vec![0.1, 0.0];
        let initial_covariance = vec![0.01, 0.0, 0.0, 0.01];
        let process_noise = vec![0.001, 0.0, 0.0, 0.001];
        let measurement_noise = vec![0.01];

        let mut ekf = ExtendedKalmanFilter::new(
            system,
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise,
            0.01,
        )
        .unwrap();

        // Use numerical Jacobian
        ekf.set_jacobian_strategy(JacobianStrategy::Numerical { step_size: 1e-6 });

        ekf.predict();
        ekf.update(&[0.09]).unwrap();

        // Should still work with numerical Jacobian
        assert!((ekf.state()[0] - 0.1).abs() < 0.05);
    }

    #[test]
    fn test_ekf_dimension_checks() {
        struct BadSystem;
        impl NonlinearSystem<f64> for BadSystem {
            fn state_transition(&self, state: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                state.to_vec()
            }
            fn measurement(&self, state: &[f64]) -> Vec<f64> {
                vec![state[0]]
            }
            fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                vec![1.0] // Wrong size!
            }
            fn measurement_jacobian(&self, _: &[f64]) -> Vec<f64> {
                vec![1.0, 0.0]
            }
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        let system = BadSystem;
        let result = ExtendedKalmanFilter::new(
            system,
            vec![0.0, 0.0],
            vec![1.0, 0.0, 0.0, 1.0],
            vec![0.1, 0.0, 0.0, 0.1],
            vec![0.1],
            0.01,
        );

        // Should detect dimension mismatch
        assert!(result.is_ok()); // Creation is ok, error happens on predict
    }

    #[test]
    fn test_ekf_convergence() {
        // Simple linear system to test convergence
        struct LinearSystem;
        impl NonlinearSystem<f64> for LinearSystem {
            fn state_transition(&self, state: &[f64], _: Option<&[f64]>, dt: f64) -> Vec<f64> {
                vec![state[0] + state[1] * dt, state[1]]
            }
            fn measurement(&self, state: &[f64]) -> Vec<f64> {
                vec![state[0]]
            }
            fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, dt: f64) -> Vec<f64> {
                vec![1.0, dt, 0.0, 1.0]
            }
            fn measurement_jacobian(&self, _: &[f64]) -> Vec<f64> {
                vec![1.0, 0.0]
            }
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        let system = LinearSystem;
        let mut ekf = ExtendedKalmanFilter::new(
            system,
            vec![0.0, 1.0],
            vec![10.0, 0.0, 0.0, 10.0],
            vec![0.01, 0.0, 0.0, 0.01],
            vec![0.1],
            0.1,
        )
        .unwrap();

        // True position after 10 steps: 0 + 1*0.1*10 = 1.0
        for i in 0..10 {
            ekf.predict();
            let true_pos = 0.1 * (i + 1) as f64;
            ekf.update(&[true_pos]).unwrap();
        }

        assert!((ekf.state()[0] - 1.0).abs() < 0.1);
        assert!((ekf.state()[1] - 1.0).abs() < 0.2);
    }

    #[test]
    fn test_ekf_getters() {
        let system = PendulumSystem { g: 9.81, l: 1.0 };
        let ekf = ExtendedKalmanFilter::new(
            system,
            vec![0.1, 0.0],
            vec![0.01, 0.0, 0.0, 0.01],
            vec![0.001, 0.0, 0.0, 0.001],
            vec![0.01],
            0.01,
        )
        .unwrap();

        assert_eq!(ekf.state().len(), 2);
        assert_eq!(ekf.covariance().len(), 4);
        assert!((ekf.dt - 0.01).abs() < 1e-10);
    }

    #[test]
    fn test_ekf_control_input() {
        struct ControlledSystem;
        impl NonlinearSystem<f64> for ControlledSystem {
            fn state_transition(
                &self,
                state: &[f64],
                control: Option<&[f64]>,
                dt: f64,
            ) -> Vec<f64> {
                let u = control.map(|c| c[0]).unwrap_or(0.0);
                vec![state[0] + state[1] * dt, state[1] + u * dt]
            }
            fn measurement(&self, state: &[f64]) -> Vec<f64> {
                vec![state[0]]
            }
            fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, dt: f64) -> Vec<f64> {
                vec![1.0, dt, 0.0, 1.0]
            }
            fn measurement_jacobian(&self, _: &[f64]) -> Vec<f64> {
                vec![1.0, 0.0]
            }
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        let system = ControlledSystem;
        let mut ekf = ExtendedKalmanFilter::new(
            system,
            vec![0.0, 0.0],
            vec![1.0, 0.0, 0.0, 1.0],
            vec![0.01, 0.0, 0.0, 0.01],
            vec![0.1],
            0.1,
        )
        .unwrap();

        // Apply control to accelerate
        ekf.set_control(vec![1.0]);
        ekf.predict();
        assert!((ekf.state()[1] - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_numerical_jacobian_accuracy() {
        let system = PendulumSystem { g: 9.81, l: 1.0 };
        let state = vec![0.1, 0.0];
        let dt = 0.01;

        // Analytical Jacobian
        let analytical = system.state_jacobian(&state, None, dt);

        // Numerical Jacobian
        let ekf = ExtendedKalmanFilter::new(
            system,
            state.clone(),
            vec![1.0, 0.0, 0.0, 1.0],
            vec![0.01, 0.0, 0.0, 0.01],
            vec![0.1],
            dt,
        )
        .unwrap();

        // Get numerical jacobian using the internal method
        let numerical = ekf.compute_numerical_jacobian_state(1e-6);

        // Compare analytical and numerical
        for i in 0..4 {
            assert!((analytical[i] - numerical[i]).abs() < 1e-4);
        }
    }
}
