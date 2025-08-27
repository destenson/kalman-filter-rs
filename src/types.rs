//! Type definitions for Kalman filter implementation

use num_traits::Float;
use std::fmt;

#[deprecated(note = "Use crate::error::{KalmanError, KalmanResult} instead")]
pub use crate::error::{KalmanError, KalmanResult};

/// Trait for numeric types that can be used in Kalman filters
pub trait KalmanScalar: Float + Default + fmt::Debug + fmt::Display + 'static {
    /// Small epsilon value for numerical stability
    fn epsilon() -> Self;
    fn Epsilon(&self) -> Self {
        <Self as KalmanScalar>::epsilon()
    }

    /// Convert to f64 for logging purposes
    fn to_f64(&self) -> f64 {
        num_traits::cast::ToPrimitive::to_f64(self).unwrap_or(f64::NAN)
    }
}

impl KalmanScalar for f32 {
    fn epsilon() -> Self {
        1e-6
    }
}

impl KalmanScalar for f64 {
    fn epsilon() -> Self {
        1e-10
    }
}

/// Trait for nonlinear system dynamics
pub trait NonlinearSystem<T: KalmanScalar> {
    /// State transition function: x_k+1 = f(x_k, u_k)
    fn state_transition(&self, state: &[T], control: Option<&[T]>, dt: T) -> Vec<T>;

    /// Measurement function: z_k = h(x_k)
    fn measurement(&self, state: &[T]) -> Vec<T>;

    /// State transition Jacobian: F = ∂f/∂x
    fn state_jacobian(&self, state: &[T], control: Option<&[T]>, dt: T) -> Vec<T>;

    /// Measurement Jacobian: H = ∂h/∂x
    fn measurement_jacobian(&self, state: &[T]) -> Vec<T>;

    /// Get state dimension
    fn state_dim(&self) -> usize;

    /// Get measurement dimension
    fn measurement_dim(&self) -> usize;
}

/// Strategy for computing Jacobian matrices
pub enum JacobianStrategy {
    /// User provides analytical Jacobian functions
    Analytical,
    /// Compute Jacobian using finite differences
    Numerical { step_size: f64 },
    /// Use automatic differentiation (future enhancement)
    #[allow(dead_code)]
    Automatic,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_scalar_f32() {
        assert_eq!(<f32 as KalmanScalar>::epsilon(), 1e-6);
        let val = 3.14_f32;
        assert_eq!(val.Epsilon(), 1e-6);
        assert!((val.to_f64() - 3.14).abs() < 0.01);
    }

    #[test]
    fn test_kalman_scalar_f64() {
        assert_eq!(<f64 as KalmanScalar>::epsilon(), 1e-10);
        let val = 3.14159_f64;
        assert_eq!(val.Epsilon(), 1e-10);
        assert!((val.to_f64() - 3.14159).abs() < 1e-10);
    }

    #[test]
    fn test_jacobian_strategy() {
        let _ = JacobianStrategy::Analytical;
        let _ = JacobianStrategy::Numerical { step_size: 1e-6 };
        // Just ensure enum variants can be created
    }

    // Mock implementation for testing NonlinearSystem trait
    struct TestSystem {
        state_dim: usize,
        measurement_dim: usize,
    }

    impl NonlinearSystem<f64> for TestSystem {
        fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
            state.iter().map(|&x| x + dt).collect()
        }

        fn measurement(&self, state: &[f64]) -> Vec<f64> {
            vec![state[0]]
        }

        fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
            let n = state.len();
            let mut jac = vec![0.0; n * n];
            for i in 0..n {
                jac[i * n + i] = 1.0;
            }
            jac
        }

        fn measurement_jacobian(&self, state: &[f64]) -> Vec<f64> {
            let n = state.len();
            let mut jac = vec![0.0; self.measurement_dim * n];
            jac[0] = 1.0;
            jac
        }

        fn state_dim(&self) -> usize {
            self.state_dim
        }

        fn measurement_dim(&self) -> usize {
            self.measurement_dim
        }
    }

    #[test]
    fn test_nonlinear_system_trait() {
        let system = TestSystem {
            state_dim: 2,
            measurement_dim: 1,
        };

        let state = vec![1.0, 2.0];
        let dt = 0.1;

        let new_state = system.state_transition(&state, None, dt);
        assert_eq!(new_state.len(), 2);
        assert!((new_state[0] - 1.1).abs() < 1e-10);
        assert!((new_state[1] - 2.1).abs() < 1e-10);

        let measurement = system.measurement(&state);
        assert_eq!(measurement.len(), 1);
        assert_eq!(measurement[0], 1.0);

        let state_jac = system.state_jacobian(&state, None, dt);
        assert_eq!(state_jac.len(), 4);
        assert_eq!(state_jac[0], 1.0);
        assert_eq!(state_jac[3], 1.0);

        let meas_jac = system.measurement_jacobian(&state);
        assert_eq!(meas_jac.len(), 2);
        assert_eq!(meas_jac[0], 1.0);
        assert_eq!(meas_jac[1], 0.0);

        assert_eq!(system.state_dim(), 2);
        assert_eq!(system.measurement_dim(), 1);
    }
}
