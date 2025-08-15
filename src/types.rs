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
