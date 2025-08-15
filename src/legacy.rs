//! Legacy API compatibility layer
//!
//! This module provides backward compatibility with the original kalman-filter crate API.
//! It wraps the new implementation to maintain the old interface.

use crate::filter::KalmanFilter as NewKalmanFilter;
use crate::KalmanError;

/// Legacy Kalman Filter with backward-compatible API
pub struct KalmanFilter {
    inner: NewKalmanFilter<f32>,
    control_dim: usize,
}

impl KalmanFilter {
    /// Create a new Kalman filter with the legacy API
    /// 
    /// # Arguments
    /// * `state_dim` - Dimension of state vector
    /// * `measurement_dim` - Dimension of measurement vector  
    /// * `control_dim` - Dimension of control vector (unused in new implementation)
    pub fn new(state_dim: usize, measurement_dim: usize, control_dim: usize) -> Self {
        let inner = NewKalmanFilter::new(state_dim, measurement_dim);
        Self {
            inner,
            control_dim,
        }
    }

    /// Set the state transition matrix (F)
    pub fn transition_matrix(&mut self, matrix: &[f32]) {
        self.inner.F = matrix.to_vec();
    }

    /// Set the measurement matrix (H)
    pub fn measurement_matrix(&mut self, matrix: &[f32]) {
        self.inner.H = matrix.to_vec();
    }

    /// Set the process noise covariance (Q)
    pub fn process_noise_cov(&mut self, matrix: &[f32]) {
        self.inner.Q = matrix.to_vec();
    }

    /// Set the measurement noise covariance (R)
    pub fn measurement_noise_cov(&mut self, matrix: &[f32]) {
        self.inner.R = matrix.to_vec();
    }

    /// Set the error covariance matrix (P)
    pub fn error_cov_post(&mut self, matrix: &[f32]) {
        self.inner.P = matrix.to_vec();
    }

    /// Set the state vector (x)
    pub fn state_post(&mut self, state: &[f32]) {
        self.inner.x = state.to_vec();
    }

    /// Predict step (with optional control input - ignored)
    pub fn predict(&mut self, _control: Option<&[f32]>) {
        self.inner.predict();
    }

    /// Correct step (measurement update)
    pub fn correct(&mut self, measurement: &[f32]) {
        // Ignore errors for backward compatibility
        let _ = self.inner.update(measurement);
    }

    /// Get the current state
    pub fn state(&self) -> &[f32] {
        &self.inner.x
    }

    /// Get the error covariance
    pub fn error_cov(&self) -> &[f32] {
        &self.inner.P
    }
}