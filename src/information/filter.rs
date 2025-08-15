//! Core Information Filter implementation
#![allow(unused)]

use crate::filter::KalmanFilter;
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use crate::information::{InformationForm, InformationMatrix, InformationVector};
use crate::logging::{format_matrix, format_state, state_norm, check_numerical_stability, log_filter_dimensions, matrix_condition_estimate};
use num_traits::{Zero, One};
use approx::assert_relative_eq;
use log::{trace, debug, info, warn, error};

/// Information state representation
#[derive(Clone, Debug)]
pub struct InformationState<T: KalmanScalar> {
    /// Information matrix Y = P^-1 (n x n)
    pub Y: Vec<T>,
    /// Information vector y = Y·x (n x 1)
    pub y: Vec<T>,
    /// State dimension
    pub dim: usize,
}

impl<T: KalmanScalar> InformationState<T> {
    /// Create new information state
    pub fn new(dim: usize) -> Self {
        log_filter_dimensions(dim, 0, None);
        info!("Information Filter: Initializing information state with dimension {}", dim);
        
        Self {
            Y: vec![T::zero(); dim * dim],
            y: vec![T::zero(); dim],
            dim,
        }
    }
    
    /// Initialize with information matrix and vector
    pub fn from_information(Y: Vec<T>, y: Vec<T>) -> KalmanResult<Self> {
        let dim = y.len();
        if Y.len() != dim * dim {
            error!("Information Filter: dimension mismatch: expected {}x{}, got len={}", dim, dim, Y.len());
            return Err(KalmanError::DimensionMismatch {
                expected: (dim, dim),
                actual: (Y.len() / dim, dim),
            });
        }
        
        debug!("Information Filter: Initialized from information matrix Y and vector y");
        check_numerical_stability(&Y, dim, "Information matrix Y");
        
        Ok(Self { Y, y, dim })
    }
    
    /// Initialize from state and covariance
    pub fn from_state_covariance(state: &[T], covariance: &[T]) -> KalmanResult<Self> {
        let dim = state.len();
        if covariance.len() != dim * dim {
            error!("Information Filter: covariance dimension mismatch: expected {}x{}, got len={}", dim, dim, covariance.len());
            return Err(KalmanError::DimensionMismatch {
                expected: (dim, dim),
                actual: (covariance.len() / dim, dim),
            });
        }
        
        debug!("Information Filter: Converting {} to information form", format_state(state, "state"));
        check_numerical_stability(covariance, dim, "Covariance matrix P");
        
        // Y = P^-1
        let Y = KalmanFilter::<T>::invert_matrix(covariance, dim)?;
        
        // y = Y·x
        let mut y = vec![T::zero(); dim];
        for i in 0..dim {
            for j in 0..dim {
                y[i] = y[i] + Y[i * dim + j] * state[j];
            }
        }
        
        debug!("Information Filter: Converted to information form with condition number ≈ {:.2e}", matrix_condition_estimate(&Y, dim));
        
        Ok(Self { Y, y, dim })
    }
}

impl<T: KalmanScalar> InformationForm<T> for InformationState<T> {
    fn information_matrix(&self) -> &[T] {
        &self.Y
    }
    
    fn information_vector(&self) -> &[T] {
        &self.y
    }
    
    fn recover_state(&self) -> KalmanResult<Vec<T>> {
        trace!("Information Filter: Recovering state from information form");
        
        // x = Y^-1 · y
        let Y_inv = KalmanFilter::<T>::invert_matrix(&self.Y, self.dim)?;
        let mut state = vec![T::zero(); self.dim];
        for i in 0..self.dim {
            for j in 0..self.dim {
                state[i] = state[i] + Y_inv[i * self.dim + j] * self.y[j];
            }
        }
        
        debug!("Information Filter: Recovered {}", format_state(&state, "state"));
        Ok(state)
    }
    
    fn recover_covariance(&self) -> KalmanResult<Vec<T>> {
        trace!("Information Filter: Recovering covariance from information matrix");
        
        // P = Y^-1
        let cov = KalmanFilter::<T>::invert_matrix(&self.Y, self.dim)?;
        
        check_numerical_stability(&cov, self.dim, "Recovered covariance P");
        
        Ok(cov)
    }
    
    fn add_information(&mut self, delta_y: &[T], delta_Y: &[T]) {
        trace!("Information Filter: Adding information update");
        
        // Y = Y + ΔY
        for i in 0..self.Y.len() {
            self.Y[i] = self.Y[i] + delta_Y[i];
        }
        // y = y + Δy
        for i in 0..self.y.len() {
            self.y[i] = self.y[i] + delta_y[i];
        }
        
        check_numerical_stability(&self.Y, self.dim, "Updated information matrix Y");
    }
}

/// Information Filter for state estimation
pub struct InformationFilter<T: KalmanScalar> {
    /// Current information state
    pub state: InformationState<T>,
    /// State dimension
    pub state_dim: usize,
    /// Measurement dimension
    pub measurement_dim: usize,
    /// State transition matrix F (n x n)
    pub F: Vec<T>,
    /// Process noise covariance Q (n x n)
    pub Q: Vec<T>,
    /// Observation matrix H (m x n)
    pub H: Vec<T>,
    /// Measurement noise covariance R (m x m)
    pub R: Vec<T>,
}

impl<T: KalmanScalar> InformationFilter<T> {
    /// Create new Information Filter
    pub fn new(
        state_dim: usize,
        measurement_dim: usize,
        initial_Y: Vec<T>,
        initial_y: Vec<T>,
        F: Vec<T>,
        Q: Vec<T>,
        H: Vec<T>,
        R: Vec<T>,
    ) -> KalmanResult<Self> {
        log_filter_dimensions(state_dim, measurement_dim, None);
        info!("Information Filter: Initializing filter with state_dim={}, measurement_dim={}", state_dim, measurement_dim);
        
        // Validate dimensions
        if initial_Y.len() != state_dim * state_dim {
            error!("Information Filter: initial Y dimension mismatch: expected {}x{}, got len={}", state_dim, state_dim, initial_Y.len());
            return Err(KalmanError::DimensionMismatch {
                expected: (state_dim, state_dim),
                actual: (initial_Y.len() / state_dim, state_dim),
            });
        }
        if initial_y.len() != state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (state_dim, 1),
                actual: (initial_y.len(), 1),
            });
        }
        if F.len() != state_dim * state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (state_dim, state_dim),
                actual: (F.len() / state_dim, state_dim),
            });
        }
        if Q.len() != state_dim * state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (state_dim, state_dim),
                actual: (Q.len() / state_dim, state_dim),
            });
        }
        if H.len() != measurement_dim * state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (measurement_dim, state_dim),
                actual: (H.len() / state_dim, state_dim),
            });
        }
        if R.len() != measurement_dim * measurement_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (measurement_dim, measurement_dim),
                actual: (R.len() / measurement_dim, measurement_dim),
            });
        }
        
        let state = InformationState::from_information(initial_Y, initial_y)?;
        
        Ok(Self {
            state,
            state_dim,
            measurement_dim,
            F,
            Q,
            H,
            R,
        })
    }
    
    /// Predict step (complex in information form)
    /// Y_k|k-1 = (F·Y_k-1^-1·F^T + Q)^-1
    /// y_k|k-1 = Y_k|k-1·F·Y_k-1^-1·y_k-1
    pub fn predict(&mut self) -> KalmanResult<()> {
        let n = self.state_dim;
        
        let prior_state = self.state.recover_state()?;
        debug!("Information Filter predict: {} prior_norm={:.4}", format_state(&prior_state, "state"), state_norm(&prior_state));
        
        // Step 1: Compute Y_k-1^-1 (covariance form)
        let P_prev = self.state.recover_covariance()?;
        
        // Step 2: Compute F·P·F^T + Q
        // First compute F·P
        let mut FP = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    FP[i * n + j] = FP[i * n + j] + self.F[i * n + k] * P_prev[k * n + j];
                }
            }
        }
        
        // Then compute F·P·F^T + Q
        let mut P_pred = self.Q.clone();
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    P_pred[i * n + j] = P_pred[i * n + j] + FP[i * n + k] * self.F[j * n + k];
                }
            }
        }
        
        // Step 3: Y_k|k-1 = P_pred^-1
        let Y_pred = KalmanFilter::<T>::invert_matrix(&P_pred, n)?;
        
        // Step 4: Compute y_k|k-1 = Y_k|k-1·F·Y_k-1^-1·y_k-1
        // First recover previous state: x_k-1 = Y_k-1^-1·y_k-1
        let x_prev = self.state.recover_state()?;
        
        // Then compute F·x_k-1
        let mut Fx = vec![T::zero(); n];
        for i in 0..n {
            for j in 0..n {
                Fx[i] = Fx[i] + self.F[i * n + j] * x_prev[j];
            }
        }
        
        // Finally compute y_k|k-1 = Y_k|k-1·F·x_k-1
        let mut y_pred = vec![T::zero(); n];
        for i in 0..n {
            for j in 0..n {
                y_pred[i] = y_pred[i] + Y_pred[i * n + j] * Fx[j];
            }
        }
        
        // Update state
        self.state.Y = Y_pred;
        self.state.y = y_pred;
        
        let posterior_state = self.state.recover_state()?;
        debug!("Information Filter predict: {} posterior_norm={:.4}", format_state(&posterior_state, "state"), state_norm(&posterior_state));
        check_numerical_stability(&self.state.Y, n, "Information Filter predict Y");
        
        Ok(())
    }
    
    /// Update step (simple in information form)
    /// Y_k = Y_k|k-1 + H^T·R^-1·H
    /// y_k = y_k|k-1 + H^T·R^-1·z
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        let n = self.state_dim;
        let m = self.measurement_dim;
        
        if measurement.len() != m {
            error!("Information Filter update: measurement dimension mismatch: expected {}x1, got {}x1", m, measurement.len());
            return Err(KalmanError::DimensionMismatch {
                expected: (m, 1),
                actual: (measurement.len(), 1),
            });
        }
        
        debug!("Information Filter update: {}", format_state(measurement, "measurement"));
        
        // Step 1: Compute R^-1
        let R_inv = KalmanFilter::<T>::invert_matrix(&self.R, m)?;
        
        // Step 2: Compute H^T·R^-1·H
        // First compute H^T·R^-1
        let mut HtR_inv = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for k in 0..m {
                    HtR_inv[i * m + j] = HtR_inv[i * m + j] + self.H[k * n + i] * R_inv[k * m + j];
                }
            }
        }
        
        // Then compute H^T·R^-1·H
        let mut delta_Y = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..m {
                    delta_Y[i * n + j] = delta_Y[i * n + j] + HtR_inv[i * m + k] * self.H[k * n + j];
                }
            }
        }
        
        // Step 3: Compute H^T·R^-1·z
        let mut delta_y = vec![T::zero(); n];
        for i in 0..n {
            for j in 0..m {
                delta_y[i] = delta_y[i] + HtR_inv[i * m + j] * measurement[j];
            }
        }
        
        // Step 4: Update information state
        self.state.add_information(&delta_y, &delta_Y);
        
        let final_state = self.state.recover_state()?;
        debug!("Information Filter update: {} posterior_norm={:.4}", format_state(&final_state, "state"), state_norm(&final_state));
        
        Ok(())
    }
    
    /// Update with no measurement (do nothing - key advantage of IF)
    pub fn skip_update(&self) {
        // No operation needed - information remains unchanged
    }
    
    /// Fuse multiple measurements at once
    pub fn fuse_measurements(&mut self, measurements: Vec<(&[T], &[T], &[T])>) -> KalmanResult<()> {
        // measurements: Vec<(z_i, H_i, R_i)>
        let n = self.state_dim;
        
        info!("Information Filter: Fusing {} measurements simultaneously", measurements.len());
        
        let mut total_delta_Y = vec![T::zero(); n * n];
        let mut total_delta_y = vec![T::zero(); n];
        
        for (z, H, R) in measurements {
            let m = z.len();
            
            // Compute R^-1
            let R_inv = KalmanFilter::<T>::invert_matrix(R, m)?;
            
            // Compute H^T·R^-1
            let mut HtR_inv = vec![T::zero(); n * m];
            for i in 0..n {
                for j in 0..m {
                    for k in 0..m {
                        HtR_inv[i * m + j] = HtR_inv[i * m + j] + H[k * n + i] * R_inv[k * m + j];
                    }
                }
            }
            
            // Add H^T·R^-1·H to total
            for i in 0..n {
                for j in 0..n {
                    for k in 0..m {
                        total_delta_Y[i * n + j] = total_delta_Y[i * n + j] + 
                            HtR_inv[i * m + k] * H[k * n + j];
                    }
                }
            }
            
            // Add H^T·R^-1·z to total
            for i in 0..n {
                for j in 0..m {
                    total_delta_y[i] = total_delta_y[i] + HtR_inv[i * m + j] * z[j];
                }
            }
        }
        
        // Update information state with total contribution
        self.state.add_information(&total_delta_y, &total_delta_Y);
        
        let fused_state = self.state.recover_state()?;
        debug!("Information Filter: Fused {} measurements, final {}", measurements.len(), format_state(&fused_state, "state"));
        
        Ok(())
    }
    
    /// Get current state estimate
    pub fn get_state(&self) -> KalmanResult<Vec<T>> {
        self.state.recover_state()
    }
    
    /// Get current covariance
    pub fn get_covariance(&self) -> KalmanResult<Vec<T>> {
        self.state.recover_covariance()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_information_state_conversion() {
        // Test conversion between state/covariance and information forms
        let state = vec![1.0, 2.0];
        let covariance = vec![
            2.0, 0.5,
            0.5, 1.0,
        ];
        
        let info_state = InformationState::from_state_covariance(&state, &covariance).unwrap();
        
        // Recover state and covariance
        let recovered_state = info_state.recover_state().unwrap();
        let recovered_cov = info_state.recover_covariance().unwrap();
        
        // Check recovery accuracy
        for i in 0..2 {
            assert_relative_eq!(state[i], recovered_state[i], epsilon = 1e-10f64);
        }
        for i in 0..4 {
            assert_relative_eq!(covariance[i], recovered_cov[i], epsilon = 1e-10f64);
        }
    }
    
    #[test]
    fn test_information_filter_update() {
        // Simple 1D test
        let Y_init = vec![1.0];  // P^-1 = 1.0, so P = 1.0
        let y_init = vec![0.0];  // x = Y^-1·y = 0.0
        
        let mut filter = InformationFilter::new(
            1, 1,
            Y_init,
            y_init,
            vec![1.0],      // F
            vec![0.01],     // Q
            vec![1.0],      // H
            vec![0.1],      // R
        ).unwrap();
        
        // Update with measurement
        filter.update(&[1.0]).unwrap();
        
        // Information should have increased
        assert!(filter.state.Y[0] > 1.0);
        
        // State should move toward measurement
        let state = filter.get_state().unwrap();
        assert!(state[0] > 0.0);
        assert!(state[0] < 1.0);
    }
}
