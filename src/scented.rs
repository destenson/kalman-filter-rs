//! Cubature Kalman Filter (CKF) implementation
//!
//! The Cubature Kalman Filter is a specialized variant of the sigma-point Kalman filters
//! that uses a spherical-radial cubature rule for numerical integration. It provides
//! better numerical stability than UKF for high-dimensional systems.
//!
//! # Theory
//!
//! The CKF uses a third-degree spherical-radial cubature rule to approximate the
//! Gaussian weighted integrals. For an n-dimensional state, it generates 2n cubature
//! points located on the surface of a hypersphere.
//!
//! ## Cubature Points
//!
//! The cubature points are generated as:
//! - ξᵢ = √n * eᵢ for i = 1, ..., n (positive direction)
//! - ξᵢ = -√n * eᵢ₋ₙ for i = n+1, ..., 2n (negative direction)
//!
//! where eᵢ is the i-th unit vector.
//!
//! All cubature points have equal weight: wᵢ = 1/(2n)
#![allow(unused)]

use crate::types::{KalmanError, KalmanResult, KalmanScalar, NonlinearSystem};
use crate::filter::KalmanFilter;
use crate::logging::{format_matrix, format_state, format_innovation, state_norm, check_numerical_stability, log_filter_dimensions};
use num_traits::{One, Zero};
use log::{trace, debug, info, warn, error};

/// Cubature Kalman Filter for nonlinear state estimation
///
/// The CKF is particularly well-suited for high-dimensional systems where
/// numerical stability is critical. Unlike UKF, it doesn't require tuning
/// parameters and provides consistent performance across different dimensions.
pub struct CubatureKalmanFilter<T, S>
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
    /// Cubature points matrix (N x 2N) - column major
    pub cubature_points: Vec<T>,
    /// Weight for each cubature point (always 1/2n)
    pub weight: T,
    /// Control input (optional)
    pub control: Option<Vec<T>>,
    /// Time step
    pub dt: T,
}

impl<T, S> CubatureKalmanFilter<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// Create a new Cubature Kalman Filter
    pub fn new(
        system: S,
        initial_state: Vec<T>,
        initial_covariance: Vec<T>,
        process_noise: Vec<T>,
        measurement_noise: Vec<T>,
        dt: T,
    ) -> KalmanResult<Self> {
        let n = system.state_dim();
        let m = system.measurement_dim();
        
        log_filter_dimensions(n, m, None);
        info!("Cubature Kalman Filter: Initializing with {} cubature points", 2*n);
        
        // Validate dimensions
        if initial_state.len() != n {
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
        
        // Cubature weight is always 1/(2n)
        let weight = T::one() / T::from(2 * n).unwrap();
        
        Ok(Self {
            system,
            state_dim: n,
            measurement_dim: m,
            x: initial_state,
            P: initial_covariance,
            Q: process_noise,
            R: measurement_noise,
            cubature_points: vec![T::zero(); n * 2 * n],
            weight,
            control: None,
            dt,
        })
    }
    
    /// Set control input
    pub fn set_control(&mut self, control: Vec<T>) {
        self.control = Some(control);
    }
    
    /// Generate cubature points from current state and covariance
    fn generate_cubature_points(&mut self) -> KalmanResult<()> {
        let n = self.state_dim;
        let sqrt_n = T::from(n).unwrap().sqrt();
        
        trace!("CKF: Generating {} cubature points with weight={:.6}", 2*n, self.weight.to_f64());
        
        // Compute matrix square root of P using Cholesky decomposition
        let L = self.cholesky_decomposition(&self.P)?;
        
        // Generate cubature points
        for i in 0..n {
            // Positive direction: x + √n * L * eᵢ
            for j in 0..n {
                self.cubature_points[j * (2 * n) + i] = 
                    self.x[j] + sqrt_n * L[j * n + i];
            }
            
            // Negative direction: x - √n * L * eᵢ
            for j in 0..n {
                self.cubature_points[j * (2 * n) + n + i] = 
                    self.x[j] - sqrt_n * L[j * n + i];
            }
        }
        
        trace!("CKF: Generated cubature points around {}", format_state(&self.x, "state"));
        
        Ok(())
    }
    
    /// Cholesky decomposition for matrix square root
    fn cholesky_decomposition(&self, matrix: &[T]) -> KalmanResult<Vec<T>> {
        let n = self.state_dim;
        let mut L = vec![T::zero(); n * n];
        
        for i in 0..n {
            for j in 0..=i {
                let mut sum = matrix[i * n + j];
                
                for k in 0..j {
                    sum = sum - L[i * n + k] * L[j * n + k];
                }
                
                if i == j {
                    if sum <= T::zero() {
                        error!("CKF: Cholesky decomposition failed - negative diagonal element at ({},{}) = {:.6e}", i, j, sum.to_f64());
                        return Err(KalmanError::SingularMatrix);
                    }
                    L[i * n + j] = sum.sqrt();
                } else {
                    L[i * n + j] = sum / L[j * n + j];
                }
            }
        }
        
        Ok(L)
    }
    
    /// Predict step: propagate state and covariance using cubature points
    pub fn predict(&mut self) -> KalmanResult<()> {
        let n = self.state_dim;
        let num_points = 2 * n;
        
        debug!("CKF predict: {} prior={:.4}", format_state(&self.x, "state"), state_norm(&self.x));
        
        // Generate cubature points
        self.generate_cubature_points()?;
        
        // Propagate cubature points through nonlinear state transition
        let mut transformed_points = vec![T::zero(); n * num_points];
        for i in 0..num_points {
            let mut point_state = vec![T::zero(); n];
            for j in 0..n {
                point_state[j] = self.cubature_points[j * num_points + i];
            }
            
            let transformed = self.system.state_transition(&point_state, self.control.as_deref(), self.dt);
            
            for j in 0..n {
                transformed_points[j * num_points + i] = transformed[j];
            }
        }
        
        // Compute predicted mean
        let mut new_x = vec![T::zero(); n];
        for i in 0..num_points {
            for j in 0..n {
                new_x[j] = new_x[j] + self.weight * transformed_points[j * num_points + i];
            }
        }
        
        // Compute predicted covariance
        let mut new_P = self.Q.clone();
        for i in 0..num_points {
            let mut diff = vec![T::zero(); n];
            for j in 0..n {
                diff[j] = transformed_points[j * num_points + i] - new_x[j];
            }
            
            for j in 0..n {
                for k in 0..n {
                    new_P[j * n + k] = new_P[j * n + k] + self.weight * diff[j] * diff[k];
                }
            }
        }
        
        // Ensure symmetry
        for i in 0..n {
            for j in i+1..n {
                let avg = (new_P[i * n + j] + new_P[j * n + i]) * T::from(0.5).unwrap();
                new_P[i * n + j] = avg;
                new_P[j * n + i] = avg;
            }
        }
        
        debug!("CKF predict: {} posterior={:.4}", format_state(&new_x, "state"), state_norm(&new_x));
        check_numerical_stability(&new_P, n, "CKF predict covariance");
        
        self.x = new_x;
        self.P = new_P;
        self.cubature_points = transformed_points;
        
        Ok(())
    }
    
    /// Update step: incorporate measurement using cubature points
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        let n = self.state_dim;
        let m = self.measurement_dim;
        let num_points = 2 * n;
        
        if measurement.len() != m {
            error!("CKF update: measurement dimension mismatch: expected {}x1, got {}x1", m, measurement.len());
            return Err(KalmanError::DimensionMismatch {
                expected: (m, 1),
                actual: (measurement.len(), 1),
            });
        }
        
        debug!("CKF update: {}", format_state(measurement, "measurement"));
        
        // Transform cubature points through measurement function
        let mut measurement_points = vec![T::zero(); m * num_points];
        for i in 0..num_points {
            let mut point_state = vec![T::zero(); n];
            for j in 0..n {
                point_state[j] = self.cubature_points[j * num_points + i];
            }
            
            let transformed = self.system.measurement(&point_state);
            
            for j in 0..m {
                measurement_points[j * num_points + i] = transformed[j];
            }
        }
        
        // Compute predicted measurement mean
        let mut z_pred = vec![T::zero(); m];
        for i in 0..num_points {
            for j in 0..m {
                z_pred[j] = z_pred[j] + self.weight * measurement_points[j * num_points + i];
            }
        }
        
        // Compute innovation covariance S = Pzz + R
        let mut S = self.R.clone();
        for i in 0..num_points {
            let mut diff = vec![T::zero(); m];
            for j in 0..m {
                diff[j] = measurement_points[j * num_points + i] - z_pred[j];
            }
            
            for j in 0..m {
                for k in 0..m {
                    S[j * m + k] = S[j * m + k] + self.weight * diff[j] * diff[k];
                }
            }
        }
        
        // Compute cross-covariance Pxz
        let mut Pxz = vec![T::zero(); n * m];
        for i in 0..num_points {
            let mut x_diff = vec![T::zero(); n];
            let mut z_diff = vec![T::zero(); m];
            
            for j in 0..n {
                x_diff[j] = self.cubature_points[j * num_points + i] - self.x[j];
            }
            for j in 0..m {
                z_diff[j] = measurement_points[j * num_points + i] - z_pred[j];
            }
            
            for j in 0..n {
                for k in 0..m {
                    Pxz[j * m + k] = Pxz[j * m + k] + self.weight * x_diff[j] * z_diff[k];
                }
            }
        }
        
        // Compute Kalman gain K = Pxz * S^-1
        let S_inv = KalmanFilter::<T>::invert_matrix(&S, m)?;
        let mut K = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for k in 0..m {
                    K[i * m + j] = K[i * m + j] + Pxz[i * m + k] * S_inv[k * m + j];
                }
            }
        }
        
        // Innovation y = z - z_pred
        let mut y = vec![T::zero(); m];
        for i in 0..m {
            y[i] = measurement[i] - z_pred[i];
        }
        
        debug!("CKF update: {}", format_innovation(&y));
        trace!("CKF update: predicted measurement {}", format_state(&z_pred, "z_pred"));
        
        // State update: x = x + K * y
        for i in 0..n {
            for j in 0..m {
                self.x[i] = self.x[i] + K[i * m + j] * y[j];
            }
        }
        
        // Covariance update: P = P - K * S * K^T
        // First compute K * S
        let mut KS = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for k in 0..m {
                    KS[i * m + j] = KS[i * m + j] + K[i * m + k] * S[k * m + j];
                }
            }
        }
        
        // Then compute K * S * K^T
        let mut KSKT = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..m {
                    KSKT[i * n + j] = KSKT[i * n + j] + KS[i * m + k] * K[j * m + k];
                }
            }
        }
        
        // Update P
        for i in 0..n {
            for j in 0..n {
                self.P[i * n + j] = self.P[i * n + j] - KSKT[i * n + j];
            }
        }
        
        // Ensure symmetry
        for i in 0..n {
            for j in i+1..n {
                let avg = (self.P[i * n + j] + self.P[j * n + i]) * T::from(0.5).unwrap();
                self.P[i * n + j] = avg;
                self.P[j * n + i] = avg;
            }
        }
        
        debug!("CKF update: {} posterior={:.4}", format_state(&self.x, "state"), state_norm(&self.x));
        check_numerical_stability(&self.P, n, "CKF update covariance");
        
        Ok(())
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
    
    /// Simple nonlinear system for testing
    struct NonlinearSystem2D;
    
    impl NonlinearSystem<f64> for NonlinearSystem2D {
        fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
            let x = state[0];
            let y = state[1];
            
            // Nonlinear dynamics with rotation
            vec![
                x * (1.0 - 0.1 * dt) + y * 0.1 * dt,
                -x * 0.1 * dt + y * (1.0 - 0.1 * dt),
            ]
        }
        
        fn measurement(&self, state: &[f64]) -> Vec<f64> {
            // Nonlinear measurement: distance from origin
            vec![(state[0].powi(2) + state[1].powi(2)).sqrt()]
        }
        
        fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
            unreachable!("CKF doesn't need Jacobians")
        }
        
        fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
            unreachable!("CKF doesn't need Jacobians")
        }
        
        fn state_dim(&self) -> usize { 2 }
        fn measurement_dim(&self) -> usize { 1 }
    }
    
    #[test]
    fn test_ckf_basic() {
        let system = NonlinearSystem2D;
        
        let initial_state = vec![1.0, 0.0];
        let initial_covariance = vec![0.1, 0.0, 0.0, 0.1];
        let process_noise = vec![0.01, 0.0, 0.0, 0.01];
        let measurement_noise = vec![0.1];
        
        let mut ckf = CubatureKalmanFilter::new(
            system,
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise,
            0.01,  // dt
        ).unwrap();
        
        // Run a few prediction and update cycles
        for _ in 0..5 {
            ckf.predict().unwrap();
            let measurement = vec![1.0];  // Simulated measurement
            ckf.update(&measurement).unwrap();
        }
        
        // Check that filter is stable
        let cov = ckf.covariance();
        assert!(cov[0] > 0.0 && cov[0] < 10.0);  // Reasonable covariance
        assert!(cov[3] > 0.0 && cov[3] < 10.0);  // Diagonal element
    }
    
    #[test]
    fn test_ckf_weight() {
        let system = NonlinearSystem2D;
        
        let initial_state = vec![1.0, 0.0];
        let initial_covariance = vec![0.1, 0.0, 0.0, 0.1];
        let process_noise = vec![0.01, 0.0, 0.0, 0.01];
        let measurement_noise = vec![0.1];
        
        let ckf = CubatureKalmanFilter::new(
            system,
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise,
            0.01,
        ).unwrap();
        
        // Check cubature weight is 1/(2n) = 1/4 for 2D system
        assert!((ckf.weight - 0.25).abs() < 1e-10);
    }
}
