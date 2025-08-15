//! Extended Information Filter (EIF) for nonlinear systems

use crate::information::{InformationState, InformationForm};
use crate::types::{KalmanError, KalmanResult, KalmanScalar, NonlinearSystem, JacobianStrategy};
use crate::filter::KalmanFilter;
use num_traits::{Zero, One};

/// Extended Information Filter for nonlinear state estimation
pub struct ExtendedInformationFilter<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// The nonlinear system model
    pub system: S,
    /// Current information state
    pub state: InformationState<T>,
    /// State dimension
    pub state_dim: usize,
    /// Measurement dimension
    pub measurement_dim: usize,
    /// Process noise covariance Q (n x n)
    pub Q: Vec<T>,
    /// Measurement noise covariance R (m x m)
    pub R: Vec<T>,
    /// Jacobian computation strategy
    pub jacobian_strategy: JacobianStrategy,
    /// Control input (optional)
    pub control: Option<Vec<T>>,
    /// Time step
    pub dt: T,
}

impl<T, S> ExtendedInformationFilter<T, S>
where
    T: KalmanScalar,
    S: NonlinearSystem<T>,
{
    /// Create new Extended Information Filter
    pub fn new(
        system: S,
        initial_Y: Vec<T>,
        initial_y: Vec<T>,
        process_noise: Vec<T>,
        measurement_noise: Vec<T>,
        dt: T,
    ) -> KalmanResult<Self> {
        let n = system.state_dim();
        let m = system.measurement_dim();
        
        // Validate dimensions
        if initial_Y.len() != n * n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, n),
                actual: (initial_Y.len() / n, n),
            });
        }
        if initial_y.len() != n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, 1),
                actual: (initial_y.len(), 1),
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
        
        let state = InformationState::from_information(initial_Y, initial_y)?;
        
        Ok(Self {
            system,
            state,
            state_dim: n,
            measurement_dim: m,
            Q: process_noise,
            R: measurement_noise,
            jacobian_strategy: JacobianStrategy::Analytical,
            control: None,
            dt,
        })
    }
    
    /// Set Jacobian computation strategy
    pub fn set_jacobian_strategy(&mut self, strategy: JacobianStrategy) {
        self.jacobian_strategy = strategy;
    }
    
    /// Set control input
    pub fn set_control(&mut self, control: Vec<T>) {
        self.control = Some(control);
    }
    
    /// Nonlinear predict step in information form
    pub fn predict(&mut self) -> KalmanResult<()> {
        let n = self.state_dim;
        
        // Step 1: Recover current state estimate
        let x_prev = self.state.recover_state()?;
        
        // Step 2: Propagate state through nonlinear function
        let x_pred = self.system.state_transition(&x_prev, self.control.as_deref(), self.dt);
        
        // Step 3: Get state transition Jacobian at current state
        let F = match self.jacobian_strategy {
            JacobianStrategy::Analytical => {
                self.system.state_jacobian(&x_prev, self.control.as_deref(), self.dt)
            },
            JacobianStrategy::Numerical { step_size } => {
                self.compute_numerical_jacobian_state(&x_prev, step_size)
            },
            _ => {
                self.system.state_jacobian(&x_prev, self.control.as_deref(), self.dt)
            }
        };
        
        // Step 4: Compute predicted information matrix
        // Y_k|k-1 = (F·Y_k-1^-1·F^T + Q)^-1
        
        // First recover covariance P_k-1 = Y_k-1^-1
        let P_prev = self.state.recover_covariance()?;
        
        // Compute F·P·F^T + Q
        let mut FP = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    FP[i * n + j] = FP[i * n + j] + F[i * n + k] * P_prev[k * n + j];
                }
            }
        }
        
        let mut P_pred = self.Q.clone();
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    P_pred[i * n + j] = P_pred[i * n + j] + FP[i * n + k] * F[j * n + k];
                }
            }
        }
        
        // Y_k|k-1 = P_pred^-1
        let Y_pred = KalmanFilter::<T>::invert_matrix(&P_pred, n)?;
        
        // Step 5: Compute predicted information vector
        // y_k|k-1 = Y_k|k-1·x_pred
        let mut y_pred = vec![T::zero(); n];
        for i in 0..n {
            for j in 0..n {
                y_pred[i] = y_pred[i] + Y_pred[i * n + j] * x_pred[j];
            }
        }
        
        // Update state
        self.state.Y = Y_pred;
        self.state.y = y_pred;
        
        Ok(())
    }
    
    /// Nonlinear update step in information form
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        let n = self.state_dim;
        let m = self.measurement_dim;
        
        if measurement.len() != m {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, 1),
                actual: (measurement.len(), 1),
            });
        }
        
        // Step 1: Recover current state estimate for linearization
        let x_pred = self.state.recover_state()?;
        
        // Step 2: Get measurement Jacobian at predicted state
        let H = match self.jacobian_strategy {
            JacobianStrategy::Analytical => {
                self.system.measurement_jacobian(&x_pred)
            },
            JacobianStrategy::Numerical { step_size } => {
                self.compute_numerical_jacobian_measurement(&x_pred, step_size)
            },
            _ => {
                self.system.measurement_jacobian(&x_pred)
            }
        };
        
        // Step 3: Compute predicted measurement
        let h_x = self.system.measurement(&x_pred);
        
        // Step 4: Compute linearized innovation
        // For EIF, we need to adjust the measurement contribution
        // δy = H^T·R^-1·(z - h(x) + H·x)
        
        // Compute R^-1
        let R_inv = KalmanFilter::<T>::invert_matrix(&self.R, m)?;
        
        // Compute H·x
        let mut Hx = vec![T::zero(); m];
        for i in 0..m {
            for j in 0..n {
                Hx[i] = Hx[i] + H[i * n + j] * x_pred[j];
            }
        }
        
        // Compute adjusted measurement: z - h(x) + H·x
        let mut z_adj = vec![T::zero(); m];
        for i in 0..m {
            z_adj[i] = measurement[i] - h_x[i] + Hx[i];
        }
        
        // Compute H^T·R^-1
        let mut HtR_inv = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for k in 0..m {
                    HtR_inv[i * m + j] = HtR_inv[i * m + j] + H[k * n + i] * R_inv[k * m + j];
                }
            }
        }
        
        // Compute δy = H^T·R^-1·z_adj
        let mut delta_y = vec![T::zero(); n];
        for i in 0..n {
            for j in 0..m {
                delta_y[i] = delta_y[i] + HtR_inv[i * m + j] * z_adj[j];
            }
        }
        
        // Compute δY = H^T·R^-1·H
        let mut delta_Y = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..m {
                    delta_Y[i * n + j] = delta_Y[i * n + j] + HtR_inv[i * m + k] * H[k * n + j];
                }
            }
        }
        
        // Step 5: Update information state
        self.state.add_information(&delta_y, &delta_Y);
        
        Ok(())
    }
    
    /// Iterated Extended Information Filter update for better linearization
    pub fn iterated_update(&mut self, measurement: &[T], max_iterations: usize) -> KalmanResult<()> {
        let n = self.state_dim;
        let m = self.measurement_dim;
        
        if measurement.len() != m {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, 1),
                actual: (measurement.len(), 1),
            });
        }
        
        // Save initial information state
        let Y_init = self.state.Y.clone();
        let y_init = self.state.y.clone();
        
        for iteration in 0..max_iterations {
            // Get current state estimate
            let x_current = self.state.recover_state()?;
            
            // Reset to initial information
            self.state.Y = Y_init.clone();
            self.state.y = y_init.clone();
            
            // Perform update with current linearization point
            self.update(measurement)?;
            
            // Check convergence
            let x_new = self.state.recover_state()?;
            let mut converged = true;
            for i in 0..n {
                if (x_new[i] - x_current[i]).abs() > T::from(1e-6).unwrap() {
                    converged = false;
                    break;
                }
            }
            
            if converged {
                break;
            }
        }
        
        Ok(())
    }
    
    /// Compute numerical Jacobian for state transition
    fn compute_numerical_jacobian_state(&self, state: &[T], step_size: f64) -> Vec<T> {
        let n = self.state_dim;
        let step = T::from(step_size).unwrap();
        let mut jacobian = vec![T::zero(); n * n];
        
        for j in 0..n {
            let mut state_plus = state.to_vec();
            let mut state_minus = state.to_vec();
            state_plus[j] = state_plus[j] + step;
            state_minus[j] = state_minus[j] - step;
            
            let f_plus = self.system.state_transition(&state_plus, self.control.as_deref(), self.dt);
            let f_minus = self.system.state_transition(&state_minus, self.control.as_deref(), self.dt);
            
            for i in 0..n {
                jacobian[i * n + j] = (f_plus[i] - f_minus[i]) / (step + step);
            }
        }
        
        jacobian
    }
    
    /// Compute numerical Jacobian for measurement
    fn compute_numerical_jacobian_measurement(&self, state: &[T], step_size: f64) -> Vec<T> {
        let n = self.state_dim;
        let m = self.measurement_dim;
        let step = T::from(step_size).unwrap();
        let mut jacobian = vec![T::zero(); m * n];
        
        for j in 0..n {
            let mut state_plus = state.to_vec();
            let mut state_minus = state.to_vec();
            state_plus[j] = state_plus[j] + step;
            state_minus[j] = state_minus[j] - step;
            
            let h_plus = self.system.measurement(&state_plus);
            let h_minus = self.system.measurement(&state_minus);
            
            for i in 0..m {
                jacobian[i * n + j] = (h_plus[i] - h_minus[i]) / (step + step);
            }
        }
        
        jacobian
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
    
    /// Simple nonlinear system for testing
    struct SimpleNonlinearSystem;
    
    impl NonlinearSystem<f64> for SimpleNonlinearSystem {
        fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
            vec![
                state[0] + state[1] * dt,
                state[1] + 0.1 * state[0].sin() * dt,  // Nonlinear term
            ]
        }
        
        fn measurement(&self, state: &[f64]) -> Vec<f64> {
            vec![state[0].powi(2)]  // Nonlinear measurement
        }
        
        fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
            vec![
                1.0, dt,
                0.1 * state[0].cos() * dt, 1.0,
            ]
        }
        
        fn measurement_jacobian(&self, state: &[f64]) -> Vec<f64> {
            vec![2.0 * state[0], 0.0]
        }
        
        fn state_dim(&self) -> usize { 2 }
        fn measurement_dim(&self) -> usize { 1 }
    }
    
    #[test]
    fn test_extended_information_filter() {
        let system = SimpleNonlinearSystem;
        
        // Initialize with small uncertainty
        let Y_init = vec![100.0, 0.0, 0.0, 100.0];  // P^-1 with P = 0.01*I
        let y_init = vec![10.0, 0.0];  // Y·x with x = [0.1, 0]
        
        let mut eif = ExtendedInformationFilter::new(
            system,
            Y_init,
            y_init,
            vec![0.001, 0.0, 0.0, 0.001],  // Q
            vec![0.01],  // R
            0.1,  // dt
        ).unwrap();
        
        // Predict and update
        eif.predict().unwrap();
        eif.update(&[0.01]).unwrap();  // Measurement of x^2
        
        // Check state estimate
        let state = eif.get_state().unwrap();
        assert!(state[0] > 0.08 && state[0] < 0.12);  // Should be close to 0.1
    }
    
    #[test]
    fn test_iterated_eif() {
        let system = SimpleNonlinearSystem;
        
        let Y_init = vec![100.0, 0.0, 0.0, 100.0];
        let y_init = vec![50.0, 0.0];  // x = [0.5, 0]
        
        let mut eif = ExtendedInformationFilter::new(
            system,
            Y_init,
            y_init,
            vec![0.001, 0.0, 0.0, 0.001],
            vec![0.01],
            0.1,
        ).unwrap();
        
        // Iterated update for better linearization
        eif.iterated_update(&[0.25], 5).unwrap();  // Measurement of 0.5^2
        
        let state = eif.get_state().unwrap();
        assert!((state[0] - 0.5).abs() < 0.1);
    }
}
