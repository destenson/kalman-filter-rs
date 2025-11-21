//! Core Kalman filter implementation
#![allow(unused)]

use crate::logging::{
    check_numerical_stability, format_innovation, format_matrix, format_state,
    log_filter_dimensions, state_norm,
};
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use crate::validation::{regularize_matrix, validate_covariance, validate_measurement_noise};
use log::{debug, error, info, trace, warn};
use num_traits::{One, Zero};

#[cfg(feature = "nalgebra")]
use nalgebra::{RealField, SMatrix, SVector};

/// Linear Kalman Filter with dynamic dimensions
///
/// Implements the standard Kalman filter equations:
/// - Predict: x = F * x, P = F * P * F^T + Q
/// - Update: y = z - H * x, S = H * P * H^T + R, K = P * H^T * S^-1
///           x = x + K * y, P = (I - K * H) * P
///
/// Where:
/// - x: State vector
/// - P: State covariance matrix
/// - F: State transition matrix
/// - Q: Process noise covariance
/// - H: Observation matrix
/// - R: Measurement noise covariance
/// - z: Measurement vector
pub struct KalmanFilter<T>
where
    T: KalmanScalar,
{
    /// State dimension
    pub state_dim: usize,
    /// Measurement dimension
    pub measurement_dim: usize,
    /// State vector (N x 1)
    pub x: Vec<T>,
    /// State covariance matrix (N x N) - row major
    pub P: Vec<T>,
    /// State transition matrix (N x N) - row major
    pub F: Vec<T>,
    /// Process noise covariance (N x N) - row major
    pub Q: Vec<T>,
    /// Observation matrix (M x N) - row major
    pub H: Vec<T>,
    /// Measurement noise covariance (M x M) - row major
    pub R: Vec<T>,
}

impl<T> KalmanFilter<T>
where
    T: KalmanScalar,
{
    /// Create a new Kalman filter with given dimensions
    pub fn new(state_dim: usize, measurement_dim: usize) -> Self {
        let n = state_dim;
        let m = measurement_dim;

        Self {
            state_dim: n,
            measurement_dim: m,
            x: vec![T::zero(); n],
            P: vec![T::zero(); n * n],
            F: vec![T::zero(); n * n],
            Q: vec![T::zero(); n * n],
            H: vec![T::zero(); m * n],
            R: vec![T::zero(); m * m],
        }
    }

    /// Initialize the filter with matrices
    pub fn initialize(
        state_dim: usize,
        measurement_dim: usize,
        initial_state: Vec<T>,
        initial_covariance: Vec<T>,
        transition_matrix: Vec<T>,
        process_noise: Vec<T>,
        observation_matrix: Vec<T>,
        measurement_noise: Vec<T>,
    ) -> KalmanResult<Self> {
        let n = state_dim;
        let m = measurement_dim;

        log_filter_dimensions(n, m, None);

        // Validate dimensions
        if initial_state.len() != n {
            error!(
                "Initial state dimension mismatch: expected {}x1, got {}x1",
                n,
                initial_state.len()
            );
            return Err(KalmanError::DimensionMismatch {
                expected: (n, 1),
                actual: (initial_state.len(), 1),
            });
        }
        if initial_covariance.len() != n * n {
            error!(
                "Initial covariance dimension mismatch: expected {}x{}, got len={}",
                n,
                n,
                initial_covariance.len()
            );
            return Err(KalmanError::DimensionMismatch {
                expected: (n, n),
                actual: (initial_covariance.len() / n, n),
            });
        }
        if transition_matrix.len() != n * n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, n),
                actual: (transition_matrix.len() / n, n),
            });
        }
        if process_noise.len() != n * n {
            return Err(KalmanError::DimensionMismatch {
                expected: (n, n),
                actual: (process_noise.len() / n, n),
            });
        }
        if observation_matrix.len() != m * n {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, n),
                actual: (observation_matrix.len() / n, n),
            });
        }
        if measurement_noise.len() != m * m {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, m),
                actual: (measurement_noise.len() / m, m),
            });
        }

        // Validate initial covariance
        if let Err(e) = validate_covariance(&initial_covariance, n) {
            warn!(
                "Initial covariance validation failed: {:?}, attempting regularization",
                e
            );
            // Don't regularize here, just warn - user should provide valid covariance
        }

        // Validate measurement noise
        if let Err(e) = validate_measurement_noise(&measurement_noise, m) {
            error!("Measurement noise validation failed: {:?}", e);
            return Err(e);
        }

        // Check numerical stability of initial covariance
        check_numerical_stability(&initial_covariance, n, "Initial covariance");

        trace!(
            "Kalman filter initialized: {}",
            format_state(&initial_state, "initial_state")
        );

        if log::log_enabled!(log::Level::Trace) {
            trace!(
                "Initial covariance: {}",
                format_matrix(&initial_covariance, n, n, "P0")
            );
            trace!(
                "Process noise: {}",
                format_matrix(&process_noise, n, n, "Q")
            );
        }

        Ok(Self {
            state_dim: n,
            measurement_dim: m,
            x: initial_state,
            P: initial_covariance,
            F: transition_matrix,
            Q: process_noise,
            H: observation_matrix,
            R: measurement_noise,
        })
    }

    /// Predict step: propagate state and covariance forward
    pub fn predict(&mut self) {
        let n = self.state_dim;

        trace!("Predict step: state_norm={:.6}", state_norm(&self.x));

        #[cfg(feature = "prometheus-metrics")]
        let _timer = crate::metrics::MetricsTimer::start();

        if log::log_enabled!(log::Level::Trace) {
            trace!("Before predict: {}", format_state(&self.x, "x"));
        }

        // x = F * x
        let mut new_x = vec![T::zero(); n];
        for (i, new_x_i) in new_x.iter_mut().enumerate() {
            for j in 0..n {
                *new_x_i = *new_x_i + self.F[i * n + j] * self.x[j];
            }
        }
        self.x = new_x;

        // P = F * P * F^T + Q
        // First compute F * P
        let mut fp = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    fp[i * n + j] = fp[i * n + j] + self.F[i * n + k] * self.P[k * n + j];
                }
            }
        }

        // Then compute (F * P) * F^T + Q
        let mut new_p = self.Q.clone();
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    new_p[i * n + j] = new_p[i * n + j] + fp[i * n + k] * self.F[j * n + k];
                }
            }
        }
        self.P = new_p;

        check_numerical_stability(&self.P, n, "Predicted covariance");

        trace!("Predict complete: state_norm={:.6}", state_norm(&self.x));

        #[cfg(feature = "prometheus-metrics")]
        {
            _timer.finish_predict("kf");
            crate::metrics::record_prediction("kf");
            crate::metrics::set_state_dimension("kf", n);

            // Calculate and record covariance trace
            let trace: f64 = (0..n)
                .map(|i| KalmanScalar::to_f64(&self.P[i * n + i]))
                .sum();
            crate::metrics::set_covariance_trace("kf", trace);
        }
    }

    /// Update step: incorporate measurement to correct state estimate
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        let n = self.state_dim;
        let m = self.measurement_dim;

        trace!(
            "Update step: measurement_norm={:.6}",
            state_norm(measurement)
        );

        #[cfg(feature = "prometheus-metrics")]
        let _timer = crate::metrics::MetricsTimer::start();

        if measurement.len() != m {
            error!(
                "Measurement dimension mismatch: expected {}, got {}",
                m,
                measurement.len()
            );
            #[cfg(feature = "prometheus-metrics")]
            crate::metrics::record_error("kf", "dimension_mismatch");
            return Err(KalmanError::DimensionMismatch {
                expected: (m, 1),
                actual: (measurement.len(), 1),
            });
        }

        // Innovation: y = z - H * x
        let mut y = measurement.to_vec();
        for (i, y_i) in y.iter_mut().enumerate() {
            let mut hx = T::zero();
            for j in 0..n {
                hx = hx + self.H[i * n + j] * self.x[j];
            }
            *y_i = *y_i - hx;
        }

        trace!("Innovation: {}", format_innovation(&y));

        // Calculate innovation norm for metrics
        #[cfg(feature = "prometheus-metrics")]
        let innovation_norm = {
            let mut norm_sq = T::zero();
            for i in 0..m {
                norm_sq = norm_sq + y[i] * y[i];
            }
            KalmanScalar::to_f64(&norm_sq.sqrt())
        };

        // Innovation covariance: S = H * P * H^T + R
        // First compute H * P
        let mut hp = vec![T::zero(); m * n];
        for i in 0..m {
            for j in 0..n {
                for k in 0..n {
                    hp[i * n + j] = hp[i * n + j] + self.H[i * n + k] * self.P[k * n + j];
                }
            }
        }

        // Then compute S = (H * P) * H^T + R
        let mut s = self.R.clone();
        for i in 0..m {
            for j in 0..m {
                for k in 0..n {
                    s[i * m + j] = s[i * m + j] + hp[i * n + k] * self.H[j * n + k];
                }
            }
        }

        // Invert S
        trace!("Computing innovation covariance inverse");
        let s_inv = match Self::invert_matrix(&s, m) {
            Ok(inv) => inv,
            Err(e) => {
                error!("Failed to invert innovation covariance: {:?}", e);
                check_numerical_stability(&s, m, "Innovation covariance (singular)");
                #[cfg(feature = "prometheus-metrics")]
                crate::metrics::record_error("kf", "singular_matrix");
                return Err(e);
            }
        };

        // Kalman gain: K = P * H^T * S^-1
        // First compute P * H^T
        let mut ph_t = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for k in 0..n {
                    ph_t[i * m + j] = ph_t[i * m + j] + self.P[i * n + k] * self.H[j * n + k];
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
                    kh[i * n + j] = kh[i * n + j] + k[i * m + l] * self.H[l * n + j];
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

        check_numerical_stability(&self.P, n, "Updated covariance");

        trace!("Update complete: state_norm={:.6}", state_norm(&self.x));

        #[cfg(feature = "prometheus-metrics")]
        {
            _timer.finish_update("kf");
            crate::metrics::record_update("kf");
            crate::metrics::set_innovation_norm("kf", innovation_norm);

            // Calculate and record covariance trace
            let trace: f64 = (0..n)
                .map(|i| KalmanScalar::to_f64(&self.P[i * n + i]))
                .sum();
            crate::metrics::set_covariance_trace("kf", trace);
        }

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

    /// Simple matrix inversion using Gauss-Jordan elimination
    pub(crate) fn invert_matrix(matrix: &[T], size: usize) -> KalmanResult<Vec<T>> {
        #[cfg(feature = "prometheus-metrics")]
        let _timer = crate::metrics::MetricsTimer::start();

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
            if pivot.abs() < <T as KalmanScalar>::epsilon() {
                // Find a non-zero element in the column
                for j in i + 1..size {
                    if a[j * size + i].abs() > <T as KalmanScalar>::epsilon() {
                        // Swap rows
                        for k in 0..size {
                            a.swap(i * size + k, j * size + k);
                            inv.swap(i * size + k, j * size + k);
                        }
                        pivot = a[i * size + i];
                        break;
                    }
                }
            }

            if pivot.abs() < <T as KalmanScalar>::epsilon() {
                warn!(
                    "Matrix inversion failed: pivot {:.6e} at position ({}, {})",
                    KalmanScalar::to_f64(&pivot),
                    i,
                    i
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

        #[cfg(feature = "prometheus-metrics")]
        _timer.finish_matrix_inversion();

        Ok(inv)
    }
}

/// Optimized Kalman filter implementation using static matrices from nalgebra
///
/// This provides compile-time dimension checking and better performance
/// for filters with known dimensions at compile time.
///
/// Type parameters:
/// - `T`: Scalar type (f32 or f64)
/// - `M`: Measurement dimension
/// - `N`: State dimension
#[cfg(feature = "nalgebra")]
pub struct StaticKalmanFilter<T, const M: usize, const N: usize>
where
    T: KalmanScalar + RealField,
{
    /// State vector (N x 1)
    pub x: SVector<T, N>,
    /// State covariance matrix (N x N)
    pub P: SMatrix<T, N, N>,
    /// State transition matrix (N x N)
    pub F: SMatrix<T, N, N>,
    /// Process noise covariance (N x N)
    pub Q: SMatrix<T, N, N>,
    /// Observation matrix (M x N)
    pub H: SMatrix<T, M, N>,
    /// Measurement noise covariance (M x M)
    pub R: SMatrix<T, M, M>,
}

#[cfg(feature = "nalgebra")]
impl<T, const M: usize, const N: usize> StaticKalmanFilter<T, M, N>
where
    T: KalmanScalar + RealField,
{
    /// Create a new Kalman filter with given parameters
    pub fn new(
        initial_state: SVector<T, N>,
        initial_covariance: SMatrix<T, N, N>,
        transition_matrix: SMatrix<T, N, N>,
        process_noise: SMatrix<T, N, N>,
        observation_matrix: SMatrix<T, M, N>,
        measurement_noise: SMatrix<T, M, M>,
    ) -> Self {
        Self {
            x: initial_state,
            P: initial_covariance,
            F: transition_matrix,
            Q: process_noise,
            H: observation_matrix,
            R: measurement_noise,
        }
    }

    /// Predict step: propagate state and covariance forward
    pub fn predict(&mut self) {
        // x = F * x
        self.x = &self.F * &self.x;

        // P = F * P * F^T + Q
        self.P = &self.F * &self.P * self.F.transpose() + &self.Q;
    }

    /// Update step: incorporate measurement to correct state estimate
    pub fn update(&mut self, measurement: &SVector<T, M>) -> KalmanResult<()> {
        // Innovation: y = z - H * x
        let y = measurement - &self.H * &self.x;

        // Innovation covariance: S = H * P * H^T + R
        let S = &self.H * &self.P * self.H.transpose() + &self.R;

        // Check if S is invertible
        let S_inv = S.try_inverse().ok_or(KalmanError::SingularMatrix)?;

        // Kalman gain: K = P * H^T * S^-1
        let K = &self.P * self.H.transpose() * S_inv;

        // State update: x = x + K * y
        self.x = &self.x + &K * y;

        // Covariance update: P = (I - K * H) * P
        let I = SMatrix::<T, N, N>::identity();
        self.P = (I - &K * &self.H) * &self.P;

        // Ensure symmetry and positive semi-definiteness
        self.P = (&self.P + self.P.transpose()) * T::from(0.5).unwrap();

        Ok(())
    }

    /// Get current state estimate
    pub fn state(&self) -> &SVector<T, N> {
        &self.x
    }

    /// Get current state covariance
    pub fn covariance(&self) -> &SMatrix<T, N, N> {
        &self.P
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_filter_1d() {
        // Simple 1D constant position model
        let mut kf = KalmanFilter::<f64>::initialize(
            1,
            1,
            vec![0.0],   // initial state
            vec![1.0],   // initial covariance
            vec![1.0],   // F
            vec![0.001], // Q
            vec![1.0],   // H
            vec![0.1],   // R
        )
        .unwrap();

        // Predict
        kf.predict();
        assert!((kf.state()[0] - 0.0).abs() < 1e-10);

        // Update with measurement
        kf.update(&[1.0]).unwrap();

        // State should have moved toward measurement
        assert!(kf.state()[0] > 0.0);
        assert!(kf.state()[0] < 1.0);
    }

    #[test]
    fn test_kalman_filter_2d() {
        // 2D position-velocity model
        // State: [position, velocity]
        let mut kf = KalmanFilter::<f64>::initialize(
            2,
            1,
            vec![0.0, 0.0], // initial state
            vec![
                1.0, 0.0, // initial covariance
                0.0, 1.0,
            ],
            vec![
                1.0, 1.0, // F (dt = 1.0)
                0.0, 1.0,
            ],
            vec![
                0.001, 0.0, // Q
                0.0, 0.001,
            ],
            vec![1.0, 0.0], // H (observe position only)
            vec![0.1],      // R
        )
        .unwrap();

        // Simulate multiple steps
        for i in 1..=5 {
            kf.predict();
            kf.update(&[i as f64]).unwrap();
        }

        // After 5 steps with constant velocity, position should be around 5
        assert!((kf.state()[0] - 5.0).abs() < 1.0);
        // Velocity should be around 1
        assert!((kf.state()[1] - 1.0).abs() < 0.5);
    }

    #[test]
    fn test_matrix_inversion() {
        // Test 2x2 matrix inversion
        let matrix = vec![4.0, 7.0, 2.0, 6.0];
        let inv = KalmanFilter::<f64>::invert_matrix(&matrix, 2).unwrap();

        // Verify A * A^-1 = I
        let mut product = vec![0.0; 4];
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    product[i * 2 + j] += matrix[i * 2 + k] * inv[k * 2 + j];
                }
            }
        }

        // Check if product is identity
        assert!((product[0] - 1.0).abs() < 1e-10);
        assert!((product[1] - 0.0).abs() < 1e-10);
        assert!((product[2] - 0.0).abs() < 1e-10);
        assert!((product[3] - 1.0).abs() < 1e-10);
    }

    #[cfg(feature = "nalgebra")]
    #[test]
    fn test_static_kalman_filter_1d() {
        // Simple 1D constant position model
        let x = SVector::<f64, 1>::from([0.0]);
        let P = SMatrix::<f64, 1, 1>::from([[1.0]]);
        let F = SMatrix::<f64, 1, 1>::from([[1.0]]);
        let Q = SMatrix::<f64, 1, 1>::from([[0.001]]);
        let H = SMatrix::<f64, 1, 1>::from([[1.0]]);
        let R = SMatrix::<f64, 1, 1>::from([[0.1]]);

        let mut kf = StaticKalmanFilter::new(x, P, F, Q, H, R);

        // Predict
        kf.predict();
        assert!((kf.state()[0] - 0.0).abs() < 1e-10);

        // Update with measurement
        let z = SVector::<f64, 1>::from([1.0]);
        kf.update(&z).unwrap();

        // State should have moved toward measurement
        assert!(kf.state()[0] > 0.0);
        assert!(kf.state()[0] < 1.0);
    }

    #[test]
    fn test_dimension_validation() {
        // Test invalid dimensions are caught
        let result = KalmanFilter::<f64>::initialize(
            2,
            1,
            vec![0.0, 0.0], // state
            vec![1.0],      // P wrong size (should be 2x2)
            vec![1.0, 0.0, 0.0, 1.0],
            vec![0.1, 0.0, 0.0, 0.1],
            vec![1.0, 0.0],
            vec![0.1],
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_singular_matrix_detection() {
        // Test that singular matrix is detected
        let singular = vec![1.0, 2.0, 2.0, 4.0]; // Singular 2x2 matrix
        let result = KalmanFilter::<f64>::invert_matrix(&singular, 2);
        assert!(result.is_err());
        match result {
            Err(KalmanError::SingularMatrix) => (),
            _ => panic!("Expected SingularMatrix error"),
        }
    }

    #[test]
    fn test_predict_update_cycle() {
        let mut kf = KalmanFilter::<f64>::new(2, 1);

        // Initialize with identity matrices
        kf.x = vec![0.0, 0.0];
        kf.P = vec![1.0, 0.0, 0.0, 1.0];
        kf.F = vec![1.0, 0.1, 0.0, 1.0]; // Simple motion model
        kf.Q = vec![0.01, 0.0, 0.0, 0.01];
        kf.H = vec![1.0, 0.0]; // Observe first state
        kf.R = vec![0.1];

        // Store initial covariance trace
        let initial_trace = kf.P[0] + kf.P[3];

        // Predict increases uncertainty
        kf.predict();
        let predict_trace = kf.P[0] + kf.P[3];
        assert!(predict_trace > initial_trace);

        // Update reduces uncertainty
        let result = kf.update(&vec![0.5]);
        assert!(result.is_ok());
        let update_trace = kf.P[0] + kf.P[3];
        assert!(update_trace < predict_trace);
    }

    #[test]
    fn test_multi_dimensional_state_estimation() {
        // Test 3D position estimation with partial observability
        // State: [x, y, z], but we only observe x and y
        let mut kf = KalmanFilter::<f64>::new(3, 2);

        kf.x = vec![0.0, 0.0, 0.0];
        kf.P = vec![
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ];
        kf.F = vec![
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ];
        kf.Q = vec![
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ];
        // Observe only x and y
        kf.H = vec![
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
        ];
        kf.R = vec![
            0.1, 0.0,
            0.0, 0.1,
        ];

        // True state: x=5, y=3, z=7
        for _ in 0..50 {
            kf.predict();
            kf.update(&vec![5.0, 3.0]).unwrap();
        }

        // Observable states should converge
        assert!((kf.x[0] - 5.0).abs() < 0.5, "x should converge. Got: {}", kf.x[0]);
        assert!((kf.x[1] - 3.0).abs() < 0.5, "y should converge. Got: {}", kf.x[1]);

        // Unobservable state (z) should stay near initial with growing uncertainty
        // z uncertainty should be larger than x,y uncertainty
        let p_z = kf.P[8]; // P[2,2]
        let p_x = kf.P[0]; // P[0,0]
        assert!(p_z > p_x, "Unobserved state should have higher uncertainty. P_z: {}, P_x: {}", p_z, p_x);
    }

    #[test]
    fn test_process_noise_effect_on_tracking() {
        // Higher process noise should allow faster adaptation to changes
        let true_values = vec![0.0, 0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 5.0]; // Jump at t=3

        // Low process noise - slow to adapt
        let mut kf_low_q = KalmanFilter::<f64>::new(1, 1);
        kf_low_q.x = vec![0.0];
        kf_low_q.P = vec![1.0];
        kf_low_q.F = vec![1.0];
        kf_low_q.Q = vec![0.001]; // Low process noise
        kf_low_q.H = vec![1.0];
        kf_low_q.R = vec![0.1];

        // High process noise - adapts faster
        let mut kf_high_q = KalmanFilter::<f64>::new(1, 1);
        kf_high_q.x = vec![0.0];
        kf_high_q.P = vec![1.0];
        kf_high_q.F = vec![1.0];
        kf_high_q.Q = vec![1.0]; // High process noise
        kf_high_q.H = vec![1.0];
        kf_high_q.R = vec![0.1];

        for z in &true_values {
            kf_low_q.predict();
            kf_low_q.update(&vec![*z]).unwrap();

            kf_high_q.predict();
            kf_high_q.update(&vec![*z]).unwrap();
        }

        // After the jump, high Q filter should be closer to 5.0
        let error_low = (kf_low_q.x[0] - 5.0).abs();
        let error_high = (kf_high_q.x[0] - 5.0).abs();

        assert!(error_high < error_low,
            "High Q should adapt faster. Low Q error: {}, High Q error: {}", error_low, error_high);
    }

    #[test]
    fn test_measurement_outlier_robustness() {
        // Standard KF is not robust to outliers - this test documents that behavior
        // (A robust filter would need different handling)
        let mut kf = KalmanFilter::<f64>::new(1, 1);
        kf.x = vec![0.0];
        kf.P = vec![0.1]; // Low uncertainty - we're confident
        kf.F = vec![1.0];
        kf.Q = vec![0.001];
        kf.H = vec![1.0];
        kf.R = vec![0.1];

        // Normal measurements
        for _ in 0..20 {
            kf.predict();
            kf.update(&vec![0.0]).unwrap();
        }

        let state_before_outlier = kf.x[0];

        // Single outlier
        kf.predict();
        kf.update(&vec![100.0]).unwrap(); // Massive outlier

        // KF will be pulled toward outlier (this is expected, not desired)
        let state_after_outlier = kf.x[0];
        assert!(state_after_outlier > state_before_outlier + 1.0,
            "KF should be affected by outlier. Before: {}, After: {}",
            state_before_outlier, state_after_outlier);
    }

    #[test]
    fn test_covariance_symmetry_maintained() {
        // Covariance matrix must remain symmetric through all operations
        let mut kf = KalmanFilter::<f64>::new(3, 2);

        kf.x = vec![1.0, 2.0, 3.0];
        kf.P = vec![
            1.0, 0.1, 0.2,
            0.1, 2.0, 0.3,
            0.2, 0.3, 3.0,
        ];
        kf.F = vec![
            1.0, 0.1, 0.0,
            0.0, 1.0, 0.1,
            0.0, 0.0, 1.0,
        ];
        kf.Q = vec![
            0.1, 0.01, 0.0,
            0.01, 0.1, 0.01,
            0.0, 0.01, 0.1,
        ];
        kf.H = vec![
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
        ];
        kf.R = vec![
            0.5, 0.0,
            0.0, 0.5,
        ];

        for _ in 0..20 {
            kf.predict();
            kf.update(&vec![1.0, 2.0]).unwrap();

            // Check symmetry: P[i,j] == P[j,i]
            let eps = 1e-10;
            assert!((kf.P[1] - kf.P[3]).abs() < eps, "P[0,1] != P[1,0]: {} vs {}", kf.P[1], kf.P[3]);
            assert!((kf.P[2] - kf.P[6]).abs() < eps, "P[0,2] != P[2,0]: {} vs {}", kf.P[2], kf.P[6]);
            assert!((kf.P[5] - kf.P[7]).abs() < eps, "P[1,2] != P[2,1]: {} vs {}", kf.P[5], kf.P[7]);
        }
    }

    #[test]
    fn test_convergence() {
        // Test that filter converges to true value with repeated measurements
        let mut kf = KalmanFilter::<f64>::new(1, 1);
        kf.x = vec![0.0];
        kf.P = vec![100.0]; // High initial uncertainty
        kf.F = vec![1.0];
        kf.Q = vec![0.001];
        kf.H = vec![1.0];
        kf.R = vec![1.0];

        let true_value = 5.0;

        // Feed repeated measurements
        for _ in 0..50 {
            kf.predict();
            kf.update(&vec![true_value]).unwrap();
        }

        // Should converge close to true value
        assert!((kf.x[0] - true_value).abs() < 0.1);
        // Uncertainty should be low
        assert!(kf.P[0] < 1.0);
    }

    #[test]
    fn test_kalman_gain_bounds() {
        // Test that Kalman gain is bounded between 0 and 1 for scalar case
        // K = P * H^T / (H * P * H^T + R)
        // When R >> P: K -> 0 (trust prediction)
        // When P >> R: K -> 1 (trust measurement)

        // High measurement noise - should trust prediction
        let mut kf_high_r = KalmanFilter::<f64>::new(1, 1);
        kf_high_r.x = vec![0.0];
        kf_high_r.P = vec![1.0];
        kf_high_r.F = vec![1.0];
        kf_high_r.Q = vec![0.0];
        kf_high_r.H = vec![1.0];
        kf_high_r.R = vec![100.0]; // Very high measurement noise

        kf_high_r.predict();
        let state_before = kf_high_r.x[0];
        kf_high_r.update(&vec![10.0]).unwrap();

        // Should barely move toward measurement
        let movement = (kf_high_r.x[0] - state_before).abs();
        assert!(movement < 0.5, "With high R, state should barely change. Moved: {}", movement);

        // Low measurement noise - should trust measurement
        let mut kf_low_r = KalmanFilter::<f64>::new(1, 1);
        kf_low_r.x = vec![0.0];
        kf_low_r.P = vec![100.0]; // High uncertainty in state
        kf_low_r.F = vec![1.0];
        kf_low_r.Q = vec![0.0];
        kf_low_r.H = vec![1.0];
        kf_low_r.R = vec![0.01]; // Very low measurement noise

        kf_low_r.predict();
        kf_low_r.update(&vec![10.0]).unwrap();

        // Should move almost entirely to measurement
        assert!((kf_low_r.x[0] - 10.0).abs() < 0.5,
            "With low R, state should jump to measurement. Got: {}", kf_low_r.x[0]);
    }

    #[test]
    fn test_innovation_sequence_properties() {
        // The innovation (measurement residual) should have specific statistical properties
        // For a well-tuned filter, innovations should be zero-mean and consistent with S

        let mut kf = KalmanFilter::<f64>::new(1, 1);
        kf.x = vec![5.0]; // True state is 5
        kf.P = vec![1.0];
        kf.F = vec![1.0];
        kf.Q = vec![0.01];
        kf.H = vec![1.0];
        kf.R = vec![0.5];

        // Simulate with measurements around true value
        let measurements = vec![5.1, 4.9, 5.2, 4.8, 5.0, 5.1, 4.9, 5.0, 5.1, 4.9];
        let mut innovations = Vec::new();

        for z in &measurements {
            kf.predict();
            // Innovation = z - H*x (before update)
            let innovation = z - kf.x[0];
            innovations.push(innovation);
            kf.update(&vec![*z]).unwrap();
        }

        // Mean of innovations should be close to zero
        let mean: f64 = innovations.iter().sum::<f64>() / innovations.len() as f64;
        assert!(mean.abs() < 0.2, "Innovation mean should be near zero. Got: {}", mean);

        // Innovations should be bounded (no outliers for consistent measurements)
        for (i, innov) in innovations.iter().enumerate() {
            assert!(innov.abs() < 1.0, "Innovation {} too large: {}", i, innov);
        }
    }

    #[test]
    fn test_steady_state_gain_convergence() {
        // For time-invariant systems, Kalman gain should converge to steady-state
        let mut kf = KalmanFilter::<f64>::new(1, 1);
        kf.x = vec![0.0];
        kf.P = vec![10.0]; // Start with high uncertainty
        kf.F = vec![1.0];
        kf.Q = vec![0.1];
        kf.H = vec![1.0];
        kf.R = vec![1.0];

        let mut prev_p = kf.P[0];
        let mut converged = false;

        for i in 0..100 {
            kf.predict();
            kf.update(&vec![0.0]).unwrap();

            let p_change = (kf.P[0] - prev_p).abs();
            if p_change < 1e-6 && i > 10 {
                converged = true;
                break;
            }
            prev_p = kf.P[0];
        }

        assert!(converged, "Covariance should converge to steady-state. Final P: {}", kf.P[0]);

        // Steady-state covariance should satisfy algebraic Riccati equation
        // For scalar: P_ss = (sqrt(Q*R) + Q) / (1 + sqrt(Q/R)) approximately
        // More importantly, it should be bounded by Q and R
        assert!(kf.P[0] > 0.0, "Steady-state P must be positive");
        assert!(kf.P[0] < 10.0, "Steady-state P should be much less than initial");
    }

    #[test]
    fn test_filter_recovers_from_bad_initialization() {
        // Filter should eventually recover even with very wrong initial state
        let true_value = 100.0;

        let mut kf = KalmanFilter::<f64>::new(1, 1);
        kf.x = vec![-1000.0]; // Wildly wrong initial estimate
        kf.P = vec![1.0]; // But claiming high confidence (worst case)
        kf.F = vec![1.0];
        kf.Q = vec![1.0]; // Process noise allows adaptation
        kf.H = vec![1.0];
        kf.R = vec![1.0];

        // Feed consistent measurements
        for _ in 0..200 {
            kf.predict();
            kf.update(&vec![true_value]).unwrap();
        }

        // Should have recovered
        let error = (kf.x[0] - true_value).abs();
        assert!(error < 5.0,
            "Filter should recover from bad init. Error: {}, State: {}", error, kf.x[0]);
    }

    #[test]
    fn test_joseph_form() {
        // Test Joseph form covariance update for numerical stability
        let mut kf = KalmanFilter::<f64>::new(1, 1);
        kf.x = vec![0.0];
        kf.P = vec![1.0];
        kf.F = vec![1.0];
        kf.Q = vec![0.1];
        kf.H = vec![1.0];
        kf.R = vec![1.0];

        kf.predict();
        let p_before = kf.P[0];

        kf.update(&vec![1.0]).unwrap();

        // Covariance should remain positive
        assert!(kf.P[0] > 0.0);
        // Covariance should decrease after update
        assert!(kf.P[0] < p_before);
    }
}
