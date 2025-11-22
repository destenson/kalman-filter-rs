//! Unified Kalman filter interface that works across all backends
//!
//! This module provides a backend-agnostic API that allows users to write
//! the same code regardless of whether the `nalgebra` feature is enabled.

use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use crate::filter::KalmanFilter;

#[cfg(feature = "nalgebra")]
use crate::filter::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::RealField;

/// Backend-agnostic Kalman filter wrapper
///
/// This enum wraps different backend implementations and provides
/// a unified interface through the `KalmanFilterTrait` trait.
///
/// Users don't typically construct this directly - use `KalmanFilterBuilder`
/// which automatically selects the optimal backend.
#[cfg(not(feature = "nalgebra"))]
pub enum UnifiedKalmanFilter<T: KalmanScalar> {
    /// Vec-based dynamic implementation
    Vec(KalmanFilter<T>),
}

#[cfg(feature = "nalgebra")]
pub enum UnifiedKalmanFilter<T>
where
    T: KalmanScalar + RealField,
{
    /// Vec-based dynamic implementation
    Vec(KalmanFilter<T>),

    /// Nalgebra-based static implementations for common dimensions
    Static1x1(StaticKalmanFilter<T, 1, 1>),
    Static2x1(StaticKalmanFilter<T, 1, 2>),
    Static3x1(StaticKalmanFilter<T, 1, 3>),
    Static4x2(StaticKalmanFilter<T, 2, 4>),
    Static6x3(StaticKalmanFilter<T, 3, 6>),
    // Add more common sizes as needed
}

/// Trait for converting backend types into unified wrapper
///
/// Similar to axum's `IntoResponse`, this allows the builder
/// to return different backend types that all convert into
/// `UnifiedKalmanFilter`.
#[cfg(not(feature = "nalgebra"))]
pub trait IntoUnified<T: KalmanScalar> {
    /// Convert into unified wrapper
    fn into_unified(self) -> UnifiedKalmanFilter<T>;
}

#[cfg(feature = "nalgebra")]
pub trait IntoUnified<T>
where
    T: KalmanScalar + RealField,
{
    /// Convert into unified wrapper
    fn into_unified(self) -> UnifiedKalmanFilter<T>;
}

/// Unified interface for Kalman filter implementations
///
/// This trait provides a consistent API that works with both Vec-based
/// and nalgebra-based backends. Users interact with filters through slices
/// (`&[T]`) rather than backend-specific types.
///
/// # Example
///
/// ```
/// use kalman_filters::KalmanFilterBuilder;
///
/// let mut kf = KalmanFilterBuilder::new(2, 1)
///     .initial_state(vec![0.0, 0.0])
///     .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
///     .transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
///     .process_noise(vec![0.001, 0.0, 0.0, 0.001])
///     .observation_matrix(vec![1.0, 0.0])
///     .measurement_noise(vec![0.1])
///     .build()
///     .unwrap();
///
/// // Works the same way regardless of backend!
/// kf.predict();
/// kf.update(&[1.0]).unwrap();
/// let state = kf.state();
/// ```
pub trait KalmanFilterTrait<T: KalmanScalar> {
    /// Perform prediction step
    ///
    /// Propagates the state estimate forward in time using the system model:
    /// - x = F * x  (state prediction)
    /// - P = F * P * F^T + Q  (covariance prediction)
    fn predict(&mut self);

    /// Perform update step with measurement
    ///
    /// Incorporates a new measurement to refine the state estimate:
    /// - y = z - H * x  (innovation)
    /// - S = H * P * H^T + R  (innovation covariance)
    /// - K = P * H^T * S^-1  (Kalman gain)
    /// - x = x + K * y  (state update)
    /// - P = (I - K * H) * P  (covariance update)
    ///
    /// # Arguments
    ///
    /// * `measurement` - Measurement vector as a slice
    ///
    /// # Errors
    ///
    /// Returns `KalmanError::DimensionMismatch` if measurement size doesn't match
    /// Returns `KalmanError::SingularMatrix` if innovation covariance is not invertible
    fn update(&mut self, measurement: &[T]) -> KalmanResult<()>;

    /// Get current state estimate
    ///
    /// Returns the current state vector as a `Vec<T>`. This may involve
    /// copying from internal representation.
    fn state(&self) -> Vec<T>;

    /// Get current state covariance
    ///
    /// Returns the current state covariance matrix as a `Vec<T>` in row-major order.
    /// This may involve copying from internal representation.
    fn covariance(&self) -> Vec<T>;

    /// Get state dimension
    fn state_dim(&self) -> usize;

    /// Get measurement dimension
    fn measurement_dim(&self) -> usize;

    /// Set the state transition matrix
    ///
    /// # Arguments
    ///
    /// * `F` - State transition matrix in row-major order
    ///
    /// # Errors
    ///
    /// Returns `KalmanError::DimensionMismatch` if F is not N x N
    fn set_transition_matrix(&mut self, F: &[T]) -> KalmanResult<()>;

    /// Set the process noise covariance
    ///
    /// # Arguments
    ///
    /// * `Q` - Process noise covariance in row-major order
    ///
    /// # Errors
    ///
    /// Returns `KalmanError::DimensionMismatch` if Q is not N x N
    fn set_process_noise(&mut self, Q: &[T]) -> KalmanResult<()>;

    /// Set the observation matrix
    ///
    /// # Arguments
    ///
    /// * `H` - Observation matrix in row-major order
    ///
    /// # Errors
    ///
    /// Returns `KalmanError::DimensionMismatch` if H is not M x N
    fn set_observation_matrix(&mut self, H: &[T]) -> KalmanResult<()>;

    /// Set the measurement noise covariance
    ///
    /// # Arguments
    ///
    /// * `R` - Measurement noise covariance in row-major order
    ///
    /// # Errors
    ///
    /// Returns `KalmanError::DimensionMismatch` if R is not M x M
    fn set_measurement_noise(&mut self, R: &[T]) -> KalmanResult<()>;
}

// Implement IntoUnified for Vec-based KalmanFilter
#[cfg(not(feature = "nalgebra"))]
impl<T: KalmanScalar> IntoUnified<T> for KalmanFilter<T> {
    fn into_unified(self) -> UnifiedKalmanFilter<T> {
        UnifiedKalmanFilter::Vec(self)
    }
}

#[cfg(feature = "nalgebra")]
impl<T> IntoUnified<T> for KalmanFilter<T>
where
    T: KalmanScalar + RealField,
{
    fn into_unified(self) -> UnifiedKalmanFilter<T> {
        UnifiedKalmanFilter::Vec(self)
    }
}

// Implement IntoUnified for nalgebra static filters
#[cfg(feature = "nalgebra")]
impl<T: KalmanScalar + RealField> IntoUnified<T> for StaticKalmanFilter<T, 1, 1> {
    fn into_unified(self) -> UnifiedKalmanFilter<T> {
        UnifiedKalmanFilter::Static1x1(self)
    }
}

#[cfg(feature = "nalgebra")]
impl<T: KalmanScalar + RealField> IntoUnified<T> for StaticKalmanFilter<T, 1, 2> {
    fn into_unified(self) -> UnifiedKalmanFilter<T> {
        UnifiedKalmanFilter::Static2x1(self)
    }
}

#[cfg(feature = "nalgebra")]
impl<T: KalmanScalar + RealField> IntoUnified<T> for StaticKalmanFilter<T, 1, 3> {
    fn into_unified(self) -> UnifiedKalmanFilter<T> {
        UnifiedKalmanFilter::Static3x1(self)
    }
}

#[cfg(feature = "nalgebra")]
impl<T: KalmanScalar + RealField> IntoUnified<T> for StaticKalmanFilter<T, 2, 4> {
    fn into_unified(self) -> UnifiedKalmanFilter<T> {
        UnifiedKalmanFilter::Static4x2(self)
    }
}

#[cfg(feature = "nalgebra")]
impl<T: KalmanScalar + RealField> IntoUnified<T> for StaticKalmanFilter<T, 3, 6> {
    fn into_unified(self) -> UnifiedKalmanFilter<T> {
        UnifiedKalmanFilter::Static6x3(self)
    }
}

// Implement KalmanFilterTrait for UnifiedKalmanFilter (delegates to wrapped type)
#[cfg(not(feature = "nalgebra"))]
impl<T: KalmanScalar> KalmanFilterTrait<T> for UnifiedKalmanFilter<T> {
    fn predict(&mut self) {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.predict(),
        }
    }

    fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.update(measurement),
        }
    }

    fn state(&self) -> Vec<T> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.x.clone(),
        }
    }

    fn covariance(&self) -> Vec<T> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.P.clone(),
        }
    }

    fn state_dim(&self) -> usize {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.state_dim,
        }
    }

    fn measurement_dim(&self) -> usize {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.measurement_dim,
        }
    }

    fn set_transition_matrix(&mut self, F: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_transition_matrix(F)
            }
        }
    }

    fn set_process_noise(&mut self, Q: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_process_noise(Q)
            }
        }
    }

    fn set_observation_matrix(&mut self, H: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_observation_matrix(H)
            }
        }
    }

    fn set_measurement_noise(&mut self, R: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_measurement_noise(R)
            }
        }
    }
}

#[cfg(feature = "nalgebra")]
impl<T> KalmanFilterTrait<T> for UnifiedKalmanFilter<T>
where
    T: KalmanScalar + RealField,
{
    fn predict(&mut self) {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.predict(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => kf.predict(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => kf.predict(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => kf.predict(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => kf.predict(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => kf.predict(),
        }
    }

    fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.update(measurement),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.update(measurement)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.update(measurement)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.update(measurement)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.update(measurement)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.update(measurement)
            }
        }
    }

    fn state(&self) -> Vec<T> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.x.clone(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => kf.x.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => kf.x.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => kf.x.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => kf.x.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => kf.x.as_slice().to_vec(),
        }
    }

    fn covariance(&self) -> Vec<T> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.P.clone(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => kf.P.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => kf.P.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => kf.P.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => kf.P.as_slice().to_vec(),
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => kf.P.as_slice().to_vec(),
        }
    }

    fn state_dim(&self) -> usize {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.state_dim,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(_) => 1,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(_) => 2,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(_) => 3,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(_) => 4,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(_) => 6,
        }
    }

    fn measurement_dim(&self) -> usize {
        match self {
            UnifiedKalmanFilter::Vec(kf) => kf.measurement_dim,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(_) => 1,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(_) => 1,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(_) => 1,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(_) => 2,
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(_) => 3,
        }
    }

    fn set_transition_matrix(&mut self, F: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_transition_matrix(F)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_transition_matrix(F)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_transition_matrix(F)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_transition_matrix(F)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_transition_matrix(F)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_transition_matrix(F)
            }
        }
    }

    fn set_process_noise(&mut self, Q: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_process_noise(Q)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_process_noise(Q)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_process_noise(Q)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_process_noise(Q)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_process_noise(Q)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_process_noise(Q)
            }
        }
    }

    fn set_observation_matrix(&mut self, H: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_observation_matrix(H)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_observation_matrix(H)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_observation_matrix(H)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_observation_matrix(H)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_observation_matrix(H)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_observation_matrix(H)
            }
        }
    }

    fn set_measurement_noise(&mut self, R: &[T]) -> KalmanResult<()> {
        match self {
            UnifiedKalmanFilter::Vec(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_measurement_noise(R)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static1x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_measurement_noise(R)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static2x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_measurement_noise(R)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static3x1(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_measurement_noise(R)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static4x2(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_measurement_noise(R)
            }
            #[cfg(feature = "nalgebra")]
            UnifiedKalmanFilter::Static6x3(kf) => {
                let trait_impl: &mut dyn KalmanFilterTrait<T> = kf;
                trait_impl.set_measurement_noise(R)
            }
        }
    }
}

// Keep original trait implementations for direct backend use
impl<T: KalmanScalar> KalmanFilterTrait<T> for KalmanFilter<T> {
    fn predict(&mut self) {
        KalmanFilter::predict(self);
    }

    fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        KalmanFilter::update(self, measurement)
    }

    fn state(&self) -> Vec<T> {
        self.x.clone()
    }

    fn covariance(&self) -> Vec<T> {
        self.P.clone()
    }

    fn state_dim(&self) -> usize {
        self.state_dim
    }

    fn measurement_dim(&self) -> usize {
        self.measurement_dim
    }

    fn set_transition_matrix(&mut self, F: &[T]) -> KalmanResult<()> {
        if F.len() != self.state_dim * self.state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (self.state_dim, self.state_dim),
                actual: (F.len() / self.state_dim, self.state_dim),
            });
        }
        self.F.copy_from_slice(F);
        Ok(())
    }

    fn set_process_noise(&mut self, Q: &[T]) -> KalmanResult<()> {
        if Q.len() != self.state_dim * self.state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (self.state_dim, self.state_dim),
                actual: (Q.len() / self.state_dim, self.state_dim),
            });
        }
        self.Q.copy_from_slice(Q);
        Ok(())
    }

    fn set_observation_matrix(&mut self, H: &[T]) -> KalmanResult<()> {
        if H.len() != self.measurement_dim * self.state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (self.measurement_dim, self.state_dim),
                actual: (H.len() / self.state_dim, self.state_dim),
            });
        }
        self.H.copy_from_slice(H);
        Ok(())
    }

    fn set_measurement_noise(&mut self, R: &[T]) -> KalmanResult<()> {
        if R.len() != self.measurement_dim * self.measurement_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (self.measurement_dim, self.measurement_dim),
                actual: (R.len() / self.measurement_dim, self.measurement_dim),
            });
        }
        self.R.copy_from_slice(R);
        Ok(())
    }
}

// Implementation for nalgebra-based StaticKalmanFilter
#[cfg(feature = "nalgebra")]
mod nalgebra_impl {
    use super::*;
    use crate::filter::StaticKalmanFilter;
    use nalgebra::{RealField, SMatrix, SVector};

    impl<T, const M: usize, const N: usize> KalmanFilterTrait<T> for StaticKalmanFilter<T, M, N>
    where
        T: KalmanScalar + RealField,
    {
        fn predict(&mut self) {
            StaticKalmanFilter::predict(self);
        }

        fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
            if measurement.len() != M {
                return Err(KalmanError::DimensionMismatch {
                    expected: (M, 1),
                    actual: (measurement.len(), 1),
                });
            }
            let z = SVector::<T, M>::from_row_slice(measurement);
            StaticKalmanFilter::update(self, &z)
        }

        fn state(&self) -> Vec<T> {
            self.x.as_slice().to_vec()
        }

        fn covariance(&self) -> Vec<T> {
            self.P.as_slice().to_vec()
        }

        fn state_dim(&self) -> usize {
            N
        }

        fn measurement_dim(&self) -> usize {
            M
        }

        fn set_transition_matrix(&mut self, F: &[T]) -> KalmanResult<()> {
            if F.len() != N * N {
                return Err(KalmanError::DimensionMismatch {
                    expected: (N, N),
                    actual: (F.len() / N, N),
                });
            }
            self.F = SMatrix::<T, N, N>::from_row_slice(F);
            Ok(())
        }

        fn set_process_noise(&mut self, Q: &[T]) -> KalmanResult<()> {
            if Q.len() != N * N {
                return Err(KalmanError::DimensionMismatch {
                    expected: (N, N),
                    actual: (Q.len() / N, N),
                });
            }
            self.Q = SMatrix::<T, N, N>::from_row_slice(Q);
            Ok(())
        }

        fn set_observation_matrix(&mut self, H: &[T]) -> KalmanResult<()> {
            if H.len() != M * N {
                return Err(KalmanError::DimensionMismatch {
                    expected: (M, N),
                    actual: (H.len() / N, N),
                });
            }
            self.H = SMatrix::<T, M, N>::from_row_slice(H);
            Ok(())
        }

        fn set_measurement_noise(&mut self, R: &[T]) -> KalmanResult<()> {
            if R.len() != M * M {
                return Err(KalmanError::DimensionMismatch {
                    expected: (M, M),
                    actual: (R.len() / M, M),
                });
            }
            self.R = SMatrix::<T, M, M>::from_row_slice(R);
            Ok(())
        }
    }
}
