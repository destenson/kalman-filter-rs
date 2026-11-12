//! Builder pattern for Kalman filter construction

use crate::filter::KalmanFilter;
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use log::{debug, error, info, trace, warn};

#[cfg(feature = "nalgebra")]
use crate::filter::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::{RealField, SMatrix, SVector};

/// Builder for constructing Kalman filters
///
/// Provides a convenient way to construct Kalman filters with validation
///
/// # Example
/// ```
/// use kalman_filters::KalmanFilterBuilder;
///
/// let kf = KalmanFilterBuilder::<f64>::new(2, 1)  // 2 states, 1 measurement
///     .initial_state(vec![0.0, 0.0])
///     .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
///     .transition_matrix(vec![1.0, 1.0, 0.0, 1.0])
///     .process_noise(vec![0.01, 0.0, 0.0, 0.01])
///     .observation_matrix(vec![1.0, 0.0])
///     .measurement_noise(vec![0.1])
///     .build()
///     .unwrap();
/// ```
pub struct KalmanFilterBuilder<T>
where
    T: KalmanScalar,
{
    state_dim: usize,
    measurement_dim: usize,
    initial_state: Option<Vec<T>>,
    initial_covariance: Option<Vec<T>>,
    transition_matrix: Option<Vec<T>>,
    process_noise: Option<Vec<T>>,
    observation_matrix: Option<Vec<T>>,
    measurement_noise: Option<Vec<T>>,
}

impl<T> KalmanFilterBuilder<T>
where
    T: KalmanScalar,
{
    /// Create a new builder with specified dimensions
    pub fn new(state_dim: usize, measurement_dim: usize) -> Self {
        trace!(
            "Creating KalmanFilterBuilder: state_dim={state_dim}, measurement_dim={measurement_dim}"
        );
        Self {
            state_dim,
            measurement_dim,
            initial_state: None,
            initial_covariance: None,
            transition_matrix: None,
            process_noise: None,
            observation_matrix: None,
            measurement_noise: None,
        }
    }

    /// Set the initial state vector
    pub fn initial_state(mut self, state: Vec<T>) -> Self {
        self.initial_state = Some(state);
        self
    }

    /// Set the initial state covariance matrix (row-major)
    pub fn initial_covariance(mut self, covariance: Vec<T>) -> Self {
        self.initial_covariance = Some(covariance);
        self
    }

    /// Set the state transition matrix (F) (row-major)
    pub fn transition_matrix(mut self, matrix: Vec<T>) -> Self {
        self.transition_matrix = Some(matrix);
        self
    }

    /// Set the process noise covariance (Q) (row-major)
    pub fn process_noise(mut self, noise: Vec<T>) -> Self {
        self.process_noise = Some(noise);
        self
    }

    /// Set the observation matrix (H) (row-major)
    pub fn observation_matrix(mut self, matrix: Vec<T>) -> Self {
        self.observation_matrix = Some(matrix);
        self
    }

    /// Set the measurement noise covariance (R) (row-major)
    pub fn measurement_noise(mut self, noise: Vec<T>) -> Self {
        self.measurement_noise = Some(noise);
        self
    }

    /// Build a Vec-backed [`KalmanFilter`]
    ///
    /// This is the constructor available since v1.0. `predict`, `update`, `state` and
    /// `covariance` are inherent methods, so no trait import is needed. It returns
    /// `KalmanFilter<T>` whichever matrix backend feature is enabled; to get a filter that
    /// can use the nalgebra static backend, call [`build_unified`](Self::build_unified).
    ///
    /// # Example
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
    /// kf.predict();
    /// kf.update(&[1.0]).unwrap();
    /// ```
    pub fn build(self) -> KalmanResult<KalmanFilter<T>> {
        let parts = self.validate()?;
        info!("Using KalmanFilter (Vec backend)");
        parts.into_vec_filter()
    }

    /// Build a backend-agnostic Kalman filter
    ///
    /// Returns `UnifiedKalmanFilter`, which provides the same API through
    /// [`KalmanFilterTrait`](crate::unified::KalmanFilterTrait) regardless of enabled features:
    /// - With `nalgebra` feature: uses the compile-time sized `StaticKalmanFilter` for the
    ///   dimensions (state, measurement) = (1, 1), (2, 1), (3, 1), (4, 2) and (6, 3), and the
    ///   Vec-backed `KalmanFilter` for any other dimensions
    /// - Without `nalgebra` feature: uses the Vec-backed `KalmanFilter`
    ///
    /// Validation is identical to [`build`](Self::build).
    ///
    /// # Example
    /// ```
    /// use kalman_filters::{KalmanFilterBuilder, KalmanFilterTrait};
    ///
    /// let mut kf = KalmanFilterBuilder::new(2, 1)
    ///     .initial_state(vec![0.0, 0.0])
    ///     .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
    ///     .transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
    ///     .process_noise(vec![0.001, 0.0, 0.0, 0.001])
    ///     .observation_matrix(vec![1.0, 0.0])
    ///     .measurement_noise(vec![0.1])
    ///     .build_unified()
    ///     .unwrap();
    ///
    /// // Works the same regardless of backend!
    /// kf.predict();
    /// kf.update(&[1.0]).unwrap();
    /// ```
    #[cfg(not(feature = "nalgebra"))]
    pub fn build_unified(self) -> KalmanResult<crate::unified::UnifiedKalmanFilter<T>> {
        use crate::unified::IntoUnified;

        Ok(self.build()?.into_unified())
    }

    #[cfg(feature = "nalgebra")]
    pub fn build_unified(self) -> KalmanResult<crate::unified::UnifiedKalmanFilter<T>>
    where
        T: RealField,
    {
        use crate::unified::IntoUnified;

        let p = self.validate()?;

        // Try to use optimized static backends for common dimensions
        match (p.state_dim, p.measurement_dim) {
            (1, 1) => {
                info!("Using StaticKalmanFilter<1, 1> (nalgebra backend)");
                let kf = StaticKalmanFilter::<T, 1, 1>::new(
                    SVector::from_row_slice(&p.initial_state),
                    SMatrix::from_row_slice(&p.initial_covariance),
                    SMatrix::from_row_slice(&p.transition_matrix),
                    SMatrix::from_row_slice(&p.process_noise),
                    SMatrix::from_row_slice(&p.observation_matrix),
                    SMatrix::from_row_slice(&p.measurement_noise),
                );
                Ok(kf.into_unified())
            }
            (2, 1) => {
                info!("Using StaticKalmanFilter<1, 2> (nalgebra backend)");
                let kf = StaticKalmanFilter::<T, 1, 2>::new(
                    SVector::from_row_slice(&p.initial_state),
                    SMatrix::from_row_slice(&p.initial_covariance),
                    SMatrix::from_row_slice(&p.transition_matrix),
                    SMatrix::from_row_slice(&p.process_noise),
                    SMatrix::from_row_slice(&p.observation_matrix),
                    SMatrix::from_row_slice(&p.measurement_noise),
                );
                Ok(kf.into_unified())
            }
            (3, 1) => {
                info!("Using StaticKalmanFilter<1, 3> (nalgebra backend)");
                let kf = StaticKalmanFilter::<T, 1, 3>::new(
                    SVector::from_row_slice(&p.initial_state),
                    SMatrix::from_row_slice(&p.initial_covariance),
                    SMatrix::from_row_slice(&p.transition_matrix),
                    SMatrix::from_row_slice(&p.process_noise),
                    SMatrix::from_row_slice(&p.observation_matrix),
                    SMatrix::from_row_slice(&p.measurement_noise),
                );
                Ok(kf.into_unified())
            }
            (4, 2) => {
                info!("Using StaticKalmanFilter<2, 4> (nalgebra backend)");
                let kf = StaticKalmanFilter::<T, 2, 4>::new(
                    SVector::from_row_slice(&p.initial_state),
                    SMatrix::from_row_slice(&p.initial_covariance),
                    SMatrix::from_row_slice(&p.transition_matrix),
                    SMatrix::from_row_slice(&p.process_noise),
                    SMatrix::from_row_slice(&p.observation_matrix),
                    SMatrix::from_row_slice(&p.measurement_noise),
                );
                Ok(kf.into_unified())
            }
            (6, 3) => {
                info!("Using StaticKalmanFilter<3, 6> (nalgebra backend)");
                let kf = StaticKalmanFilter::<T, 3, 6>::new(
                    SVector::from_row_slice(&p.initial_state),
                    SMatrix::from_row_slice(&p.initial_covariance),
                    SMatrix::from_row_slice(&p.transition_matrix),
                    SMatrix::from_row_slice(&p.process_noise),
                    SMatrix::from_row_slice(&p.observation_matrix),
                    SMatrix::from_row_slice(&p.measurement_noise),
                );
                Ok(kf.into_unified())
            }
            (n, m) => {
                // Fall back to Vec-based for uncommon dimensions
                info!("Using KalmanFilter (Vec backend) for dimensions {n}x{m}");
                Ok(p.into_vec_filter()?.into_unified())
            }
        }
    }

    /// Check that every matrix is present and the covariances are symmetric
    ///
    /// The single validation path shared by `build` and `build_unified`.
    fn validate(self) -> KalmanResult<BuilderParts<T>> {
        trace!(
            "Building Kalman filter: state_dim={}, measurement_dim={}",
            self.state_dim, self.measurement_dim
        );
        let initial_state = self.initial_state.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing initial_state");
            KalmanError::BuilderIncomplete("initial_state".to_string())
        })?;

        let initial_covariance = self.initial_covariance.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing initial_covariance");
            KalmanError::BuilderIncomplete("initial_covariance".to_string())
        })?;

        let transition_matrix = self.transition_matrix.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing transition_matrix");
            KalmanError::BuilderIncomplete("transition_matrix".to_string())
        })?;

        let process_noise = self.process_noise.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing process_noise");
            KalmanError::BuilderIncomplete("process_noise".to_string())
        })?;

        let observation_matrix = self.observation_matrix.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing observation_matrix");
            KalmanError::BuilderIncomplete("observation_matrix".to_string())
        })?;

        let measurement_noise = self.measurement_noise.ok_or_else(|| {
            error!("KalmanFilterBuilder: missing measurement_noise");
            KalmanError::BuilderIncomplete("measurement_noise".to_string())
        })?;

        let n = self.state_dim;
        let m = self.measurement_dim;

        if !is_symmetric(&initial_covariance, n) {
            error!("KalmanFilterBuilder: initial_covariance is not symmetric (size={n}x{n})");
            return Err(KalmanError::InvalidCovariance);
        }
        trace!("Initial covariance matrix validated as symmetric");
        if !is_symmetric(&process_noise, n) {
            error!("KalmanFilterBuilder: process_noise is not symmetric (size={n}x{n})");
            return Err(KalmanError::InvalidNoiseCovariance);
        }
        trace!("Process noise matrix validated as symmetric");
        if !is_symmetric(&measurement_noise, m) {
            error!("KalmanFilterBuilder: measurement_noise is not symmetric (size={m}x{m})");
            return Err(KalmanError::InvalidNoiseCovariance);
        }
        trace!("Measurement noise matrix validated as symmetric");

        Ok(BuilderParts {
            state_dim: n,
            measurement_dim: m,
            initial_state,
            initial_covariance,
            transition_matrix,
            process_noise,
            observation_matrix,
            measurement_noise,
        })
    }
}

/// A builder's matrices after validation
struct BuilderParts<T> {
    state_dim: usize,
    measurement_dim: usize,
    initial_state: Vec<T>,
    initial_covariance: Vec<T>,
    transition_matrix: Vec<T>,
    process_noise: Vec<T>,
    observation_matrix: Vec<T>,
    measurement_noise: Vec<T>,
}

impl<T> BuilderParts<T>
where
    T: KalmanScalar,
{
    fn into_vec_filter(self) -> KalmanResult<KalmanFilter<T>> {
        KalmanFilter::initialize(
            self.state_dim,
            self.measurement_dim,
            self.initial_state,
            self.initial_covariance,
            self.transition_matrix,
            self.process_noise,
            self.observation_matrix,
            self.measurement_noise,
        )
    }
}

impl<T> Default for KalmanFilterBuilder<T>
where
    T: KalmanScalar,
{
    fn default() -> Self {
        Self::new(1, 1)
    }
}

fn is_symmetric<T>(matrix: &[T], size: usize) -> bool
where
    T: KalmanScalar,
{
    for i in 0..size {
        for j in i + 1..size {
            let diff = (matrix[i * size + j] - matrix[j * size + i]).abs();
            if diff > <T as KalmanScalar>::epsilon() {
                warn!(
                    "Matrix symmetry check failed at ({},{}) and ({},{}): diff={:.6e}",
                    i,
                    j,
                    j,
                    i,
                    KalmanScalar::to_f64(&diff)
                );
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_complete() {
        let result = KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
            .transition_matrix(vec![1.0, 1.0, 0.0, 1.0])
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .observation_matrix(vec![1.0, 0.0])
            .measurement_noise(vec![0.1])
            .build();

        assert!(result.is_ok());
    }

    #[test]
    fn test_builder_incomplete() {
        let result = KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .build();

        assert!(result.is_err());
        match result {
            Err(KalmanError::BuilderIncomplete(field)) => {
                assert_eq!(field, "initial_covariance");
            }
            _ => panic!("Expected BuilderIncomplete error"),
        }
    }

    #[test]
    fn test_builder_invalid_covariance() {
        let result = KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .initial_covariance(vec![1.0, 2.0, 0.0, 1.0]) // Not symmetric
            .transition_matrix(vec![1.0, 1.0, 0.0, 1.0])
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .observation_matrix(vec![1.0, 0.0])
            .measurement_noise(vec![0.1])
            .build();

        assert!(result.is_err());
        match result {
            Err(KalmanError::InvalidCovariance) => {}
            _ => panic!("Expected InvalidCovariance error"),
        }
    }

    fn tracker_2x1() -> KalmanFilterBuilder<f64> {
        KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
            .transition_matrix(vec![1.0, 1.0, 0.0, 1.0])
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .observation_matrix(vec![1.0, 0.0])
            .measurement_noise(vec![0.1])
    }

    #[test]
    fn test_build_returns_vec_filter() {
        // The v1.0.1 signature: inherent methods, no trait import needed
        let mut kf: KalmanFilter<f64> = tracker_2x1().build().unwrap();
        kf.predict();
        kf.update(&[1.0]).unwrap();
        assert_eq!(kf.state().len(), 2);
    }

    #[test]
    fn test_build_and_build_unified_agree() {
        use crate::unified::KalmanFilterTrait;

        let mut vec_kf = tracker_2x1().build().unwrap();
        let mut unified_kf = tracker_2x1().build_unified().unwrap();
        for z in [1.0, 2.1, 2.9, 4.2] {
            vec_kf.predict();
            vec_kf.update(&[z]).unwrap();
            unified_kf.predict();
            unified_kf.update(&[z]).unwrap();
        }
        let unified_state = KalmanFilterTrait::state(&unified_kf);
        for (a, b) in vec_kf.state().iter().zip(unified_state.iter()) {
            assert!((a - b).abs() < 1e-10, "{a} != {b}");
        }
    }

    #[test]
    fn test_build_unified_validates_like_build() {
        let result = KalmanFilterBuilder::<f64>::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .build_unified();
        match result {
            Err(KalmanError::BuilderIncomplete(field)) => {
                assert_eq!(field, "initial_covariance");
            }
            _ => panic!("Expected BuilderIncomplete error"),
        }
    }

    #[cfg(feature = "nalgebra")]
    #[test]
    fn test_build_unified_selects_static_backend() {
        use crate::unified::UnifiedKalmanFilter;

        let kf = tracker_2x1().build_unified().unwrap();
        assert!(matches!(kf, UnifiedKalmanFilter::Static2x1(_)));
    }
}
