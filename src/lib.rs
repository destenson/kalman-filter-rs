//! # Kalman Filter Implementation
//! 
//! This crate provides a Kalman filter implementation for state estimation
//! in linear dynamic systems. The Kalman filter is an optimal recursive
//! estimator that combines predictions from a system model with measurements
//! to estimate the true state of a system.
//!
//! ## Features
//! 
//! - Linear Kalman filter with predict and update steps
//! - Dynamic dimension support without external dependencies
//! - Support for both f32 and f64 precision
//! - Builder pattern for easy initialization
//! - Extended Kalman Filter (EKF) support (future)
//! - Unscented Kalman Filter (UKF) support (future)
//!
//! ## Example
//!
//! ```
//! use kalman_filter::KalmanFilterBuilder;
//!
//! // Create a simple 1D Kalman filter using the builder
//! let mut kf = KalmanFilterBuilder::new(1, 1)
//!     .initial_state(vec![0.0])
//!     .initial_covariance(vec![1.0])
//!     .transition_matrix(vec![1.0])
//!     .process_noise(vec![0.001])
//!     .observation_matrix(vec![1.0])
//!     .measurement_noise(vec![0.1])
//!     .build()
//!     .unwrap();
//!
//! // Predict step
//! kf.predict();
//!
//! // Update with measurement
//! kf.update(&[1.0]).unwrap();
//! ```

#![allow(unused, non_snake_case)] // DO NOT CHANGE

pub mod builder;
pub mod ensemble;
pub mod error;
pub mod extended;
pub mod filter;
pub mod information;
pub mod logging;
pub mod particle;
pub mod scented;
pub mod types;
pub mod unscented;

// Re-export main types for convenience
pub use builder::KalmanFilterBuilder;
pub use ensemble::{EnsembleKalmanFilter, EnsembleStatistics};
pub use error::KalmanError;
pub use extended::ExtendedKalmanFilter;

// Export modern KalmanFilter only when legacy is not enabled
#[cfg(not(feature = "legacy"))]
pub use filter::KalmanFilter;

// Export as ModernKalmanFilter when legacy is enabled
#[cfg(feature = "legacy")]
pub use filter::KalmanFilter as ModernKalmanFilter;

pub use particle::{ParticleFilter, Particle, ResamplingStrategy};
pub use scented::CubatureKalmanFilter;
pub use types::{JacobianStrategy, KalmanResult, KalmanScalar, NonlinearSystem};
pub use unscented::{UnscentedKalmanFilter, UKFParameters};

#[cfg(feature = "nalgebra")]
pub use filter::StaticKalmanFilter;

#[cfg(feature = "adskalman")]
pub use adskalman;

// Legacy API compatibility
#[cfg(feature = "legacy")]
pub mod legacy;

// When legacy feature is enabled, use the legacy API as default KalmanFilter
#[cfg(feature = "legacy")]
pub use legacy::KalmanFilter;
