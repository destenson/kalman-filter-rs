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
//! use kalman_filter::{KalmanFilter, KalmanFilterBuilder};
//!
//! // Create a simple 1D Kalman filter
//! let mut kf = KalmanFilter::<f64>::initialize(
//!     1, 1,                // 1 state, 1 measurement
//!     vec![0.0],           // initial state
//!     vec![1.0],           // initial covariance
//!     vec![1.0],           // state transition matrix
//!     vec![0.001],         // process noise
//!     vec![1.0],           // observation matrix
//!     vec![0.1],           // measurement noise
//! ).unwrap();
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
