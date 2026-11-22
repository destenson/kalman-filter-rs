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
//! - Extended Kalman Filter (EKF) support (future)
//! - Unscented Kalman Filter (UKF) support (future)
//!
//! ## Example
//!
//! ```
//! use kalman_filters::linear::LinearKalmanFilter;
//!
//! // n = 1 state, m = 1 measurement; matrices are row-major slices:
//! //                                        x0      P0     F      Q        H      R
//! let mut kf = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.001], &[1.0], &[0.1])
//!     .unwrap();
//!
//! kf.predict().unwrap();
//! kf.update(&[1.0]).unwrap();
//! assert!(kf.state()[0] > 0.0);
//! ```

#![allow(unused, non_snake_case)] // DO NOT CHANGE

pub mod builders;
pub mod ensemble;
/// Error types and result definitions for the Kalman filter library
pub mod error;
pub mod extended;
pub mod filter;
pub mod information;
pub mod logging;
pub(crate) mod matrix;

#[cfg(feature = "prometheus-metrics")]
pub mod metrics;

pub mod particle;
pub mod scented;
pub mod types;
pub mod unscented;
pub(crate) mod validation;

// Re-export main types for convenience
pub use builders::{
    CubatureKalmanFilterBuilder, EnsembleKalmanFilterBuilder, ExtendedKalmanFilterBuilder,
    InformationFilterBuilder, ParticleFilterBuilder, UnscentedKalmanFilterBuilder,
};
pub use ensemble::{EnsembleKalmanFilter, EnsembleStatistics};
pub use error::KalmanError;
pub use extended::ExtendedKalmanFilter;

/// The linear Kalman filter (see SPEC.md).
pub use filter::linear;
pub use filter::LinearKalmanFilter;

pub use information::filter::InformationFilter;
pub use particle::{Particle, ParticleFilter, ResamplingStrategy};
pub use scented::CubatureKalmanFilter;
pub use types::{JacobianStrategy, KalmanResult, KalmanScalar, NonlinearSystem};
pub use unscented::{UKFParameters, UnscentedKalmanFilter};

#[cfg(feature = "nalgebra")]
pub use nalgebra;
