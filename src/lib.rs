//! # Kalman Filter Implementation
//!
//! This crate provides a Kalman filter implementation for state estimation
//! in linear dynamic systems. The Kalman filter is an optimal recursive
//! estimator that combines predictions from a system model with measurements
//! to estimate the true state of a system.
//!
//! ## Features
//!
//! - Linear Kalman filter (`kalman_filters::linear`): Joseph-form update,
//!   Cholesky solve with S, no heap allocation per step
//! - Dynamic dimension support without external dependencies
//! - Support for both f32 and f64 precision
//! - Extended, unscented, cubature, information, ensemble and particle filters
//!
//! ## Example
//!
//! ```
//! use kalman_filters::linear::LinearKalmanFilter;
//!
//! // n = 1 state, m = 1 measurement; matrices are row-major slices:
//! //                                             x0      P0     F      Q        H      R
//! let mut kf = LinearKalmanFilter::<f64>::try_new(1, 1, &[0.0], &[1.0], &[1.0], &[0.001], &[1.0], &[0.1])?;
//!
//! kf.predict()?;
//! kf.update(&[1.0])?;
//! assert!(kf.state()[0] > 0.0);
//! # Ok::<(), kalman_filters::KalmanError>(())
//! ```
//!
//! ## Precision
//!
//! The scalar is a type parameter: instantiate the filter as `LinearKalmanFilter<f32>` or
//! `LinearKalmanFilter<f64>` and every matrix and intermediate is held at that precision.
//! Both are supported by both numeric engines (see the `nalgebra` feature).

#![allow(unused, non_snake_case)] // DO NOT CHANGE

mod builder;
pub mod builders;
pub mod ensemble;
/// Error types and result definitions for the Kalman filter library
pub mod error;
pub mod extended;
mod filter;
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

pub use information::filter::InformationFilter;
pub use particle::{Particle, ParticleFilter, ResamplingStrategy};
pub use scented::CubatureKalmanFilter;
pub use types::{JacobianStrategy, KalmanResult, KalmanScalar, NonlinearSystem};
pub use unscented::{UKFParameters, UnscentedKalmanFilter};

#[cfg(feature = "nalgebra")]
pub use nalgebra;
