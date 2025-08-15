//! Ensemble Kalman Filter (EnKF) module for high-dimensional data assimilation
//!
//! The EnKF uses a Monte Carlo approach to represent uncertainty through an
//! ensemble of state realizations, avoiding explicit covariance storage.

pub mod filter;

pub use filter::{EnsembleKalmanFilter, EnsembleStatistics};
