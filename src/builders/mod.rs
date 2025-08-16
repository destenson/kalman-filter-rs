//! Builder patterns for constructing Kalman filters
//!
//! This module provides builder patterns for all filter types, enabling
//! type-safe construction with validation deferred to the build phase.

mod extended;
mod unscented;
mod scented;
mod ensemble;
mod information;
mod particle;

pub use extended::ExtendedKalmanFilterBuilder;
pub use unscented::UnscentedKalmanFilterBuilder;
pub use scented::CubatureKalmanFilterBuilder;
pub use ensemble::EnsembleKalmanFilterBuilder;
pub use information::InformationFilterBuilder;
pub use particle::ParticleFilterBuilder;
