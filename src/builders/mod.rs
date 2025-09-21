//! Builder patterns for constructing Kalman filters
//!
//! This module provides builder patterns for all filter types, enabling
//! type-safe construction with validation deferred to the build phase.

mod ensemble;
mod extended;
mod information;
mod particle;
mod scented;
mod unscented;

pub use ensemble::EnsembleKalmanFilterBuilder;
pub use extended::ExtendedKalmanFilterBuilder;
pub use information::InformationFilterBuilder;
pub use particle::ParticleFilterBuilder;
pub use scented::CubatureKalmanFilterBuilder;
pub use unscented::UnscentedKalmanFilterBuilder;
