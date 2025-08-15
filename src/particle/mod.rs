//! Particle Filter (Sequential Monte Carlo) module
//!
//! The particle filter uses weighted samples to represent arbitrary probability
//! distributions, making it suitable for highly nonlinear and non-Gaussian systems.

pub mod filter;

pub use filter::{ParticleFilter, Particle, ResamplingStrategy};