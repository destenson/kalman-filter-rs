//! Example demonstrating the logging infrastructure with tracing_subscriber
//!
//! This example shows how to configure logging for the Kalman filter library
//! and demonstrates the various log levels and diagnostic information available.
//!
//! Run with different log levels:
//! ```bash
//! RUST_LOG=error cargo run --example logging_demo
//! RUST_LOG=warn cargo run --example logging_demo
//! RUST_LOG=info cargo run --example logging_demo
//! RUST_LOG=debug cargo run --example logging_demo
//! RUST_LOG=trace cargo run --example logging_demo
//! ```
//!
//! Or with tracing_subscriber feature:
//! ```bash
//! cargo run --example logging_demo --features tracing-subscriber
//! ```
//!
//! Command-line options:
//! ```bash
//! # Use a specific random seed for reproducibility
//! cargo run --example logging_demo -- --seed 42
//!
//! # Adjust number of cycles for demos
//! cargo run --example logging_demo -- --pendulum-cycles 5 --linear-cycles 10
//! ```

use clap::Parser;
use kalman_filters::{ExtendedKalmanFilter, KalmanFilterBuilder, NonlinearSystem};
use log::{debug, info, warn};

#[cfg(feature = "tracing-subscriber")]
use tracing_subscriber::{EnvFilter, fmt, prelude::*};

/// Command-line arguments for the logging demo
#[derive(Parser, Debug)]
#[command(author, version, about = "Demonstrates logging infrastructure for Kalman filters", long_about = None)]
struct Args {
    /// Random seed for reproducibility of the numerical issues demo
    #[arg(long)]
    seed: Option<u64>,

    /// Number of cycles to run for the pendulum (EKF) demo
    #[arg(long, default_value_t = 3)]
    pendulum_cycles: usize,

    /// Number of cycles to run for the linear Kalman filter demo
    #[arg(long, default_value_t = 3)]
    linear_cycles: usize,
}

/// Simple pendulum system for demonstrating EKF logging
struct PendulumSystem {
    g: f64, // gravity
    l: f64, // length
}

impl NonlinearSystem<f64> for PendulumSystem {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let theta = state[0];
        let theta_dot = state[1];
        vec![
            theta + theta_dot * dt,
            theta_dot - (self.g / self.l) * theta.sin() * dt,
        ]
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        vec![state[0]] // Measure angle only
    }

    fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let theta = state[0];
        vec![1.0, dt, -(self.g / self.l) * theta.cos() * dt, 1.0]
    }

    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        vec![1.0, 0.0]
    }

    fn state_dim(&self) -> usize {
        2
    }

    fn measurement_dim(&self) -> usize {
        1
    }
}

fn main() {
    let args = Args::parse();

    // Initialize logging
    #[cfg(feature = "tracing-subscriber")]
    {
        tracing_subscriber::registry()
            .with(fmt::layer())
            .with(EnvFilter::from_default_env())
            .init();
        info!("Initialized tracing_subscriber for structured logging");
    }

    #[cfg(not(feature = "tracing-subscriber"))]
    {
        // Simple env_logger initialization
        env_logger::init();
        info!("Using env_logger (install tracing-subscriber feature for structured logging)");
    }

    info!("Starting Kalman Filter logging demonstration");
    debug!("Arguments: {:?}", args);

    // Demonstrate linear Kalman filter logging
    demo_linear_kf(args.linear_cycles);

    // Demonstrate Extended Kalman filter logging
    demo_extended_kf(args.pendulum_cycles);

    // Demonstrate numerical stability warnings
    demo_numerical_issues(args.seed);

    info!("Logging demonstration complete");
}

fn demo_linear_kf(cycles: usize) {
    info!("=== Linear Kalman Filter Demo ===");

    // Create a simple 1D Kalman filter using the builder
    let mut kf = KalmanFilterBuilder::<f64>::new(1, 1)
        .initial_state(vec![0.0])
        .initial_covariance(vec![1.0])
        .transition_matrix(vec![1.0])
        .process_noise(vec![0.001])
        .observation_matrix(vec![1.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();

    info!("Running {} predict/update cycles", cycles);

    // Run specified number of cycles
    for i in 1..=cycles {
        info!("Cycle {}", i);

        // Predict
        kf.predict();

        // Update with measurement
        let measurement = vec![i as f64 * 0.5];
        kf.update(&measurement).unwrap();
    }
}

fn demo_extended_kf(cycles: usize) {
    info!("=== Extended Kalman Filter Demo ===");

    let system = PendulumSystem { g: 9.81, l: 1.0 };

    let mut ekf: ExtendedKalmanFilter<f64, PendulumSystem> = ExtendedKalmanFilter::new(
        system,
        vec![0.1, 0.0], // initial state [angle, angular_velocity]
        vec![
            0.01, 0.0, // initial covariance
            0.0, 0.01,
        ],
        vec![
            0.001, 0.0, // process noise
            0.0, 0.001,
        ],
        vec![0.1], // measurement noise
        0.01,      // dt
    )
    .unwrap();

    info!("Running {} EKF predict/update cycles", cycles);

    // Run specified number of cycles
    for i in 1..=cycles {
        info!("EKF Cycle {}", i);

        // Predict
        ekf.predict();

        // Update with measurement
        let measurement = vec![0.05 * i as f64];
        ekf.update(&measurement).unwrap();
    }
}

fn demo_numerical_issues(seed: Option<u64>) {
    info!("=== Numerical Stability Demo ===");

    if let Some(s) = seed {
        info!("Using random seed: {}", s);
        // Note: This demo uses hardcoded values for demonstration purposes.
        // The seed parameter is provided for future enhancements that might
        // use randomized test cases.
    }

    // Create a filter with near-singular covariance
    let mut kf = KalmanFilterBuilder::<f64>::new(2, 1)
        .initial_state(vec![0.0, 0.0])
        .initial_covariance(vec![1e-12, 0.0, 0.0, 1e-12]) // near-singular
        .transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
        .process_noise(vec![0.001, 0.0, 0.0, 0.001])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();

    warn!("Created filter with near-singular covariance to trigger warnings");

    // This should trigger numerical stability warnings
    kf.predict();

    // Try to update - may trigger warnings about innovation covariance
    let _ = kf.update(&[1.0]);

    // Create a filter that will have a singular innovation covariance
    let mut bad_kf = KalmanFilterBuilder::<f64>::new(2, 2)
        .initial_state(vec![0.0, 0.0])
        .initial_covariance(vec![0.0, 0.0, 0.0, 0.0]) // zero covariance!
        .transition_matrix(vec![1.0, 0.0, 0.0, 1.0])
        .process_noise(vec![0.0, 0.0, 0.0, 0.0]) // zero process noise
        .observation_matrix(vec![1.0, 0.0, 0.0, 1.0])
        .measurement_noise(vec![0.0, 0.0, 0.0, 0.0]) // zero measurement noise!
        .build()
        .unwrap();

    warn!("Created filter with zero covariances to demonstrate error logging");

    bad_kf.predict();

    // This should fail with a singular matrix error
    match bad_kf.update(&[1.0, 1.0]) {
        Ok(_) => debug!("Update succeeded unexpectedly"),
        Err(e) => info!("Update failed as expected: {:?}", e),
    }
}
