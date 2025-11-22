//! Unscented Kalman Filter example: Nonlinear target tracking
//!
//! This example demonstrates tracking a target with nonlinear dynamics
//! using the UKF, which handles nonlinearity better than EKF without
//! requiring Jacobian calculations.
//!
//! ## Usage
//! ```bash
//! # Run with random noise (different output each time)
//! cargo run --example ukf_nonlinear_tracking
//!
//! # Run with a specific seed for reproducible results
//! cargo run --example ukf_nonlinear_tracking -- --seed 42
//! ```
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use kalman_filters::{NonlinearSystem, UKFParameters, UnscentedKalmanFilter};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

/// Nonlinear target dynamics with coordinated turn model
struct CoordinatedTurnModel {
    /// Turn rate (rad/s)
    omega: f64,
}

impl NonlinearSystem<f64> for CoordinatedTurnModel {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let x = state[0]; // x position
        let y = state[1]; // y position
        let vx = state[2]; // x velocity
        let vy = state[3]; // y velocity

        // Coordinated turn model
        let sin_wt = (self.omega * dt).sin();
        let cos_wt = (self.omega * dt).cos();

        if self.omega.abs() < 1e-6 {
            // Linear motion when turn rate is near zero
            vec![x + vx * dt, y + vy * dt, vx, vy]
        } else {
            // Nonlinear coordinated turn
            vec![
                x + (vx * sin_wt - vy * (1.0 - cos_wt)) / self.omega,
                y + (vx * (1.0 - cos_wt) + vy * sin_wt) / self.omega,
                vx * cos_wt - vy * sin_wt,
                vx * sin_wt + vy * cos_wt,
            ]
        }
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        // Radar measurement: range and bearing
        let x = state[0];
        let y = state[1];

        vec![
            (x * x + y * y).sqrt(), // Range
            y.atan2(x),             // Bearing
        ]
    }

    fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
        unreachable!("UKF doesn't need Jacobians")
    }

    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        unreachable!("UKF doesn't need Jacobians")
    }

    fn state_dim(&self) -> usize {
        4
    }
    fn measurement_dim(&self) -> usize {
        2
    }
}

fn main() {
    println!("=== UKF Nonlinear Target Tracking Example ===\n");

    // Parse command-line arguments for optional seed
    let args: Vec<String> = std::env::args().collect();
    let seed = if args.len() > 2 && args[1] == "--seed" {
        let seed_value = args[2].parse::<u64>().expect("Invalid seed value");
        println!("Using seed: {}\n", seed_value);
        seed_value
    } else {
        let seed_value = rand::random::<u64>();
        println!("Using random seed: {} (use --seed {} to reproduce)\n", seed_value, seed_value);
        seed_value
    };

    // Create RNG with the selected seed
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

    // Create coordinated turn model
    let system = CoordinatedTurnModel { omega: 0.1 }; // 0.1 rad/s turn rate

    // Initial state: [x, y, vx, vy]
    let initial_state = vec![100.0, 50.0, 10.0, 5.0];
    let initial_covariance = vec![
        10.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    // Process and measurement noise
    let process_noise = vec![
        0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.01,
    ];
    let measurement_noise = vec![
        1.0, 0.0, // Range noise
        0.0, 0.01, // Bearing noise (radians)
    ];

    // Create UKF
    let mut ukf = UnscentedKalmanFilter::new(
        system,
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise,
        1.0, // dt = 1 second
    )
    .unwrap();

    // Set UKF parameters for better performance
    let mut params = UKFParameters::default();
    params.alpha = 1e-3;
    params.beta = 2.0;
    params.kappa = 0.0;
    ukf.set_parameters(params);

    println!(
        "Initial state: x={:.1}, y={:.1}, vx={:.1}, vy={:.1}",
        ukf.state()[0],
        ukf.state()[1],
        ukf.state()[2],
        ukf.state()[3]
    );

    // Simulate tracking for 10 time steps
    println!("\nTracking target with coordinated turn model:");
    println!("Time | Range | Bearing | Est. X | Est. Y | Est. Vx | Est. Vy");
    println!("-----|-------|---------|--------|--------|---------|--------");

    // Create noise distributions matching the measurement noise covariance
    let range_noise = Normal::new(0.0, 1.0_f64.sqrt()).unwrap(); // σ² = 1.0
    let bearing_noise = Normal::new(0.0, 0.01_f64.sqrt()).unwrap(); // σ² = 0.01

    for t in 1..=10 {
        // Predict
        ukf.predict().unwrap();

        // Generate simulated measurement (with noise)
        let true_state = ukf.state();
        let true_range = (true_state[0].powi(2) + true_state[1].powi(2)).sqrt();
        let true_bearing = true_state[1].atan2(true_state[0]);

        // Add random measurement noise
        let measured_range = true_range + range_noise.sample(&mut rng);
        let measured_bearing = true_bearing + bearing_noise.sample(&mut rng);

        // Update with measurement
        ukf.update(&[measured_range, measured_bearing]).unwrap();

        // Print results
        let state = ukf.state();
        println!(
            "{:4} | {:5.1} | {:7.3} | {:6.1} | {:6.1} | {:7.2} | {:6.2}",
            t, measured_range, measured_bearing, state[0], state[1], state[2], state[3]
        );
    }

    // Print final covariance
    println!("\nFinal position uncertainty (std dev):");
    let cov = ukf.covariance();
    println!("  σ_x = {:.2} m", cov[0].sqrt());
    println!("  σ_y = {:.2} m", cov[5].sqrt());

    println!("\n✓ UKF successfully tracked nonlinear target motion");
}
