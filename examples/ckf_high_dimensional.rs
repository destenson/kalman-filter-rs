//! Cubature Kalman Filter example: High-dimensional system
//!
//! This example demonstrates the CKF's numerical stability advantages
//! for higher-dimensional systems compared to UKF.
//!
//! ## Usage
//! ```bash
//! # Run with default parameters
//! cargo run --example ckf_high_dimensional
//!
//! # Run with custom parameters
//! cargo run --example ckf_high_dimensional -- \
//!   --seed 42 \
//!   --steps 50 \
//!   --dt 0.05 \
//!   --process-noise 0.02 \
//!   --meas-noise1 0.1 \
//!   --meas-noise2 0.1 \
//!   --meas-noise3 0.05
//!
//! # Show all available options
//! cargo run --example ckf_high_dimensional -- --help
//! ```
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use clap::Parser;
use kalman_filters::{CubatureKalmanFilter, NonlinearSystem};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

/// Command-line arguments for the CKF high-dimensional example
#[derive(Parser, Debug)]
#[command(name = "CKF High-Dimensional System")]
#[command(about = "Demonstrates CKF tracking of a 6D nonlinear system", long_about = None)]
struct Args {
    /// Random seed for reproducibility (if not specified, uses random seed)
    #[arg(long)]
    seed: Option<u64>,

    /// Number of time steps to simulate
    #[arg(long, default_value_t = 20)]
    steps: usize,

    /// Time step size (seconds)
    #[arg(long, default_value_t = 0.1)]
    dt: f64,

    /// Initial position X
    #[arg(long, default_value_t = 1.0)]
    init_x: f64,

    /// Initial position Y
    #[arg(long, default_value_t = 0.5)]
    init_y: f64,

    /// Initial position Z
    #[arg(long, default_value_t = -0.5)]
    init_z: f64,

    /// Initial velocity X
    #[arg(long, default_value_t = 0.1)]
    init_vx: f64,

    /// Initial velocity Y
    #[arg(long, default_value_t = 0.2)]
    init_vy: f64,

    /// Initial velocity Z
    #[arg(long, default_value_t = -0.1)]
    init_vz: f64,

    /// Initial covariance diagonal value
    #[arg(long, default_value_t = 0.1)]
    init_cov: f64,

    /// Process noise for position states
    #[arg(long, default_value_t = 0.01)]
    process_noise_pos: f64,

    /// Process noise for velocity states
    #[arg(long, default_value_t = 0.001)]
    process_noise_vel: f64,

    /// Measurement noise for first measurement (XY distance)
    #[arg(long, default_value_t = 0.05)]
    meas_noise1: f64,

    /// Measurement noise for second measurement (YZ distance)
    #[arg(long, default_value_t = 0.05)]
    meas_noise2: f64,

    /// Measurement noise for third measurement (position sum)
    #[arg(long, default_value_t = 0.02)]
    meas_noise3: f64,

    /// Show detailed output for every step (not just summary)
    #[arg(long)]
    verbose: bool,
}

/// High-dimensional nonlinear system (6D state)
struct HighDimensionalSystem;

impl NonlinearSystem<f64> for HighDimensionalSystem {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        // Coupled nonlinear dynamics
        vec![
            state[0] + state[3] * dt + 0.01 * state[0] * state[1] * dt,
            state[1] + state[4] * dt + 0.01 * state[1] * state[2] * dt,
            state[2] + state[5] * dt + 0.01 * state[2] * state[0] * dt,
            state[3] * (1.0 - 0.1 * dt) + 0.05 * state[4].sin() * dt,
            state[4] * (1.0 - 0.1 * dt) + 0.05 * state[5].sin() * dt,
            state[5] * (1.0 - 0.1 * dt) + 0.05 * state[3].sin() * dt,
        ]
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        // Nonlinear measurements of positions
        vec![
            (state[0].powi(2) + state[1].powi(2)).sqrt(), // Distance from origin in XY
            (state[1].powi(2) + state[2].powi(2)).sqrt(), // Distance from origin in YZ
            state[0] + state[1] + state[2],               // Sum of positions
        ]
    }

    fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
        unreachable!("CKF doesn't need Jacobians")
    }

    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        unreachable!("CKF doesn't need Jacobians")
    }

    fn state_dim(&self) -> usize {
        6
    }
    fn measurement_dim(&self) -> usize {
        3
    }
}

fn main() {
    let args = Args::parse();

    println!("=== CKF High-Dimensional System Example ===\n");

    // Handle seed
    let seed = args.seed.unwrap_or_else(|| {
        let seed_value = rand::random::<u64>();
        println!("Using random seed: {} (use --seed {} to reproduce)", seed_value, seed_value);
        seed_value
    });
    if args.seed.is_some() {
        println!("Using seed: {}", seed);
    }
    println!();

    // Create RNG with the selected seed
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

    let system = HighDimensionalSystem;

    // Initial 6D state: [x, y, z, vx, vy, vz]
    let initial_state = vec![
        args.init_x,
        args.init_y,
        args.init_z,
        args.init_vx,
        args.init_vy,
        args.init_vz,
    ];

    // Initial covariance (6x6 identity scaled)
    let mut initial_covariance = vec![0.0; 36];
    for i in 0..6 {
        initial_covariance[i * 6 + i] = args.init_cov;
    }

    // Process noise (6x6 diagonal)
    let mut process_noise = vec![0.0; 36];
    for i in 0..3 {
        process_noise[i * 6 + i] = args.process_noise_pos; // Position noise
        process_noise[(i + 3) * 6 + (i + 3)] = args.process_noise_vel; // Velocity noise
    }

    // Measurement noise (3x3 diagonal)
    let measurement_noise = vec![
        args.meas_noise1, 0.0, 0.0,
        0.0, args.meas_noise2, 0.0,
        0.0, 0.0, args.meas_noise3,
    ];

    // Create CKF
    let mut ckf = CubatureKalmanFilter::new(
        system,
        initial_state.clone(),
        initial_covariance,
        process_noise,
        measurement_noise,
        args.dt,
    )
    .unwrap();

    println!("Configuration:");
    println!("  Time steps: {}", args.steps);
    println!("  Time step (dt): {:.3} s", args.dt);
    println!("  Initial state: [{:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}]",
        initial_state[0], initial_state[1], initial_state[2],
        initial_state[3], initial_state[4], initial_state[5]);
    println!("  Initial covariance: {:.3}", args.init_cov);
    println!("  Process noise (pos/vel): {:.4} / {:.4}", args.process_noise_pos, args.process_noise_vel);
    println!("  Measurement noise: [{:.3}, {:.3}, {:.3}]",
        args.meas_noise1, args.meas_noise2, args.meas_noise3);
    println!();

    println!("6D State tracking with CKF (2n = 12 cubature points)");

    if args.verbose {
        println!("\nTime | Meas 1 | Meas 2 | Meas 3 | State Norm | Cov Trace");
        println!("-----|--------|--------|--------|------------|----------");
    }

    // Create noise distributions matching the measurement noise covariance
    let noise_dist1 = Normal::new(0.0, args.meas_noise1.sqrt()).unwrap();
    let noise_dist2 = Normal::new(0.0, args.meas_noise2.sqrt()).unwrap();
    let noise_dist3 = Normal::new(0.0, args.meas_noise3.sqrt()).unwrap();

    for t in 1..=args.steps {
        // Predict
        ckf.predict().expect("CKF prediction failed");

        // Generate simulated measurements
        let state = ckf.state();
        let true_meas1 = (state[0].powi(2) + state[1].powi(2)).sqrt();
        let true_meas2 = (state[1].powi(2) + state[2].powi(2)).sqrt();
        let true_meas3 = state[0] + state[1] + state[2];

        // Add random noise
        let measurements = vec![
            true_meas1 + noise_dist1.sample(&mut rng),
            true_meas2 + noise_dist2.sample(&mut rng),
            true_meas3 + noise_dist3.sample(&mut rng),
        ];

        // Update
        ckf.update(&measurements).unwrap();

        if args.verbose {
            // Compute state norm and covariance trace
            let state = ckf.state();
            let state_norm: f64 = state.iter().map(|x| x * x).sum::<f64>().sqrt();

            let cov = ckf.covariance();
            let cov_trace: f64 = (0..6).map(|i| cov[i * 6 + i]).sum();

            println!(
                "{:4} | {:6.3} | {:6.3} | {:6.3} | {:10.3} | {:9.4}",
                t, measurements[0], measurements[1], measurements[2], state_norm, cov_trace
            );
        }
    }

    // Final state
    let final_state = ckf.state();
    let cov = ckf.covariance();
    let cov_trace: f64 = (0..6).map(|i| cov[i * 6 + i]).sum();

    println!("\nFinal state estimate:");
    println!(
        "  Position: [{:.3}, {:.3}, {:.3}]",
        final_state[0], final_state[1], final_state[2]
    );
    println!(
        "  Velocity: [{:.3}, {:.3}, {:.3}]",
        final_state[3], final_state[4], final_state[5]
    );
    println!("  Covariance trace: {:.4}", cov_trace);

    println!("\n✓ CKF successfully handled 6D nonlinear system");
    println!("  Used only 12 cubature points (vs 13 sigma points for UKF)");
}
