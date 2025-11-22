//! Simple 1D Kalman filter example
//!
//! This example demonstrates tracking a 1D position with noisy measurements.
//! The system models constant velocity motion where:
//! - State: [position, velocity]
//! - Measurement: position only
//!
//! ## Usage
//! ```bash
//! # Run with default parameters
//! cargo run --example simple_1d
//!
//! # Run with custom parameters
//! cargo run --example simple_1d -- \
//!   --seed 42 \
//!   --steps 100 \
//!   --dt 0.05 \
//!   --velocity 2.0 \
//!   --meas-noise-std 0.5
//!
//! # Show all available options
//! cargo run --example simple_1d -- --help
//! ```
//!
//! This example works with both Vec-based and nalgebra implementations.
//! Use `cargo run --example simple_1d` for Vec-based
//! Use `cargo run --example simple_1d --features nalgebra` for nalgebra-based
#![allow(unused, non_snake_case)]

use clap::Parser;
use rand::SeedableRng;
use rand_distr::{Distribution, Uniform};

#[cfg(not(feature = "nalgebra"))]
use kalman_filters::{KalmanFilter, KalmanFilterBuilder};

#[cfg(feature = "nalgebra")]
use kalman_filters::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::{SMatrix, SVector};

/// Command-line arguments for the simple 1D Kalman filter example
#[derive(Parser, Debug)]
#[command(name = "Simple 1D Kalman Filter")]
#[command(about = "Demonstrates 1D position tracking with constant velocity model", long_about = None)]
struct Args {
    /// Random seed for reproducibility (if not specified, uses random seed)
    #[arg(long)]
    seed: Option<u64>,

    /// Number of time steps to simulate
    #[arg(long, default_value_t = 50)]
    steps: usize,

    /// Time step size (seconds)
    #[arg(long, default_value_t = 0.1)]
    dt: f64,

    /// True velocity of the object (m/s)
    #[arg(long, default_value_t = 1.0)]
    velocity: f64,

    /// Initial position estimate
    #[arg(long, default_value_t = 0.0)]
    init_pos: f64,

    /// Initial velocity estimate
    #[arg(long, default_value_t = 1.0)]
    init_vel: f64,

    /// Initial position covariance
    #[arg(long, default_value_t = 1.0)]
    init_pos_cov: f64,

    /// Initial velocity covariance
    #[arg(long, default_value_t = 1.0)]
    init_vel_cov: f64,

    /// Process noise for position
    #[arg(long, default_value_t = 0.001)]
    process_noise_pos: f64,

    /// Process noise for velocity
    #[arg(long, default_value_t = 0.001)]
    process_noise_vel: f64,

    /// Measurement noise variance
    #[arg(long, default_value_t = 0.1)]
    meas_noise: f64,

    /// Measurement noise standard deviation (for simulation)
    #[arg(long, default_value_t = 0.3)]
    meas_noise_std: f64,

    /// Print interval (show every Nth step)
    #[arg(long, default_value_t = 10)]
    print_interval: usize,
}

#[cfg(not(feature = "nalgebra"))]
fn main() {
    let args = Args::parse();

    println!("Simple 1D Kalman Filter Example (Vec-based)");
    println!("============================================");

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

    // Create state transition matrix (constant velocity model)
    // [position_new] = [1  dt] * [position_old]
    // [velocity_new]   [0  1 ]   [velocity_old]
    let f = vec![1.0, args.dt, 0.0, 1.0];

    // Initial state
    let x0 = vec![args.init_pos, args.init_vel];

    // Initial covariance (uncertainty)
    let p0 = vec![args.init_pos_cov, 0.0, 0.0, args.init_vel_cov];

    // Process noise (how much the system can change randomly)
    let q = vec![args.process_noise_pos, 0.0, 0.0, args.process_noise_vel];

    // Observation matrix (we only measure position)
    let h = vec![1.0, 0.0];

    // Measurement noise
    let r = vec![args.meas_noise];

    // Build the Kalman filter
    let mut kf = KalmanFilterBuilder::<f64>::new(2, 1)
        .initial_state(x0)
        .initial_covariance(p0)
        .transition_matrix(f)
        .process_noise(q)
        .observation_matrix(h)
        .measurement_noise(r)
        .build()
        .expect("Failed to build Kalman filter");

    println!("Configuration:");
    println!("  Time steps: {}", args.steps);
    println!("  Time step (dt): {:.3} s", args.dt);
    println!("  True velocity: {:.3} m/s", args.velocity);
    println!("  Initial estimate: pos={:.2}, vel={:.2}", args.init_pos, args.init_vel);
    println!("  Process noise: pos={:.4}, vel={:.4}", args.process_noise_pos, args.process_noise_vel);
    println!("  Measurement noise: {:.3}", args.meas_noise);
    println!("  Measurement std dev: {:.3}", args.meas_noise_std);
    println!();

    // Simulate true system and noisy measurements
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let noise_dist = Uniform::new(-args.meas_noise_std, args.meas_noise_std).unwrap();

    println!("\nTime | True Pos | Measured | Estimated | Est. Vel");
    println!("-----|----------|----------|-----------|----------");

    for step in 0..args.steps {
        let t = step as f64 * args.dt;
        let true_position = args.velocity * t;

        // Generate noisy measurement
        let noise = noise_dist.sample(&mut rng);
        let measured_position = true_position + noise;

        // Kalman filter predict step
        kf.predict();

        // Kalman filter update step
        kf.update(&[measured_position]).expect("Update failed");

        // Get estimated state
        let state = kf.state();
        let estimated_position = state[0];
        let estimated_velocity = state[1];

        // Print results at specified intervals
        if step % args.print_interval == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:9.3} | {:8.3}",
                t, true_position, measured_position, estimated_position, estimated_velocity
            );
        }
    }

    println!("\nFinal state covariance:");
    let p = kf.covariance();
    println!("  Position variance: {:.6}", p[0]);
    println!("  Velocity variance: {:.6}", p[3]);

    println!("\nThe Kalman filter successfully tracked the position and");
    println!("estimated the velocity from noisy position measurements.");
}

#[cfg(feature = "nalgebra")]
fn main() {
    let args = Args::parse();

    println!("Simple 1D Kalman Filter Example (nalgebra-based)");
    println!("=================================================");

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

    // Create state transition matrix (constant velocity model)
    let F = SMatrix::<f64, 2, 2>::from_row_slice(&[1.0, args.dt, 0.0, 1.0]);

    // Initial state
    let x0 = SVector::<f64, 2>::from([args.init_pos, args.init_vel]);

    // Initial covariance (uncertainty)
    let P0 = SMatrix::<f64, 2, 2>::from_row_slice(&[args.init_pos_cov, 0.0, 0.0, args.init_vel_cov]);

    // Process noise (how much the system can change randomly)
    let Q = SMatrix::<f64, 2, 2>::from_row_slice(&[args.process_noise_pos, 0.0, 0.0, args.process_noise_vel]);

    // Observation matrix (we only measure position)
    let H = SMatrix::<f64, 1, 2>::from_row_slice(&[1.0, 0.0]);

    // Measurement noise
    let R = SMatrix::<f64, 1, 1>::from_row_slice(&[args.meas_noise]);

    // Create the Kalman filter
    let mut kf = StaticKalmanFilter::new(x0, P0, F, Q, H, R);

    println!("Configuration:");
    println!("  Time steps: {}", args.steps);
    println!("  Time step (dt): {:.3} s", args.dt);
    println!("  True velocity: {:.3} m/s", args.velocity);
    println!("  Initial estimate: pos={:.2}, vel={:.2}", args.init_pos, args.init_vel);
    println!("  Process noise: pos={:.4}, vel={:.4}", args.process_noise_pos, args.process_noise_vel);
    println!("  Measurement noise: {:.3}", args.meas_noise);
    println!("  Measurement std dev: {:.3}", args.meas_noise_std);
    println!();

    // Simulate true system and noisy measurements
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let noise_dist = Uniform::new(-args.meas_noise_std, args.meas_noise_std).unwrap();

    println!("\nTime | True Pos | Measured | Estimated | Est. Vel");
    println!("-----|----------|----------|-----------|----------");

    for step in 0..args.steps {
        let t = step as f64 * args.dt;
        let true_position = args.velocity * t;

        // Generate noisy measurement
        let noise = noise_dist.sample(&mut rng);
        let measured_position = true_position + noise;

        // Kalman filter predict step
        kf.predict();

        // Kalman filter update step
        let measurement = SVector::<f64, 1>::from([measured_position]);
        kf.update(&measurement).expect("Update failed");

        // Get estimated state
        let state = kf.state();
        let estimated_position = state[0];
        let estimated_velocity = state[1];

        // Print results at specified intervals
        if step % args.print_interval == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:9.3} | {:8.3}",
                t, true_position, measured_position, estimated_position, estimated_velocity
            );
        }
    }

    println!("\nFinal state covariance:");
    let P = kf.covariance();
    println!("  Position variance: {:.6}", P[(0, 0)]);
    println!("  Velocity variance: {:.6}", P[(1, 1)]);

    println!("\nThe Kalman filter successfully tracked the position and");
    println!("estimated the velocity from noisy position measurements.");
}
