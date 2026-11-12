//! 2D Position tracking example
//!
//! This example demonstrates tracking a 2D position (x, y) with velocity.
//! The state vector contains [x, y, vx, vy] and we measure [x, y].
//!
//! ## Usage
//! ```bash
//! # Run with default parameters
//! cargo run --example position_2d
//!
//! # Run with custom parameters
//! cargo run --example position_2d -- \
//!   --seed 42 \
//!   --steps 100 \
//!   --dt 0.1 \
//!   --omega 0.1 \
//!   --radius 5.0 \
//!   --init-x 0.0 \
//!   --init-y 0.0 \
//!   --init-vx 1.0 \
//!   --init-vy 0.5 \
//!   --process-noise 0.01 \
//!   --meas-noise 0.1 \
//!   --meas-noise-std 0.3
//!
//! # Show all available options
//! cargo run --example position_2d -- --help
//! ```
//!
//! This example works with both Vec-based and nalgebra implementations.
//! Use `cargo run --example position_2d` for Vec-based
//! Use `cargo run --example position_2d --features nalgebra` for nalgebra-based
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use clap::Parser;
use rand::SeedableRng;
use rand_distr::{Distribution, Uniform};

#[cfg(not(feature = "nalgebra"))]
use kalman_filters::{KalmanFilter, KalmanFilterBuilder};

#[cfg(feature = "nalgebra")]
use kalman_filters::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::{SMatrix, SVector};

/// Command-line arguments for the 2D position tracking example
#[derive(Parser, Debug)]
#[command(name = "2D Position Tracking")]
#[command(about = "Demonstrates 2D position tracking with circular motion", long_about = None)]
struct Args {
    /// Random seed for reproducibility (if not specified, uses random seed)
    #[arg(long)]
    seed: Option<u64>,

    /// Number of time steps to simulate
    #[arg(long, default_value_t = 100)]
    steps: usize,

    /// Time step size (seconds)
    #[arg(long, default_value_t = 0.1)]
    dt: f64,

    /// Angular velocity for circular motion (rad/s)
    #[arg(long, default_value_t = 0.1)]
    omega: f64,

    /// Radius of circular motion (m)
    #[arg(long, default_value_t = 5.0)]
    radius: f64,

    /// Initial X position estimate
    #[arg(long, default_value_t = 0.0)]
    init_x: f64,

    /// Initial Y position estimate
    #[arg(long, default_value_t = 0.0)]
    init_y: f64,

    /// Initial X velocity estimate
    #[arg(long, default_value_t = 1.0)]
    init_vx: f64,

    /// Initial Y velocity estimate
    #[arg(long, default_value_t = 0.5)]
    init_vy: f64,

    /// Process noise strength (affects position and velocity uncertainty)
    #[arg(long, default_value_t = 0.01)]
    process_noise: f64,

    /// Measurement noise variance
    #[arg(long, default_value_t = 0.1)]
    meas_noise: f64,

    /// Measurement noise standard deviation (for simulation)
    #[arg(long, default_value_t = 0.3)]
    meas_noise_std: f64,

    /// Print interval (show every Nth step)
    #[arg(long, default_value_t = 20)]
    print_interval: usize,
}

#[cfg(not(feature = "nalgebra"))]
fn main() {
    let args = Args::parse();

    println!("2D Position Tracking with Kalman Filter (Vec-based)");
    println!("====================================================");

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

    // State transition matrix for 2D constant velocity model
    // State: [x, y, vx, vy]
    let f = vec![
        1.0, 0.0, args.dt, 0.0, // x = x + vx * dt
        0.0, 1.0, 0.0, args.dt, // y = y + vy * dt
        0.0, 0.0, 1.0, 0.0, // vx = vx
        0.0, 0.0, 0.0, 1.0, // vy = vy
    ];

    // Initial state
    let x0 = vec![args.init_x, args.init_y, args.init_vx, args.init_vy];

    // Initial covariance
    let p0 = vec![
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ];

    // Process noise (acceleration uncertainty)
    let q_val = args.process_noise;
    let dt = args.dt;
    let q = vec![
        q_val * dt * dt * dt / 3.0,
        0.0,
        q_val * dt * dt / 2.0,
        0.0,
        0.0,
        q_val * dt * dt * dt / 3.0,
        0.0,
        q_val * dt * dt / 2.0,
        q_val * dt * dt / 2.0,
        0.0,
        q_val * dt,
        0.0,
        0.0,
        q_val * dt * dt / 2.0,
        0.0,
        q_val * dt,
    ];

    // Observation matrix (we measure x and y positions)
    let h = vec![
        1.0, 0.0, 0.0, 0.0, // Measure x
        0.0, 1.0, 0.0, 0.0, // Measure y
    ];

    // Measurement noise
    let r_val = args.meas_noise;
    let r = vec![r_val, 0.0, 0.0, r_val];

    // Build the Kalman filter
    let mut kf = KalmanFilterBuilder::<f64>::new(4, 2)
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
    println!("  Angular velocity (omega): {:.3} rad/s", args.omega);
    println!("  Radius: {:.3} m", args.radius);
    println!("  Initial state: x={:.2}, y={:.2}, vx={:.2}, vy={:.2}",
             args.init_x, args.init_y, args.init_vx, args.init_vy);
    println!("  Process noise: {:.4}", args.process_noise);
    println!("  Measurement noise: {:.3}", args.meas_noise);
    println!("  Measurement std dev: {:.3}", args.meas_noise_std);
    println!();

    // Simulate a moving object with circular motion
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let noise_dist = Uniform::new(-args.meas_noise_std, args.meas_noise_std).unwrap();

    println!(
        "\nTime |  True X  |  True Y  | Meas. X  | Meas. Y  | Est. X   | Est. Y   | Est. Vx  | Est. Vy"
    );
    println!(
        "-----|----------|----------|----------|----------|----------|----------|----------|--------"
    );

    for step in 0..args.steps {
        let t = step as f64 * args.dt;

        // True position (circular motion)
        let true_x = args.radius * (args.omega * t).cos();
        let true_y = args.radius * (args.omega * t).sin();

        // Generate noisy measurements
        let noise_x = noise_dist.sample(&mut rng);
        let noise_y = noise_dist.sample(&mut rng);
        let measured_x = true_x + noise_x;
        let measured_y = true_y + noise_y;

        // Kalman filter predict step
        kf.predict();

        // Kalman filter update step
        kf.update(&[measured_x, measured_y]).expect("Update failed");

        // Get estimated state
        let state = kf.state();

        // Print results at specified intervals
        if step % args.print_interval == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:7.3}",
                t, true_x, true_y, measured_x, measured_y, state[0], state[1], state[2], state[3]
            );
        }
    }

    println!("\nFinal state covariance (diagonal elements):");
    let p = kf.covariance();
    println!("X position variance:  {:.6}", p[0 * 4 + 0]);
    println!("Y position variance:  {:.6}", p[1 * 4 + 1]);
    println!("X velocity variance:  {:.6}", p[2 * 4 + 2]);
    println!("Y velocity variance:  {:.6}", p[3 * 4 + 3]);

    println!("\nThe filter tracked the 2D position and estimated velocities");
    println!("from noisy position measurements.");
}

#[cfg(feature = "nalgebra")]
fn main() {
    let args = Args::parse();

    println!("2D Position Tracking with Kalman Filter (nalgebra-based)");
    println!("=========================================================");

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

    // State transition matrix for 2D constant velocity model
    // State: [x, y, vx, vy]
    let F = SMatrix::<f64, 4, 4>::from([
        [1.0, 0.0, args.dt, 0.0],  // x = x + vx * dt
        [0.0, 1.0, 0.0, args.dt],  // y = y + vy * dt
        [0.0, 0.0, 1.0, 0.0], // vx = vx
        [0.0, 0.0, 0.0, 1.0], // vy = vy
    ]);

    // Initial state
    let x0 = SVector::<f64, 4>::from([args.init_x, args.init_y, args.init_vx, args.init_vy]);

    // Initial covariance
    let P0 = SMatrix::<f64, 4, 4>::from([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);

    // Process noise (acceleration uncertainty)
    let q_val = args.process_noise;
    let dt = args.dt;
    let Q = SMatrix::<f64, 4, 4>::from([
        [q_val * dt * dt * dt / 3.0, 0.0, q_val * dt * dt / 2.0, 0.0],
        [0.0, q_val * dt * dt * dt / 3.0, 0.0, q_val * dt * dt / 2.0],
        [q_val * dt * dt / 2.0, 0.0, q_val * dt, 0.0],
        [0.0, q_val * dt * dt / 2.0, 0.0, q_val * dt],
    ]);

    // Observation matrix (we measure x and y positions)
    let H = SMatrix::<f64, 2, 4>::from_row_slice(&[
        1.0, 0.0, 0.0, 0.0, // Measure x
        0.0, 1.0, 0.0, 0.0, // Measure y
    ]);

    // Measurement noise
    let r_val = args.meas_noise;
    let R = SMatrix::<f64, 2, 2>::from([[r_val, 0.0], [0.0, r_val]]);

    // Create the Kalman filter
    let mut kf = StaticKalmanFilter::new(x0, P0, F, Q, H, R);

    println!("Configuration:");
    println!("  Time steps: {}", args.steps);
    println!("  Time step (dt): {:.3} s", args.dt);
    println!("  Angular velocity (omega): {:.3} rad/s", args.omega);
    println!("  Radius: {:.3} m", args.radius);
    println!("  Initial state: x={:.2}, y={:.2}, vx={:.2}, vy={:.2}",
             args.init_x, args.init_y, args.init_vx, args.init_vy);
    println!("  Process noise: {:.4}", args.process_noise);
    println!("  Measurement noise: {:.3}", args.meas_noise);
    println!("  Measurement std dev: {:.3}", args.meas_noise_std);
    println!();

    // Simulate a moving object with circular motion
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let noise_dist = Uniform::new(-args.meas_noise_std, args.meas_noise_std).unwrap();

    println!(
        "\nTime |  True X  |  True Y  | Meas. X  | Meas. Y  | Est. X   | Est. Y   | Est. Vx  | Est. Vy"
    );
    println!(
        "-----|----------|----------|----------|----------|----------|----------|----------|--------"
    );

    for step in 0..args.steps {
        let t = step as f64 * args.dt;

        // True position (circular motion)
        let true_x = args.radius * (args.omega * t).cos();
        let true_y = args.radius * (args.omega * t).sin();

        // Generate noisy measurements
        let noise_x = noise_dist.sample(&mut rng);
        let noise_y = noise_dist.sample(&mut rng);
        let measured_x = true_x + noise_x;
        let measured_y = true_y + noise_y;

        // Kalman filter predict step
        kf.predict();

        // Kalman filter update step
        let measurement = SVector::<f64, 2>::from([measured_x, measured_y]);
        kf.update(&measurement).expect("Update failed");

        // Get estimated state
        let state = kf.state();

        // Print results at specified intervals
        if step % args.print_interval == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:7.3}",
                t, true_x, true_y, measured_x, measured_y, state[0], state[1], state[2], state[3]
            );
        }
    }

    println!("\nFinal state covariance (diagonal elements):");
    let P = kf.covariance();
    println!("X position variance:  {:.6}", P[(0, 0)]);
    println!("Y position variance:  {:.6}", P[(1, 1)]);
    println!("X velocity variance:  {:.6}", P[(2, 2)]);
    println!("Y velocity variance:  {:.6}", P[(3, 3)]);

    println!("\nThe filter tracked the 2D position and estimated velocities");
    println!("from noisy position measurements.");
}
