//! Simple 1D Kalman filter example with unified API
//!
//! This example demonstrates the backend-agnostic unified API.
//! The same code works regardless of whether the `nalgebra` feature is enabled!
//!
//! ## Usage
//! ```bash
//! # Run with Vec backend (default)
//! cargo run --example simple_1d_unified
//!
//! # Run with nalgebra backend (automatically selected for 2x1 dimensions)
//! cargo run --example simple_1d_unified --features nalgebra
//!
//! # Custom parameters
//! cargo run --example simple_1d_unified -- --seed 42 --steps 100 --velocity 2.0
//! ```

use clap::Parser;
use kalman_filters::{KalmanFilterBuilder, KalmanFilterTrait};
use rand::SeedableRng;
use rand_distr::{Distribution, Uniform};

#[derive(Parser, Debug)]
#[command(name = "Simple 1D Kalman Filter (Unified API)")]
#[command(about = "Backend-agnostic 1D position tracking example")]
struct Args {
    /// Random seed for reproducibility
    #[arg(long)]
    seed: Option<u64>,

    /// Number of time steps
    #[arg(long, default_value_t = 50)]
    steps: usize,

    /// Time step size (seconds)
    #[arg(long, default_value_t = 0.1)]
    dt: f64,

    /// True velocity (m/s)
    #[arg(long, default_value_t = 1.0)]
    velocity: f64,

    /// Initial position estimate
    #[arg(long, default_value_t = 0.0)]
    init_pos: f64,

    /// Initial velocity estimate
    #[arg(long, default_value_t = 1.0)]
    init_vel: f64,

    /// Process noise for position
    #[arg(long, default_value_t = 0.001)]
    process_noise_pos: f64,

    /// Process noise for velocity
    #[arg(long, default_value_t = 0.001)]
    process_noise_vel: f64,

    /// Measurement noise variance
    #[arg(long, default_value_t = 0.1)]
    meas_noise: f64,

    /// Measurement noise std dev (simulation)
    #[arg(long, default_value_t = 0.3)]
    meas_noise_std: f64,

    /// Print interval
    #[arg(long, default_value_t = 10)]
    print_interval: usize,
}

fn main() {
    let args = Args::parse();

    println!("=== Simple 1D Kalman Filter (Unified API) ===\n");

    // Handle seed
    let seed = args.seed.unwrap_or_else(|| {
        let s = rand::random::<u64>();
        println!("Using random seed: {} (use --seed {} to reproduce)\n", s, s);
        s
    });
    if args.seed.is_some() {
        println!("Using seed: {}\n", seed);
    }

    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

    println!("Configuration:");
    println!("  Steps: {}", args.steps);
    println!("  Time step (dt): {} s", args.dt);
    println!("  True velocity: {} m/s", args.velocity);
    println!("  Initial estimate: pos={}, vel={}", args.init_pos, args.init_vel);
    println!("  Process noise: pos={}, vel={}", args.process_noise_pos, args.process_noise_vel);
    println!("  Measurement noise: {}", args.meas_noise);
    println!();

    // Build Kalman filter - works identically regardless of backend!
    let mut kf = KalmanFilterBuilder::new(2, 1)
        .initial_state(vec![args.init_pos, args.init_vel])
        .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
        .transition_matrix(vec![1.0, args.dt, 0.0, 1.0])
        .process_noise(vec![
            args.process_noise_pos, 0.0,
            0.0, args.process_noise_vel
        ])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![args.meas_noise])
        .build()
        .unwrap();

    println!("Step | True Pos | Measured | Estimated | Error");
    println!("-----|----------|----------|-----------|-------");

    let noise_dist = Uniform::new(-args.meas_noise_std, args.meas_noise_std).unwrap();

    for step in 0..args.steps {
        let t = step as f64 * args.dt;
        let true_pos = args.velocity * t;

        // Simulate noisy measurement
        let measurement = true_pos + noise_dist.sample(&mut rng);

        // Kalman filter predict and update - same API regardless of backend!
        kf.predict();
        kf.update(&[measurement]).unwrap();

        // Get state estimate
        let state = kf.state();
        let error = (state[0] - true_pos).abs();

        if step % args.print_interval == 0 {
            println!(
                "{:4} | {:8.2} | {:8.2} | {:9.2} | {:5.2}",
                step, true_pos, measurement, state[0], error
            );
        }
    }

    let final_state = kf.state();
    println!("\nFinal estimate:");
    println!("  Position: {:.2} m", final_state[0]);
    println!("  Velocity: {:.2} m/s", final_state[1]);

    #[cfg(feature = "nalgebra")]
    println!("\n✓ Using nalgebra backend (compile-time optimized)");

    #[cfg(not(feature = "nalgebra"))]
    println!("\n✓ Using Vec backend (runtime flexible)");

    println!("\nThis code works identically regardless of backend!");
}
