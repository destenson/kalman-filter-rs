//! Unscented Kalman Filter example: Nonlinear target tracking
//!
//! This example demonstrates tracking a target with nonlinear dynamics
//! using the UKF, which handles nonlinearity better than EKF without
//! requiring Jacobian calculations.
//!
//! ## Usage
//! ```bash
//! # Run with default parameters
//! cargo run --example ukf_nonlinear_tracking
//!
//! # Run with custom parameters
//! cargo run --example ukf_nonlinear_tracking -- \
//!   --seed 42 \
//!   --steps 50 \
//!   --dt 0.5 \
//!   --turn-rate 0.05 \
//!   --alpha 0.001
//!
//! # Show all available options
//! cargo run --example ukf_nonlinear_tracking -- --help
//! ```
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use clap::Parser;
use kalman_filters::{NonlinearSystem, UKFParameters, UnscentedKalmanFilter};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

/// Command-line arguments for the UKF nonlinear tracking example
#[derive(Parser, Debug)]
#[command(name = "UKF Nonlinear Target Tracking")]
#[command(about = "Demonstrates UKF tracking of a target with coordinated turn dynamics", long_about = None)]
struct Args {
    /// Random seed for reproducibility (if not specified, uses random seed)
    #[arg(long)]
    seed: Option<u64>,

    /// Number of time steps to simulate
    #[arg(long, default_value_t = 10)]
    steps: usize,

    /// Time step size (seconds)
    #[arg(long, default_value_t = 1.0)]
    dt: f64,

    /// Turn rate (rad/s)
    #[arg(long, default_value_t = 0.1)]
    turn_rate: f64,

    /// Initial position X
    #[arg(long, default_value_t = 100.0)]
    init_x: f64,

    /// Initial position Y
    #[arg(long, default_value_t = 50.0)]
    init_y: f64,

    /// Initial velocity X
    #[arg(long, default_value_t = 10.0)]
    init_vx: f64,

    /// Initial velocity Y
    #[arg(long, default_value_t = 5.0)]
    init_vy: f64,

    /// Initial position covariance
    #[arg(long, default_value_t = 10.0)]
    init_pos_cov: f64,

    /// Initial velocity covariance
    #[arg(long, default_value_t = 1.0)]
    init_vel_cov: f64,

    /// Process noise for position
    #[arg(long, default_value_t = 0.1)]
    process_noise_pos: f64,

    /// Process noise for velocity
    #[arg(long, default_value_t = 0.01)]
    process_noise_vel: f64,

    /// Measurement noise for range
    #[arg(long, default_value_t = 1.0)]
    meas_noise_range: f64,

    /// Measurement noise for bearing (radians)
    #[arg(long, default_value_t = 0.01)]
    meas_noise_bearing: f64,

    /// UKF alpha parameter (spread of sigma points)
    #[arg(long, default_value_t = 1e-3)]
    alpha: f64,

    /// UKF beta parameter (prior knowledge of distribution)
    #[arg(long, default_value_t = 2.0)]
    beta: f64,

    /// UKF kappa parameter (secondary scaling)
    #[arg(long, default_value_t = 0.0)]
    kappa: f64,

    /// Print interval (show every Nth step, 0 means show all)
    #[arg(long, default_value_t = 1)]
    print_interval: usize,
}

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
    let args = Args::parse();

    println!("=== UKF Nonlinear Target Tracking Example ===\n");

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

    // Create coordinated turn model
    let system = CoordinatedTurnModel { omega: args.turn_rate };

    // Initial state: [x, y, vx, vy]
    let initial_state = vec![args.init_x, args.init_y, args.init_vx, args.init_vy];
    let initial_covariance = vec![
        args.init_pos_cov, 0.0, 0.0, 0.0,
        0.0, args.init_pos_cov, 0.0, 0.0,
        0.0, 0.0, args.init_vel_cov, 0.0,
        0.0, 0.0, 0.0, args.init_vel_cov,
    ];

    // Process and measurement noise
    let process_noise = vec![
        args.process_noise_pos, 0.0, 0.0, 0.0,
        0.0, args.process_noise_pos, 0.0, 0.0,
        0.0, 0.0, args.process_noise_vel, 0.0,
        0.0, 0.0, 0.0, args.process_noise_vel,
    ];
    let measurement_noise = vec![
        args.meas_noise_range, 0.0,
        0.0, args.meas_noise_bearing,
    ];

    // Create UKF
    let mut ukf = UnscentedKalmanFilter::new(
        system,
        initial_state.clone(),
        initial_covariance,
        process_noise,
        measurement_noise,
        args.dt,
    )
    .unwrap();

    // Set UKF parameters
    let mut params = UKFParameters::default();
    params.alpha = args.alpha;
    params.beta = args.beta;
    params.kappa = args.kappa;
    ukf.set_parameters(params);

    println!("Configuration:");
    println!("  Time steps: {}", args.steps);
    println!("  Time step (dt): {:.3} s", args.dt);
    println!("  Turn rate: {:.3} rad/s", args.turn_rate);
    println!("  Initial state: [{:.1}, {:.1}, {:.1}, {:.1}]",
        initial_state[0], initial_state[1], initial_state[2], initial_state[3]);
    println!("  Initial covariance: pos={:.2}, vel={:.2}", args.init_pos_cov, args.init_vel_cov);
    println!("  Process noise: pos={:.4}, vel={:.4}", args.process_noise_pos, args.process_noise_vel);
    println!("  Measurement noise: range={:.3}, bearing={:.4}", args.meas_noise_range, args.meas_noise_bearing);
    println!("  UKF parameters: alpha={:.3e}, beta={:.1}, kappa={:.1}", args.alpha, args.beta, args.kappa);
    println!();

    println!("Tracking target with coordinated turn model:");
    println!("Time | Range | Bearing | Est. X | Est. Y | Est. Vx | Est. Vy");
    println!("-----|-------|---------|--------|--------|---------|--------");

    // Create noise distributions matching the measurement noise covariance
    let range_noise = Normal::new(0.0, args.meas_noise_range.sqrt()).unwrap();
    let bearing_noise = Normal::new(0.0, args.meas_noise_bearing.sqrt()).unwrap();

    for t in 1..=args.steps {
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

        // Print results at specified intervals
        if args.print_interval > 0 && t % args.print_interval == 0 {
            let state = ukf.state();
            println!(
                "{:4} | {:5.1} | {:7.3} | {:6.1} | {:6.1} | {:7.2} | {:6.2}",
                t, measured_range, measured_bearing, state[0], state[1], state[2], state[3]
            );
        }
    }

    // Print final covariance
    println!("\nFinal position uncertainty (std dev):");
    let cov = ukf.covariance();
    println!("  σ_x = {:.2} m", cov[0].sqrt());
    println!("  σ_y = {:.2} m", cov[5].sqrt());

    println!("\n✓ UKF successfully tracked nonlinear target motion");
}
