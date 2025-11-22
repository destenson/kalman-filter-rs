//! Ensemble Kalman Filter example: Weather model data assimilation
//!
//! This example demonstrates EnKF for high-dimensional data assimilation
//! using a simplified weather model (Lorenz-96).
//!
//! ## Usage
//! ```bash
//! # Run with random noise (different output each time)
//! cargo run --example enkf_weather_model
//!
//! # Run with a specific seed for reproducible results
//! cargo run --example enkf_weather_model -- --seed 42
//!
//! # Customize simulation parameters
//! cargo run --example enkf_weather_model -- --steps 200 --dt 0.01 --n 60 --ensemble-size 100
//! ```
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use clap::Parser;
use kalman_filters::{EnsembleKalmanFilter, NonlinearSystem};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

/// EnKF Weather Model Data Assimilation Example
///
/// Demonstrates ensemble Kalman filtering for high-dimensional data assimilation
/// using the Lorenz-96 model, a simplified weather model.
#[derive(Parser, Debug)]
#[command(name = "enkf_weather_model")]
#[command(about = "EnKF data assimilation with Lorenz-96 weather model", long_about = None)]
struct Args {
    /// Random seed for reproducibility (if not set, uses random seed)
    #[arg(long)]
    seed: Option<u64>,

    /// Number of time steps to simulate
    #[arg(long, default_value_t = 100)]
    steps: usize,

    /// Time step size for integration
    #[arg(long, default_value_t = 0.005)]
    dt: f64,

    /// Number of grid points in Lorenz-96 model
    #[arg(long, default_value_t = 40)]
    n: usize,

    /// Forcing parameter for Lorenz-96 model
    #[arg(long, default_value_t = 8.0)]
    forcing: f64,

    /// Number of ensemble members
    #[arg(long, default_value_t = 50)]
    ensemble_size: usize,

    /// Inflation factor to maintain ensemble spread
    #[arg(long, default_value_t = 1.02)]
    inflation: f64,

    /// Process noise variance (diagonal)
    #[arg(long, default_value_t = 0.01)]
    process_noise: f64,

    /// Measurement noise variance
    #[arg(long, default_value_t = 0.1)]
    measurement_noise: f64,

    /// Print interval (print every N steps)
    #[arg(long, default_value_t = 10)]
    print_interval: usize,
}

/// Lorenz-96 model - a simplified weather model
struct Lorenz96Model {
    /// Number of grid points
    n: usize,
    /// Forcing parameter
    forcing: f64,
}

impl NonlinearSystem<f64> for Lorenz96Model {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let n = self.n;
        let mut dxdt = vec![0.0; n];

        // Lorenz-96 dynamics: dx_i/dt = (x_{i+1} - x_{i-2}) * x_{i-1} - x_i + F
        for i in 0..n {
            let im2 = if i >= 2 { i - 2 } else { n + i - 2 };
            let im1 = if i >= 1 { i - 1 } else { n - 1 };
            let ip1 = (i + 1) % n;

            dxdt[i] = (state[ip1] - state[im2]) * state[im1] - state[i] + self.forcing;
        }

        // Euler integration
        let mut new_state = vec![0.0; n];
        for i in 0..n {
            new_state[i] = state[i] + dxdt[i] * dt;
        }

        new_state
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        // Observe every other grid point
        let mut obs = Vec::new();
        for i in (0..self.n).step_by(2) {
            obs.push(state[i]);
        }
        obs
    }

    fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
        unreachable!("EnKF doesn't need Jacobians")
    }

    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        unreachable!("EnKF doesn't need Jacobians")
    }

    fn state_dim(&self) -> usize {
        self.n
    }
    fn measurement_dim(&self) -> usize {
        self.n / 2
    }
}

fn main() {
    // Parse command-line arguments
    let args = Args::parse();

    println!("=== EnKF Weather Model Data Assimilation Example ===\n");

    // Print configuration summary
    println!("Configuration:");
    println!("  Model parameters:");
    println!("    Grid points (n): {}", args.n);
    println!("    Forcing: {}", args.forcing);
    println!("    Time step (dt): {}", args.dt);
    println!("  Filter parameters:");
    println!("    Ensemble size: {}", args.ensemble_size);
    println!("    Inflation factor: {}", args.inflation);
    println!("    Process noise: {}", args.process_noise);
    println!("    Measurement noise: {}", args.measurement_noise);
    println!("  Simulation:");
    println!("    Time steps: {}", args.steps);
    println!("    Print interval: {}", args.print_interval);

    // Handle seed
    let seed = if let Some(seed_value) = args.seed {
        println!("    Seed: {} (reproducible)\n", seed_value);
        seed_value
    } else {
        let seed_value = rand::random::<u64>();
        println!("    Seed: {} (random, use --seed {} to reproduce)\n", seed_value, seed_value);
        seed_value
    };

    // Create RNG with the selected seed
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

    // Create Lorenz-96 model
    let n = args.n;
    let system = Lorenz96Model {
        n,
        forcing: args.forcing
    };

    // Initial mean state (small perturbation from equilibrium)
    let mut initial_mean = vec![args.forcing; n];
    initial_mean[n / 2] = args.forcing + 0.01; // Small perturbation

    // Initial spread
    let initial_spread = vec![0.1; n];

    // Process noise (n x n diagonal matrix)
    let mut process_noise = vec![0.0; n * n];
    for i in 0..n {
        process_noise[i * n + i] = args.process_noise;
    }

    // Measurement noise (m x m diagonal matrix for observing half the points)
    let m = n / 2;
    let mut measurement_noise = vec![0.0; m * m];
    for i in 0..m {
        measurement_noise[i * m + i] = args.measurement_noise;
    }

    // Create EnKF
    let mut enkf = EnsembleKalmanFilter::new(
        system,
        initial_mean,
        initial_spread,
        args.ensemble_size,
        process_noise,
        measurement_noise,
        args.dt,
    )
    .unwrap();

    // Set inflation to maintain ensemble spread
    enkf.set_inflation(args.inflation);

    println!("Lorenz-96 model with {} grid points", n);
    println!("Observing {} locations (every other point)", m);
    println!("Ensemble size: {} members", args.ensemble_size);
    println!("\nAssimilating observations over time:");
    println!("Time | Ensemble Mean Energy | Ensemble Spread | ESS");
    println!("-----|---------------------|-----------------|--------");

    // Create noise distribution for observations
    let obs_noise = Normal::new(0.0, args.measurement_noise.sqrt()).unwrap();

    // Run data assimilation
    for t in 1..=args.steps {
        // Forecast step
        enkf.forecast();

        // Generate synthetic observations (truth + noise)
        let ensemble_mean = enkf.mean();
        let mut observations = Vec::new();
        for i in (0..n).step_by(2) {
            observations.push(ensemble_mean[i] + obs_noise.sample(&mut rng));
        }

        // Analysis step (update with observations)
        enkf.update(&observations).unwrap();

        // Compute diagnostics
        let mean = enkf.mean();
        let spread = enkf.spread();

        // Energy (sum of squares)
        let energy: f64 = mean.iter().map(|x| x * x).sum::<f64>() / n as f64;

        // Average spread
        let avg_spread: f64 = spread.iter().sum::<f64>() / n as f64;

        // Print at specified interval
        if t % args.print_interval == 0 {
            println!(
                "{:4} | {:19.3} | {:15.3} | {:6.1}",
                t, energy, avg_spread, args.ensemble_size as f64
            );
        }
    }

    // Final statistics
    let final_mean = enkf.mean();
    let final_spread = enkf.spread();

    println!("\nFinal state statistics:");
    println!(
        "  Mean state range: [{:.2}, {:.2}]",
        final_mean.iter().fold(f64::INFINITY, |a, &b| a.min(b)),
        final_mean.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b))
    );
    println!(
        "  Average uncertainty: {:.3}",
        final_spread.iter().sum::<f64>() / n as f64
    );

    println!(
        "\nEnKF successfully assimilated data in {}-dimensional system",
        n
    );
    println!(
        "  Used only {} ensemble members (vs {} x {} = {} for full covariance)",
        args.ensemble_size,
        n,
        n,
        n * n
    );
}
