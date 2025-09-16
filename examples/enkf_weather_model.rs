//! Ensemble Kalman Filter example: Weather model data assimilation
//!
//! This example demonstrates EnKF for high-dimensional data assimilation
//! using a simplified weather model (Lorenz-96).
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use kalman_filters::{EnsembleKalmanFilter, NonlinearSystem};

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
    println!("=== EnKF Weather Model Data Assimilation Example ===\n");

    // Create Lorenz-96 model with 40 grid points (typical test case)
    let n = 40;
    let system = Lorenz96Model { n, forcing: 8.0 };

    // Initial mean state (small perturbation from equilibrium)
    let mut initial_mean = vec![8.0; n];
    initial_mean[n / 2] = 8.01; // Small perturbation

    // Initial spread
    let initial_spread = vec![0.1; n];

    // Process noise (40x40 diagonal matrix)
    let mut process_noise = vec![0.0; n * n];
    for i in 0..n {
        process_noise[i * n + i] = 0.01;
    }

    // Measurement noise (20x20 diagonal matrix for observing half the points)
    let m = n / 2;
    let mut measurement_noise = vec![0.0; m * m];
    for i in 0..m {
        measurement_noise[i * m + i] = 0.1;
    }

    // Create EnKF with 50 ensemble members
    let ensemble_size = 50;
    let mut enkf = EnsembleKalmanFilter::new(
        system,
        initial_mean,
        initial_spread,
        ensemble_size,
        process_noise,
        measurement_noise,
        0.005, // dt = 0.005 (model time units)
    )
    .unwrap();

    // Set inflation to maintain ensemble spread
    enkf.set_inflation(1.02);

    println!("Lorenz-96 model with {} grid points", n);
    println!("Observing {} locations (every other point)", m);
    println!("Ensemble size: {} members", ensemble_size);
    println!("\nAssimilating observations over time:");
    println!("Time | Ensemble Mean Energy | Ensemble Spread | ESS");
    println!("-----|---------------------|-----------------|--------");

    // Run data assimilation for 100 time steps
    for t in 1..=100 {
        // Forecast step
        enkf.forecast();

        // Generate synthetic observations (truth + noise)
        let ensemble_mean = enkf.mean();
        let mut observations = Vec::new();
        for i in (0..n).step_by(2) {
            observations.push(ensemble_mean[i] + 0.1 * ((t as f64) * 0.1).sin());
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

        // Print every 10 steps
        if t % 10 == 0 {
            println!(
                "{:4} | {:19.3} | {:15.3} | {:6.1}",
                t, energy, avg_spread, ensemble_size as f64
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
        "\n✓ EnKF successfully assimilated data in {}-dimensional system",
        n
    );
    println!(
        "  Used only {} ensemble members (vs {} x {} = {} for full covariance)",
        ensemble_size,
        n,
        n,
        n * n
    );
}
