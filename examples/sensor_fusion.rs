//! Sensor fusion example
//!
//! This example demonstrates fusing data from multiple sensors with different
//! characteristics to track a moving object. We simulate:
//! - GPS sensor: Provides position measurements with moderate noise
//! - IMU accelerometer: Provides acceleration measurements
//! - Radar: Provides position and velocity with different noise characteristics
//!
//! The Kalman filter combines these measurements optimally based on their
//! respective noise characteristics.
//!
//! ## Usage
//! ```bash
//! # Run with default parameters
//! cargo run --example sensor_fusion
//!
//! # Run with a specific seed for reproducible results
//! cargo run --example sensor_fusion -- --seed 42
//!
//! # Run with custom parameters
//! cargo run --example sensor_fusion -- --steps 200 --dt 0.05 --omega 0.3
//!
//! # Customize noise levels
//! cargo run --example sensor_fusion -- --gps-noise 1.0 --radar-noise 0.3 --imu-noise 0.1
//! ```
#![allow(unused, non_snake_case)]

use clap::Parser;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

#[cfg(not(feature = "nalgebra"))]
use kalman_filters::{KalmanFilter, KalmanFilterBuilder};

#[cfg(feature = "nalgebra")]
use kalman_filters::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::{SMatrix, SVector};

/// Multi-sensor fusion Kalman filter example
///
/// Fuses GPS, IMU (accelerometer), and Radar measurements to track a moving object
/// following a circular trajectory. Each sensor has different noise characteristics:
/// - GPS: Moderate position noise (~0.5m)
/// - Radar: Better position accuracy (~0.2m), includes velocity
/// - IMU: High-frequency acceleration measurements (~0.05m/s²)
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Random seed for reproducible results (random if not specified)
    #[arg(long)]
    seed: Option<u64>,

    /// Number of simulation steps
    #[arg(long, default_value_t = 100)]
    steps: usize,

    /// Time step (seconds)
    #[arg(long, default_value_t = 0.1)]
    dt: f64,

    /// Angular frequency for circular motion (rad/s)
    #[arg(long, default_value_t = 0.2)]
    omega: f64,

    /// Radius for circular motion (m)
    #[arg(long, default_value_t = 10.0)]
    radius: f64,

    /// Initial x position (m)
    #[arg(long, default_value_t = 0.0)]
    x0: f64,

    /// Initial y position (m)
    #[arg(long, default_value_t = 0.0)]
    y0: f64,

    /// Initial x velocity (m/s)
    #[arg(long, default_value_t = 2.0)]
    vx0: f64,

    /// Initial y velocity (m/s)
    #[arg(long, default_value_t = 1.0)]
    vy0: f64,

    /// Initial x acceleration (m/s²)
    #[arg(long, default_value_t = 0.0)]
    ax0: f64,

    /// Initial y acceleration (m/s²)
    #[arg(long, default_value_t = 0.0)]
    ay0: f64,

    /// Acceleration process noise
    #[arg(long, default_value_t = 0.1)]
    q_acc: f64,

    /// GPS measurement noise standard deviation (m)
    #[arg(long, default_value_t = 0.5)]
    gps_noise: f64,

    /// Radar measurement noise standard deviation (m)
    #[arg(long, default_value_t = 0.2)]
    radar_noise: f64,

    /// IMU measurement noise standard deviation (m/s²)
    #[arg(long, default_value_t = 0.05)]
    imu_noise: f64,

    /// Print output every N steps
    #[arg(long, default_value_t = 10)]
    print_interval: usize,
}

#[cfg(not(feature = "nalgebra"))]
fn main() {
    let args = Args::parse();

    println!("Multi-Sensor Fusion Example (Vec-based)");
    println!("========================================");
    println!("Fusing GPS, IMU, and Radar measurements\n");

    // Set up random seed
    let seed = args.seed.unwrap_or_else(|| {
        let seed_value = rand::random::<u64>();
        println!("Using random seed: {} (use --seed {} to reproduce)\n", seed_value, seed_value);
        seed_value
    });
    if args.seed.is_some() {
        println!("Using seed: {}\n", seed);
    }

    let dt = args.dt;

    // State vector: [x, y, vx, vy, ax, ay] - position, velocity, acceleration
    // State transition matrix (constant acceleration model)
    let f = vec![
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0,
        0.0, // x
        0.0,
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0, // y
        0.0,
        0.0,
        1.0,
        0.0,
        dt,
        0.0, // vx
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        dt, // vy
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0, // ax
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0, // ay
    ];

    // Initial state from arguments
    let x0 = vec![args.x0, args.y0, args.vx0, args.vy0, args.ax0, args.ay0];

    // Initial covariance
    let p0 = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    // Process noise (uncertainty in acceleration)
    let q_acc = args.q_acc;
    let mut q = vec![0.0; 36];
    // Fill process noise matrix
    q[0 * 6 + 0] = dt.powi(4) / 4.0 * q_acc; // x-x
    q[1 * 6 + 1] = dt.powi(4) / 4.0 * q_acc; // y-y
    q[2 * 6 + 2] = dt.powi(2) * q_acc; // vx-vx
    q[3 * 6 + 3] = dt.powi(2) * q_acc; // vy-vy
    q[4 * 6 + 4] = q_acc; // ax-ax
    q[5 * 6 + 5] = q_acc; // ay-ay

    // GPS observation matrix (measures x, y position only)
    let h_gps = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // x
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // y
    ];

    // GPS measurement noise (moderate position uncertainty)
    let gps_var = args.gps_noise * args.gps_noise;
    let r_gps = vec![
        gps_var, 0.0, // GPS position noise
        0.0, gps_var,
    ];

    // Radar observation matrix (measures position and velocity)
    let h_radar = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // x
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // y
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // vx
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // vy
    ];

    // Radar measurement noise (better than GPS for position, includes velocity)
    let radar_pos_var = args.radar_noise * args.radar_noise;
    let radar_vel_var = (args.radar_noise * 1.5) * (args.radar_noise * 1.5);
    let r_radar = vec![
        radar_pos_var, 0.0, 0.0, 0.0, // Better position accuracy
        0.0, radar_pos_var, 0.0, 0.0,
        0.0, 0.0, radar_vel_var, 0.0, // Velocity measurements
        0.0, 0.0, 0.0, radar_vel_var,
    ];

    // IMU observation matrix (measures acceleration)
    let h_imu = vec![
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // ax
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, // ay
    ];

    // IMU measurement noise
    let imu_var = args.imu_noise * args.imu_noise;
    let r_imu = vec![
        imu_var, 0.0, // Accelerometer noise
        0.0, imu_var,
    ];

    // Build Kalman filter (we'll switch observation matrices as needed)
    let mut kf = KalmanFilter::<f64>::initialize(
        6,
        2, // 6 states, 2 measurements (will adjust for each sensor)
        x0,
        p0,
        f,
        q,
        h_gps.clone(), // Start with GPS
        r_gps.clone(),
    )
    .expect("Failed to build Kalman filter");

    // Simulation parameters
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let gps_noise_dist = Normal::new(0.0, args.gps_noise).unwrap();
    let radar_noise_dist = Normal::new(0.0, args.radar_noise).unwrap();
    let imu_noise_dist = Normal::new(0.0, args.imu_noise).unwrap();

    // True trajectory: circular motion with changing acceleration
    let omega = args.omega;
    let radius = args.radius;

    println!(
        "Time | True X | True Y | GPS X  | GPS Y  | Radar X| Radar Y| Est. X | Est. Y | Est. Vx| Est. Vy"
    );
    println!(
        "-----|--------|--------|--------|--------|--------|--------|--------|--------|--------|--------"
    );

    for step in 0..args.steps {
        let t = step as f64 * dt;

        // True state (circular motion)
        let true_x = radius * t.cos() * 0.1;
        let true_y = radius * t.sin() * 0.1;
        let true_vx = -radius * omega * t.sin() * 0.1;
        let true_vy = radius * omega * t.cos() * 0.1;
        let true_ax = -radius * omega * omega * t.cos() * 0.1;
        let true_ay = -radius * omega * omega * t.sin() * 0.1;

        // Predict step
        kf.predict();

        // Sensor measurements (alternating between sensors)
        match step % 3 {
            0 => {
                // GPS measurement (every 3rd step)
                let gps_x = true_x + gps_noise_dist.sample(&mut rng);
                let gps_y = true_y + gps_noise_dist.sample(&mut rng);

                // Update observation model for GPS
                kf.measurement_dim = 2;
                kf.H = h_gps.clone();
                kf.R = r_gps.clone();

                kf.update(&[gps_x, gps_y]).expect("GPS update failed");

                if step % args.print_interval == 0 {
                    print!(
                        "{:4.1} | {:6.2} | {:6.2} | {:6.2} | {:6.2} |",
                        t, true_x, true_y, gps_x, gps_y
                    );
                }
            }
            1 => {
                // Radar measurement
                let radar_x = true_x + radar_noise_dist.sample(&mut rng);
                let radar_y = true_y + radar_noise_dist.sample(&mut rng);
                let radar_vx = true_vx + radar_noise_dist.sample(&mut rng) * 1.5;
                let radar_vy = true_vy + radar_noise_dist.sample(&mut rng) * 1.5;

                // Update observation model for Radar
                kf.measurement_dim = 4;
                kf.H = h_radar.clone();
                kf.R = r_radar.clone();

                kf.update(&[radar_x, radar_y, radar_vx, radar_vy])
                    .expect("Radar update failed");

                if step % args.print_interval == 1 {
                    print!(" {:6.2} | {:6.2} |", radar_x, radar_y);
                }
            }
            _ => {
                // IMU measurement
                let imu_ax = true_ax + imu_noise_dist.sample(&mut rng);
                let imu_ay = true_ay + imu_noise_dist.sample(&mut rng);

                // Update observation model for IMU
                kf.measurement_dim = 2;
                kf.H = h_imu.clone();
                kf.R = r_imu.clone();

                kf.update(&[imu_ax, imu_ay]).expect("IMU update failed");
            }
        }

        // Print state estimate periodically
        if step % args.print_interval == 0 || step % args.print_interval == 1 {
            let state = kf.state();
            println!(
                " {:6.2} | {:6.2} | {:6.2} | {:6.2}",
                state[0], state[1], state[2], state[3]
            );
        }
    }

    println!("\nFinal state covariance (diagonal elements):");
    let p = kf.covariance();
    println!("X position variance:   {:.6}", p[0 * 6 + 0]);
    println!("Y position variance:   {:.6}", p[1 * 6 + 1]);
    println!("X velocity variance:   {:.6}", p[2 * 6 + 2]);
    println!("Y velocity variance:   {:.6}", p[3 * 6 + 3]);
    println!("X acceleration variance: {:.6}", p[4 * 6 + 4]);
    println!("Y acceleration variance: {:.6}", p[5 * 6 + 5]);

    println!("\nSensor fusion successfully combined GPS, Radar, and IMU measurements");
    println!("to provide optimal state estimation with different sensor characteristics.");
}

#[cfg(feature = "nalgebra")]
fn main() {
    let args = Args::parse();

    println!("Multi-Sensor Fusion Example (nalgebra-based)");
    println!("=============================================");
    println!("Fusing GPS, IMU, and Radar measurements\n");

    // Set up random seed
    let seed = args.seed.unwrap_or_else(|| {
        let seed_value = rand::random::<u64>();
        println!("Using random seed: {} (use --seed {} to reproduce)\n", seed_value, seed_value);
        seed_value
    });
    if args.seed.is_some() {
        println!("Using seed: {}\n", seed);
    }

    let dt = args.dt;

    // State transition matrix (constant acceleration model)
    let F = SMatrix::<f64, 6, 6>::from_row_slice(&[
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0,
        0.0, // x
        0.0,
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0, // y
        0.0,
        0.0,
        1.0,
        0.0,
        dt,
        0.0, // vx
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        dt, // vy
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0, // ax
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0, // ay
    ]);

    // Initial state from arguments
    let x0 = SVector::<f64, 6>::from([args.x0, args.y0, args.vx0, args.vy0, args.ax0, args.ay0]);

    // Initial covariance
    let P0 = SMatrix::<f64, 6, 6>::identity();

    // Process noise
    let q_acc = args.q_acc;
    let mut Q = SMatrix::<f64, 6, 6>::zeros();
    Q[(0, 0)] = dt.powi(4) / 4.0 * q_acc;
    Q[(1, 1)] = dt.powi(4) / 4.0 * q_acc;
    Q[(2, 2)] = dt.powi(2) * q_acc;
    Q[(3, 3)] = dt.powi(2) * q_acc;
    Q[(4, 4)] = q_acc;
    Q[(5, 5)] = q_acc;

    // For nalgebra version, we'll create separate filters for each sensor type
    // since we can't dynamically change dimensions

    // GPS Kalman Filter (2 measurements)
    let H_gps = SMatrix::<f64, 2, 6>::from_row_slice(&[
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    ]);
    let gps_var = args.gps_noise * args.gps_noise;
    let R_gps = SMatrix::<f64, 2, 2>::from_row_slice(&[gps_var, 0.0, 0.0, gps_var]);

    let mut kf = StaticKalmanFilter::new(x0, P0, F, Q, H_gps, R_gps);

    // Simulation parameters
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let gps_noise_dist = Normal::new(0.0, args.gps_noise).unwrap();

    // True trajectory
    let omega = args.omega;
    let radius = args.radius;

    println!("Time | True X | True Y | GPS X  | GPS Y  | Est. X | Est. Y | Est. Vx| Est. Vy");
    println!("-----|--------|--------|--------|--------|--------|--------|--------|--------");

    for step in 0..args.steps {
        let t = step as f64 * dt;

        // True state
        let true_x = radius * t.cos() * 0.1;
        let true_y = radius * t.sin() * 0.1;

        // Predict
        kf.predict();

        // GPS measurement
        let gps_x = true_x + gps_noise_dist.sample(&mut rng);
        let gps_y = true_y + gps_noise_dist.sample(&mut rng);
        let measurement = SVector::<f64, 2>::from([gps_x, gps_y]);

        // Update
        kf.update(&measurement).expect("Update failed");

        // Print results
        if step % args.print_interval == 0 {
            let state = kf.state();
            println!(
                "{:4.1} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2}",
                t, true_x, true_y, gps_x, gps_y, state[0], state[1], state[2], state[3]
            );
        }
    }

    println!("\nFinal state covariance (diagonal elements):");
    let P = kf.covariance();
    println!("X position variance:   {:.6}", P[(0, 0)]);
    println!("Y position variance:   {:.6}", P[(1, 1)]);
    println!("X velocity variance:   {:.6}", P[(2, 2)]);
    println!("Y velocity variance:   {:.6}", P[(3, 3)]);
    println!("X acceleration variance: {:.6}", P[(4, 4)]);
    println!("Y acceleration variance: {:.6}", P[(5, 5)]);

    println!(
        "\nNote: nalgebra version simplified to GPS-only due to static dimension constraints."
    );
    println!(
        "Full multi-sensor fusion would require separate filter instances or dynamic matrices."
    );
}
