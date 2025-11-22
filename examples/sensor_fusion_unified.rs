//! Sensor fusion example with unified API
//!
//! This example demonstrates fusing data from multiple sensors with different
//! characteristics using the backend-agnostic unified API.
//! We simulate:
//! - GPS sensor: Provides position measurements with moderate noise
//! - IMU accelerometer: Provides acceleration measurements
//! - Radar: Provides position and velocity with different noise characteristics
//!
//! The Kalman filter combines these measurements optimally based on their
//! respective noise characteristics.
//!
//! ## Usage
//! ```bash
//! # Run with Vec backend (default)
//! cargo run --example sensor_fusion_unified
//!
//! # Run with nalgebra backend (falls back to Vec for dynamic sensor switching)
//! cargo run --example sensor_fusion_unified --features nalgebra
//!
//! # Run with a specific seed for reproducible results
//! cargo run --example sensor_fusion_unified -- --seed 42
//!
//! # Run with custom parameters
//! cargo run --example sensor_fusion_unified -- --steps 200 --dt 0.05 --omega 0.3
//!
//! # Customize noise levels
//! cargo run --example sensor_fusion_unified -- --gps-noise 1.0 --radar-noise 0.3 --imu-noise 0.1
//! ```

use clap::Parser;
use kalman_filters::{KalmanFilterBuilder, KalmanFilterTrait};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

/// Multi-sensor fusion Kalman filter example
///
/// Fuses GPS, IMU (accelerometer), and Radar measurements to track a moving object
/// following a circular trajectory. Each sensor has different noise characteristics:
/// - GPS: Moderate position noise (~0.5m)
/// - Radar: Better position accuracy (~0.2m), includes velocity
/// - IMU: High-frequency acceleration measurements (~0.05m/s²)
#[derive(Parser, Debug)]
#[command(name = "Multi-Sensor Fusion (Unified API)")]
#[command(about = "Backend-agnostic sensor fusion example")]
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

fn main() {
    let args = Args::parse();

    println!("=== Multi-Sensor Fusion (Unified API) ===");
    println!("Fusing GPS, IMU, and Radar measurements\n");

    // Set up random seed
    let seed = args.seed.unwrap_or_else(|| {
        let s = rand::random::<u64>();
        println!("Using random seed: {} (use --seed {} to reproduce)\n", s, s);
        s
    });
    if args.seed.is_some() {
        println!("Using seed: {}\n", seed);
    }

    let dt = args.dt;

    // State vector: [x, y, vx, vy, ax, ay] - position, velocity, acceleration
    // State transition matrix (constant acceleration model)
    let f = vec![
        1.0, 0.0, dt, 0.0, dt * dt / 2.0, 0.0,                // x
        0.0, 1.0, 0.0, dt, 0.0, dt * dt / 2.0,                // y
        0.0, 0.0, 1.0, 0.0, dt, 0.0,                          // vx
        0.0, 0.0, 0.0, 1.0, 0.0, dt,                          // vy
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,                         // ax
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,                         // ay
    ];

    // Initial state from arguments
    let x0 = vec![args.x0, args.y0, args.vx0, args.vy0, args.ax0, args.ay0];

    // Initial covariance
    let p0 = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    // Process noise (uncertainty in acceleration)
    let q_acc = args.q_acc;
    let mut q = vec![0.0; 36];
    q[0 * 6 + 0] = dt.powi(4) / 4.0 * q_acc; // x-x
    q[1 * 6 + 1] = dt.powi(4) / 4.0 * q_acc; // y-y
    q[2 * 6 + 2] = dt.powi(2) * q_acc;       // vx-vx
    q[3 * 6 + 3] = dt.powi(2) * q_acc;       // vy-vy
    q[4 * 6 + 4] = q_acc;                    // ax-ax
    q[5 * 6 + 5] = q_acc;                    // ay-ay

    // GPS observation matrix (measures x, y position only)
    let h_gps = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // x
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // y
    ];

    // GPS measurement noise (moderate position uncertainty)
    let gps_var = args.gps_noise * args.gps_noise;
    let r_gps = vec![
        gps_var, 0.0,
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
        radar_pos_var, 0.0, 0.0, 0.0,
        0.0, radar_pos_var, 0.0, 0.0,
        0.0, 0.0, radar_vel_var, 0.0,
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
        imu_var, 0.0,
        0.0, imu_var,
    ];

    // Build Kalman filter - works identically regardless of backend!
    // Start with GPS configuration (2 measurements)
    let mut kf = KalmanFilterBuilder::new(6, 2)
        .initial_state(x0)
        .initial_covariance(p0)
        .transition_matrix(f)
        .process_noise(q)
        .observation_matrix(h_gps.clone())
        .measurement_noise(r_gps.clone())
        .build()
        .unwrap();

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

        // Predict step - same API regardless of backend!
        kf.predict();

        // Sensor measurements (alternating between sensors)
        // We switch observation matrices using the unified API
        match step % 3 {
            0 => {
                // GPS measurement (every 3rd step)
                let gps_x = true_x + gps_noise_dist.sample(&mut rng);
                let gps_y = true_y + gps_noise_dist.sample(&mut rng);

                // Update observation model for GPS using unified API
                kf.set_observation_matrix(&h_gps).unwrap();
                kf.set_measurement_noise(&r_gps).unwrap();

                kf.update(&[gps_x, gps_y]).unwrap();

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

                // Update observation model for Radar using unified API
                kf.set_observation_matrix(&h_radar).unwrap();
                kf.set_measurement_noise(&r_radar).unwrap();

                kf.update(&[radar_x, radar_y, radar_vx, radar_vy]).unwrap();

                if step % args.print_interval == 1 {
                    print!(" {:6.2} | {:6.2} |", radar_x, radar_y);
                }
            }
            _ => {
                // IMU measurement
                let imu_ax = true_ax + imu_noise_dist.sample(&mut rng);
                let imu_ay = true_ay + imu_noise_dist.sample(&mut rng);

                // Update observation model for IMU using unified API
                kf.set_observation_matrix(&h_imu).unwrap();
                kf.set_measurement_noise(&r_imu).unwrap();

                kf.update(&[imu_ax, imu_ay]).unwrap();
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
    println!("  X position variance:       {:.6}", p[0 * 6 + 0]);
    println!("  Y position variance:       {:.6}", p[1 * 6 + 1]);
    println!("  X velocity variance:       {:.6}", p[2 * 6 + 2]);
    println!("  Y velocity variance:       {:.6}", p[3 * 6 + 3]);
    println!("  X acceleration variance:   {:.6}", p[4 * 6 + 4]);
    println!("  Y acceleration variance:   {:.6}", p[5 * 6 + 5]);

    #[cfg(feature = "nalgebra")]
    println!("\n✓ Using Vec backend (required for dynamic sensor switching)");

    #[cfg(not(feature = "nalgebra"))]
    println!("\n✓ Using Vec backend (runtime flexible)");

    println!("\nThis code works identically regardless of backend!");
    println!("Sensor fusion successfully combined GPS, Radar, and IMU measurements");
    println!("to provide optimal state estimation with different sensor characteristics.");
}
