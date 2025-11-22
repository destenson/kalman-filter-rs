//! Particle Filter example: Robot localization with multimodal distributions
//!
//! This example demonstrates particle filter for robot localization where
//! the posterior can be multimodal (multiple possible locations).
//!
//! ## Usage
//! ```bash
//! # Run with default parameters
//! cargo run --example particle_filter_robot
//!
//! # Run with custom parameters
//! cargo run --example particle_filter_robot -- \
//!   --seed 42 \
//!   --steps 20 \
//!   --dt 1.0 \
//!   --num-particles 1000 \
//!   --init-mean-x 5.0 --init-mean-y 5.0 --init-mean-theta 0.0 \
//!   --init-std-x 3.0 --init-std-y 3.0 --init-std-theta 3.14159 \
//!   --process-noise-x 0.1 --process-noise-y 0.1 --process-noise-theta 0.05 \
//!   --meas-noise-std 0.5 \
//!   --forward-velocity 1.0 --angular-velocity 0.1 \
//!   --print-interval 1
//!
//! # Show all available options
//! cargo run --example particle_filter_robot -- --help
//! ```

use clap::Parser;
use kalman_filters::{ParticleFilter, ResamplingStrategy};
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};

/// Command-line arguments for the particle filter robot localization example
#[derive(Parser, Debug)]
#[command(name = "Particle Filter Robot Localization")]
#[command(about = "Demonstrates particle filter for robot localization with multimodal distributions", long_about = None)]
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

    /// Number of particles
    #[arg(long, default_value_t = 500)]
    num_particles: usize,

    /// Initial mean X position
    #[arg(long, default_value_t = 5.0)]
    init_mean_x: f64,

    /// Initial mean Y position
    #[arg(long, default_value_t = 5.0)]
    init_mean_y: f64,

    /// Initial mean theta (heading angle in radians)
    #[arg(long, default_value_t = 0.0)]
    init_mean_theta: f64,

    /// Initial standard deviation for X position
    #[arg(long, default_value_t = 3.0)]
    init_std_x: f64,

    /// Initial standard deviation for Y position
    #[arg(long, default_value_t = 3.0)]
    init_std_y: f64,

    /// Initial standard deviation for theta (heading angle)
    #[arg(long, default_value_t = 3.14159265358979323846)]
    init_std_theta: f64,

    /// Process noise standard deviation for X position
    #[arg(long, default_value_t = 0.1)]
    process_noise_x: f64,

    /// Process noise standard deviation for Y position
    #[arg(long, default_value_t = 0.1)]
    process_noise_y: f64,

    /// Process noise standard deviation for theta
    #[arg(long, default_value_t = 0.05)]
    process_noise_theta: f64,

    /// Measurement noise standard deviation
    #[arg(long, default_value_t = 0.5)]
    meas_noise_std: f64,

    /// Forward velocity (m/s)
    #[arg(long, default_value_t = 1.0)]
    forward_velocity: f64,

    /// Angular velocity (rad/s)
    #[arg(long, default_value_t = 0.1)]
    angular_velocity: f64,

    /// Landmark 1 X position
    #[arg(long, default_value_t = 2.0)]
    landmark1_x: f64,

    /// Landmark 1 Y position
    #[arg(long, default_value_t = 8.0)]
    landmark1_y: f64,

    /// Landmark 2 X position
    #[arg(long, default_value_t = 8.0)]
    landmark2_x: f64,

    /// Landmark 2 Y position
    #[arg(long, default_value_t = 8.0)]
    landmark2_y: f64,

    /// Landmark 3 X position
    #[arg(long, default_value_t = 8.0)]
    landmark3_x: f64,

    /// Landmark 3 Y position
    #[arg(long, default_value_t = 2.0)]
    landmark3_y: f64,

    /// Landmark 4 X position
    #[arg(long, default_value_t = 2.0)]
    landmark4_x: f64,

    /// Landmark 4 Y position
    #[arg(long, default_value_t = 2.0)]
    landmark4_y: f64,

    /// Print interval (show every Nth step, 0 means show all steps)
    #[arg(long, default_value_t = 1)]
    print_interval: usize,
}

fn main() {
    let args = Args::parse();

    println!("=== Particle Filter Robot Localization Example ===\n");

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

    // Robot state: [x, y, theta] (position and heading)
    let state_dim = 3;

    // Initial uncertainty (robot could be anywhere in a 10x10 area)
    let initial_mean = vec![args.init_mean_x, args.init_mean_y, args.init_mean_theta];
    let initial_std = vec![args.init_std_x, args.init_std_y, args.init_std_theta];

    // Process noise (motion uncertainty)
    let process_noise_std = vec![args.process_noise_x, args.process_noise_y, args.process_noise_theta];

    // Measurement noise
    let measurement_noise_std = vec![args.meas_noise_std, args.meas_noise_std];

    // Create particle filter
    let mut pf = ParticleFilter::new(
        state_dim,
        args.num_particles,
        initial_mean.clone(),
        initial_std.clone(),
        process_noise_std.clone(),
        measurement_noise_std.clone(),
        args.dt,
    )
    .unwrap();

    // Use systematic resampling for better performance
    pf.set_resampling_strategy(ResamplingStrategy::Systematic);

    // Known landmarks in the environment
    let landmarks = vec![
        (args.landmark1_x, args.landmark1_y),
        (args.landmark2_x, args.landmark2_y),
        (args.landmark3_x, args.landmark3_y),
        (args.landmark4_x, args.landmark4_y),
    ];

    println!("Configuration:");
    println!("  Time steps: {}", args.steps);
    println!("  Time step (dt): {:.3} s", args.dt);
    println!("  Number of particles: {}", args.num_particles);
    println!("  Initial mean: [{:.2}, {:.2}, {:.2}]", initial_mean[0], initial_mean[1], initial_mean[2]);
    println!("  Initial std dev: [{:.2}, {:.2}, {:.2}]", initial_std[0], initial_std[1], initial_std[2]);
    println!("  Process noise std: [{:.3}, {:.3}, {:.3}]", process_noise_std[0], process_noise_std[1], process_noise_std[2]);
    println!("  Measurement noise std: {:.3}", args.meas_noise_std);
    println!("  Forward velocity: {:.3} m/s", args.forward_velocity);
    println!("  Angular velocity: {:.3} rad/s", args.angular_velocity);
    println!("  Landmarks:");
    for (i, (x, y)) in landmarks.iter().enumerate() {
        println!("    L{}: ({:.1}, {:.1})", i + 1, x, y);
    }
    println!("  Print interval: {}", args.print_interval);
    println!();

    // Motion model: simple differential drive robot
    let v = args.forward_velocity;
    let omega = args.angular_velocity;
    let motion_model = move |state: &[f64], dt: f64| -> Vec<f64> {
        let x = state[0];
        let y = state[1];
        let theta = state[2];

        vec![
            x + v * theta.cos() * dt,
            y + v * theta.sin() * dt,
            theta + omega * dt,
        ]
    };

    // Likelihood function based on landmark observations
    let likelihood_fn = |state: &[f64], measurement: &[f64]| -> f64 {
        let x = state[0];
        let y = state[1];

        // Find closest landmark
        let observed_landmark_idx = measurement[0] as usize;
        if observed_landmark_idx >= landmarks.len() {
            return 1e-10;
        }

        let (lx, ly) = landmarks[observed_landmark_idx];
        let measured_dist = measurement[1];

        // Calculate expected distance
        let expected_dist = ((x - lx).powi(2) + (y - ly).powi(2)).sqrt();

        // Gaussian likelihood
        let diff = measured_dist - expected_dist;
        let sigma = 0.5;
        (-0.5 * diff * diff / (sigma * sigma)).exp()
    };

    println!("Time | Observed | Distance | Est. X | Est. Y | Est. θ | ESS");
    println!("-----|----------|----------|--------|--------|--------|------");

    // Create noise distribution for measurements
    let measurement_noise = Normal::new(0.0, args.meas_noise_std).unwrap();

    // Simulate robot motion and observations
    for t in 1..=args.steps {
        // Predict step (motion update)
        pf.predict(motion_model);

        // Simulate landmark observation
        let landmark_idx = t % landmarks.len();
        let (lx, ly) = landmarks[landmark_idx];

        // True robot position (for simulation)
        let best = pf.best_particle();
        let true_dist = ((best.state[0] - lx).powi(2) + (best.state[1] - ly).powi(2)).sqrt();

        // Observed distance with random noise
        let observed_dist = true_dist + measurement_noise.sample(&mut rng);

        // Update step with measurement
        let measurement = vec![landmark_idx as f64, observed_dist];
        pf.update(&measurement, likelihood_fn).unwrap();

        // Get state estimate (weighted mean)
        let state = pf.mean();
        let ess = pf.effective_sample_size();

        // Print results at specified intervals
        if args.print_interval > 0 && t % args.print_interval == 0 {
            println!(
                "{:4} | L{:7} | {:8.2} | {:6.2} | {:6.2} | {:6.2} | {:4.0}",
                t,
                landmark_idx + 1,
                observed_dist,
                state[0],
                state[1],
                state[2],
                ess
            );
        }
    }

    // Final estimate
    let final_state = pf.mean();
    let best_particle = pf.best_particle();

    println!("\nFinal estimates:");
    println!(
        "  Weighted mean: x={:.2}, y={:.2}, θ={:.2}",
        final_state[0], final_state[1], final_state[2]
    );
    println!(
        "  Best particle: x={:.2}, y={:.2}, θ={:.2} (weight={:.3})",
        best_particle.state[0],
        best_particle.state[1],
        best_particle.state[2],
        best_particle.weight
    );

    // Check for multimodality
    let covariance = pf.covariance();
    let position_variance = covariance[0] + covariance[4]; // σ²_x + σ²_y

    if position_variance > 2.0 {
        println!("\nHigh variance detected - possible multimodal distribution");
        println!("  The robot might be uncertain between multiple locations");
    } else {
        println!("\nRobot successfully localized with particle filter");
    }

    println!("  Handled non-Gaussian, potentially multimodal distributions");
}
