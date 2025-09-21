# kalman_filters

[![Crates.io](https://img.shields.io/crates/v/kalman_filters.svg)](https://crates.io/crates/kalman_filters)
[![Documentation](https://docs.rs/kalman_filters/badge.svg)](https://docs.rs/kalman_filters)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive Kalman filter library in Rust, providing multiple filter variants for state estimation in noisy systems.

## Features

- **7 Filter Variants**: From basic linear to advanced non-linear and distributed filters
- **Generic Scalar Types**: Support for both `f32` and `f64` precision
- **Builder Pattern**: Ergonomic filter construction with compile-time validation
- **Numerical Stability**: Joseph form covariance updates and symmetry enforcement
- **Logging Support**: Built-in diagnostics via the `log` crate
- **Optional Features**: Static matrices via `nalgebra`, parallel processing, and more

## Filter Variants

### Linear Filters
- **Kalman Filter (KF)**: Standard linear filter for systems with Gaussian noise
- **Information Filter (IF)**: Dual form using information matrix (inverse covariance)

### Non-linear Filters
- **Extended Kalman Filter (EKF)**: Linearization via Jacobians for non-linear systems
- **Unscented Kalman Filter (UKF)**: Sigma point sampling for better non-linear handling
- **Cubature Kalman Filter (CKF)**: High-dimensional non-linear estimation

### Advanced Filters
- **Ensemble Kalman Filter (EnKF)**: Monte Carlo approach with ensemble members
- **Particle Filter (PF)**: Sequential Monte Carlo for non-Gaussian distributions

## Quick Start

Add to your `Cargo.toml`:

```toml
[dependencies]
kalman_filters = "1.0.0"
```

### Basic Example

```rust
use kalman_filters::KalmanFilterBuilder;

// Create a 2D position/velocity tracker
let mut kf = KalmanFilterBuilder::new(2, 1)  // 2 states, 1 measurement
    .initial_state(vec![0.0, 0.0])            // position, velocity
    .initial_covariance(vec![
        1.0, 0.0,
        0.0, 1.0,
    ])
    .transition_matrix(vec![
        1.0, 0.1,  // position += velocity * dt
        0.0, 1.0,  // velocity unchanged
    ])
    .process_noise(vec![
        0.001, 0.0,
        0.0,   0.001,
    ])
    .observation_matrix(vec![1.0, 0.0])       // measure position only
    .measurement_noise(vec![0.1])
    .build()
    .unwrap();

// Prediction step
kf.predict();

// Update with measurement
kf.update(&[1.5]).unwrap();

let state = kf.state();
println!("Position: {:.2}, Velocity: {:.2}", state[0], state[1]);
```

### Non-linear Example (EKF)

```rust
use kalman_filters::{ExtendedKalmanFilter, NonlinearSystem};

// Define your non-linear system
struct PendulumSystem {
    g: f64,  // gravity
    l: f64,  // length
}

impl NonlinearSystem<f64> for PendulumSystem {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let theta = state[0];
        let theta_dot = state[1];
        vec![
            theta + theta_dot * dt,
            theta_dot - (self.g / self.l) * theta.sin() * dt,
        ]
    }
    
    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        vec![state[0]]  // measure angle only
    }
    
    fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let theta = state[0];
        vec![
            1.0, dt,
            -(self.g / self.l) * theta.cos() * dt, 1.0,
        ]
    }
    
    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        vec![1.0, 0.0]
    }
    
    fn state_dim(&self) -> usize { 2 }
    fn measurement_dim(&self) -> usize { 1 }
}

// Use the EKF
let system = PendulumSystem { g: 9.81, l: 1.0 };
let mut ekf = ExtendedKalmanFilter::new(
    system,
    vec![0.1, 0.0],        // initial state
    vec![0.01, 0.0, 0.0, 0.01],  // initial covariance
    vec![0.001, 0.0, 0.0, 0.001], // process noise
    vec![0.01],            // measurement noise
    0.01,                  // dt
).unwrap();

ekf.predict();
ekf.update(&[0.09]).unwrap();
```

## Optional Features

Enable features in your `Cargo.toml`:

```toml
[dependencies]
kalman_filters = { version = "1.0.0", features = ["nalgebra", "parallel"] }
```

| Feature | Description |
|---------|-------------|
| `std` | Standard library support (default) |
| `nalgebra` | Static matrix support with compile-time dimensions |
| `parallel` | Parallel processing via `rayon` for ensemble/particle filters |
| `serde` | Serialization support |
| `legacy` | Backward compatibility with `kalman-filter` v0.1.x |
| `adskalman` | Comparison with reference implementation |
| `opencv` | OpenCV integration for computer vision |
| `tracing-subscriber` | Enhanced logging for examples |

## Advanced Usage

### Information Filter for Sensor Networks

```rust
use kalman_filters::information::{InformationFilter, DistributedFilter};

// Create distributed filter for sensor network
let mut network = DistributedFilter::new(4, 2);  // 4 states, 2 measurements

// Add sensor nodes
network.add_node(0, information_filter_1);
network.add_node(1, information_filter_2);

// Connect nodes
network.add_edge(0, 1, 0.5);  // nodes 0 and 1 communicate

// Run consensus algorithm
network.consensus_update(10, 1e-6).unwrap();
```

### Particle Filter for Non-Gaussian Systems

```rust
use kalman_filters::{ParticleFilter, ResamplingStrategy};

let mut pf = ParticleFilter::new(
    state_dim: 2,
    measurement_dim: 1,
    num_particles: 1000,
    process_noise: vec![0.1, 0.1],
    measurement_noise: vec![0.01],
    resampling_strategy: ResamplingStrategy::Systematic,
).unwrap();

// Predict with custom dynamics
pf.predict_with(|state, _control| {
    // Your non-linear dynamics here
    vec![state[0] + 0.1, state[1] * 0.9]
});

// Update with measurement
pf.update(&[1.0], |state| {
    // Your measurement model here
    vec![state[0]]
}).unwrap();
```

## Logging and Diagnostics

The library uses the `log` crate for diagnostics. Enable logging to see filter operations:

```rust
use log::info;
env_logger::init();

// Your filter operations will now log diagnostic information
info!("Starting Kalman filter estimation");
```

Set log level via environment variable:
```bash
RUST_LOG=debug cargo run
```

## Performance

The library is optimized for:
- **Cache efficiency**: Row-major matrix storage
- **Numerical stability**: Joseph form updates, condition checking
- **Zero-cost abstractions**: Generic programming with monomorphization
- **Optional parallelism**: Multi-threading for ensemble/particle filters

## Examples

Run the examples:

```bash
# Simple 1D tracking
cargo run --example simple_1d

# Sensor network with Information Filter
cargo run --example if_sensor_network

# Logging demonstration
RUST_LOG=debug cargo run --example logging_demo
```

## Documentation

Full API documentation is available at [docs.rs/kalman_filters](https://docs.rs/kalman_filters).

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

This library was originally created in a weekend with Claude Code. Special thanks to all contributors and the Rust scientific computing community.
