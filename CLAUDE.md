# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Codebase Overview

This is a comprehensive Kalman filter library (v1.0.0-alpha0) with 7 filter variants, extracted from the GAI ecosystem to be a standalone crate. The library implements state-of-the-art estimation algorithms with a focus on numerical stability and performance.

## Build and Test Commands

```bash
# Basic build
cargo build

# Build with all features
cargo build --all-features

# Run all tests
cargo test

# Run specific test suite
cargo test --test information_tests

# Run a single test
cargo test test_kf_if_equivalence

# Run examples
cargo run --example simple_1d
cargo run --example if_sensor_network
cargo run --example legacy --features legacy

# Run benchmarks (when implemented)
cargo bench --bench kalman_benchmarks

# Check code without building
cargo check --all-features

# Format code
cargo fmt

# Lint with clippy
cargo clippy --all-features -- -D warnings

# Generate documentation
cargo doc --all-features --open

# Build for release
cargo build --release
```

## Architecture

### Filter Hierarchy
The crate implements 7 filter variants, each in its own module:
- `filter.rs` - Standard linear Kalman Filter (KF) with dynamic dimensions
- `extended.rs` - Extended Kalman Filter (EKF) for nonlinear systems via linearization
- `unscented.rs` - Unscented Kalman Filter (UKF) using sigma points
- `information/` - Information Filter (IF) dual form for sparse measurements and distributed fusion
- `ensemble/` - Ensemble Kalman Filter (EnKF) using Monte Carlo
- `particle/` - Particle Filter (PF) for non-Gaussian distributions
- `scented.rs` - Cubature Kalman Filter (CKF) for high dimensions

### Core Abstractions
- `KalmanScalar` trait - Allows f32/f64 generic programming
- `NonlinearSystem` trait - Interface for nonlinear dynamics in EKF/UKF
- `KalmanResult<T>` / `KalmanError` - Consistent error handling
- Builder pattern in `builder.rs` for ergonomic filter construction

### Feature Architecture
Features control optional dependencies and capabilities:
- `std` (default) - Standard library support
- `nalgebra` - Static matrix support for compile-time dimension checking
- `legacy` - Backward compatibility with original kalman-filter API
- `parallel` - Parallel processing via rayon
- `adskalman` - Comparison with reference implementation
- `opencv` - OpenCV integration for computer vision

### Information Filter Architecture
The Information Filter (`information/` module) is the most complex subsystem:
- `filter.rs` - Core IF with Y=P^-1 representation
- `sparse.rs` - Sparse matrix operations for high-dimensional systems
- `distributed.rs` - Multi-node sensor networks with message passing
- `consensus.rs` - Average/weighted consensus algorithms
- `conversion.rs` - Bidirectional KF↔IF conversion
- `extended.rs` - Nonlinear Extended Information Filter

### Matrix Storage Convention
All matrices are stored in row-major order as flat `Vec<T>`:
- A 2x2 matrix [[a,b],[c,d]] is stored as vec![a,b,c,d]
- Matrix multiplication and inversion in `filter.rs` follow this convention
- When using nalgebra feature, automatic conversion happens

## Current State and PRPs

The codebase has 5 Project Requirements and Planning documents (PRPs) in `PRPs/`:
1. **01-comprehensive-documentation.md** - Add missing documentation
2. **02-performance-benchmarks.md** - Implement criterion benchmarks
3. **03-algorithm-validation.md** - Mathematical correctness testing
4. **04-state-of-art-review.md** - Competitive analysis
5. **05-cross-validation-testing.md** - Validate against FilterPy/PyKalman

Current issues from codebase review (`codebase-review-report.md`):
- Empty README.md (1 line)
- Empty benchmark file (`benches/kalman_benchmarks.rs`)
- 155 `.unwrap()` calls in non-test code
- No CI/CD pipeline

## Key Implementation Details

### Error Handling Pattern
The codebase uses `KalmanResult<T>` throughout but currently has many `.unwrap()` calls. When modifying, prefer proper error propagation:
```rust
// Avoid
let result = matrix_inverse(&m).unwrap();

// Prefer
let result = matrix_inverse(&m)?;
```

### Testing Approach
- Unit tests are inline with `#[cfg(test)]` modules
- Integration tests in `tests/` directory
- Examples serve as both documentation and integration tests
- Property-based testing planned via quickcheck (PRP-03)

### Numerical Stability
- The Information Filter uses Joseph form for covariance updates
- Matrix inversion checks for singularity with epsilon = 1e-10
- Some filters have `#![allow(unused, non_snake_case)]` - DO NOT CHANGE these

### Performance Considerations
- Matrix operations dominate runtime, especially inversion
- Information Filter has sparse matrix support for efficiency
- Particle Filter performance scales with particle count
- When optimizing, avoid unnecessary clones of state vectors

## Important Notes

1. **Legacy Compatibility**: The `legacy` feature provides backward compatibility with the original kalman-filter crate API. When enabled, it changes the default `KalmanFilter` export.

2. **No GAI Dependencies**: This crate was extracted from gai-rs-kalman and has zero dependencies on the GAI ecosystem.

3. **Validation Status**: The implementation has been tested but needs cross-validation against reference implementations (see PRP-05).

4. **Documentation Priority**: The README is essentially empty - this is the highest priority issue blocking publication to crates.io.
