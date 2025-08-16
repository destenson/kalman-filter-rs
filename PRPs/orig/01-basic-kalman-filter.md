# PRP: Basic Kalman Filter Implementation

## Overview
Implement a working linear Kalman filter in the gai-rs-kalman crate with comprehensive API, examples, and tests. The current crate is non-functional with all implementations commented out or stubbed with `todo!()`.

## Context and Research

### Current State
- **Location**: `gai-rs/crates/gai-rs-kalman/`
- **Status**: Empty shell with no working functionality
- **Dependencies**: Multiple Kalman libraries added (kalman-rust, kalmanfilt, adskalman, minikalman) but none integrated
- **Compilation**: Fails due to nalgebra import issues when feature disabled

### Reference Implementations
- **adskalman-rs**: https://github.com/strawlab/adskalman-rs - Well-structured with static typing
- **kalmanrs**: https://github.com/nravic/kalmanrs - Generic dimension support
- **Dom Wilson's Tutorial**: https://domwil.co.uk/posts/kalman/ - Clear Rust implementation guide

### Workspace Patterns to Follow
- **Math Matrix Module**: `gai-rs/crates/gai-rs-math-matrices/src/lib.rs` - Shows matrix abstraction patterns
- **Test Pattern**: Inline tests using `#[test]` attribute within module files
- **Example Pattern**: Simple examples in `examples/` directory demonstrating key use cases

## Requirements

### Core Functionality
1. **Basic Linear Kalman Filter**
   - State prediction (predict step)
   - Measurement update (update/correct step)
   - Configurable process and measurement noise

2. **Matrix Types**
   - Use nalgebra's `SMatrix` for compile-time dimension checking
   - Support both f32 and f64 precision
   - Generic over state dimension N and measurement dimension M

3. **API Design**
   - Clear separation between predict and update phases
   - Builder pattern for filter initialization
   - Support for both static and dynamic dimensions

### File Structure
```
src/
├── lib.rs          # Public API and re-exports
├── filter.rs       # Core KalmanFilter implementation
├── builder.rs      # Builder pattern for filter construction
├── types.rs        # Type aliases and matrix definitions
└── examples.rs     # Module with inline doc examples

examples/
├── simple_1d.rs    # 1D position tracking
├── position_2d.rs  # 2D position/velocity tracking
└── sensor_fusion.rs # Multiple sensor fusion example
```

## Implementation Blueprint

### Phase 1: Fix Compilation Issues
1. Fix nalgebra imports in `src/lib.rs` and `src/filter.rs`
2. Ensure proper feature gating for nalgebra dependency
3. Clean up commented code to start fresh

### Phase 2: Core Implementation
1. Define types in `src/types.rs`:
   - StateVector, CovarianceMatrix, TransitionMatrix, etc.
   - Use nalgebra::SMatrix with const generics

2. Implement KalmanFilter struct in `src/filter.rs`:
   - Fields: state, covariance, F, Q, H, R matrices
   - Methods: predict(), update(), get_state(), get_covariance()

3. Create builder in `src/builder.rs`:
   - Initialize with dimensions
   - Set matrices with validation
   - Build method returns Result<KalmanFilter>

### Phase 3: Examples and Tests
1. Create three working examples demonstrating:
   - Simple 1D tracking (constant velocity model)
   - 2D position tracking (x,y coordinates with velocity)
   - Sensor fusion (combining multiple noisy measurements)

2. Add comprehensive inline tests:
   - Unit tests for predict/update cycles
   - Property tests for matrix dimensions
   - Integration tests with known scenarios

### Phase 4: Documentation
1. Add module-level documentation explaining Kalman filter theory
2. Document all public APIs with examples
3. Create README.md with quick start guide

## Implementation Tasks (in order)

1. Fix nalgebra import issues in lib.rs and filter.rs
2. Remove all commented code and start fresh
3. Define core types using nalgebra::SMatrix
4. Implement basic KalmanFilter struct with predict/update
5. Add builder pattern for filter construction
6. Create simple_1d example with constant velocity model
7. Add inline unit tests for core functionality
8. Implement 2D position tracking example
9. Add sensor fusion example
10. Write comprehensive documentation
11. Add property-based tests for matrix operations
12. Benchmark against existing Kalman libraries

## Validation Gates

```bash
# Build with default features
cargo build

# Build with all features
cargo build --all-features

# Run tests
cargo test --all-features

# Check formatting and linting
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings

# Run examples
cargo run --example simple_1d
cargo run --example position_2d
cargo run --example sensor_fusion

# Generate documentation
cargo doc --no-deps --open

# Benchmark (if implemented)
cargo bench
```

## Error Handling Strategy
- Use `Result` types for fallible operations (matrix inversions, builder validation)
- Custom error enum for Kalman-specific errors (dimension mismatch, singular matrix)
- Clear error messages indicating the problem and solution

## Success Criteria
- All tests passing (100% of new tests)
- Three working examples demonstrating practical use cases
- Clean compilation with both default and all features
- Documentation with mathematical background and usage examples
- Performance within 20% of adskalman benchmark

## References for Implementation
- Kalman Filter Theory: https://www.kalmanfilter.net/
- nalgebra Documentation: https://docs.rs/nalgebra/latest/nalgebra/
- adskalman Source: https://github.com/strawlab/adskalman-rs/blob/main/src/lib.rs
- Existing workspace patterns: `gai-rs/crates/gai-rs-math-matrices/`

## Notes for AI Agent
- Start by fixing the compilation issues before adding new functionality
- Use the existing adskalman dependency as reference but implement from scratch
- Follow the workspace's testing pattern with inline `#[test]` functions
- Keep the API simple initially - avoid over-engineering
- Focus on getting a working predict/update cycle before adding advanced features

## Quality Score: 8/10
Strong context with specific implementation steps, clear validation gates, and references to existing patterns. Deducted points for not having specific numerical test cases ready, but the AI agent can generate those during implementation.