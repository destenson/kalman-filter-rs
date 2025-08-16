# PRP: Extended Kalman Filter (EKF) Implementation

## Goal
Implement a comprehensive Extended Kalman Filter (EKF) for nonlinear state estimation, supporting both additive and non-additive noise models with user-provided Jacobian functions or automatic differentiation options.

## Why
- **Nonlinear Systems**: Many real-world systems are inherently nonlinear (robotics, aerospace, finance)
- **Industry Standard**: EKF is the de facto standard in nonlinear state estimation for navigation systems and GPS
- **Foundation for Advanced Applications**: Required for SLAM, sensor fusion, and autonomous vehicle applications
- **Complements Linear KF**: Extends the existing linear Kalman filter to handle a broader class of problems

## What
Create an Extended Kalman Filter that linearizes nonlinear models around current estimates using Jacobian matrices, with support for both continuous and discrete time models.

### Success Criteria
- [ ] EKF handles arbitrary nonlinear state transition and measurement functions
- [ ] User can provide analytical Jacobians or use numerical approximation
- [ ] Performance within 20% of linear KF for nearly-linear systems
- [ ] Examples demonstrate robot localization and GPS/IMU fusion
- [ ] Comprehensive tests for convergence and stability
- [ ] Documentation explains linearization theory and practical usage

## All Needed Context

### Documentation & References
```yaml
- url: https://simondlevy.github.io/ekf-tutorial/
  why: Interactive EKF tutorial with clear mathematical explanations and examples
  
- url: https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
  why: Comparison of EKF vs UKF, understanding EKF limitations
  
- url: https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter
  why: Object tracking implementation showing radar/lidar fusion with Jacobians
  
- url: https://medium.com/@opinoquintana/i-wrote-an-extended-kalman-filter-for-uav-attitude-estimation-from-scratch-in-rust-b8748ff33b12
  why: Rust EKF implementation for UAV with quaternion-based attitude estimation
  
- url: https://domwil.co.uk/posts/kalman/
  why: Rust Kalman implementation showing nonlinear system traits and Jacobian handling
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\filter.rs
  why: Linear KF implementation to extend - reuse matrix operations and structure
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\extended.rs
  why: Existing stub file to implement in
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\types.rs
  why: Error types and traits to extend for EKF-specific needs
```

### Suggested Implementation Structure
```bash
gai-rs-kalman/
├── src/
│   ├── extended.rs         # Main EKF implementation
│   ├── extended/
│   │   ├── mod.rs          # Module exports
│   │   ├── jacobian.rs     # Jacobian computation strategies
│   │   ├── continuous.rs   # Continuous-time EKF
│   │   └── discrete.rs     # Discrete-time EKF
│   └── types.rs            # Extended with EKF types
├── examples/
│   ├── ekf_robot_localization.rs
│   ├── ekf_radar_tracking.rs
│   └── ekf_battery_nonlinear.rs
```

### Known Gotchas & Library Quirks
- CRITICAL: Jacobian must be evaluated at predicted state, not current state
- CRITICAL: Poor linearization can cause filter divergence
- CRITICAL: Numerical Jacobian computation needs appropriate step size (typically sqrt(machine_epsilon))
- CRITICAL: Matrix condition number affects stability - monitor covariance eigenvalues
- CRITICAL: EKF assumes local linearity - fails for highly nonlinear systems
- CRITICAL: Process noise may need tuning differently than linear KF

## Implementation Blueprint

### Core Design Concepts

**Nonlinear System Trait**:
- State transition function: x_k+1 = f(x_k, u_k) + w_k
- Measurement function: z_k = h(x_k) + v_k
- State transition Jacobian: F = ∂f/∂x evaluated at x̂
- Measurement Jacobian: H = ∂h/∂x evaluated at x̂
- Optional control input Jacobian: B = ∂f/∂u

**Jacobian Computation Strategies**:
1. Analytical: User provides Jacobian functions
2. Numerical: Finite differences approximation
3. Automatic differentiation: Future enhancement

**EKF Algorithm Steps**:
1. Prediction:
   - State: x̂_k|k-1 = f(x̂_k-1|k-1, u_k)
   - Jacobian: F_k = ∂f/∂x|x̂_k-1|k-1
   - Covariance: P_k|k-1 = F_k P_k-1|k-1 F_k^T + Q_k

2. Update:
   - Innovation: y_k = z_k - h(x̂_k|k-1)
   - Jacobian: H_k = ∂h/∂x|x̂_k|k-1
   - Innovation covariance: S_k = H_k P_k|k-1 H_k^T + R_k
   - Kalman gain: K_k = P_k|k-1 H_k^T S_k^-1
   - State update: x̂_k|k = x̂_k|k-1 + K_k y_k
   - Covariance update: P_k|k = (I - K_k H_k) P_k|k-1

### Task List

```yaml
Task 1: Define nonlinear system traits
MODIFY src/types.rs:
  - Add NonlinearSystem trait with state_transition and measurement functions
  - Add JacobianProvider trait for different Jacobian strategies
  - Add EKF-specific error types (JacobianComputation, Divergence)

Task 2: Implement core EKF structure
MODIFY src/extended.rs:
  - Define ExtendedKalmanFilter struct with generic parameters
  - Add fields for nonlinear functions and Jacobian providers
  - Implement initialization with dimension validation

Task 3: Create Jacobian computation module
CREATE src/extended/jacobian.rs:
  - Implement analytical Jacobian provider
  - Implement numerical Jacobian via finite differences
  - Add Jacobian validation and conditioning checks

Task 4: Implement discrete-time EKF
CREATE src/extended/discrete.rs:
  - Implement predict() with nonlinear propagation and Jacobian
  - Implement update() with nonlinear measurement and Jacobian
  - Add stability monitoring (covariance trace, eigenvalues)

Task 5: Implement continuous-time EKF
CREATE src/extended/continuous.rs:
  - Continuous state propagation via RK4 or similar
  - Continuous Riccati equation for covariance
  - Discretization methods for mixed continuous/discrete

Task 6: Create EKF builder
MODIFY src/builder.rs:
  - Extend builder for EKF-specific parameters
  - Add nonlinear function setters
  - Add Jacobian strategy selection

Task 7: Implement robot localization example
CREATE examples/ekf_robot_localization.rs:
  - 2D robot with nonlinear motion model
  - Landmark measurements with bearing/range
  - Compare with ground truth trajectory

Task 8: Implement radar tracking example
CREATE examples/ekf_radar_tracking.rs:
  - Cartesian state, polar measurements
  - Coordinate transformation nonlinearity
  - Track maneuvering target

Task 9: Add comprehensive tests
CREATE tests/ekf_tests.rs:
  - Test linearization accuracy
  - Test convergence for weakly nonlinear systems
  - Test numerical vs analytical Jacobians
  - Test filter consistency (NEES/NIS)

Task 10: Create documentation
MODIFY src/extended.rs:
  - Add module documentation with theory
  - Document when to use EKF vs UKF
  - Include convergence criteria and tuning guide
```

### Integration Points
```yaml
TYPES:
  - Extend KalmanScalar trait for Jacobian operations
  - Add convergence monitoring types
  - Define nonlinear function signatures

BUILDER:
  - Support both linear and nonlinear filter construction
  - Validate Jacobian dimensions match state/measurement
  - Default to numerical Jacobian if not provided

EXAMPLES:
  - Show migration from linear to extended filter
  - Demonstrate performance vs computation tradeoff
  - Include real-world sensor models
```

## Validation Loop

### Level 1: Syntax & Style
```bash
# Build with EKF implementation
cargo build --features extended

# Check all code quality
cargo clippy --all-features -- -D warnings
cargo fmt --check
```

### Level 2: Unit Tests
Test cases to implement:
- Verify EKF reduces to linear KF for linear systems
- Test Jacobian computation accuracy (analytical vs numerical)
- Test prediction step with nonlinear dynamics
- Test update step with nonlinear measurements
- Test filter consistency via normalized estimation error squared (NEES)
- Test innovation consistency via normalized innovation squared (NIS)

### Level 3: Integration Tests
```bash
# Run EKF examples
cargo run --example ekf_robot_localization
cargo run --example ekf_radar_tracking
cargo run --example ekf_battery_nonlinear

# Compare performance
cargo bench --features extended
```

### Level 4: Convergence Tests
Validation metrics:
- Root mean square error (RMSE) vs ground truth
- Consistency: χ² test on NEES/NIS
- Cramer-Rao lower bound comparison
- Observability/controllability analysis

## Final Validation Checklist
- [ ] EKF implementation compiles without warnings
- [ ] All unit tests pass including edge cases
- [ ] Examples produce expected tracking performance
- [ ] Numerical Jacobian matches analytical within tolerance
- [ ] Filter remains stable for 1000+ iterations
- [ ] Documentation includes mathematical derivation
- [ ] Performance benchmarks vs linear KF documented

## Anti-Patterns to Avoid
- ❌ Don't evaluate Jacobian at wrong point (use predicted state)
- ❌ Don't ignore numerical issues in matrix operations
- ❌ Don't use fixed step size for all numerical derivatives
- ❌ Don't assume EKF will work for all nonlinear systems
- ❌ Don't neglect consistency checks (NEES/NIS)
- ❌ Don't mix continuous and discrete time incorrectly

## Error Handling Strategy
- Detect divergence via covariance trace monitoring
- Provide fallback to numerical Jacobian if analytical fails
- Check matrix conditioning before inversion
- Validate nonlinear function outputs are finite
- Reset filter if covariance becomes non-positive definite

## References for Implementation
- EKF Tutorial: https://simondlevy.github.io/ekf-tutorial/
- Optimal State Estimation (Dan Simon): Comprehensive EKF theory
- Probabilistic Robotics (Thrun et al.): EKF-SLAM applications
- Dom Wilson's Rust implementation: https://github.com/dw-labs-org/kfilter
- UAV EKF in Rust: Implementation patterns for quaternions

## Notes for AI Agent
- Start with discrete-time EKF as it's more commonly used
- Implement analytical Jacobian first, numerical as fallback
- Use the existing linear KF as base, override predict/update
- Test with simple nonlinear system first (e.g., pendulum)
- Monitor covariance eigenvalues for stability detection
- Consider using `approx` crate for floating-point comparisons
- Benchmark Jacobian computation as it's the main overhead
- Document linearization assumptions clearly

## Quality Score: 9/10
Comprehensive PRP with strong theoretical foundation, practical implementation guidance, and extensive references. Includes both discrete and continuous formulations with multiple Jacobian strategies. Clear validation criteria with consistency tests. Deducted one point as automatic differentiation details are deferred to future enhancement.
