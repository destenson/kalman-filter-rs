# PRP: Unscented Kalman Filter (UKF) Implementation

## Goal
Implement a robust Unscented Kalman Filter (UKF) using sigma points to handle nonlinear transformations more accurately than EKF, without requiring Jacobian calculations.

## Why
- **Superior Accuracy**: Captures mean and covariance to 3rd order Taylor expansion (vs 1st order for EKF)
- **Jacobian-Free**: No need for analytical derivatives, easier to implement and maintain
- **Better Stability**: More robust than EKF for highly nonlinear systems
- **Proven Applications**: Widely used in aerospace, robotics, and automotive industries
- **Parameter Estimation**: Can jointly estimate states and model parameters

## What
Create a UKF that uses deterministic sigma point sampling to approximate probability distributions through nonlinear transformations, supporting both additive and augmented state formulations.

### Success Criteria
- [ ] UKF handles arbitrary nonlinear functions without Jacobians
- [ ] Supports both standard and square-root formulations
- [ ] Sigma point generation uses Van der Merwe's scaled algorithm
- [ ] Examples demonstrate superiority over EKF for highly nonlinear systems
- [ ] Augmented state handles non-additive noise
- [ ] Performance benchmarks vs EKF for various nonlinearity levels

## All Needed Context

### Documentation & References
```yaml
- url: https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
  why: Original UKF paper by Julier & Uhlmann with mathematical foundation
  
- url: https://github.com/hortovanyi/Unscented-Kalman-Filter-Rust
  why: Rust UKF implementation based on Udacity course with strong typing
  
- url: https://filterpy.readthedocs.io/en/latest/kalman/UnscentedKalmanFilter.html
  why: Comprehensive UKF documentation with sigma point algorithms
  
- url: https://docs.duckietown.com/daffy/course-intro-to-drones/ukf/theory/ukf-specifics.html
  why: Clear explanation of sigma points and unscented transform
  
- url: https://github.com/sfwa/ukf
  why: C++ reference showing square-root UKF and quaternion handling
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\filter.rs
  why: Base filter structure to follow
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\unscented.rs
  why: Existing stub file to implement
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\extended.rs
  why: EKF implementation for comparison and shared traits
```

### Suggested Implementation Structure
```bash
gai-rs-kalman/
├── src/
│   ├── unscented.rs        # Main UKF implementation
│   ├── unscented/
│   │   ├── mod.rs          # Module exports
│   │   ├── sigma_points.rs # Sigma point generation algorithms
│   │   ├── transform.rs    # Unscented transform
│   │   ├── augmented.rs    # Augmented state UKF
│   │   └── square_root.rs  # Square-root UKF (SR-UKF)
│   └── types.rs            # Extended with UKF types
├── examples/
│   ├── ukf_van_der_pol.rs     # Highly nonlinear oscillator
│   ├── ukf_reentry_vehicle.rs # Aerospace application
│   └── ukf_parameter_estimation.rs
```

### Known Gotchas & Library Quirks
- CRITICAL: Sigma point weights can be negative (for mean calculation)
- CRITICAL: Covariance weights are always positive
- CRITICAL: Number of sigma points = 2n+1 for standard, 2(n+q)+1 for augmented
- CRITICAL: Parameter tuning (α, β, κ) significantly affects performance
- CRITICAL: Square-root form prevents covariance from losing positive definiteness
- CRITICAL: Augmented form needed when process noise is non-additive

## Implementation Blueprint

### Core Design Concepts

**Sigma Point Generation**:
Van der Merwe's scaled sigma point algorithm:
- Scaling parameters: α (spread), β (prior knowledge), κ (secondary scaling)
- λ = α²(n + κ) - n (compound scaling parameter)
- 2n+1 sigma points for n-dimensional state
- Weights for mean: W_m[0] = λ/(n+λ), W_m[i] = 1/2(n+λ)
- Weights for covariance: W_c[0] = λ/(n+λ) + (1-α²+β), W_c[i] = 1/2(n+λ)

**Unscented Transform**:
1. Generate sigma points X from state x and covariance P
2. Transform sigma points: Y[i] = f(X[i])
3. Compute transformed mean: y = Σ W_m[i] * Y[i]
4. Compute transformed covariance: P_y = Σ W_c[i] * (Y[i]-y)(Y[i]-y)^T

**UKF Algorithm**:
1. Prediction:
   - Generate sigma points from x̂_k-1|k-1, P_k-1|k-1
   - Propagate through process model: X_k|k-1 = f(X_k-1|k-1)
   - Compute predicted mean and covariance via unscented transform
   
2. Update:
   - Generate sigma points from x̂_k|k-1, P_k|k-1
   - Transform through measurement model: Z_k|k-1 = h(X_k|k-1)
   - Compute predicted measurement and innovation covariance
   - Calculate cross-covariance and Kalman gain
   - Update state and covariance

**Augmented State Formulation**:
For non-additive noise, augment state with noise terms:
- Augmented state: x_a = [x^T, w^T, v^T]^T
- Augmented covariance includes Q and R
- Generate sigma points in augmented space

### Task List

```yaml
Task 1: Define sigma point types and parameters
MODIFY src/types.rs:
  - Add SigmaPointParameters struct (α, β, κ)
  - Add SigmaPoints container with points and weights
  - Add UKF-specific configuration types

Task 2: Implement sigma point generation
CREATE src/unscented/sigma_points.rs:
  - Van der Merwe's scaled algorithm
  - Simplex sigma points (optional)
  - Minimal skew simplex (optional)
  - Weight calculation for mean and covariance

Task 3: Implement unscented transform
CREATE src/unscented/transform.rs:
  - Generic transform for any nonlinear function
  - Mean and covariance computation
  - Cross-covariance calculation
  - Handle constraints (e.g., quaternion normalization)

Task 4: Implement standard UKF
MODIFY src/unscented.rs:
  - UnscentedKalmanFilter struct
  - Predict step using unscented transform
  - Update step with cross-covariance
  - State and covariance getters

Task 5: Implement augmented UKF
CREATE src/unscented/augmented.rs:
  - Augmented state construction
  - Modified sigma point generation
  - Handle non-additive process and measurement noise
  - Dimension management for augmented state

Task 6: Implement square-root UKF
CREATE src/unscented/square_root.rs:
  - Propagate Cholesky factors instead of covariance
  - QR decomposition for updates
  - Guaranteed positive definiteness
  - Improved numerical stability

Task 7: Create Van der Pol oscillator example
CREATE examples/ukf_van_der_pol.rs:
  - Highly nonlinear dynamics
  - Compare UKF vs EKF performance
  - Show superiority for strong nonlinearity

Task 8: Create reentry vehicle example
CREATE examples/ukf_reentry_vehicle.rs:
  - Ballistic reentry dynamics
  - Exponential atmospheric density
  - Radar measurements

Task 9: Create parameter estimation example
CREATE examples/ukf_parameter_estimation.rs:
  - Joint state and parameter estimation
  - Augment state with unknown parameters
  - Online system identification

Task 10: Add comprehensive tests
CREATE tests/ukf_tests.rs:
  - Verify sigma point symmetry
  - Test weight normalization
  - Compare with EKF on weakly nonlinear systems
  - Test augmented vs standard for additive noise
  - Verify square-root maintains positive definiteness
```

### Integration Points
```yaml
TRAITS:
  - Share NonlinearSystem trait with EKF
  - Add UnscentedTransform trait
  - Define SigmaPointGenerator trait for different algorithms

BUILDER:
  - Extend for UKF-specific parameters
  - Set sigma point algorithm
  - Configure augmented state
  - Choose standard vs square-root

BENCHMARKS:
  - Compare UKF vs EKF vs Linear KF
  - Measure sigma point generation overhead
  - Profile different sigma point algorithms
```

## Validation Loop

### Level 1: Syntax & Style
```bash
# Build with UKF implementation
cargo build --features unscented

# Code quality checks
cargo clippy --all-features -- -D warnings
cargo fmt --check
```

### Level 2: Unit Tests
Test requirements:
- Sigma points capture mean exactly
- Sigma points capture covariance exactly for linear systems
- Weight normalization: Σ W_m = 1, Σ W_c = 1
- Unscented transform preserves Gaussian through linear function
- Augmented state produces same result as standard for additive noise
- Square-root form maintains Cholesky factor properties

### Level 3: Integration Tests
```bash
# Run UKF examples
cargo run --example ukf_van_der_pol
cargo run --example ukf_reentry_vehicle
cargo run --example ukf_parameter_estimation

# Comparative analysis
cargo run --example compare_ekf_ukf --features "extended unscented"
```

### Level 4: Performance Validation
Metrics to evaluate:
- RMSE comparison: UKF vs EKF for different nonlinearity levels
- Consistency via NEES and NIS statistics
- Computational cost vs accuracy tradeoff
- Convergence rate for parameter estimation
- Stability for long-duration runs

## Final Validation Checklist
- [ ] UKF implementation compiles without warnings
- [ ] Sigma points correctly generated and weighted
- [ ] Unscented transform preserves mean/covariance for linear systems
- [ ] UKF outperforms EKF for highly nonlinear examples
- [ ] Square-root form maintains numerical stability
- [ ] Augmented state handles non-additive noise correctly
- [ ] Documentation explains parameter tuning guidelines

## Anti-Patterns to Avoid
- ❌ Don't use fixed parameters (α, β, κ) for all problems
- ❌ Don't ignore negative weights in covariance calculation
- ❌ Don't assume n sigma points sufficient (need 2n+1)
- ❌ Don't forget to normalize quaternions or angles
- ❌ Don't use standard form when covariance may lose positive definiteness
- ❌ Don't overlook augmented state for non-additive noise

## Error Handling Strategy
- Validate sigma point parameters (0 < α ≤ 1)
- Check weight normalization after calculation
- Monitor covariance eigenvalues for positive definiteness
- Detect numerical issues in Cholesky decomposition
- Provide fallback to EKF if UKF fails
- Reset filter if state diverges

## References for Implementation
- Original UKF paper: https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
- Rust implementation: https://github.com/hortovanyi/Unscented-Kalman-Filter-Rust
- Van der Merwe dissertation: Comprehensive sigma point algorithms
- FilterPy documentation: Python reference implementation
- Square-root UKF: Merwe & Wan (2001) paper

## Notes for AI Agent
- Start with standard UKF before square-root form
- Implement Van der Merwe's algorithm as default
- Use 2n+1 sigma points initially (minimal set)
- Default parameters: α=1e-3, β=2, κ=3-n
- Test with Van der Pol oscillator for nonlinearity
- Profile sigma point generation as main computational cost
- Consider generic const parameters for compile-time optimization
- Add runtime checks for covariance positive definiteness
- Document why UKF superior to EKF for specific cases

## Quality Score: 9/10
Comprehensive PRP covering standard, augmented, and square-root UKF formulations with clear mathematical foundation. Includes multiple sigma point algorithms and practical examples. Strong emphasis on numerical stability and comparison with EKF. Deducted one point as some advanced topics (constrained UKF, iterative UKF) are not covered in detail.
