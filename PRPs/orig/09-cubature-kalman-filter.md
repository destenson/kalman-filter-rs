# PRP: Cubature Kalman Filter (CKF) for High-Accuracy Nonlinear Estimation

## Goal
Implement a numerically stable Cubature Kalman Filter using spherical-radial cubature rules for high-accuracy nonlinear state estimation, supporting both third-degree and higher-degree cubature rules.

## Why
- **Numerical Stability**: More stable than UKF with deterministic cubature points
- **Mathematical Rigor**: Based on exact integration rules for Gaussian integrals
- **Improved Accuracy**: Third-degree accuracy with potential for higher degrees
- **Fewer Parameters**: No tuning parameters unlike UKF (α, β, κ)
- **Computational Efficiency**: 2n cubature points vs 2n+1 sigma points

## What
Create a CKF implementation using spherical-radial cubature rules to numerically compute Gaussian-weighted integrals, with support for square-root formulation and adaptive high-degree variants.

### Success Criteria
- [ ] Third-degree spherical-radial cubature rule implementation
- [ ] Support for fifth and seventh-degree rules
- [ ] Square-root CKF for numerical stability
- [ ] Embedded CKF with adaptive parameter selection
- [ ] Performance comparison with UKF showing stability advantages
- [ ] Examples demonstrate spacecraft attitude estimation
- [ ] Accuracy validation against Gauss-Hermite quadrature

## All Needed Context

### Documentation & References
```yaml
- url: https://www.researchgate.net/publication/224471882_Cubature_Kalman_Filters
  why: Original Arasaratnam & Haykin (2009) CKF paper
  
- url: https://www.sciencedirect.com/science/article/abs/pii/S000510981200550X
  why: High-degree cubature Kalman filter extensions
  
- url: https://www.sciencedirect.com/science/article/abs/pii/S0005109821000613
  why: CKF revisited - corrections and improvements
  
- url: https://www.sciencedirect.com/science/article/abs/pii/S0947358021000182
  why: Square-root high-degree CKF for continuous-discrete systems
  
- url: https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/iet-spr.2012.0085
  why: Cubature quadrature Kalman filter combining multiple rules
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\unscented.rs
  why: UKF implementation to compare and share infrastructure
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\types.rs
  why: Common types for nonlinear filters
```

### Suggested Implementation Structure
```bash
gai-rs-kalman/
├── src/
│   ├── cubature/
│   │   ├── mod.rs              # Module exports
│   │   ├── filter.rs           # Core CKF implementation
│   │   ├── cubature_rule.rs    # Spherical-radial rules
│   │   ├── high_degree.rs      # 5th, 7th degree rules
│   │   ├── square_root.rs      # Square-root CKF
│   │   ├── embedded.rs         # Embedded CKF with adaptation
│   │   ├── quadrature.rs       # Gauss-Laguerre combination
│   │   └── continuous.rs       # Continuous-discrete CKF
├── examples/
│   ├── ckf_spacecraft.rs       # Attitude estimation
│   ├── ckf_target_tracking.rs  # Radar tracking
│   ├── ckf_vs_ukf.rs          # Comparison example
│   └── ckf_high_degree.rs     # High-degree demonstration
├── benches/
│   └── ckf_accuracy.rs        # Accuracy vs computational cost
```

### Known Gotchas & Library Quirks
- CRITICAL: Cubature points lie on hypersphere surface, not volume
- CRITICAL: Weight equality (1/2n) is fundamental property
- CRITICAL: Numerical issues in high dimensions (n > 100)
- CRITICAL: Square-root form essential for covariance preservation
- CRITICAL: Higher degrees increase points exponentially
- CRITICAL: Embedded rule free parameter affects accuracy

## Implementation Blueprint

### Core CKF Mathematics

**Third-Degree Spherical-Radial Rule**:
For integral I(f) = ∫ f(x)N(x; 0, I)dx:
- Cubature points: ξᵢ = √n eᵢ and ξᵢ₊ₙ = -√n eᵢ
- Weights: ωᵢ = 1/(2n) for all i
- Where eᵢ are unit vectors

**Cubature Transform**:
1. Generate cubature points:
   - Xᵢ = √P ξᵢ + x̂ for i = 1...2n
2. Propagate through nonlinearity:
   - Yᵢ = f(Xᵢ)
3. Compute statistics:
   - Mean: ȳ = Σ ωᵢ Yᵢ
   - Covariance: Pᵧ = Σ ωᵢ (Yᵢ - ȳ)(Yᵢ - ȳ)ᵀ

**CKF Algorithm**:
1. Prediction:
   - Generate cubature points from x̂ₖ₋₁, Pₖ₋₁
   - Propagate: Xᵢ,ₖ|ₖ₋₁ = f(Xᵢ,ₖ₋₁)
   - Predicted mean: x̂ₖ|ₖ₋₁ = Σ ωᵢ Xᵢ,ₖ|ₖ₋₁
   - Predicted covariance: Pₖ|ₖ₋₁ = Σ ωᵢ (Xᵢ - x̂)(Xᵢ - x̂)ᵀ + Q

2. Update:
   - Generate cubature points from x̂ₖ|ₖ₋₁, Pₖ|ₖ₋₁
   - Transform: Zᵢ = h(Xᵢ)
   - Predicted measurement: ẑ = Σ ωᵢ Zᵢ
   - Innovation covariance: Pzz = Σ ωᵢ (Zᵢ - ẑ)(Zᵢ - ẑ)ᵀ + R
   - Cross-covariance: Pxz = Σ ωᵢ (Xᵢ - x̂)(Zᵢ - ẑ)ᵀ
   - Kalman gain: K = Pxz Pzz⁻¹
   - Update state and covariance

**High-Degree Rules**:
- Fifth-degree: n² + 3n + 3 points
- Seventh-degree: n³ + 5n² + 5n + 1 points
- Trade-off: accuracy vs computational cost

**Square-Root CKF**:
- Propagate Cholesky factor S: P = SSᵀ
- Use QR decomposition for updates
- Ensures positive definiteness

### Task List

```yaml
Task 1: Define cubature rule structures
CREATE src/cubature/cubature_rule.rs:
  - CubatureRule trait for different degrees
  - ThirdDegreeRule with 2n points
  - Point generation on unit hypersphere
  - Weight calculation (uniform 1/2n)

Task 2: Implement core CKF
CREATE src/cubature/filter.rs:
  - CubatureKalmanFilter struct
  - Cubature transform implementation
  - Predict step with cubature points
  - Update step with cross-covariance
  - State and covariance accessors

Task 3: Add high-degree rules
CREATE src/cubature/high_degree.rs:
  - Fifth-degree spherical-radial rule
  - Seventh-degree rule
  - Adaptive degree selection
  - Point/weight generation algorithms

Task 4: Implement square-root CKF
CREATE src/cubature/square_root.rs:
  - Cholesky factor propagation
  - QR-based covariance update
  - Triangular matrix operations
  - Numerical stability guarantees

Task 5: Create embedded CKF
CREATE src/cubature/embedded.rs:
  - Embedded cubature rule
  - Free parameter optimization
  - Maximum likelihood adaptation
  - Stability-accuracy trade-off

Task 6: Add quadrature combination
CREATE src/cubature/quadrature.rs:
  - Gauss-Laguerre quadrature integration
  - Combined CQKF implementation
  - Improved accuracy for special cases
  - Computational efficiency analysis

Task 7: Implement continuous-discrete variant
CREATE src/cubature/continuous.rs:
  - Continuous process model
  - Discrete measurements
  - ODE integration for cubature points
  - Mixed time-scale handling

Task 8: Create spacecraft attitude example
CREATE examples/ckf_spacecraft.rs:
  - Quaternion-based attitude
  - Gyroscope and star tracker fusion
  - Comparison with EKF/UKF
  - Real-time performance

Task 9: Add comparison example
CREATE examples/ckf_vs_ukf.rs:
  - Same nonlinear system
  - CKF vs UKF performance
  - Numerical stability tests
  - Accuracy metrics

Task 10: Create comprehensive tests
CREATE tests/ckf_tests.rs:
  - Cubature point symmetry
  - Weight normalization
  - Integration accuracy tests
  - Comparison with analytical integrals
  - Stability in high dimensions
```

### Integration Points
```yaml
SHARED INFRASTRUCTURE:
  - Reuse nonlinear system traits from EKF/UKF
  - Common matrix operations
  - Shared validation utilities
  - Unified builder pattern

NUMERICAL LIBRARIES:
  - Use nalgebra for matrix operations
  - QR decomposition from LAPACK
  - Cholesky factorization
  - Eigenvalue monitoring

OPTIMIZATION:
  - SIMD for point generation
  - Parallel cubature point evaluation
  - Cache-efficient memory layout
  - Const generics for compile-time optimization
```

## Validation Loop

### Level 1: Syntax & Style
```bash
# Build CKF module
cargo build --features cubature

# Code quality
cargo clippy --features cubature -- -D warnings
cargo fmt --check
```

### Level 2: Unit Tests
Test requirements:
- Cubature points on unit sphere
- Weight sum equals 1
- Third-degree polynomial integration exact
- Square-root form maintains positive definiteness
- High-degree rules improve accuracy

### Level 3: Integration Tests
```bash
# Run spacecraft example
cargo run --example ckf_spacecraft --features cubature

# Compare with UKF
cargo run --example ckf_vs_ukf --features "cubature unscented"

# Test high-degree rules
cargo run --example ckf_high_degree --features cubature
```

### Level 4: Numerical Validation
Metrics to verify:
- Integration accuracy vs analytical
- Stability in ill-conditioned problems
- Convergence rate analysis
- Computational cost vs accuracy
- Comparison with Gauss-Hermite quadrature

## Final Validation Checklist
- [ ] Third-degree rule matches published results
- [ ] Square-root form prevents covariance issues
- [ ] Higher degrees improve accuracy as expected
- [ ] Embedded CKF adapts parameter correctly
- [ ] Performance competitive with or better than UKF
- [ ] Examples demonstrate practical applications
- [ ] Documentation explains cubature theory

## Anti-Patterns to Avoid
- ❌ Don't confuse spherical with Cartesian coordinates
- ❌ Don't neglect square-root form for long runs
- ❌ Don't use high degrees unnecessarily (cost)
- ❌ Don't ignore dimension limits for stability
- ❌ Don't mix cubature rules incorrectly
- ❌ Don't forget to validate positive definiteness

## Error Handling Strategy
- Check matrix conditioning before inversion
- Monitor covariance eigenvalues
- Detect numerical overflow in high dimensions
- Fallback to lower degree if unstable
- Automatic square-root form switching
- Graceful degradation to EKF if needed

## References for Implementation
- Arasaratnam & Haykin (2009): Original CKF paper
- Stroud (1971): "Approximate Calculation of Multiple Integrals"
- Jia et al. (2013): High-degree cubature filters
- Zhang et al. (2021): CKF revisited
- Bhaumik & Swati (2013): Cubature quadrature filter

## Notes for AI Agent
- Start with third-degree rule (simplest and most common)
- Implement square-root form early for stability
- Use const generics for compile-time dimensions
- Test with Van der Pol oscillator
- Compare with UKF using same test problems
- Profile cubature point generation
- Consider caching unit sphere points
- Default to third-degree for most applications
- Document mathematical derivations clearly

## Quality Score: 9/10
Comprehensive PRP specifically for CKF with deep mathematical foundation and practical implementation guidance. Covers standard, high-degree, and square-root variants with clear algorithmic specifications. Strong emphasis on numerical stability and accuracy validation. Deducted one point as some advanced topics (simplex cubature, Gaussian mixture CKF) are not covered.
