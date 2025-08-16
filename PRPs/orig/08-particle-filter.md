# PRP: Particle Filter for Non-Gaussian and Nonlinear State Estimation

## Goal
Implement a robust Sequential Monte Carlo (SMC) particle filter capable of handling arbitrary probability distributions and highly nonlinear systems, with applications in robotics SLAM, target tracking, and financial modeling.

## Why
- **Non-Gaussian Distributions**: Handles multimodal, skewed, and arbitrary PDFs
- **Highly Nonlinear Systems**: No linearization required, works with discontinuous functions
- **Global Optimization**: Can track multiple hypotheses simultaneously
- **SLAM Applications**: Industry standard for robot localization and mapping
- **Flexibility**: Can incorporate any likelihood function and motion model

## What
Create a particle filter framework using weighted samples to represent probability distributions, supporting various resampling strategies, Rao-Blackwellization, and GPU acceleration for real-time applications.

### Success Criteria
- [ ] Handle multimodal distributions correctly
- [ ] Support 1000+ particles in real-time
- [ ] Multiple resampling algorithms (systematic, stratified, deterministic)
- [ ] Effective sample size monitoring
- [ ] Rao-Blackwellized variant for SLAM
- [ ] Examples demonstrate robot localization and tracking
- [ ] GPU acceleration support for large particle sets

## All Needed Context

### Documentation & References
```yaml
- url: https://www.stats.ox.ac.uk/~doucet/smc_resources.html
  why: Comprehensive SMC/particle filter resources from leading researcher
  
- url: https://github.com/rsasaki0109/RustRobotics
  why: Rust implementation of particle filter and FastSLAM
  
- url: https://github.com/0shimax/rust-particle-filter
  why: Dedicated Rust particle filter implementation
  
- url: https://www.sciencedirect.com/science/article/abs/pii/S0263224122001312
  why: Review of resampling techniques to handle degeneracy
  
- url: https://arxiv.org/html/2504.18056
  why: Modern GPU-accelerated particle filter for 6-DoF SLAM
  
- url: https://en.wikipedia.org/wiki/Particle_filter
  why: Mathematical foundations and algorithm variants
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\filter.rs
  why: Base filter trait to implement
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\types.rs
  why: Type system to extend for particle representation
```

### Suggested Implementation Structure
```bash
gai-rs-kalman/
├── src/
│   ├── particle/
│   │   ├── mod.rs              # Module exports
│   │   ├── filter.rs           # Core particle filter
│   │   ├── particle.rs         # Particle representation
│   │   ├── resampling.rs       # Resampling strategies
│   │   ├── rao_blackwell.rs    # Rao-Blackwellized PF
│   │   ├── auxiliary.rs        # Auxiliary particle filter
│   │   ├── gpu.rs              # GPU acceleration
│   │   └── diagnostics.rs      # ESS, particle diversity
├── examples/
│   ├── pf_robot_localization.rs   # 2D robot localization
│   ├── pf_slam.rs                 # FastSLAM implementation
│   ├── pf_multi_target.rs         # Multi-target tracking
│   └── pf_financial.rs            # Option pricing with jumps
├── benches/
│   └── particle_scaling.rs        # Performance vs particle count
```

### Known Gotchas & Library Quirks
- CRITICAL: Particle degeneracy - all weight concentrates on few particles
- CRITICAL: Sample impoverishment after resampling
- CRITICAL: Curse of dimensionality - exponential particles needed
- CRITICAL: Resampling breaks parallelization
- CRITICAL: Numerical underflow in weight calculations (use log-weights)
- CRITICAL: GPU memory transfer overhead for small particle sets

## Implementation Blueprint

### Core Particle Filter Concepts

**Particle Representation**:
- Particles: {xᵢ, wᵢ} for i=1...N
- State samples: xᵢ ~ p(x)
- Weights: wᵢ represent probability
- Normalization: Σwᵢ = 1

**Sequential Importance Sampling (SIS)**:
1. Initialize: xᵢ₀ ~ p(x₀), wᵢ₀ = 1/N
2. Predict: xᵢₖ ~ p(xₖ|xᵢₖ₋₁)
3. Weight update: wᵢₖ = wᵢₖ₋₁ × p(yₖ|xᵢₖ)
4. Normalize: wᵢₖ = wᵢₖ / Σwⱼₖ

**Resampling Strategies**:
- Multinomial: Simple but high variance
- Systematic: Lower variance, deterministic spacing
- Stratified: Divides CDF into strata
- Residual: Deterministic + stochastic
- Metropolis-Hastings: MCMC moves after resampling

**Effective Sample Size (ESS)**:
- ESS = 1 / Σ(wᵢ)²
- Resample when ESS < N_threshold (typically N/2)

**Rao-Blackwellization**:
- Marginalize linear/Gaussian substructure analytically
- Particle for nonlinear states
- Kalman filter for linear states given particle

**Auxiliary Particle Filter**:
- Look-ahead to future measurement
- Sample from better proposal distribution
- Reduces particle degeneracy

### Task List

```yaml
Task 1: Define particle data structures
CREATE src/particle/particle.rs:
  - Particle struct with state and weight
  - ParticleSet container with operations
  - LogWeight representation for numerical stability
  - Particle pool for memory efficiency

Task 2: Implement core particle filter
CREATE src/particle/filter.rs:
  - Sequential importance sampling
  - Bootstrap particle filter
  - Generic state and measurement types
  - Log-weight arithmetic for stability

Task 3: Implement resampling algorithms
CREATE src/particle/resampling.rs:
  - Multinomial resampling
  - Systematic resampling
  - Stratified resampling
  - Residual resampling
  - Metropolis-Hastings moves
  - ESS calculation and trigger

Task 4: Add Rao-Blackwellized variant
CREATE src/particle/rao_blackwell.rs:
  - Marginalized particle filter
  - Particle with Kalman filter per particle
  - FastSLAM as special case
  - Conditional independence exploitation

Task 5: Implement auxiliary particle filter
CREATE src/particle/auxiliary.rs:
  - Look-ahead proposal
  - Auxiliary weights computation
  - Improved sampling efficiency
  - Adaptive proposal selection

Task 6: Add GPU acceleration
CREATE src/particle/gpu.rs:
  - CUDA/OpenCL kernels for particle operations
  - Parallel weight computation
  - GPU-friendly resampling
  - Memory transfer optimization

Task 7: Create robot localization example
CREATE examples/pf_robot_localization.rs:
  - 2D robot with odometry noise
  - Landmark measurements
  - Multimodal posterior handling
  - Visualization of particle cloud

Task 8: Implement FastSLAM example
CREATE examples/pf_slam.rs:
  - Rao-Blackwellized SLAM
  - Particle for robot pose
  - EKF for landmarks per particle
  - Loop closure handling

Task 9: Add diagnostics module
CREATE src/particle/diagnostics.rs:
  - Effective sample size
  - Particle diversity metrics
  - Weight entropy
  - Degeneracy detection
  - Convergence monitoring

Task 10: Create comprehensive tests
CREATE tests/particle_tests.rs:
  - Test resampling unbiasedness
  - Verify weight normalization
  - Test multimodal tracking
  - Benchmark vs analytical solutions
  - Convergence tests
```

### Integration Points
```yaml
MEMORY MANAGEMENT:
  - Object pool for particles
  - In-place resampling when possible
  - Avoid particle copying
  - Custom allocator for real-time

PARALLELIZATION:
  - Parallel particle propagation
  - Parallel weight computation
  - GPU kernels for large N
  - SIMD for weight operations

NUMERICAL STABILITY:
  - Log-space weight arithmetic
  - Stable resampling algorithms
  - Overflow/underflow protection
  - Condition monitoring
```

## Validation Loop

### Level 1: Syntax & Style
```bash
# Build particle filter module
cargo build --features particle

# Code quality
cargo clippy --features particle -- -D warnings
cargo fmt --check
```

### Level 2: Unit Tests
Test requirements:
- Resampling preserves mean
- ESS calculation correct
- Weight normalization maintained
- Particle diversity after resampling
- Log-weight numerical stability

### Level 3: Integration Tests
```bash
# Run robot localization
cargo run --example pf_robot_localization --features particle

# Run SLAM example
cargo run --example pf_slam --features particle

# Benchmark scaling
cargo bench --features particle
```

### Level 4: Statistical Validation
Metrics to verify:
- Posterior mean accuracy
- Posterior variance estimation
- Multimodal distribution tracking
- Convergence rate vs particle count
- Computational complexity scaling

## Final Validation Checklist
- [ ] Handles multimodal distributions correctly
- [ ] No particle degeneracy with proper resampling
- [ ] ESS triggers resampling appropriately
- [ ] Rao-Blackwellization improves efficiency
- [ ] Real-time performance with 1000+ particles
- [ ] GPU acceleration provides speedup
- [ ] Examples demonstrate practical applications

## Anti-Patterns to Avoid
- ❌ Don't resample every timestep (loses diversity)
- ❌ Don't use linear weights (numerical underflow)
- ❌ Don't ignore ESS (degeneracy undetected)
- ❌ Don't use multinomial resampling (high variance)
- ❌ Don't apply to high-dimensional states naively
- ❌ Don't forget proposal distribution importance

## Error Handling Strategy
- Detect weight degeneracy (all weights → 0)
- Monitor ESS for particle collapse
- Adaptive particle count based on complexity
- Regularization for sample impoverishment
- Automatic switch to auxiliary PF if needed
- Graceful fallback to bootstrap filter

## References for Implementation
- Doucet et al. (2001): "Sequential Monte Carlo Methods in Practice"
- Arulampalam et al. (2002): "A Tutorial on Particle Filters"
- Montemerlo et al. (2002): "FastSLAM"
- Gordon et al. (1993): Original bootstrap filter
- Liu & Chen (1998): Sequential Monte Carlo methods

## Notes for AI Agent
- Start with bootstrap filter (simplest)
- Implement systematic resampling first
- Use log-weights from the beginning
- Test with bimodal Gaussian mixture
- Profile resampling as bottleneck
- Consider arena allocator for particles
- Add visualization for debugging
- Default N=100 particles for real-time
- Benchmark against Kalman filter for linear case

## Quality Score: 9/10
Comprehensive PRP for particle filter implementation with strong focus on practical challenges like degeneracy and resampling. Covers both basic and advanced variants (Rao-Blackwellized, auxiliary) with clear mathematical specifications. Includes GPU acceleration and real-world applications. Deducted one point as some advanced topics (smoothing, parameter learning) are not covered in detail.
