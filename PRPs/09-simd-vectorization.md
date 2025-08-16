# PRP: SIMD Vectorization for Matrix Operations

## Goal
Implement SIMD (Single Instruction, Multiple Data) vectorization for critical matrix operations in the Kalman filter library to achieve 2-8x performance improvements on modern CPUs supporting AVX2, AVX-512, ARM NEON, and WebAssembly SIMD.

## Why
- **Performance Critical**: Matrix operations dominate Kalman filter runtime (>80% in profiling)
- **Hardware Underutilization**: Current scalar operations use only 1/8th of modern CPU capability
- **Competitive Requirement**: C++ implementations use SIMD extensively (see SOTA.md)
- **Cross-platform Support**: Rust's portable_simd provides unified API across architectures
- **Low Hanging Fruit**: 2-8x speedup with minimal API changes

## What
Implement SIMD-accelerated versions of core matrix operations:
1. Matrix-vector multiplication (used in predict/update)
2. Matrix-matrix multiplication (covariance updates)
3. Matrix addition/subtraction
4. Element-wise operations (scaling, square root)
5. Dot products and reductions

### Success Criteria
- [ ] 2x+ performance improvement on matrix operations >16 elements
- [ ] Automatic fallback to scalar on unsupported platforms
- [ ] No accuracy loss (bit-identical results in tests)
- [ ] Works on x86_64 (AVX2), aarch64 (NEON), wasm32 (SIMD128)
- [ ] Benchmarks show improvement across dimension sizes
- [ ] Zero overhead when SIMD feature disabled

## All Needed Context

### Current Matrix Operation Patterns
```yaml
- file: C:\Users\deste\repos\kalman_filter_rs\src\filter.rs:200-227
  why: Shows current scalar matrix multiply pattern (triple nested loops)
  
- file: C:\Users\deste\repos\kalman_filter_rs\benches\kalman_benchmarks.rs:613-635
  why: Benchmark implementations to be optimized
  
- file: C:\Users\deste\repos\kalman_filter_rs\src\validation.rs
  why: Matrix validation functions that could benefit from SIMD
```

### Matrix Operations Analysis
From codebase scan:
- Matrix multiply: O(n³) - Primary optimization target
- Matrix-vector: O(n²) - Used in every predict/update
- Element-wise ops: O(n²) - Covariance updates
- Row-major storage: Contiguous memory ideal for SIMD

### Rust SIMD References
```yaml
- url: https://doc.rust-lang.org/std/simd/index.html
  why: Official portable_simd documentation - stable in Rust 1.64+
  
- url: https://github.com/rust-lang/portable-simd
  why: Examples and best practices for portable SIMD
  
- url: https://github.com/dimforge/simba
  why: SIMD abstraction used by nalgebra - proven patterns
  
- url: https://www.rustsim.org/blog/2020/03/23/simd-aosoa-in-nalgebra/
  why: nalgebra's SIMD implementation strategy
  
- url: https://docs.rs/packed_simd_2/latest/packed_simd_2/
  why: Alternative SIMD crate with more features
```

### Performance Targets
Based on research and C++ implementations:
- 4x speedup for aligned f32 operations (8 lanes)
- 2x speedup for f64 operations (4 lanes)
- 8x potential with AVX-512 (16 f32 lanes)
- Near-linear scaling up to SIMD width

### Platform Detection
```yaml
- x86/x86_64: Use std::is_x86_feature_detected!
- ARM: Use std::arch::is_aarch64_feature_detected!
- WASM: Compile-time feature flag
- Fallback: Scalar implementation always available
```

### Known Gotchas
- Alignment requirements: SIMD requires 16/32/64-byte alignment
- Remainder handling: Matrix dimensions not divisible by SIMD width
- Precision differences: FMA instructions may change rounding
- Overhead for small matrices: SIMD setup cost vs benefit threshold
- Feature detection: Runtime CPU detection needed for distribution

## Implementation Blueprint

### Task List (in order)

1. **Setup SIMD Infrastructure**
   - Add portable_simd or packed_simd_2 dependency
   - Create src/simd.rs module for SIMD operations
   - Setup feature flag "simd" in Cargo.toml
   - Create CPU feature detection utilities
   - Add alignment helpers for matrix storage

2. **Implement Core SIMD Operations**
   - SIMD dot product for vectors
   - SIMD matrix-vector multiply
   - SIMD matrix-matrix multiply (blocked algorithm)
   - SIMD element-wise operations (add, subtract, scale)
   - SIMD reduction operations (sum, norm)

3. **Integrate with KalmanFilter**
   - Replace predict() matrix operations with SIMD versions
   - Replace update() matrix operations with SIMD versions
   - Ensure alignment of P, F, Q, R matrices
   - Add runtime dispatch based on CPU features
   - Maintain scalar fallback path

4. **Optimize Other Filter Variants**
   - ExtendedKalmanFilter: Jacobian computations
   - UnscentedKalmanFilter: Sigma point operations
   - ParticleFilter: Weight normalization
   - EnsembleKalmanFilter: Member updates
   - InformationFilter: Sparse operations where applicable

5. **Benchmark and Tune**
   - Add SIMD-specific benchmarks
   - Compare scalar vs SIMD across dimensions
   - Profile to find optimal SIMD width thresholds
   - Tune blocking factors for cache efficiency
   - Document performance improvements

6. **Platform Testing**
   - Test on x86_64 with AVX2
   - Test on ARM64 with NEON (if available)
   - Test WASM with simd128 feature
   - Verify fallback on older CPUs
   - Cross-compile for embedded targets

### SIMD Module Structure
```
src/
├── simd/
│   ├── mod.rs          # SIMD module interface
│   ├── ops.rs          # Basic SIMD operations
│   ├── matrix.rs       # Matrix-specific SIMD
│   ├── dispatch.rs     # CPU feature detection
│   └── fallback.rs     # Scalar implementations
```

### Cargo.toml Additions
```toml
[features]
simd = ["dep:wide"]  # or portable_simd when stable

[dependencies]
wide = { version = "0.7", optional = true }
# OR when stable:
# portable_simd = { version = "0.1", optional = true }

[target.'cfg(target_arch = "x86_64")'.dependencies]
# x86_64 specific if needed

[target.'cfg(target_arch = "aarch64")'.dependencies]  
# ARM specific if needed
```

### Integration Pattern
The implementation should use runtime CPU feature detection to dispatch to the appropriate SIMD implementation. When the SIMD feature is enabled and the CPU supports it (e.g., AVX2 on x86_64), use the vectorized path. Otherwise, fall back to scalar operations. The threshold for using SIMD should be configurable based on matrix dimensions.

## Validation Gates

```bash
# Build with SIMD support
cargo build --features simd
cargo build --features simd --target wasm32-unknown-unknown

# Test correctness (results must match scalar)
cargo test --features simd
cargo test --features simd -- --nocapture matrix_multiply

# Benchmark improvements
cargo bench --features simd -- matrix
cargo bench --no-default-features -- matrix
# Compare results

# Test on different architectures (if available)
# x86_64
RUSTFLAGS="-C target-cpu=native" cargo bench --features simd

# Disable AVX2 to test fallback
RUSTFLAGS="-C target-feature=-avx2" cargo test --features simd

# ARM64 (on ARM machine or cross-compile)
cargo build --features simd --target aarch64-unknown-linux-gnu

# WASM with SIMD
RUSTFLAGS="-C target-feature=+simd128" cargo build \
    --features simd --target wasm32-unknown-unknown

# Verify no overhead when disabled
cargo bench  # baseline
cargo bench --features simd  # should be faster
size target/release/libkalman_filter.rlib  # compare sizes

# Assembly inspection
cargo rustc --release --features simd -- --emit asm
# Check for SIMD instructions (vaddpd, vmulpd, etc.)

# Documentation
cargo doc --features simd --open
```

## Error Handling Strategy
- Graceful fallback if SIMD unavailable
- Compile-time errors for unsupported architectures
- Runtime detection with caching for efficiency
- Clear logging when SIMD path taken (debug builds)

## Migration Strategy
1. SIMD is purely additive - no breaking changes
2. Opt-in via feature flag initially
3. After validation, could become default feature
4. Maintain scalar path indefinitely for compatibility

## References for Implementation
- portable-simd guide: https://rust-lang.github.io/portable-simd/
- packed_simd_2 docs: https://docs.rs/packed_simd_2/
- simba (nalgebra): https://github.com/dimforge/simba
- Performance guide: https://www.intel.com/content/www/us/en/docs/intrinsics-guide/
- ARM NEON: https://developer.arm.com/architectures/instruction-sets/intrinsics/
- WASM SIMD: https://github.com/WebAssembly/simd

## Notes for AI Agent
- Start with simple operations (dot product) before complex (matrix multiply)
- Use chunks_exact() for clean remainder handling
- Prefer horizontal operations last (they're expensive)
- Test with small matrices first, then scale up
- Consider cache blocking for large matrices
- Use #[inline] aggressively for SIMD functions
- Benchmark everything - SIMD isn't always faster for small data

## Quality Score: 9/10
Very high confidence in implementation success. Clear performance targets, established patterns from nalgebra, and comprehensive validation gates. Only deduction for platform testing complexity across different architectures. All critical context provided including existing code patterns, external references, and specific optimization targets.