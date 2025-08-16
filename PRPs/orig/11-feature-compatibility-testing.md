# PRP: Comprehensive Feature Compatibility Testing for gai-rs-kalman

## Goal
Ensure all tests and examples in the gai-rs-kalman crate work correctly with all combinations of feature flags, properly utilizing optional features when enabled and gracefully handling their absence.

## Why
- **Feature Fragmentation**: Current examples don't handle optional features consistently
- **CI Coverage**: No systematic testing of feature combinations
- **User Experience**: Users enabling different features should get working examples
- **Maintenance**: Feature-gated code paths need validation to prevent bit rot
- **No-std Support**: Future no_std support requires careful feature management

## What
Create a comprehensive feature compatibility framework that ensures all code paths work with every valid feature combination, with conditional compilation for examples and tests that adapt to available features.

### Success Criteria
- [ ] All examples compile and run with default features
- [ ] All examples compile and run with --no-default-features
- [ ] All examples compile and run with --all-features
- [ ] All tests pass with every feature combination
- [ ] Examples utilize optional features when available
- [ ] CI validates all feature combinations
- [ ] Clear documentation of feature requirements

## All Needed Context

### Current Feature Landscape
```yaml
Features:
  default: ["std", "alloc"]
  std: ["rand_distr/std"]
  alloc: ["rand_distr/alloc"]
  nalgebra: ["dep:nalgebra", "nalgebra/alloc", "gai-rs-math/nalgebra"]
  adskalman: ["dep:adskalman"]
  minikalman: ["dep:minikalman"]
  all: ["adskalman", "minikalman"]
```

### Problematic Files
```yaml
Issues Found:
  - examples/ukf_nonlinear_tracking.rs: No feature handling
  - examples/ckf_high_dimensional.rs: No feature handling
  - examples/enkf_weather_model.rs: No feature handling
  - examples/particle_filter_robot.rs: No feature handling
  - src/filter.rs: Has nalgebra feature gates but not complete
  - No CI testing of feature combinations
```

### Reference Implementation
```yaml
Good Pattern:
  - file: examples/simple_1d.rs
    why: Already handles both nalgebra and vec-based implementations
  - file: examples/position_2d.rs
    why: Shows conditional compilation pattern
```

### External References
```yaml
Documentation:
  - url: https://doc.rust-lang.org/cargo/reference/features.html
    why: Cargo feature documentation for best practices
  - url: https://github.com/rust-lang/rust/issues/45599
    why: Discussion on testing with feature combinations
  - url: https://github.com/taiki-e/cargo-hack
    why: Tool for testing all feature combinations automatically
```

## Requirements

### Feature Matrix Testing
All code must work with these combinations:
1. `--no-default-features`
2. Default features (std + alloc)
3. `--features nalgebra`
4. `--features adskalman`
5. `--features minikalman`
6. `--all-features`
7. Each individual feature alone
8. All pairwise combinations

### Example Adaptation Strategy
Examples should:
- Use conditional compilation to switch implementations
- Provide meaningful functionality with minimal features
- Enhance functionality when optional features are available
- Document which features enable which functionality

### Test Coverage Requirements
- Unit tests must pass with all feature combinations
- Integration tests should adapt to available features
- Doc tests should be feature-aware
- Benchmarks (if any) should handle feature variations

## Implementation Blueprint

### Phase 1: Audit Current State
1. Map which code depends on which features
2. Identify implicit feature dependencies
3. Document minimum feature requirements
4. List enhancement opportunities with optional features

### Phase 2: Refactor Examples
1. Add feature detection to all examples
2. Implement fallback behavior for missing features
3. Add feature-specific enhancements
4. Document feature requirements in example headers

### Phase 3: Test Infrastructure
1. Create feature matrix test script
2. Add CI workflow for feature testing
3. Implement feature-aware test macros
4. Add feature combination benchmarks

### Phase 4: Documentation
1. Create feature guide in README
2. Document each feature's purpose and benefits
3. Add feature requirement badges to examples
4. Create migration guide for feature changes

## Task List

```yaml
Task 1: Create feature detection utilities
LOCATION: src/features.rs (new file)
  - Feature detection macros
  - Conditional type aliases
  - Feature-aware trait implementations
  - Runtime feature reporting

Task 2: Update UKF example for features
MODIFY: examples/ukf_nonlinear_tracking.rs
  - Add nalgebra variant using SMatrix when available
  - Ensure works without any features
  - Document feature benefits in comments

Task 3: Update CKF example for features
MODIFY: examples/ckf_high_dimensional.rs
  - Add optimized nalgebra path
  - Fallback to vec implementation
  - Show performance difference in output

Task 4: Update EnKF example for features
MODIFY: examples/enkf_weather_model.rs
  - Use rayon for parallel ensemble when available
  - Add nalgebra matrix operations option
  - Document scalability with features

Task 5: Update Particle Filter example
MODIFY: examples/particle_filter_robot.rs
  - Add SIMD acceleration when available
  - Use nalgebra for transformations if present
  - Show particle count scaling with features

Task 6: Create feature test matrix
CREATE: tests/feature_matrix.rs
  - Test each filter with each feature combo
  - Verify examples compile with all combos
  - Check for feature leakage

Task 7: Add CI workflow
CREATE: .github/workflows/feature-matrix.yml
  - Test all feature combinations
  - Run examples with each combo
  - Generate feature coverage report

Task 8: Create no_std compatibility layer
MODIFY: src/lib.rs
  - Add #![cfg_attr(not(feature = "std"), no_std)]
  - Import alloc where needed
  - Provide no_std alternatives

Task 9: Document feature requirements
MODIFY: README.md
  - Feature comparison table
  - Performance implications
  - Migration guide from vec to nalgebra

Task 10: Add feature-aware benchmarks
CREATE: benches/feature_comparison.rs
  - Compare vec vs nalgebra performance
  - Show scaling with different features
  - Document optimization opportunities
```

## Validation Gates

```bash
# Test with no features
cargo test --no-default-features

# Test with default features
cargo test

# Test with each feature individually
cargo test --no-default-features --features std
cargo test --no-default-features --features alloc
cargo test --no-default-features --features nalgebra
cargo test --no-default-features --features adskalman
cargo test --no-default-features --features minikalman

# Test with all features
cargo test --all-features

# Test all combinations using cargo-hack
cargo install cargo-hack
cargo hack test --feature-powerset

# Run all examples with different features
for example in examples/*.rs; do
    name=$(basename $example .rs)
    cargo run --example $name --no-default-features
    cargo run --example $name
    cargo run --example $name --all-features
done

# Check that feature gates are correct
cargo clippy --all-targets --no-default-features
cargo clippy --all-targets --all-features

# Verify documentation builds with all features
cargo doc --all-features --no-deps
```

## Error Handling Strategy

### Feature Detection Errors
- Compile-time errors for incompatible feature combinations
- Clear error messages when required features are missing
- Suggest which features to enable for specific functionality

### Runtime Adaptation
- Gracefully degrade when optional features absent
- Warn (not panic) when performance could be improved with features
- Provide feature availability queries

### Migration Path
- Support both old and new APIs during transition
- Deprecation warnings for feature-dependent APIs
- Clear upgrade instructions in errors

## Success Metrics

1. **Compatibility**: Zero compilation failures across feature matrix
2. **Coverage**: 100% of examples work with minimal features
3. **Performance**: Optional features provide measurable improvements
4. **Documentation**: Every feature's purpose is clearly documented
5. **CI**: All feature combinations tested automatically

## Anti-Patterns to Avoid

- ❌ Hard dependencies on optional features in core functionality
- ❌ Feature flags that change API semantics (only performance/backend)
- ❌ Undocumented feature requirements in examples
- ❌ Test-only features that affect library behavior
- ❌ Circular feature dependencies
- ❌ Features that can't be combined

## Notes for AI Agent

- Start with the feature detection utilities to establish patterns
- Use `cfg_attr` for conditional derives and attributes
- Test incrementally - fix one example at a time
- Use `cargo expand` to verify macro expansions
- Consider using `cfg-if` crate for complex conditions
- Remember that no_std still allows core and alloc
- Feature names should be descriptive and non-overlapping
- Always provide fallback implementations

## References for Implementation

- Cargo Features Best Practices: https://doc.rust-lang.org/cargo/reference/features.html
- no_std Rust Book: https://docs.rust-embedded.org/book/intro/no-std.html
- cargo-hack for testing: https://github.com/taiki-e/cargo-hack
- Example of good feature management: https://github.com/rust-num/num-traits

## Quality Score: 9/10

Comprehensive PRP with clear implementation path, extensive validation gates, and thorough context. Includes specific file modifications, testing strategy, and CI integration. Deducted one point as some edge cases in feature combinations might emerge during implementation that aren't fully anticipated.