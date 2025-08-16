# PRP-00: Constructor Builder Pattern Refactoring

## Summary
Refactor all filter constructors that currently return `Result<Self>` to use the builder pattern, separating fallible validation from infallible construction. This addresses the architectural issue where `new()` functions returning `Result` violates Rust conventions and creates partially initialized filter states.

## Motivation
The current codebase has several filters with `new()` methods that return `Result<Self>`, which is considered poor practice in Rust. The convention is that `new()` should be infallible, with validation deferred to a `build()` method in the builder pattern. KalmanFilter already follows this pattern successfully, but other filters need to be refactored for consistency.

## Current State Analysis

### Filters with Fallible new() Methods
Based on codebase analysis, these filters return `KalmanResult<Self>` from their constructors:
- `ExtendedKalmanFilter` (src/extended.rs:103)
- `UnscentedKalmanFilter` (src/unscented.rs:126) 
- `CubatureKalmanFilter` (src/scented.rs:73)
- `EnsembleKalmanFilter` (src/ensemble/filter.rs:69)
- `ExtendedInformationFilter` (src/information/extended.rs:40)

### Partially Modified Filters
The user has started modifying:
- `InformationFilter` (src/information/filter.rs) - parameters commented out, incomplete
- `ParticleFilter` (src/particle/filter.rs) - parameters commented out, incomplete

### Existing Builder Pattern
`KalmanFilter` already has a well-implemented builder in src/builder.rs that:
- Uses `KalmanFilterBuilder<T>` struct with Optional fields
- Has setter methods returning `self` for chaining
- Validates all requirements in `build()` method returning `KalmanResult<KalmanFilter<T>>`
- Calls `KalmanFilter::initialize()` which performs dimension validation

## Implementation Blueprint

### Pattern to Follow (from existing KalmanFilterBuilder)
1. Create builder struct with Optional fields for all parameters
2. Implement `new()` on builder taking only required dimensions  
3. Add setter methods for each parameter returning `self`
4. Implement `build()` method that validates and constructs the filter
5. Modify filter's `new()` to be infallible (taking all validated params)
6. Keep or create an `initialize()` method for the actual validation logic

### Architectural Approach

#### For Filters with NonlinearSystem Trait
ExtendedKalmanFilter, UnscentedKalmanFilter, CubatureKalmanFilter, EnsembleKalmanFilter all take a `system: S where S: NonlinearSystem<T>` parameter. Their builders should:
- Store the system in the builder
- Extract dimensions from `system.state_dim()` and `system.measurement_dim()`
- Validate that provided matrices match system dimensions

#### For Information Filters
Information filters have dual representation (Y=P^-1, y=Y*x). Builders should:
- Allow initialization from either information form (Y,y) or covariance form (P,x)
- Provide conversion methods in the builder
- Handle sparse matrix initialization for SparseInformationFilter

#### For Particle Filter
Currently incomplete implementation. Builder should:
- Handle particle initialization strategies
- Configure resampling methods
- Set particle count and state dimensions

## Detailed Tasks

### Phase 1: Create Builder Infrastructure
1. Create src/builders/ module directory
2. Create builder files for each filter type:
   - extended_builder.rs
   - unscented_builder.rs
   - scented_builder.rs (for CubatureKalmanFilter)
   - ensemble_builder.rs
   - information_builder.rs
   - particle_builder.rs
3. Add mod.rs to export all builders

### Phase 2: Implement ExtendedKalmanFilterBuilder
1. Create struct with fields: system, initial_state, initial_covariance, process_noise, measurement_noise, dt
2. Implement new(system) constructor
3. Add setter methods for each parameter
4. Implement build() with dimension validation from src/extended.rs:110-145
5. Modify ExtendedKalmanFilter::new() to take all params and return Self
6. Create ExtendedKalmanFilter::initialize() for validated construction

### Phase 3: Implement UnscentedKalmanFilterBuilder
1. Follow same pattern as ExtendedKalmanFilterBuilder
2. Add UKF-specific parameters: alpha, beta, kappa
3. Move validation logic from src/unscented.rs:134-160 to build()

### Phase 4: Implement CubatureKalmanFilterBuilder
1. Follow same pattern, using src/scented.rs as reference
2. Handle cubature point generation parameters

### Phase 5: Implement EnsembleKalmanFilterBuilder
1. Add ensemble_size parameter
2. Handle initial_spread vs initial_covariance
3. Move validation from src/ensemble/filter.rs:77-100

### Phase 6: Implement InformationFilterBuilder
1. Support dual initialization modes (information vs covariance)
2. Add conversion methods: from_covariance(), from_information()
3. Fix the incomplete changes in src/information/filter.rs

### Phase 7: Implement ParticleFilterBuilder
1. Complete the partial implementation
2. Add particle initialization strategies
3. Configure resampling methods

### Phase 8: Update Examples and Tests
1. Update all examples to use builders
2. Update all tests to use builders
3. Add builder-specific tests

### Phase 9: Documentation
1. Update module documentation for each filter
2. Add builder examples to each filter's doc comments
3. Update CLAUDE.md with new builder patterns

## External References

### Rust Builder Pattern Best Practices
- Official Rust patterns book: https://rust-unofficial.github.io/patterns/patterns/creational/builder.html
- Constructor conventions: https://rust-unofficial.github.io/patterns/idioms/ctor.html
- std::process::Command as builder example: https://doc.rust-lang.org/std/process/struct.Command.html

### Error Handling Conventions
- Idiomatic error handling discussion: https://users.rust-lang.org/t/idiomatic-error-handling-in-new-and-builder-pattern/86361
- Result type documentation: https://doc.rust-lang.org/std/result/

### Similar Implementations
- nalgebra builders: https://docs.rs/nalgebra/latest/nalgebra/base/struct.MatrixBuilder.html
- ndarray builders: https://docs.rs/ndarray/latest/ndarray/struct.ArrayBuilder.html

## Validation Gates

```bash
# Check that no new() methods return Result (except in tests)
! cargo clippy --all-features 2>&1 | grep -E "fn new.*-> .*Result"

# Ensure all builders follow the pattern
cargo test --all-features -- builder

# Verify examples compile with new builders
cargo build --examples --all-features

# Run all tests to ensure functionality preserved
cargo test --all-features

# Check documentation builds correctly
cargo doc --all-features --no-deps

# Lint and format checks
cargo fmt --check && cargo clippy --all-features -- -D warnings
```

## Migration Strategy

### Backward Compatibility
1. Keep existing constructors during transition (mark as deprecated)
2. Add #[deprecated] attribute pointing to builder
3. Remove deprecated constructors in next major version

### Incremental Rollout
1. Start with ExtendedKalmanFilter as proof of concept
2. Get user feedback on API design
3. Roll out to remaining filters
4. Update all documentation and examples

## Success Criteria
- All filter constructors are infallible (new() returns Self)
- All validation happens in builder's build() method
- Consistent API across all filter types
- No loss of functionality or safety
- All tests pass without modification (except constructor calls)

## Risks and Mitigations
- **Risk**: Breaking API changes for users
  - **Mitigation**: Provide migration guide, keep deprecated methods temporarily
- **Risk**: Increased code complexity with builders
  - **Mitigation**: Use macro to reduce boilerplate if patterns emerge
- **Risk**: Performance overhead from builder pattern
  - **Mitigation**: Builders are zero-cost abstractions in Rust

## Implementation Notes
- Revert the incomplete changes in src/information/filter.rs and src/particle/filter.rs before starting
- Consider using a macro to generate similar builders if patterns are repetitive
- Ensure all builders implement Debug and Clone traits
- Consider implementing Default for builders where sensible

## Quality Score: 8/10
High confidence in successful implementation due to:
- Existing working builder pattern to follow (KalmanFilterBuilder)
- Clear validation logic to migrate from existing constructors
- Well-defined Rust conventions and best practices
- Comprehensive task breakdown with clear dependencies

Points deducted for:
- Large scope requiring multiple builder implementations
- Some filters (ParticleFilter) have incomplete implementations that need design decisions
