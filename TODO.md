# TODO List for kalman_filters

Last updated: 2025-08-27

## Critical (Blocking v0.9.0/v1.0.0 Release)

### Crate Publication
- [ ] **Prepare for crates.io publication** (New PRPs: 13-17)
  - [ ] Create LICENSE file with MIT text (PRP-14)
  - [ ] Update Cargo.toml with `readme = "README.md"` field (PRP-14)
  - [ ] Update README.md badges/references to use `kalman_filters` name (PRP-15)
  - [ ] Fix feature flag propagation to dependencies (PRP-17)
  - [ ] Run `cargo publish --dry-run` to verify

### Error Handling
- [ ] **Replace 300+ .unwrap() calls in non-test code** - Major panic risk
  - Files with most unwraps: unscented.rs (32), particle/filter.rs (21), information/distributed.rs (17)
  - Use proper error propagation with `?` operator
  - Add context to errors before propagating

## High Priority

### Testing & Validation
- [ ] **Implement comprehensive test coverage** (PRP-12)
  - Current coverage needs improvement
  - Add unit tests for all public APIs
  - Implement property-based testing with quickcheck
  
- [ ] **Fix InformationFilter API test** (`tests/quickcheck_invariants.rs:226`)
  - Currently skipped - "Skip this test for now as InformationFilter API has changed"
  - Need to update test to work with new Y/y parameter API

- [ ] **Complete cross-validation testing** (PRP-05)
  - Validate against FilterPy/PyKalman reference implementations
  - Adapt test suites from established libraries
  - Prove mathematical equivalence

### Performance
- [ ] **Implement performance benchmarks** (PRP-02)
  - File `benches/kalman_benchmarks.rs` exists but appears incomplete
  - Add criterion benchmarks for all filter variants
  - Benchmark matrix operations (main bottleneck)
  - Compare with adskalman reference implementation

### Infrastructure
- [ ] **Set up CI/CD pipeline**
  - Add GitHub Actions workflow
  - Run tests, clippy, rustfmt
  - Generate and publish documentation
  - Calculate and report test coverage

## Medium Priority

### Code Quality
- [ ] **Address 84 clippy warnings** (mentioned in codebase-review-report.md)
  - Most are style issues
  - Some may indicate potential bugs

- [ ] **Review underscore-prefixed parameters**
  - Many `_control`, `_state`, `_dt` parameters in trait implementations
  - Determine if these should be used or if API needs redesign

### Features & Enhancements
- [ ] **Add logging infrastructure** (PRP-06)
  - Structured logging with log/tracing
  - Debug matrix operations
  - Performance metrics logging

- [ ] **Implement Prometheus metrics** (PRP-07)
  - Export performance metrics
  - Track prediction/update cycles
  - Monitor numerical stability

- [ ] **Add no_std support** (PRP-08)
  - Make std optional
  - Support embedded systems
  - Careful feature gating

## Low Priority

### Performance Optimizations
- [ ] **SIMD vectorization** (PRP-09)
  - Use packed_simd for matrix operations
  - Significant performance gains possible

- [ ] **GPU acceleration** (PRP-10)
  - CUDA/OpenCL support for large-scale operations
  - Useful for particle filters with many particles

### Ecosystem
- [ ] **Hardware sensor plugins** (PRP-11)
  - Direct integration with IMUs, GPS, etc.
  - Real-time data streaming support

- [ ] **Python bindings** (PRP-99)
  - PyO3 bindings for Python interop
  - Enable use in ML/data science workflows

### Architecture
- [ ] **Complete constructor/builder refactor** (PRP-00)
  - Move validation logic to builders
  - Make constructors infallible
  - Improve API ergonomics

## Completed
✅ Basic filter implementations (all 7 variants)
✅ Builder pattern for filter construction
✅ Examples for each filter type
✅ Basic error types and handling

## Recent Changes (2025-08-27)
- **Version changed to 0.9.0-alpha0** in Cargo.toml (was 1.0.0-alpha1)
- **Crate name changed to `kalman_filters`** (plural) to avoid naming conflict
- **Edition changed to 2021** (was incorrectly set to 2024)
- **New PRPs added (13-17)** for crates.io publication and automation

## Notes

### From Code Comments  
- **TODO in tests/quickcheck_invariants.rs:222** - Fix InformationFilter API to allow setting state in covariance form
- **TODO in docs/SOTA.md:101** - Benchmarks section needs implementation
- Missing `_timer` metrics collection points throughout codebase
  - Variables created but unused (in filter.rs, unscented.rs, particle/filter.rs)
  - Suggests incomplete metrics implementation

### Validation Criteria References
- CLAUDE.md mentions validation against established algorithms
- PRPs contain detailed implementation requirements
- Cross-validation with Python libraries critical for trust
- docs/VALIDATION.md notes precision differences with FilterPy reference

### Underscore Parameters
- Many `_control`, `_state`, `_dt` parameters in NonlinearSystem trait implementations
- These are intentional for traits where control input is optional
