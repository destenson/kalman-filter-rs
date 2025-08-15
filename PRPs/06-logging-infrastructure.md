# PRP: Add Logging Infrastructure

## Goal
Add comprehensive logging support using the `log` crate for library diagnostics and optional `tracing_subscriber` for examples, enabling debugging, performance monitoring, and numerical issue detection without impacting runtime performance when disabled.

## Why
- **Debugging Support**: Currently no visibility into filter operations
- **Numerical Diagnostics**: Track convergence, condition numbers, and stability issues
- **Performance Monitoring**: Log timing for expensive operations (matrix inversions)
- **Error Context**: Add context to errors before they propagate
- **User Control**: Library uses `log` facade, users choose implementation

## What
Implement structured logging that:
1. Uses `log` crate macros throughout the library
2. Provides optional `tracing_subscriber` setup for examples
3. Logs at appropriate levels (trace, debug, info, warn, error)
4. Includes numerical diagnostics (matrix condition, convergence)
5. Zero overhead when logging is disabled

### Success Criteria
- [ ] All major operations have debug-level logging
- [ ] Numerical warnings for near-singular matrices
- [ ] Performance traces for predict/update cycles
- [ ] Examples demonstrate logging setup
- [ ] No performance impact when RUST_LOG is not set
- [ ] Documentation explains logging configuration

## All Needed Context

### Logging References
```yaml
- url: https://docs.rs/log/latest/log/
  why: log crate documentation - the facade we'll use
  
- url: https://docs.rs/tracing-subscriber/latest/tracing_subscriber/
  why: tracing_subscriber for structured logging in examples
  
- url: https://rust-lang.github.io/rust-clippy/master/index.html#print_stdout
  why: Avoid println! in libraries, use log macros instead
  
- file: C:\Users\deste\repos\kalman_filter-0.1.2\Cargo.toml
  why: Dependencies already added - log = "0.4.27", tracing-subscriber optional
  
- file: C:\Users\deste\repos\kalman_filter-0.1.2\src\error.rs
  why: Error types that should trigger warnings/errors
  
- file: C:\Users\deste\repos\kalman_filter-0.1.2\src\filter.rs
  why: Core operations that need instrumentation
```

### Logging Patterns for Numerical Libraries
Based on best practices for scientific computing libraries:
- **TRACE**: Matrix operations, intermediate values
- **DEBUG**: Predict/update steps, state changes
- **INFO**: Filter initialization, major mode changes
- **WARN**: Near-singular matrices, numerical instability
- **ERROR**: Dimension mismatches, singular matrices

### Key Locations for Logging
1. **Filter Operations** (src/filter.rs, extended.rs, etc.):
   - Predict step entry/exit with state norm
   - Update step with innovation and gain
   - Matrix inversions with condition number

2. **Error Conditions** (all modules):
   - Before returning Err() to add context
   - When detecting numerical issues

3. **Initialization** (builder.rs):
   - Log filter configuration
   - Validate and warn about parameters

4. **Convergence** (information/consensus.rs, distributed.rs):
   - Iteration count and convergence metrics
   - Network communication in distributed filter

### Performance Considerations
- Use `log::log_enabled!()` to skip expensive formatting
- Compute diagnostics only when logging is active
- Use closures for lazy evaluation of log messages

### Known Gotchas
- Libraries should use `log`, not `println!` or `eprintln!`
- Don't log in hot loops without level checking
- Matrix printing can be expensive - use lazy evaluation
- Avoid logging sensitive data (though not applicable here)

## Implementation Blueprint

### Task List (in order)

1. **Setup logging infrastructure**
   - Add `use log::{trace, debug, info, warn, error};` to modules
   - Create logging utilities module for common patterns
   - Add feature flag guard for expensive diagnostics
   - Setup benchmarks to verify zero overhead

2. **Instrument core Kalman filter (src/filter.rs)**
   - Log initialization with dimensions and parameters
   - Debug log predict step with state norm
   - Debug log update step with innovation residual
   - Warn on near-singular covariance (determinant < epsilon)
   - Trace matrix operations when enabled

3. **Add logging to all filter variants**
   - Extended KF: Log linearization points and Jacobians
   - Unscented KF: Log sigma points generation
   - Information Filter: Log information matrix condition
   - Particle Filter: Log effective sample size
   - Ensemble KF: Log ensemble spread

4. **Instrument error paths**
   - Add context before each `Err()` return
   - Log dimension mismatches with actual values
   - Warn on numerical issues before they cause errors
   - Create error recovery suggestions in logs

5. **Add numerical diagnostics utilities**
   - Matrix condition number calculator
   - Covariance positive-definiteness checker
   - Innovation consistency checker
   - Convergence rate monitor

6. **Setup example logging**
   - Add tracing_subscriber initialization
   - Show different log levels in examples
   - Create debug example showing all diagnostics
   - Document RUST_LOG configuration

7. **Performance validation**
   - Benchmark with logging disabled
   - Benchmark with RUST_LOG=error
   - Benchmark with RUST_LOG=trace
   - Ensure < 1% overhead when disabled

### Logging Module Structure
```
src/
├── logging.rs          # Utilities and helpers
│   ├── matrix_fmt()    # Lazy matrix formatting
│   ├── condition_number() # Numerical diagnostics
│   └── log_state()    # State vector logging
└── [each filter module gets logging added inline]
```

### Example Logging Patterns
The implementation should follow these patterns:
- Entry/exit logging for major operations
- Conditional expensive diagnostics
- Structured data using key=value format
- Lazy evaluation for expensive computations

## Validation Gates

```bash
# Check no println! or eprintln! in library code
! grep -r "println!\|eprintln!" src/

# Verify log macros are used
grep -r "log::\|trace!\|debug!\|info!\|warn!\|error!" src/ | wc -l
# Should be > 50 occurrences

# Test with different log levels
RUST_LOG=off cargo test --release 2>&1 | grep -c "^\[" # Should be 0
RUST_LOG=error cargo test 2>&1 | grep "^\[ERROR" # Should show errors
RUST_LOG=debug cargo test 2>&1 | grep "^\[DEBUG" # Should show debug

# Run example with logging
RUST_LOG=debug cargo run --example simple_1d 2>&1 | head -20

# Run example with tracing
cargo run --example simple_1d --features tracing-subscriber

# Benchmark performance impact
cargo bench --bench logging_overhead # Create this benchmark

# Check documentation
cargo doc --open # Should document logging configuration

# Verify zero overhead
cargo build --release
objdump -t target/release/libkalman_filter.rlib | grep log
# Log symbols should be optimized out when not used
```

## Error Handling Strategy
- Log errors before returning them for context
- Include relevant state in error logs
- Suggest recovery actions in warning messages
- Use structured logging for machine parsing

## References for Implementation
- Logging best practices: https://rust-lang-nursery.github.io/rust-cookbook/development_tools/debugging/log.html
- Zero-cost logging: https://github.com/rust-lang/log#in-libraries
- Tracing examples: https://github.com/tokio-rs/tracing/tree/master/examples
- Scientific computing logging: https://github.com/rust-ndarray/ndarray/search?q=log%3A%3A

## Notes for AI Agent
- Start with core filter.rs as the template
- Use log_enabled! for expensive operations
- Keep messages concise but informative
- Include units in numerical values
- Use consistent key names across modules
- Test with RUST_LOG=off to ensure zero overhead
- Don't log in Clone, Drop, or Debug impls

## Quality Score: 8/10
Comprehensive logging plan with specific patterns and performance considerations. Clear task structure and validation gates. Deducted points for complexity of ensuring zero overhead and potential for over-logging in numerical computations. All necessary context provided for successful implementation.