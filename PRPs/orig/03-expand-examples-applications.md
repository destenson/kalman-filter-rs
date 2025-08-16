# PRP: Expand Kalman Filter Examples with Real-World Applications

## Goal
Create comprehensive, practical examples demonstrating real-world applications of the Kalman filter including battery state estimation, sensor fusion, financial market prediction, temperature monitoring, and object tracking. Each example should be self-contained, well-documented, and demonstrate best practices.

## Why
- **Educational Value**: Current examples only show basic 1D and 2D position tracking; users need practical applications
- **Integration Showcase**: Demonstrate how Kalman filters integrate with the broader gai-rs ecosystem (markets, physics, sensors)
- **Industry Relevance**: Cover applications in robotics, finance, IoT, and energy management that users actively search for
- **Adoption Driver**: Quality examples are the primary way developers evaluate and learn new libraries

## What
Create 5-7 new examples covering diverse domains with realistic scenarios, noise models, and performance metrics.

### Success Criteria
- [ ] At least 5 new examples covering different domains
- [ ] Each example includes realistic noise models and parameters
- [ ] Examples demonstrate integration with other gai-rs crates where applicable
- [ ] Comprehensive documentation explaining the physics/math behind each application
- [ ] Performance metrics and comparison with ground truth
- [ ] Examples compile and run with clear output

## All Needed Context

### Documentation & References
```yaml
- url: https://github.com/AlterWL/Battery_SOC_Estimation
  why: MATLAB battery SOC estimation using EKF - reference for battery model equations
  
- url: https://github.com/Janudis/Extended-Kalman-Filter-GPS_IMU
  why: GPS/IMU fusion implementation showing state vector design and noise parameters
  
- url: https://github.com/jogrady23/kalman-filter-battery-soc/blob/master/kalman_filter_operation.py
  why: Python implementation of battery Kalman filter with practical parameters
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\examples\simple_1d.rs
  why: Existing example pattern to follow - structure and documentation style
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\examples\position_2d.rs
  why: 2D tracking example showing multi-dimensional state handling
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-markets\examples\alpaca_check_acct.rs
  why: Pattern for feature-gated examples and async integration

- url: https://www.kalmanfilter.net/
  why: Mathematical reference for Kalman filter equations and applications
  
- url: https://en.wikipedia.org/wiki/Kalman_filter#Applications
  why: Comprehensive list of real-world applications with technical details
```

### Current Codebase Structure
```bash
gai-rs-kalman/
├── examples/
│   ├── simple_1d.rs       # Basic 1D position tracking
│   └── position_2d.rs      # 2D position with velocity
```

### Desired Codebase Structure
```bash
gai-rs-kalman/
├── examples/
│   ├── simple_1d.rs               # Keep existing
│   ├── position_2d.rs             # Keep existing
│   ├── battery_soc.rs             # Battery state of charge estimation
│   ├── imu_gps_fusion.rs          # 6DOF IMU + GPS sensor fusion
│   ├── market_prediction.rs       # Financial time series prediction
│   ├── temperature_fusion.rs      # Multi-sensor temperature monitoring
│   ├── object_tracking.rs         # Video object tracking with occlusion
│   ├── robot_localization.rs      # Robot SLAM with landmarks
│   └── benchmark_comparison.rs    # Performance comparison across scenarios
└── examples_data/                 # Data files for examples
    ├── battery_discharge.csv      # Battery test data
    ├── imu_gps_log.csv            # Sensor fusion test data
    └── market_prices.csv          # Financial time series
```

### Known Gotchas & Library Quirks
- CRITICAL: Kalman filter requires positive semi-definite covariance matrices
- CRITICAL: Process noise Q and measurement noise R must be tuned for each application
- CRITICAL: Battery models are nonlinear - may need EKF in future
- CRITICAL: Financial data is non-Gaussian - Kalman assumptions may be violated
- CRITICAL: IMU has bias drift that needs separate estimation
- CRITICAL: Examples should work without external data files (generate synthetic data)

## Implementation Blueprint

### Example Structure Pattern
Each example should follow this structure:
- Application-specific Kalman filter example header
- Documentation explaining the specific application
- Real-world use case description
- Theory section with mathematical model explanation
- State vector documentation listing each state component
- Measurements documentation listing what is observed
- Main function structure:
  - Clear title and separator
  - Model parameters with explanatory comments
  - Synthetic data generation
  - Filter initialization
  - Simulation loop with metrics
  - Results visualization/summary
- Helper functions for data generation
- Helper functions for metrics calculation

### Task List

```yaml
Task 1: Create battery SOC estimation example
CREATE examples/battery_soc.rs:
  - PATTERN: Follow simple_1d.rs structure
  - State: [SOC, voltage, current]
  - Model: Equivalent circuit battery model
  - Include: Coulomb counting comparison
  - Noise: Realistic current sensor noise

Task 2: Create IMU/GPS sensor fusion example
CREATE examples/imu_gps_fusion.rs:
  - State: [x, y, z, vx, vy, vz, ax, ay, az]
  - Measurements: GPS position (1Hz), IMU acceleration (100Hz)
  - Handle: Different sensor rates
  - Include: Bias estimation for gyroscope

Task 3: Create market prediction example
CREATE examples/market_prediction.rs:
  - INTEGRATE: Use gai-rs-markets types if available
  - State: [price, momentum, volatility]
  - Model: Mean-reverting price process
  - Include: Comparison with moving average

Task 4: Create temperature sensor fusion example
CREATE examples/temperature_fusion.rs:
  - Multiple sensors with different accuracies
  - State: [true_temperature, sensor_biases]
  - Handle: Sensor failures/dropouts
  - Include: Weighted average comparison

Task 5: Create object tracking example
CREATE examples/object_tracking.rs:
  - State: [x, y, vx, vy, width, height]
  - Measurements: Bounding box from detector
  - Handle: Occlusions and missing detections
  - Include: Constant velocity vs acceleration models

Task 6: Create robot localization example
CREATE examples/robot_localization.rs:
  - State: [x, y, theta, v, omega]
  - Measurements: Landmark bearings and ranges
  - Model: Differential drive robot
  - Include: Dead reckoning comparison

Task 7: Create benchmark comparison example
CREATE examples/benchmark_comparison.rs:
  - Run multiple scenarios
  - Compare: Execution time, accuracy, convergence
  - Output: Performance table
  - Test: Different matrix sizes

Task 8: Create example data directory
CREATE examples_data/:
  - Add README explaining data formats
  - Include sample CSV generators
  - Document data sources

Task 9: Update examples README
CREATE examples/README.md:
  - Overview of all examples
  - Quick start guide
  - Theory references
  - Performance expectations

Task 10: Add integration tests
MODIFY Cargo.toml:
  - Add dev-dependencies for CSV parsing
  - Add feature flags for domain-specific examples
CREATE tests/examples_test.rs:
  - Test each example runs successfully
  - Verify output format
```

### Integration Points
```yaml
CARGO:
  - add to: Cargo.toml
  - dev-dependencies:
    csv = "1.3"           # For data file parsing
    rand_distr = "0.4"    # For realistic noise generation
    
  - features:
    example-markets = ["gai-rs-markets"]  # Optional integration
    example-physics = ["gai-rs-physics"]  # Optional integration
    all-examples = ["example-markets", "example-physics"]

DOCUMENTATION:
  - add to: README.md
  - Section: "## Examples"
  - Link to examples/README.md
  - Highlight key applications

CI/CD:
  - add to: .github/workflows/test.yml
  - Run all examples in CI
  - Check examples compile with --all-features
```

## Validation Loop

### Level 1: Syntax & Style
```bash
# Check all examples compile
for example in battery_soc imu_gps_fusion market_prediction temperature_fusion object_tracking robot_localization benchmark_comparison; do
  cargo build --example $example
done

# Lint all examples
cargo clippy --examples -- -D warnings
cargo fmt --check -- examples/*.rs
```

### Level 2: Example Tests
Test framework for validating example outputs:
- Create test module for example validation
- Test each example can be executed successfully
- Capture and verify output contains expected metrics
- For battery_soc: verify SOC estimate and RMSE present
- For imu_gps_fusion: verify position and velocity estimates
- For market_prediction: verify price predictions and error metrics
- For temperature_fusion: verify fused temperature and individual sensor readings
- For object_tracking: verify tracked positions and velocities
- For robot_localization: verify position, orientation estimates
- For benchmark_comparison: verify performance table output

### Level 3: Integration Tests
```bash
# Run all examples and verify completion
cargo test --test examples_test

# Run with all features
cargo run --example benchmark_comparison --all-features

# Check performance benchmarks
cargo run --example benchmark_comparison --release
```

### Level 4: Documentation Build
```bash
# Ensure all examples are documented
cargo doc --examples --no-deps

# Check for broken links in example docs
cargo doc --examples --open
```

## Final Validation Checklist
- [ ] All 7+ examples compile: `cargo build --examples`
- [ ] No clippy warnings: `cargo clippy --examples -- -D warnings`
- [ ] Examples produce meaningful output with metrics
- [ ] Each example has comprehensive documentation header
- [ ] Examples work without external data files
- [ ] Performance benchmark shows expected results
- [ ] README.md updated with example descriptions
- [ ] Integration with other gai-rs crates works (where applicable)

## Anti-Patterns to Avoid
- ❌ Don't require external data files for basic functionality
- ❌ Don't use unrealistic noise models - research actual sensor specs
- ❌ Don't ignore numerical stability - check covariance remains positive
- ❌ Don't assume specific random seeds - examples should work with any seed
- ❌ Don't create overly complex examples - each should focus on one application
- ❌ Don't skip performance metrics - users need to evaluate efficiency

## Error Handling Strategy
- Generate synthetic data if files not found
- Provide clear error messages for matrix singularities
- Handle sensor dropouts gracefully
- Warn when filter diverges (covariance grows unbounded)
- Include recovery strategies in examples

## References for Implementation
- Battery Model: https://www.mathworks.com/help/autoblks/ug/lithium-battery-cell-two-rc-branch-equivalent-circuit.html
- IMU/GPS Fusion: https://github.com/ethz-asl/ethzasl_msf
- Market Models: https://en.wikipedia.org/wiki/Ornstein%E2%80%93Uhlenbeck_process
- Temperature Fusion: https://www.ti.com/lit/an/snoa967/snoa967.pdf
- Object Tracking: https://github.com/abewley/sort
- Robot Localization: https://github.com/AtsushiSakai/PythonRobotics

## Notes for AI Agent
- Start with battery_soc.rs as it's most straightforward to implement
- Use rand and rand_distr for realistic noise generation
- Keep examples under 300 lines for readability
- Include ASCII art diagrams where helpful (state transitions, sensor layout)
- Add performance timing using std::time::Instant
- Consider using const generics for compile-time dimension checking where applicable
- Each example should be runnable in under 1 second
- Include links to papers/references in documentation comments
- Test with both f32 and f64 to ensure numerical stability

## Quality Score: 9/10
Comprehensive PRP with detailed implementation plan, real-world applications backed by research, clear validation criteria, and integration with existing codebase patterns. Includes specific references to external implementations and mathematical models. Deducted one point as some applications (like market prediction) may need adjustment based on actual gai-rs-markets API availability.