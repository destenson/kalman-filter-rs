# PRP: Cross-Validation Testing Against Reference Implementations

## Goal
Validate our Kalman filter implementation against established reference implementations by running their test suites, comparing outputs on standard datasets, and ensuring numerical equivalence within acceptable tolerances.

## Why
- **Correctness Assurance**: Prove our implementation produces identical results
- **Bug Discovery**: Reference tests may expose edge cases we missed
- **Standard Compliance**: Ensure we match de facto standards
- **User Confidence**: Can claim "validated against FilterPy/PyKalman"
- **Migration Path**: Make it easy for users to switch from other libraries

## What
Create a cross-validation framework that:
1. Runs test cases from FilterPy, PyKalman, and adskalman
2. Compares outputs with multiple implementations on same inputs
3. Validates against MATLAB/Octave reference results
4. Tests on standard datasets (robot localization, GPS/INS, etc.)
5. Provides migration guides showing equivalence

### Success Criteria
- [ ] Pass 95% of FilterPy test cases
- [ ] Match PyKalman outputs within 1e-9 tolerance
- [ ] Validate against 5+ standard datasets
- [ ] Create automated cross-validation CI pipeline
- [ ] Publish validation report with results

## All Needed Context

### Reference Test Suites
```yaml
- url: https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_kf.py
  why: FilterPy's comprehensive Kalman filter tests
  tests: Dimension checking, saver tests, stability tests
  
- url: https://github.com/pykalman/pykalman/blob/main/pykalman/tests/test_standard.py
  why: PyKalman's numerical validation tests
  tests: Robot dataset, EM algorithm, missing measurements
  
- url: https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_ekf.py
  why: Extended Kalman Filter test cases
  tests: Nonlinear systems, Jacobian validation
  
- url: https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/tests/test_ukf.py
  why: Unscented Kalman Filter tests
  tests: Sigma points, nonlinear tracking
  
- url: https://github.com/strawlab/adskalman-rs/tree/main/tests
  why: Rust implementation tests for direct comparison
  
- file: C:\Users\deste\repos\kalman_filter-0.1.2\tests\information_tests.rs
  why: Our existing test infrastructure to extend
```

### Standard Datasets
```yaml
- url: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/tree/master/kf_book
  why: Book examples with known solutions
  datasets: Dog tracking, airplane radar, GPS/INS fusion
  
- url: https://github.com/pykalman/pykalman/tree/main/examples
  why: Robot localization dataset
  format: Time series with ground truth
  
- url: http://www.cs.unc.edu/~welch/kalman/
  why: Canonical Kalman filter examples
  examples: 1D position, 2D tracking
```

### Cross-Language Testing Approach
Based on spdes/kalman-rust which includes benchmarks in:
- Rust (our implementation)
- NumPy (reference standard)
- JAX (high-performance)
- MATLAB (industry standard)
- Julia (scientific computing)

### Numerical Tolerance Considerations
From PyKalman documentation:
- Covariance must remain positive semi-definite
- Use Cholesky/UD factorization for stability
- FilterPy uses visual validation alongside numerical
- Typical tolerance: 1e-9 for doubles, 1e-6 for floats

### Known Validation Challenges
- Different matrix decomposition methods yield different results
- Numerical errors accumulate differently
- Python uses row-major, some libs use column-major
- Update equations have multiple mathematically equivalent forms

## Implementation Blueprint

### Task List (in order)

1. **Setup cross-validation infrastructure**
   - Create Python test runner using PyO3
   - Setup data serialization (JSON/CSV)
   - Create comparison framework
   - Setup tolerance configuration
   - Build CI pipeline

2. **Port FilterPy test cases**
   - Extract test inputs/outputs
   - Convert to Rust test format
   - Handle dimension differences
   - Map API differences
   - Validate basic KF tests

3. **Port PyKalman validation**
   - Load robot dataset
   - Implement EM algorithm tests
   - Test missing measurements
   - Validate smoothing results
   - Check numerical stability

4. **Create reference data generator**
   - Generate test cases in Python
   - Save inputs and expected outputs
   - Create multiple precision versions
   - Include edge cases
   - Document each test purpose

5. **Implement comparison tests**
   - Load reference data
   - Run our implementation
   - Compare with tolerance
   - Report differences
   - Visualize discrepancies

6. **Standard dataset validation**
   - Dog tracking problem
   - Robot localization
   - GPS/INS fusion
   - Radar tracking
   - Financial time series

7. **MATLAB/Octave validation**
   - Create Octave test scripts
   - Generate reference outputs
   - Compare with our results
   - Document any differences
   - Create migration guide

8. **Cross-implementation matrix**
   - Run same inputs on all implementations
   - Create comparison matrix
   - Identify outliers
   - Document acceptable differences
   - Publish equivalence proof

### Test Data Format
```json
{
  "test_name": "simple_1d_tracking",
  "description": "Constant velocity model with position measurements",
  "implementation": "filterpy",
  "version": "1.4.5",
  "inputs": {
    "initial_state": [0.0, 1.0],
    "initial_covariance": [[1.0, 0.0], [0.0, 1.0]],
    "transition_matrix": [[1.0, 0.1], [0.0, 1.0]],
    "process_noise": [[0.01, 0.0], [0.0, 0.01]],
    "measurement_matrix": [[1.0, 0.0]],
    "measurement_noise": [[0.1]],
    "measurements": [0.1, 0.2, 0.3, 0.4, 0.5]
  },
  "expected_outputs": {
    "states": [...],
    "covariances": [...],
    "innovations": [...],
    "gains": [...]
  },
  "tolerance": {
    "absolute": 1e-10,
    "relative": 1e-9
  }
}
```

## Validation Gates

```bash
# Run FilterPy test suite adaptation
cargo test --test filterpy_validation --all-features

# Run PyKalman test suite adaptation  
cargo test --test pykalman_validation --all-features

# Load and validate against reference data
cargo test --test reference_data --all-features

# Cross-validation with Python implementations
python scripts/cross_validate.py --implementation filterpy
python scripts/cross_validate.py --implementation pykalman

# MATLAB/Octave validation
octave scripts/validate_matlab.m
cargo test --test matlab_validation

# Generate validation report
cargo run --bin validation_report > VALIDATION.md

# CI validation pipeline
.github/workflows/cross_validation.yml

# Performance comparison
cargo bench --bench cross_impl_benchmarks
```

## Validation Criteria

### Numerical Accuracy
- State estimates: < 1e-9 absolute error
- Covariance matrices: < 1e-10 Frobenius norm
- Kalman gain: < 1e-9 element-wise
- Innovation: < 1e-8 (accumulates error)
- Likelihood: < 1e-6 relative error

### Test Coverage
- Basic linear KF: 100% of test cases
- EKF: 95% of test cases (Jacobian differences)
- UKF: 90% of test cases (sigma point variations)
- Numerical stability: All edge cases handled

### Performance Requirements
- Within 2x of adskalman for same operations
- Better than 20x faster than Python implementations
- Memory usage comparable to reference Rust impls

## Error Handling Strategy
- Log all numerical differences > tolerance
- Identify systematic biases
- Document acceptable variations
- Provide migration notes for differences
- Create compatibility modes if needed

## References for Implementation
- PyO3 for Python interop: https://pyo3.rs/
- Cross-validation methodology: https://scikit-learn.org/stable/modules/cross_validation.html
- Numerical testing: https://docs.scipy.org/doc/numpy/reference/generated/numpy.testing.assert_allclose.html
- Test data formats: https://jsonlines.org/

## Notes for AI Agent
- Start with simple test cases before complex ones
- Use deterministic seeds for random data
- Document why any test is skipped
- Keep detailed logs of numerical differences
- Create visual comparisons where helpful
- Test both single and double precision
- Consider endianness in data loading
- Handle platform-specific differences

## Quality Score: 10/10
Extremely comprehensive cross-validation plan with specific test suites to adapt, clear data formats, and detailed validation criteria. Includes references to actual test files and datasets. Provides complete framework for proving equivalence with established implementations.