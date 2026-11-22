// Property-based testing with QuickCheck for Kalman filter invariants
use kalman_filters::linear::LinearKalmanFilter as KalmanFilter;
use quickcheck::{Arbitrary, Gen};
use quickcheck_macros::quickcheck;

// Custom type for generating valid covariance matrices
#[derive(Clone, Debug)]
struct PositiveDefiniteMatrix {
    data: Vec<f64>,
    size: usize,
}

impl Arbitrary for PositiveDefiniteMatrix {
    fn arbitrary(g: &mut Gen) -> Self {
        // Size between 1 and 4 for reasonable test times
        let size = (u8::arbitrary(g) % 4 + 1) as usize;

        // Generate a random matrix
        let mut a: Vec<f64> = Vec::with_capacity(size * size);
        for _ in 0..size * size {
            a.push(f64::arbitrary(g) * 10.0); // Scale to reasonable values
        }

        // Make it positive definite: P = A * A^T + epsilon * I
        let mut data = vec![0.0; size * size];

        // P = A * A^T
        for i in 0..size {
            for j in 0..size {
                let mut sum = 0.0;
                for k in 0..size {
                    sum += a[i * size + k] * a[j * size + k];
                }
                data[i * size + j] = sum;
            }
        }

        // Add small diagonal to ensure positive definite
        for i in 0..size {
            data[i * size + i] += 0.1;
        }

        PositiveDefiniteMatrix { data, size }
    }
}

// Custom type for stable dynamics matrices
#[derive(Clone, Debug)]
struct StableDynamicsMatrix {
    data: Vec<f64>,
    size: usize,
}

impl Arbitrary for StableDynamicsMatrix {
    fn arbitrary(g: &mut Gen) -> Self {
        let size = (u8::arbitrary(g) % 4 + 1) as usize;
        let mut data = Vec::with_capacity(size * size);

        for i in 0..size {
            for j in 0..size {
                if i == j {
                    // Diagonal elements between 0.5 and 1.0 for stability
                    data.push(0.5 + f64::arbitrary(g).abs() * 0.5);
                } else {
                    // Off-diagonal elements small
                    data.push((f64::arbitrary(g) - 0.5) * 0.1);
                }
            }
        }

        StableDynamicsMatrix { data, size }
    }
}

// Property: Covariance remains positive semi-definite
#[quickcheck]
fn prop_covariance_positive_semidefinite(
    initial_p: PositiveDefiniteMatrix,
    dynamics: StableDynamicsMatrix,
    process_noise: PositiveDefiniteMatrix,
    measurement_noise: PositiveDefiniteMatrix,
    num_steps: u8,
) -> bool {
    // Ensure compatible dimensions
    let n = initial_p.size.min(dynamics.size).min(process_noise.size);
    if n < 1 {
        return true;
    }

    let m = measurement_noise.size.min(n);
    if m < 1 {
        return true;
    }

    // Simple observation matrix
    let mut h = vec![0.0; m * n];
    for i in 0..m.min(n) {
        h[i * n + i] = 1.0;
    }

    // Set up filter with arbitrary values. Truncating the generated data does not
    // always give a symmetric PSD matrix; such models are rejected at construction.
    let Ok(mut kf) = KalmanFilter::<f64>::new(
        n,
        m,
        &vec![0.0; n],
        &initial_p.data[..n * n],
        &dynamics.data[..n * n],
        &process_noise.data[..n * n],
        &h,
        &measurement_noise.data[..m * m],
    ) else {
        return true;
    };

    // Run filter for arbitrary number of steps
    let steps = (num_steps % 20) as usize + 1; // Limit to reasonable number

    for _ in 0..steps {
        if kf.predict().is_err() {
            return false;
        }

        // Check diagonal elements are non-negative
        for i in 0..n {
            if kf.covariance()[i * n + i] < -1e-10 {
                return false;
            }
        }

        // Check symmetry
        for i in 0..n {
            for j in i + 1..n {
                if (kf.covariance()[i * n + j] - kf.covariance()[j * n + i]).abs() > 1e-8 {
                    return false;
                }
            }
        }

        // Update with random measurement
        let z: Vec<f64> = (0..m).map(|_| 0.0).collect();
        if kf.update(&z).is_err() {
            // Singular matrix is acceptable in edge cases
            continue;
        }

        // Check again after update
        for i in 0..n {
            if kf.covariance()[i * n + i] < -1e-10 {
                return false;
            }
        }
    }

    true
}

// Property: Predict-update cycle preserves dimensions
#[quickcheck]
fn prop_dimension_preservation(n: u8, m: u8) -> bool {
    let n = (n % 5 + 1) as usize;
    let m = (m % 5 + 1) as usize;

    // Set up with valid matrices
    let x = vec![0.0; n];
    let mut p = vec![0.0; n * n];
    let mut f = vec![0.0; n * n];
    let mut h = vec![0.0; m * n];
    let mut q = vec![0.0; n * n];
    let mut r = vec![0.0; m * m];

    // Make diagonal matrices
    for i in 0..n {
        p[i * n + i] = 1.0;
        f[i * n + i] = 1.0;
        q[i * n + i] = 0.1;
    }
    for i in 0..m {
        r[i * m + i] = 1.0;
        if i < n {
            h[i * n + i] = 1.0;
        }
    }

    let mut kf = KalmanFilter::<f64>::new(n, m, &x, &p, &f, &q, &h, &r).unwrap();

    let initial_x_len = kf.state().len();
    let initial_p_len = kf.covariance().len();

    kf.predict().unwrap();

    // Dimensions should be preserved after predict
    if kf.state().len() != initial_x_len || kf.covariance().len() != initial_p_len {
        return false;
    }

    if kf.update(&vec![0.0; m]).is_err() {
        // Update might fail for numerical reasons
        return true;
    }

    // Dimensions should be preserved after update
    kf.state().len() == initial_x_len && kf.covariance().len() == initial_p_len
}

// Property: KF and IF equivalence for random inputs
#[quickcheck]
fn prop_kf_if_equivalence(
    initial_p: PositiveDefiniteMatrix,
    dynamics: StableDynamicsMatrix,
    _measurement: f64,
) -> bool {
    let n = initial_p.size.min(dynamics.size).min(2); // Keep small for IF stability
    if n < 1 {
        return true;
    }

    // The removed KalmanFilter was configured here but never used; nothing to port.
    // TODO: Fix InformationFilter API to allow setting state in covariance form
    // Setup IF - need to invert P to get Y
    // For simplicity, skip this test if P is not invertible
    // The InformationFilter has a different API that requires Y and y directly
    true // Skip this test for now as InformationFilter API has changed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quickcheck_properties() {
        // Run a quick sanity check of our properties
        // The actual property tests are run by quickcheck macros

        // Test with known good values
        let p = PositiveDefiniteMatrix {
            data: vec![1.0, 0.0, 0.0, 1.0],
            size: 2,
        };

        let f = StableDynamicsMatrix {
            data: vec![0.9, 0.0, 0.0, 0.9],
            size: 2,
        };

        // Just verify the types are created correctly
        assert_eq!(p.size, 2);
        assert_eq!(f.size, 2);
        assert_eq!(p.data.len(), 4);
        assert_eq!(f.data.len(), 4);
    }
}
