// Comprehensive tests for matrix operations
//
// These tests verify that all backends produce consistent results.

use super::*;
use approx::assert_relative_eq;

#[cfg(test)]
fn test_backend<B: MatrixBackend<f64>>() {
    test_matvec::<B>();
    test_matmul::<B>();
    test_matmul_transpose::<B>();
    test_transpose::<B>();
    test_cholesky::<B>();
    test_invert::<B>();
    test_symmetrize::<B>();
}

fn test_matvec<B: MatrixBackend<f64>>() {
    let a = vec![1.0, 2.0, 3.0, 4.0];
    let x = vec![5.0, 7.0];
    let mut y = vec![0.0; 2];

    B::matvec(&a, &x, &mut y, 2, 2);

    assert_relative_eq!(y[0], 19.0, epsilon = 1e-10);
    assert_relative_eq!(y[1], 43.0, epsilon = 1e-10);
}

fn test_matmul<B: MatrixBackend<f64>>() {
    let a = vec![1.0, 2.0, 3.0, 4.0];
    let b = vec![5.0, 6.0, 7.0, 8.0];
    let mut c = vec![0.0; 4];

    B::matmul(&a, &b, &mut c, 2, 2, 2);

    assert_relative_eq!(c[0], 19.0, epsilon = 1e-10);
    assert_relative_eq!(c[1], 22.0, epsilon = 1e-10);
    assert_relative_eq!(c[2], 43.0, epsilon = 1e-10);
    assert_relative_eq!(c[3], 50.0, epsilon = 1e-10);
}

fn test_matmul_transpose<B: MatrixBackend<f64>>() {
    // A = [1 2]  B = [5 7]
    //     [3 4]      [6 8]
    // A * B^T = [1 2] * [5 6]^T = [19 22]
    //           [3 4]   [7 8]     [43 50]
    let a = vec![1.0, 2.0, 3.0, 4.0];
    let b = vec![5.0, 7.0, 6.0, 8.0];
    let mut c = vec![0.0; 4];

    B::matmul_transpose(&a, &b, &mut c, 2, 2, 2);

    assert_relative_eq!(c[0], 19.0, epsilon = 1e-10);
    assert_relative_eq!(c[1], 22.0, epsilon = 1e-10);
    assert_relative_eq!(c[2], 43.0, epsilon = 1e-10);
    assert_relative_eq!(c[3], 50.0, epsilon = 1e-10);
}

fn test_transpose<B: MatrixBackend<f64>>() {
    let a = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    let mut b = vec![0.0; 6];

    B::transpose(&a, &mut b, 2, 3);

    assert_relative_eq!(b[0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(b[1], 4.0, epsilon = 1e-10);
    assert_relative_eq!(b[2], 2.0, epsilon = 1e-10);
    assert_relative_eq!(b[3], 5.0, epsilon = 1e-10);
    assert_relative_eq!(b[4], 3.0, epsilon = 1e-10);
    assert_relative_eq!(b[5], 6.0, epsilon = 1e-10);
}

fn test_cholesky<B: MatrixBackend<f64>>() {
    let p = vec![
        4.0, 12.0, -16.0,
        12.0, 37.0, -43.0,
        -16.0, -43.0, 98.0
    ];
    let mut l = vec![0.0; 9];

    let result = B::cholesky(&p, &mut l, 3);
    assert!(result.is_ok());

    assert_relative_eq!(l[0], 2.0, epsilon = 1e-10);
    assert_relative_eq!(l[3], 6.0, epsilon = 1e-10);
    assert_relative_eq!(l[4], 1.0, epsilon = 1e-10);
    assert_relative_eq!(l[6], -8.0, epsilon = 1e-10);
    assert_relative_eq!(l[7], 5.0, epsilon = 1e-10);
    assert_relative_eq!(l[8], 3.0, epsilon = 1e-10);
}

fn test_invert<B: MatrixBackend<f64>>() {
    let a = vec![4.0, 7.0, 2.0, 6.0];
    let mut a_inv = vec![0.0; 4];

    let result = B::invert(&a, &mut a_inv, 2);
    assert!(result.is_ok());

    assert_relative_eq!(a_inv[0], 0.6, epsilon = 1e-10);
    assert_relative_eq!(a_inv[1], -0.7, epsilon = 1e-10);
    assert_relative_eq!(a_inv[2], -0.2, epsilon = 1e-10);
    assert_relative_eq!(a_inv[3], 0.4, epsilon = 1e-10);
}

fn test_symmetrize<B: MatrixBackend<f64>>() {
    let mut a = vec![1.0, 2.0, 3.0, 4.0];
    B::symmetrize(&mut a, 2);

    assert_relative_eq!(a[0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(a[1], 2.5, epsilon = 1e-10);
    assert_relative_eq!(a[2], 2.5, epsilon = 1e-10);
    assert_relative_eq!(a[3], 4.0, epsilon = 1e-10);
}

#[cfg(feature = "naive-matrix")]
#[test]
fn test_naive_backend() {
    use super::naive::NaiveBackend;
    test_backend::<NaiveBackend>();
}

#[cfg(feature = "nalgebra")]
#[test]
fn test_nalgebra_backend() {
    use super::nalgebra_backend::NalgebraBackend;
    test_backend::<NalgebraBackend>();
}
