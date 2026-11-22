//! Randomised robustness checks for `LinearKalmanFilter` beyond the frozen
//! spec models: every `Ok` step leaves P finite, exactly symmetric and PSD;
//! every `Err` leaves the filter unchanged. Deterministic (fixed seeds).

use kalman_filters::KalmanError;
use kalman_filters::linear::LinearKalmanFilter;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn sym(&mut self) -> f64 {
        2.0 * self.next() - 1.0
    }
    fn mat(&mut self, r: usize, c: usize) -> Vec<f64> {
        (0..r * c).map(|_| self.sym()).collect()
    }
}

/// D (A Aᵀ) D with A n×k (rank ≤ k) and per-component scales D = 10^U(-e, e).
fn random_psd(rng: &mut Rng, n: usize, k: usize, e: f64) -> Vec<f64> {
    let a = rng.mat(n, k);
    let d: Vec<f64> = (0..n).map(|_| 10f64.powf(e * rng.sym())).collect();
    let mut p = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            let s: f64 = (0..k).map(|t| a[i * k + t] * a[j * k + t]).sum();
            p[i * n + j] = d[i] * s * d[j];
        }
    }
    p
}

fn min_max_eig(a: &[f64], n: usize) -> (f64, f64) {
    let mut a = a.to_vec();
    for _ in 0..60 {
        for p in 0..n {
            for q in p + 1..n {
                let apq = a[p * n + q];
                if apq == 0.0 {
                    continue;
                }
                let theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let (x, y) = (a[k * n + p], a[k * n + q]);
                    a[k * n + p] = c * x - s * y;
                    a[k * n + q] = s * x + c * y;
                }
                for k in 0..n {
                    let (x, y) = (a[p * n + k], a[q * n + k]);
                    a[p * n + k] = c * x - s * y;
                    a[q * n + k] = s * x + c * y;
                }
            }
        }
    }
    let d = (0..n).map(|i| a[i * n + i]);
    (d.clone().fold(f64::INFINITY, f64::min), d.fold(f64::NEG_INFINITY, f64::max))
}

macro_rules! robustness {
    ($name:ident, $t:ty, $tol:expr) => {
        #[test]
        fn $name() {
            let mut rng = Rng(0xC0FF_EE00 ^ ($tol as f64).to_bits());
            let c = |v: &[f64]| v.iter().map(|&x| x as $t).collect::<Vec<$t>>();
            let (mut ok_steps, mut models) = (0, 0);
            for _ in 0..200 {
                let n = 1 + (rng.next() * 6.0) as usize;
                let m = 1 + (rng.next() * 4.0) as usize;
                let f: Vec<f64> = (0..n * n)
                    .map(|i| if i % (n + 1) == 0 { 1.0 } else { 0.3 * rng.sym() })
                    .collect();
                let p0 = random_psd(&mut rng, n, n, 3.0);
                let q_rank = 1 + (rng.next() * n as f64) as usize;
                let q = random_psd(&mut rng, n, q_rank, 2.0);
                let h = rng.mat(m, n);
                let r_rank = 1 + (rng.next() * m as f64) as usize;
                let r = random_psd(&mut rng, m, r_rank, 3.0);
                let Ok(mut kf) = LinearKalmanFilter::<$t>::new(n, m, &c(&[0.0; 6][..n]), &c(&p0), &c(&f), &c(&q), &c(&h), &c(&r))
                else {
                    continue; // a rounding-indefinite random input is a legitimate rejection
                };
                models += 1;
                for k in 0..100 {
                    let before = (kf.state().to_vec(), kf.covariance().to_vec());
                    let res = if k % 2 == 0 {
                        kf.predict()
                    } else {
                        let z: Vec<$t> = (0..m).map(|_| rng.sym() as $t).collect();
                        kf.update(&z)
                    };
                    match res {
                        Ok(()) => {
                            ok_steps += 1;
                            let p: Vec<f64> = kf.covariance().iter().map(|&x| x as f64).collect();
                            assert!(p.iter().all(|x| x.is_finite()), "non-finite P");
                            assert!(kf.state().iter().all(|x| x.is_finite()), "non-finite x");
                            for i in 0..n {
                                for j in 0..n {
                                    assert_eq!(p[i * n + j], p[j * n + i], "P not exactly symmetric");
                                }
                            }
                            assert!(p.iter().step_by(n + 1).all(|&v| v >= 0.0), "negative variance");
                            // The B11/B21 criterion: λ_min ≥ −tol · λ_max.
                            let (lo, hi) = min_max_eig(&p, n);
                            assert!(lo >= -$tol * hi, "P not PSD (step {k}, n={n}, m={m}): λ ∈ [{lo:e}, {hi:e}]");
                        }
                        Err(KalmanError::SingularMatrix) | Err(KalmanError::FilterDivergence(_)) => {
                            assert_eq!((kf.state().to_vec(), kf.covariance().to_vec()), before);
                        }
                        Err(e) => panic!("unexpected error {e:?}"),
                    }
                }
            }
            assert!(models > 150 && ok_steps > 10_000, "too few cases exercised: {models} models, {ok_steps} ok steps");
        }
    };
}

robustness!(random_models_stay_psd_f64, f64, 1e-9);
robustness!(random_models_stay_psd_f32, f32, 1e-3);
