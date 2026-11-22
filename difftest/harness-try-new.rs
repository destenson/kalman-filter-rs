//! Differential-test harness for the linear-core tournament (installed untracked as
//! tests/zz_difftest.rs, run, removed). Uses only the SPEC.md public interface.
//! With DIFFTEST_REFERENCE=1 it prints results of its own f64 textbook oracle
//! (Joseph-form update, pivoted Gaussian solve) instead of calling the crate.
//! Invalid-input cases have no oracle line, so they are judged by majority.

use kalman_filters::linear::LinearKalmanFilter;

// ---------------------------------------------------------------- PRNG (stable)
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        // splitmix64
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn uni(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
    }
    fn range(&mut self, lo: usize, hi: usize) -> usize {
        lo + (self.next() % (hi - lo + 1) as u64) as usize
    }
}

// ---------------------------------------------------------------- dense helpers
fn mm(a: &[f64], b: &[f64], r: usize, k: usize, c: usize) -> Vec<f64> {
    let mut o = vec![0.0; r * c];
    for i in 0..r {
        for j in 0..c {
            o[i * c + j] = (0..k).map(|t| a[i * k + t] * b[t * c + j]).sum();
        }
    }
    o
}
fn tr(a: &[f64], r: usize, c: usize) -> Vec<f64> {
    let mut o = vec![0.0; r * c];
    for i in 0..r {
        for j in 0..c {
            o[j * r + i] = a[i * c + j];
        }
    }
    o
}
fn add(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x + y).collect()
}
fn eye(n: usize) -> Vec<f64> {
    let mut o = vec![0.0; n * n];
    for i in 0..n {
        o[i * n + i] = 1.0;
    }
    o
}
/// Solve A X = B (A n×n, B n×c) by Gaussian elimination with partial pivoting.
fn solve(a: &[f64], b: &[f64], n: usize, c: usize) -> Option<Vec<f64>> {
    let mut a = a.to_vec();
    let mut b = b.to_vec();
    let scale = a.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    for col in 0..n {
        let p = (col..n).max_by(|&i, &j| a[i * n + col].abs().total_cmp(&a[j * n + col].abs()))?;
        if a[p * n + col].abs() <= scale * 1e-14 {
            return None;
        }
        for k in 0..n {
            a.swap(col * n + k, p * n + k);
        }
        for k in 0..c {
            b.swap(col * c + k, p * c + k);
        }
        for i in 0..n {
            if i != col {
                let f = a[i * n + col] / a[col * n + col];
                for k in 0..n {
                    a[i * n + k] -= f * a[col * n + k];
                }
                for k in 0..c {
                    b[i * c + k] -= f * b[col * c + k];
                }
            }
        }
    }
    for i in 0..n {
        for k in 0..c {
            b[i * c + k] /= a[i * n + i];
        }
    }
    Some(b)
}
fn psd(rng: &mut Rng, n: usize, scale: f64, floor: f64) -> Vec<f64> {
    let a: Vec<f64> = (0..n * n).map(|_| rng.uni()).collect();
    let mut p = mm(&a, &tr(&a, n, n), n, n, n);
    for i in 0..n {
        p[i * n + i] += floor;
    }
    p.iter().map(|v| v * scale).collect()
}

// ---------------------------------------------------------------- model + oracle
#[derive(Clone)]
struct Model {
    n: usize,
    m: usize,
    x0: Vec<f64>,
    p0: Vec<f64>,
    f: Vec<f64>,
    q: Vec<f64>,
    h: Vec<f64>,
    r: Vec<f64>,
    /// Operation script: 'p' = predict, 'u' = update with next measurement.
    ops: Vec<char>,
    zs: Vec<Vec<f64>>,
}

fn random_model(rng: &mut Rng, s: f64) -> Model {
    let n = rng.range(1, 6);
    let m = rng.range(1, (n + 1).min(6));
    let f: Vec<f64> = (0..n * n)
        .map(|k| if k % (n + 1) == 0 { 0.95 } else { 0.0 } + 0.1 * rng.uni())
        .collect();
    let h: Vec<f64> = (0..m * n).map(|_| rng.uni()).collect();
    let steps = rng.range(5, 30);
    let mut ops = Vec::new();
    for _ in 0..steps {
        // mostly predict+update, sometimes repeated updates or predict-only
        match rng.range(0, 9) {
            0 => ops.extend(['u', 'u']),
            1 => ops.push('p'),
            _ => ops.extend(['p', 'u']),
        }
    }
    let nz = ops.iter().filter(|&&o| o == 'u').count();
    Model {
        n,
        m,
        x0: (0..n).map(|_| rng.uni() * s).collect(),
        p0: psd(rng, n, s * s, 0.1),
        f,
        q: psd(rng, n, 0.01 * s * s, 0.01),
        h,
        r: psd(rng, m, 0.1 * s * s, 0.05),
        ops,
        zs: (0..nz).map(|_| (0..m).map(|_| rng.uni() * s).collect()).collect(),
    }
}

/// Textbook f64 filter: x=Fx, P=FPFᵀ+Q; S=HPHᵀ+R, K=PHᵀS⁻¹ (solved), Joseph update.
fn oracle(md: &Model) -> Result<Vec<f64>, String> {
    let (n, m) = (md.n, md.m);
    let (mut x, mut p) = (md.x0.clone(), md.p0.clone());
    let mut zi = 0;
    for &op in &md.ops {
        if op == 'p' {
            x = mm(&md.f, &x, n, n, 1);
            p = add(&mm(&mm(&md.f, &p, n, n, n), &tr(&md.f, n, n), n, n, n), &md.q);
        } else {
            let z = &md.zs[zi];
            zi += 1;
            let hx = mm(&md.h, &x, m, n, 1);
            let y: Vec<f64> = z.iter().zip(&hx).map(|(a, b)| a - b).collect();
            let ht = tr(&md.h, m, n);
            let pht = mm(&p, &ht, n, n, m);
            let s = add(&mm(&md.h, &pht, m, n, m), &md.r);
            // K = P Hᵀ S⁻¹  <=>  Kᵀ = S⁻ᵀ H P = solve(Sᵀ, (P Hᵀ)ᵀ)
            let kt = solve(&tr(&s, m, m), &tr(&pht, n, m), m, n).ok_or("SingularMatrix")?;
            let k = tr(&kt, m, n);
            let ky = mm(&k, &y, n, m, 1);
            x = add(&x, &ky);
            let mut ikh = eye(n);
            let kh = mm(&k, &md.h, n, m, n);
            for i in 0..n * n {
                ikh[i] -= kh[i];
            }
            let a = mm(&mm(&ikh, &p, n, n, n), &tr(&ikh, n, n), n, n, n);
            let krk = mm(&mm(&k, &md.r, n, m, m), &kt, n, m, n);
            p = add(&a, &krk);
        }
    }
    Ok(x.into_iter().chain(p).collect())
}

// ---------------------------------------------------------------- crate runner
fn kind(e: impl std::fmt::Debug) -> String {
    let s = format!("{e:?}");
    s.split(|c: char| c == '(' || c == '{' || c == ' ').next().unwrap_or("").to_string()
}

macro_rules! run_crate {
    ($t:ty, $md:expr) => {{
        let md: &Model = $md;
        let c = |v: &[f64]| v.iter().map(|&x| x as $t).collect::<Vec<$t>>();
        match LinearKalmanFilter::<$t>::try_new(
            md.n, md.m, &c(&md.x0), &c(&md.p0), &c(&md.f), &c(&md.q), &c(&md.h), &c(&md.r),
        ) {
            Err(e) => Err(kind(e)),
            Ok(mut kf) => {
                let mut zi = 0;
                let mut res = Ok(());
                for &op in &md.ops {
                    let r = if op == 'p' {
                        kf.predict()
                    } else {
                        zi += 1;
                        kf.update(&c(&md.zs[zi - 1]))
                    };
                    if let Err(e) = r {
                        res = Err(kind(e));
                        break;
                    }
                }
                res.map(|_| {
                    let st: Vec<f64> = kf.state().as_ref().iter().map(|&v| v as f64).collect();
                    let cv: Vec<f64> = kf.covariance().as_ref().iter().map(|&v| v as f64).collect();
                    st.into_iter().chain(cv).collect::<Vec<f64>>()
                })
            }
        }
    }};
}

fn guard(f: impl FnOnce() -> Result<Vec<f64>, String>) -> Result<Vec<f64>, String> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or_else(|_| Err("PANIC".into()))
}

fn emit(case: &str, r: Result<Vec<f64>, String>, tol: f64, abs: f64) {
    match r {
        Ok(v) => {
            let vals: Vec<String> = v
                .iter()
                .map(|x| if x.is_finite() { format!("{x:e}") } else { "null".into() })
                .collect();
            println!(
                "{{\"case\":\"{case}\",\"ok\":true,\"tol\":{tol:e},\"abs\":{abs:e},\"values\":[{}]}}",
                vals.join(",")
            );
        }
        Err(e) => println!("{{\"case\":\"{case}\",\"ok\":false,\"error\":\"{e}\"}}"),
    }
}

#[test]
fn difftest() {
    let reference = std::env::var("DIFFTEST_REFERENCE").is_ok();
    std::panic::set_hook(Box::new(|_| {})); // panics are reported as status "PANIC"
    let mut rng = Rng(0x5EED_2026);

    // 1. random f64 models, and the same models in f32 (looser tolerance)
    for i in 0..150 {
        let md = random_model(&mut rng, 1.0);
        let o = if reference { oracle(&md) } else { guard(|| run_crate!(f64, &md)) };
        emit(&format!("rand{i:03}_f64"), o, 1e-7, 1e-9);
        if i % 3 == 0 {
            let o = if reference { oracle(&md) } else { guard(|| run_crate!(f32, &md)) };
            emit(&format!("rand{i:03}_f32"), o, 2e-3, 1e-4);
        }
    }
    // 2. scale invariance: same structure at very different magnitudes
    for i in 0..40 {
        let s = [1e-6, 1e-3, 1e3, 1e6][i % 4];
        let md = random_model(&mut rng, s);
        let o = if reference { oracle(&md) } else { guard(|| run_crate!(f64, &md)) };
        emit(&format!("scale{i:02}_{s:e}"), o, 1e-6, 1e-12 * s * s);
    }
    // 3. ill-conditioned: huge prior, tiny measurement noise
    for i in 0..20 {
        let mut md = random_model(&mut rng, 1.0);
        if md.m > md.n {
            // keep S well conditioned: drop the extra measurement row
            md.m = md.n;
            md.h.truncate(md.m * md.n);
            md.zs.iter_mut().for_each(|z| z.truncate(md.m));
        }
        md.p0 = eye(md.n).iter().map(|v| v * 1e8).collect();
        md.r = eye(md.m).iter().map(|v| v * 1e-8).collect();
        let o = if reference { oracle(&md) } else { guard(|| run_crate!(f64, &md)) };
        emit(&format!("illcond{i:02}"), o, 1e-4, 1e-6);
    }
    if reference {
        return; // no oracle for invalid inputs: judged by majority
    }
    // 4. invalid / edge inputs: status only (values compared if all succeed)
    let base = random_model(&mut rng, 1.0);
    let mut cases: Vec<(&str, Model)> = Vec::new();
    let mut z = base.clone();
    z.n = 0;
    (z.x0, z.p0, z.f, z.q, z.h) = (vec![], vec![], vec![], vec![], vec![]);
    cases.push(("zero_state_dim", z));
    let mut z = base.clone();
    z.m = 0;
    (z.h, z.r) = (vec![], vec![]);
    z.zs = z.zs.iter().map(|_| vec![]).collect();
    cases.push(("zero_meas_dim", z));
    let mut z = base.clone();
    z.x0.pop();
    cases.push(("short_x0", z));
    let mut z = base.clone();
    z.h.push(0.0);
    cases.push(("long_h", z));
    let mut z = base.clone();
    if z.n > 1 {
        z.p0[1] += 0.5;
    } else {
        z.p0[0] = -1.0;
    }
    cases.push(("bad_p0", z));
    let mut z = base.clone();
    z.r[0] = -1.0;
    cases.push(("negative_r", z));
    let mut z = base.clone();
    z.q[0] = f64::NAN;
    cases.push(("nan_q", z));
    let mut z = base.clone();
    z.f[0] = f64::INFINITY;
    cases.push(("inf_f", z));
    let mut z = base.clone();
    if let Some(v) = z.zs.first_mut() {
        v[0] = f64::NAN;
    }
    cases.push(("nan_measurement", z));
    let mut z = base.clone();
    z.ops = vec!['u'; 20];
    z.zs = vec![z.zs[0].clone(); 20];
    cases.push(("updates_only", z));
    for (name, md) in &cases {
        emit(&format!("edge_{name}"), guard(|| run_crate!(f64, md)), 1e-7, 1e-9);
    }
}
