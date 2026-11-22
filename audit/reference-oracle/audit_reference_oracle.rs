//! Audit tests: compare `LinearKalmanFilter` against an independent, naive,
//! obviously-correct f64 reference (textbook equations, Joseph covariance
//! update, S solved via Gauss elimination with full pivoting) on seeded random
//! models and on deliberately hard inputs. Public interface of SPEC.md only.

use kalman_filters::KalmanError;
use kalman_filters::linear::LinearKalmanFilter;

// ---------------------------------------------------------------------------
// Reference oracle (f64, slow, naive)
// ---------------------------------------------------------------------------

fn mm(a: &[f64], b: &[f64], r: usize, k: usize, c: usize) -> Vec<f64> {
    let mut o = vec![0.0; r * c];
    for i in 0..r {
        for j in 0..c {
            let mut s = 0.0;
            for p in 0..k {
                s += a[i * k + p] * b[p * c + j];
            }
            o[i * c + j] = s;
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
/// Solve A X = B (A: n×n, B: n×k) by Gaussian elimination with complete pivoting.
fn solve(a: &[f64], b: &[f64], n: usize, k: usize) -> Vec<f64> {
    let mut a = a.to_vec();
    let mut b = b.to_vec();
    let mut perm: Vec<usize> = (0..n).collect();
    for col in 0..n {
        let (mut pr, mut pc, mut best) = (col, col, -1.0);
        for i in col..n {
            for j in col..n {
                if a[i * n + j].abs() > best {
                    best = a[i * n + j].abs();
                    pr = i;
                    pc = j;
                }
            }
        }
        for j in 0..n {
            a.swap(col * n + j, pr * n + j);
        }
        for j in 0..k {
            b.swap(col * k + j, pr * k + j);
        }
        for i in 0..n {
            a.swap(i * n + col, i * n + pc);
        }
        perm.swap(col, pc);
        for i in col + 1..n {
            let f = a[i * n + col] / a[col * n + col];
            for j in col..n {
                a[i * n + j] -= f * a[col * n + j];
            }
            for j in 0..k {
                b[i * k + j] -= f * b[col * k + j];
            }
        }
    }
    let mut x = vec![0.0; n * k];
    for j in 0..k {
        for i in (0..n).rev() {
            let mut s = b[i * k + j];
            for p in i + 1..n {
                s -= a[i * n + p] * x[p * k + j];
            }
            x[i * k + j] = s / a[i * n + i];
        }
    }
    // undo column permutation
    let mut out = vec![0.0; n * k];
    for i in 0..n {
        for j in 0..k {
            out[perm[i] * k + j] = x[i * k + j];
        }
    }
    out
}

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
}

struct Oracle {
    md: Model,
    x: Vec<f64>,
    p: Vec<f64>,
}
impl Oracle {
    fn new(md: &Model) -> Self {
        Oracle { md: md.clone(), x: md.x0.clone(), p: md.p0.clone() }
    }
    fn predict(&mut self) {
        let (n, f, q) = (self.md.n, &self.md.f, &self.md.q);
        self.x = mm(f, &self.x, n, n, 1);
        let fp = mm(f, &self.p, n, n, n);
        let mut p = mm(&fp, &tr(f, n, n), n, n, n);
        for i in 0..n * n {
            p[i] += q[i];
        }
        self.p = p;
    }
    fn update(&mut self, z: &[f64]) {
        let (n, m) = (self.md.n, self.md.m);
        let (h, r) = (&self.md.h, &self.md.r);
        let hx = mm(h, &self.x, m, n, 1);
        let y: Vec<f64> = (0..m).map(|i| z[i] - hx[i]).collect();
        let ht = tr(h, m, n);
        let pht = mm(&self.p, &ht, n, n, m);
        let mut s = mm(h, &pht, m, n, m);
        for i in 0..m * m {
            s[i] += r[i];
        }
        // K = P H^T S^-1  <=>  S K^T = H P   (S, P symmetric)
        let kt = solve(&s, &tr(&pht, n, m), m, n);
        let k = tr(&kt, m, n);
        let ky = mm(&k, &y, n, m, 1);
        for i in 0..n {
            self.x[i] += ky[i];
        }
        let kh = mm(&k, h, n, m, n);
        let mut a = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                a[i * n + j] = if i == j { 1.0 } else { 0.0 } - kh[i * n + j];
            }
        }
        let apa = mm(&mm(&a, &self.p, n, n, n), &tr(&a, n, n), n, n, n);
        let krk = mm(&mm(&k, r, n, m, m), &kt, n, m, n);
        for i in 0..n * n {
            self.p[i] = apa[i] + krk[i];
        }
        // exact symmetrization
        for i in 0..n {
            for j in 0..i {
                let v = 0.5 * (self.p[i * n + j] + self.p[j * n + i]);
                self.p[i * n + j] = v;
                self.p[j * n + i] = v;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// RNG + helpers
// ---------------------------------------------------------------------------

struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn sym(&mut self) -> f64 {
        2.0 * self.uniform() - 1.0
    }
    fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-300);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
    fn usize(&mut self, lo: usize, hi: usize) -> usize {
        lo + (self.next_u64() % (hi - lo + 1) as u64) as usize
    }
}

/// Random well-conditioned SPD matrix: A A^T / k + d I, d in [0.5, 1.5].
fn rand_spd(rng: &mut Rng, n: usize, scale: f64) -> Vec<f64> {
    let a: Vec<f64> = (0..n * n).map(|_| rng.sym()).collect();
    let mut s = mm(&a, &tr(&a, n, n), n, n, n);
    for v in s.iter_mut() {
        *v /= n as f64;
    }
    let d = 0.5 + rng.uniform();
    for i in 0..n {
        s[i * n + i] += d;
    }
    for i in 0..n {
        for j in 0..i {
            s[j * n + i] = s[i * n + j];
        }
    }
    s.iter().map(|v| v * scale).collect()
}

fn rand_model(rng: &mut Rng, n: usize, m: usize) -> Model {
    // F: rotation-ish, spectral radius ~<= 1 so 50 steps stay bounded.
    let mut f: Vec<f64> = (0..n * n).map(|_| 0.3 * rng.sym() / (n as f64).sqrt()).collect();
    for i in 0..n {
        f[i * n + i] += 0.7;
    }
    Model {
        n,
        m,
        x0: (0..n).map(|_| rng.normal()).collect(),
        p0: rand_spd(rng, n, 2.0),
        f,
        q: rand_spd(rng, n, 0.1),
        h: (0..m * n).map(|_| rng.sym()).collect(),
        r: rand_spd(rng, m, 0.5),
    }
}

fn max_abs(a: &[f64]) -> f64 {
    a.iter().fold(0.0f64, |acc, v| acc.max(v.abs()))
}

fn rel_err(a: &[f64], b: &[f64]) -> f64 {
    let d: Vec<f64> = a.iter().zip(b).map(|(x, y)| x - y).collect();
    max_abs(&d) / max_abs(b).max(f64::MIN_POSITIVE)
}

fn build64(md: &Model) -> Result<LinearKalmanFilter<f64>, KalmanError> {
    LinearKalmanFilter::<f64>::new(md.n, md.m, &md.x0, &md.p0, &md.f, &md.q, &md.h, &md.r)
}
fn f32v(a: &[f64]) -> Vec<f32> {
    a.iter().map(|&x| x as f32).collect()
}
fn build32(md: &Model) -> Result<LinearKalmanFilter<f32>, KalmanError> {
    LinearKalmanFilter::<f32>::new(
        md.n,
        md.m,
        &f32v(&md.x0),
        &f32v(&md.p0),
        &f32v(&md.f),
        &f32v(&md.q),
        &f32v(&md.h),
        &f32v(&md.r),
    )
}
fn st64<A: AsRef<[f64]>>(a: A) -> Vec<f64> {
    a.as_ref().to_vec()
}
fn st32<A: AsRef<[f32]>>(a: A) -> Vec<f64> {
    a.as_ref().iter().map(|&x| x as f64).collect()
}

// ---------------------------------------------------------------------------
// Sanity: random models of every shape 1..=6 x 1..=6 agree with the oracle.
// (Expected to PASS on correct code; guards the oracle itself.)
// ---------------------------------------------------------------------------

#[test]
fn audit_random_shapes_match_oracle_f64_and_f32() {
    let mut rng = Rng(0xA0D1_7001);
    for n in 1..=6 {
        for m in 1..=6 {
            for _rep in 0..3 {
                let md = rand_model(&mut rng, n, m);
                let mut o = Oracle::new(&md);
                let mut k64 = build64(&md).unwrap();
                let mut k32 = build32(&md).unwrap();
                for step in 0..50 {
                    let z: Vec<f64> = (0..m).map(|_| 3.0 * rng.normal()).collect();
                    o.predict();
                    k64.predict().unwrap();
                    k32.predict().unwrap();
                    o.update(&z);
                    k64.update(&z).unwrap();
                    k32.update(&f32v(&z)).unwrap_or_else(|e| panic!("f32 n={n} m={m} step={step}: {e:?}"));
                    let ex = rel_err(&st64(k64.state()), &o.x).max(rel_err(&st64(k64.covariance()), &o.p));
                    assert!(ex < 1e-9, "f64 n={n} m={m} step={step}: rel err {ex:e}");
                    let ex = rel_err(&st32(k32.state()), &o.x).max(rel_err(&st32(k32.covariance()), &o.p));
                    assert!(ex < 1e-4, "f32 n={n} m={m} step={step}: rel err {ex:e}");
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// AUD-1: a tiny but strictly positive measurement-noise variance is rejected.
// validate_measurement_noise uses an ABSOLUTE floor of 1e-12 on diag(R)
// (src/validation.rs:59 via src/filter.rs:150). R = 1e-13·I is SPD and valid
// per B5 ("q and r must be PSD"); rejection depends only on the chosen units.
// ---------------------------------------------------------------------------

#[test]
fn audit_tiny_positive_measurement_noise_is_accepted() {
    // Same well-conditioned 1-D problem expressed in units 1e7 times smaller:
    // P0 = R = 1e-13 (variance), z = 1e-6. Oracle: x = z/2, P = R/2.
    let r = LinearKalmanFilter::<f64>::new(1, 1, &[0.0], &[1e-13], &[1.0], &[0.0], &[1.0], &[1e-13]);
    assert!(r.is_ok(), "f64: R = 1e-13 (SPD) rejected: {:?}", r.err());
    let r = LinearKalmanFilter::<f32>::new(1, 1, &[0.0], &[1e-13], &[1.0], &[0.0], &[1.0], &[1e-13]);
    assert!(r.is_ok(), "f32: R = 1e-13 (SPD, normal f32) rejected: {:?}", r.err());
    // Correlated 2x2 R whose diagonal straddles the floor: diag(1, 1e-13).
    let r = LinearKalmanFilter::<f64>::new(
        2, 2, &[0.0, 0.0], &[1.0, 0.0, 0.0, 1.0], &[1.0, 0.0, 0.0, 1.0], &[0.0; 4],
        &[1.0, 0.0, 0.0, 1.0], &[1.0, 0.0, 0.0, 1e-13],
    );
    assert!(r.is_ok(), "f64: R = diag(1, 1e-13) rejected: {:?}", r.err());
}

// ---------------------------------------------------------------------------
// AUD-2: results are not invariant under a change of measurement units.
// Rescaling measurement component i by c_i (z_i -> c_i z_i, row i of H by c_i,
// R -> D R D, D = diag(c)) leaves the exact posterior unchanged. The current
// code reports SingularMatrix when one component is expressed in small units,
// because the pivot test in Gauss-Jordan is absolute (src/matrix/naive.rs:143).
// Unlike B10's test (uniform scaling of a 1-D problem), here S's LARGEST
// entry is O(1): a candidate that only makes the threshold relative to
// max|S| still fails this.
// ---------------------------------------------------------------------------

fn unit_scaled(md: &Model, c: &[f64]) -> Model {
    let (n, m) = (md.n, md.m);
    let mut out = md.clone();
    for i in 0..m {
        for j in 0..n {
            out.h[i * n + j] = md.h[i * n + j] * c[i];
        }
        for j in 0..m {
            out.r[i * m + j] = md.r[i * m + j] * c[i] * c[j];
        }
    }
    out
}

#[test]
fn audit_measurement_unit_change_does_not_change_estimate_f64() {
    let mut rng = Rng(0xA0D1_7002);
    let md = rand_model(&mut rng, 3, 2);
    let zs: Vec<Vec<f64>> = (0..20).map(|_| (0..2).map(|_| rng.normal()).collect()).collect();
    // 3e-6 keeps every diag(R) above the absolute 1e-12 floor (AUD-1), so this
    // isolates the singularity test.
    for &c1 in &[3e-6, 1e6] {
        let c = [1.0, c1];
        let sm = unit_scaled(&md, &c);
        let mut o = Oracle::new(&md);
        // Feasibility: the naive oracle run directly on the rescaled model.
        let mut os = Oracle::new(&sm);
        let mut kf = build64(&sm).unwrap_or_else(|e| panic!("c={c1:e}: construction failed: {e:?}"));
        for (k, z) in zs.iter().enumerate() {
            o.predict();
            os.predict();
            kf.predict().unwrap();
            o.update(z);
            let zs_: Vec<f64> = z.iter().zip(&c).map(|(a, b)| a * b).collect();
            os.update(&zs_);
            let eo = rel_err(&os.x, &o.x).max(rel_err(&os.p, &o.p));
            assert!(eo < 1e-10, "oracle itself not unit-invariant: {eo:e}");
            kf.update(&zs_).unwrap_or_else(|e| panic!("c={c1:e} step {k}: update failed: {e:?}"));
            let ex = rel_err(&st64(kf.state()), &o.x).max(rel_err(&st64(kf.covariance()), &o.p));
            assert!(ex < 1e-8, "c={c1:e} step {k}: rel err {ex:e}");
        }
    }
}

#[test]
fn audit_measurement_unit_change_does_not_change_estimate_f32() {
    // f32: component 2 in units 1e-3 smaller -> S_22 ~ 1e-6, cond(D^-1 S D^-1) ~ 1.
    let mut rng = Rng(0xA0D1_7003);
    let md = rand_model(&mut rng, 3, 2);
    let zs: Vec<Vec<f64>> = (0..20).map(|_| (0..2).map(|_| rng.normal()).collect()).collect();
    let c = [1.0, 1e-3];
    let sm = unit_scaled(&md, &c);
    let mut o = Oracle::new(&md);
    let mut kf = build32(&sm).unwrap_or_else(|e| panic!("construction failed: {e:?}"));
    for (k, z) in zs.iter().enumerate() {
        o.predict();
        kf.predict().unwrap();
        o.update(z);
        let zs_: Vec<f32> = z.iter().zip(&c).map(|(a, b)| (a * b) as f32).collect();
        kf.update(&zs_).unwrap_or_else(|e| panic!("step {k}: update failed: {e:?}"));
        let ex = rel_err(&st32(kf.state()), &o.x).max(rel_err(&st32(kf.covariance()), &o.p));
        assert!(ex < 1e-3, "step {k}: rel err {ex:e}");
    }
}

// ---------------------------------------------------------------------------
// AUD-3: a numerically singular S of LARGE magnitude is not detected.
// S = H P H^T + R with R = r h h^T and H = h (n = 1, m = 2) has rank 1.
// With the absolute pivot floor (1e-10) the rounding residual of the second
// pivot (~1e-16·|S|) passes when |S| >~ 1e6, so S is "inverted" and K blows
// up along the null direction of S. B9 requires SingularMatrix (atomic) for a
// singular S. Accept either Err(SingularMatrix) with unchanged state, or a
// result consistent with the exact pseudo-solution (x stays O(|z|), P >= 0).
// ---------------------------------------------------------------------------

#[test]
fn audit_large_scale_singular_innovation_is_detected() {
    let mut bad = Vec::new();
    let mut total = 0;
    let mut hbs = vec![1.0 / 3.0, 0.1, 0.7, 1.0 / 7.0, 0.123456789, std::f64::consts::SQRT_2];
    let mut rng = Rng(0xA0D1_7006);
    hbs.extend((0..40).map(|_| rng.sym()));
    for &hb in &hbs {
        for &scale in &[1.0f64, 1e2, 1e4, 1e6, 1e8, 1e10, 1e12] {
            total += 1;
            let h = [1.0, hb];
            let r = [h[0] * h[0] * scale, h[0] * h[1] * scale, h[1] * h[0] * scale, h[1] * h[1] * scale];
            let mut kf = LinearKalmanFilter::<f64>::new(1, 2, &[0.0], &[scale], &[1.0], &[0.0], &h, &r).unwrap();
            // z has a component orthogonal to range(S) = span(h).
            let res = kf.update(&[0.0, 1.0]);
            let x = st64(kf.state())[0];
            let p = st64(kf.covariance())[0];
            match res {
                Err(KalmanError::SingularMatrix) if (x, p) == (0.0, scale) => {}
                // Tolerated only if the result is still a sane posterior.
                Ok(()) if x.is_finite() && x.abs() < 10.0 && p >= -1e-9 * scale && p <= scale => {}
                other => bad.push(format!("h=[1,{hb}] scale={scale:e}: {other:?}, x={x:e}, P={p:e}")),
            }
        }
    }
    // Feasibility: a plain Cholesky with a pivot tolerance RELATIVE to the
    // diagonal (d_k <= 1e3·eps·S_kk => singular) flags every one of these S.
    for &hb in &hbs {
        for &scale in &[1.0f64, 1e2, 1e4, 1e6, 1e8, 1e10, 1e12] {
            let h = [1.0, hb];
            let (p, r) = (scale, [h[0] * h[0] * scale, h[0] * h[1] * scale, h[1] * h[0] * scale, h[1] * h[1] * scale]);
            let s = [p * h[0] * h[0] + r[0], p * h[0] * h[1] + r[1], p * h[1] * h[0] + r[2], p * h[1] * h[1] + r[3]];
            assert!(ref_cholesky_is_singular(&s, 2), "reference failed to flag h=[1,{hb}] scale={scale:e}");
        }
    }
    assert!(bad.is_empty(), "singular S mishandled in {}/{total} cases:\n{}", bad.len(), bad.join("\n"));
}

fn ref_cholesky_is_singular(s: &[f64], m: usize) -> bool {
    let mut l = vec![0.0; m * m];
    for j in 0..m {
        let mut d = s[j * m + j];
        for k in 0..j {
            d -= l[j * m + k] * l[j * m + k];
        }
        if !(d > 1e3 * f64::EPSILON * s[j * m + j]) {
            return true;
        }
        l[j * m + j] = d.sqrt();
        for i in j + 1..m {
            let mut v = s[i * m + j];
            for k in 0..j {
                v -= l[i * m + k] * l[j * m + k];
            }
            l[i * m + j] = v / l[j * m + j];
        }
    }
    false
}

// ---------------------------------------------------------------------------
// AUD-4 (drift): long run of a random 4x2 model vs. oracle, f64, 1e4 steps.
// ---------------------------------------------------------------------------

#[test]
fn audit_long_run_no_drift_from_oracle_f64() {
    let mut rng = Rng(0xA0D1_7004);
    let md = rand_model(&mut rng, 4, 2);
    let mut o = Oracle::new(&md);
    let mut kf = build64(&md).unwrap();
    for k in 0..10_000 {
        let z: Vec<f64> = (0..2).map(|_| 2.0 * rng.normal()).collect();
        o.predict();
        kf.predict().unwrap();
        o.update(&z);
        kf.update(&z).unwrap();
        if k % 500 == 0 || k == 9_999 {
            let ex = rel_err(&st64(kf.state()), &o.x).max(rel_err(&st64(kf.covariance()), &o.p));
            assert!(ex < 1e-9, "step {k}: rel err {ex:e}");
        }
    }
}

// ---------------------------------------------------------------------------
// AUD-5: extreme (but representable) magnitudes, uniform scaling up.
// Uniform scaling by c (P0, Q, R by c^2, z and x0 by c) must scale x by c and
// P by c^2. Scaling DOWN is B10; this checks scaling UP near the top of range.
// ---------------------------------------------------------------------------

#[test]
fn audit_uniform_large_scale_matches_oracle() {
    let mut rng = Rng(0xA0D1_7005);
    let md = rand_model(&mut rng, 3, 2);
    let zs: Vec<Vec<f64>> = (0..30).map(|_| (0..2).map(|_| rng.normal()).collect()).collect();
    for &c in &[1e50f64, 1e100, 1e140] {
        let mut s = md.clone();
        s.x0.iter_mut().for_each(|v| *v *= c);
        for a in [&mut s.p0, &mut s.q] {
            a.iter_mut().for_each(|v| *v *= c * c);
        }
        s.r.iter_mut().for_each(|v| *v *= c * c);
        let mut o = Oracle::new(&md);
        let mut kf = build64(&s).unwrap();
        for (k, z) in zs.iter().enumerate() {
            o.predict();
            kf.predict().unwrap();
            o.update(z);
            let zc: Vec<f64> = z.iter().map(|v| v * c).collect();
            kf.update(&zc).unwrap_or_else(|e| panic!("c={c:e} step {k}: {e:?}"));
            let x: Vec<f64> = st64(kf.state()).iter().map(|v| v / c).collect();
            let p: Vec<f64> = st64(kf.covariance()).iter().map(|v| v / c / c).collect();
            let ex = rel_err(&x, &o.x).max(rel_err(&p, &o.p));
            assert!(ex < 1e-9, "c={c:e} step {k}: rel err {ex:e}");
        }
    }
    // f32: up to 1e15 (c^2 = 1e30 < f32::MAX).
    for &c in &[1e6f64, 1e12, 1e15] {
        let mut s = md.clone();
        s.x0.iter_mut().for_each(|v| *v *= c);
        for a in [&mut s.p0, &mut s.q, &mut s.r] {
            a.iter_mut().for_each(|v| *v *= c * c);
        }
        let mut o = Oracle::new(&md);
        let mut kf = build32(&s).unwrap();
        for (k, z) in zs.iter().enumerate() {
            o.predict();
            kf.predict().unwrap();
            o.update(z);
            let zc: Vec<f32> = z.iter().map(|v| (v * c) as f32).collect();
            kf.update(&zc).unwrap_or_else(|e| panic!("f32 c={c:e} step {k}: {e:?}"));
            let x: Vec<f64> = st32(kf.state()).iter().map(|v| v / c).collect();
            let p: Vec<f64> = st32(kf.covariance()).iter().map(|v| v / c / c).collect();
            let ex = rel_err(&x, &o.x).max(rel_err(&p, &o.p));
            assert!(ex < 1e-4, "f32 c={c:e} step {k}: rel err {ex:e}");
        }
    }
}
