//! Finding from the differential-test harness (orchestrator), not an auditor agent.
use kalman_filters::linear::LinearKalmanFilter;

/// Run a 1-D random-walk model whose variances are all multiplied by s².
fn run(s: f64) -> Result<(f64, f64), String> {
    let s2 = s * s;
    let mut kf = LinearKalmanFilter::<f64>::new(
        1, 1, &[0.0], &[1.0 * s2], &[1.0], &[0.01 * s2], &[1.0], &[0.5 * s2],
    )
    .map_err(|e| format!("new: {e:?}"))?;
    for k in 0..10 {
        kf.predict().map_err(|e| format!("predict: {e:?}"))?;
        kf.update(&[(k as f64 * 0.1) * s]).map_err(|e| format!("update: {e:?}"))?;
    }
    Ok((kf.state().as_ref()[0] / s, kf.covariance().as_ref()[0] / s2))
}

/// Validation and filtering must be scale-invariant: a model expressed in different
/// units (all variances × s², all states × s) is equally valid and gives the same
/// result in scaled units. Today, P0/R diagonals below an absolute 1e-12
/// (src/validation.rs:6, used at :26 and :59) are rejected as invalid.
#[test]
fn audit_validation_is_scale_invariant() {
    let unit = run(1.0).expect("unit-scale model must be accepted");
    for s in [1e-7, 1e-4, 1e4, 1e7] {
        let r = run(s).unwrap_or_else(|e| panic!("scale {s:e}: valid model rejected: {e}"));
        assert!((r.0 - unit.0).abs() <= 1e-9 * unit.0.abs().max(1.0), "scale {s:e}: state {r:?} vs {unit:?}");
        assert!((r.1 - unit.1).abs() <= 1e-9 * unit.1.abs(), "scale {s:e}: covariance {r:?} vs {unit:?}");
    }
}
