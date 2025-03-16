extern crate kalman_filter;

use kalman_filter::*;

fn main() {
    let mut kf = KalmanFilter::new(2, 1, 0);
    kf.transition_matrix(&[1.0, 1.0, 0.0, 1.0]);
    kf.measurement_matrix(&[1.0, 0.0]);
    kf.process_noise_cov(&[1e-5, 0.0, 0.0, 1e-5]);
    kf.measurement_noise_cov(&[0.1]);
    kf.error_cov_post(&[1.0, 0.0, 0.0, 1.0]);

    kf.state_post(&[10.0, 0.0]);

    let sys_time = std::time::SystemTime::now();
    for i in 0..10000 {
        kf.predict(None);
        kf.correct(&[i as f32 + 0.1]);
    }
    println!("{}", sys_time.elapsed().unwrap().subsec_nanos());
}
