extern crate nalgebra;

use nalgebra::DMatrix;

// use variable name in opencv
pub struct KalmanFilter {
    /// `x(k - 1)`
    pub state_pre: DMatrix<f32>,
    /// `x(k)`
    pub state_post: DMatrix<f32>,
    /// `A`
    pub transition_matrix: DMatrix<f32>,
    /// `B`
    pub control_matrix: Option<DMatrix<f32>>,
    /// `H`
    pub measurement_matrix: DMatrix<f32>,
    /// `Q`
    pub process_noise_cov: DMatrix<f32>,
    /// `R`
    pub measurement_noise_cov: DMatrix<f32>,
    /// `P(k - 1)`
    pub error_cov_pre: DMatrix<f32>,
    /// `K(k)`
    pub gain: DMatrix<f32>,
    /// `P(k)`
    pub error_cov_post: DMatrix<f32>,

    tmp: DMatrix<f32>,
}

macro_rules! create_setter {
    ($matrix:ident) => {
        /// setter
        pub fn $matrix(&mut self, slice: &[f32]){
            assert_eq!(
                self.$matrix.nrows() * self.$matrix.ncols(),
                slice.len(),
                "{} must be {} * {}",
                stringify!($matrix),
                self.$matrix.nrows(),
                self.$matrix.ncols()
            );
            self.$matrix = DMatrix::from_row_slice(self.$matrix.nrows(), self.$matrix.ncols(), slice);
        }
    };
}

impl KalmanFilter {
    /// `dynamParams`: Dimensionality of the state.
    /// `measureParams`: Dimensionality of the measurement.
    /// `controlParams`: Dimensionality of the control vector.
    pub fn new(dynam_params: usize, measure_params: usize, control_params: usize) -> Self {
        assert!(dynam_params > 0 && measure_params > 0);
        KalmanFilter {
            state_pre: DMatrix::zeros(dynam_params, 1),
            state_post: DMatrix::zeros(dynam_params, 1),
            transition_matrix: DMatrix::identity(dynam_params, dynam_params),
            control_matrix: match control_params {
                0 => None,
                _ => Some(DMatrix::zeros(dynam_params, control_params)),
            },
            measurement_matrix: DMatrix::zeros(measure_params, dynam_params),
            process_noise_cov: DMatrix::identity(dynam_params, dynam_params),
            measurement_noise_cov: DMatrix::identity(measure_params, measure_params),
            error_cov_pre: DMatrix::zeros(dynam_params, dynam_params),
            gain: DMatrix::zeros(dynam_params, measure_params),
            error_cov_post: DMatrix::zeros(dynam_params, dynam_params),

            tmp: DMatrix::zeros(measure_params, dynam_params),
        }
    }

    /// Computes a predicted state.
    pub fn predict(&mut self, control: Option<&[f32]>) -> &DMatrix<f32> {
        // x(k) = A * x(k - 1)
        self.state_pre = &self.transition_matrix * &self.state_post;
        if let Some(_control) = control {
            // x(k) = A * x(k - 1) + B * u(k)
            let control_matrix = self.control_matrix
                .clone()
                .expect("control matrix is empty");
            self.state_pre +=
                &control_matrix * &DMatrix::from_row_slice(control_matrix.ncols(), 1, _control);
        }
        // P'(k) = A * P(k) * At + Q
        self.error_cov_pre = &self.transition_matrix * &self.error_cov_post
            * &self.transition_matrix.transpose()
            + &self.process_noise_cov;

        self.state_post = self.state_pre.clone();
        self.error_cov_post = self.error_cov_pre.clone();

        &self.state_pre
    }

    /// Updates the predicted state from the measurement.
    pub fn correct(&mut self, measurement: &[f32]) -> &DMatrix<f32> {
        // tmp = H * P(k - 1)
        self.tmp = &self.measurement_matrix * &self.error_cov_pre;

        // K(k) = ((H * P(k - 1) * Ht + R).inv() * H * P(k - 1)).t()
        self.gain = ((&self.tmp * &self.measurement_matrix.transpose()
            + &self.measurement_noise_cov)
            .pseudo_inverse(1e-6) * &self.tmp)
            .transpose();

        // x(k) = x(k - 1) + K(k) * (z(k) - H * x(k - 1))
        self.state_post = &self.state_pre
            + &self.gain
                * (&DMatrix::from_row_slice(self.gain.ncols(), 1, measurement)
                    - &self.measurement_matrix * &self.state_pre);

        // P(k) = P(k - 1) - K(k) * H * P(k - 1)
        self.error_cov_post = &self.error_cov_pre - &self.gain * &self.tmp;

        &self.state_post
    }

    /// setter
    pub fn control_matrix(&mut self, slice: &[f32]) {
        let control_matrix = self.control_matrix
            .clone()
            .expect("control matrix is empty");
        assert_eq!(
            control_matrix.nrows() * control_matrix.ncols(),
            slice.len(),
            "control_matrix must be {} * {}",
            control_matrix.nrows(),
            control_matrix.ncols()
        );
        self.control_matrix = Some(DMatrix::from_row_slice(
            control_matrix.nrows(),
            control_matrix.ncols(),
            slice,
        ));
    }

    create_setter!(state_post);
    create_setter!(transition_matrix);
    create_setter!(measurement_matrix);
    create_setter!(process_noise_cov);
    create_setter!(measurement_noise_cov);
    create_setter!(error_cov_post);
}
