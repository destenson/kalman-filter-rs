//! Sparse matrix operations for Information Filter

use crate::information::{InformationFilter, InformationState, InformationForm};
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use num_traits::Zero;
use std::collections::HashMap;

/// Sparse matrix in Compressed Sparse Row (CSR) format
#[derive(Clone, Debug)]
pub struct SparseMatrix<T: KalmanScalar> {
    /// Non-zero values
    pub values: Vec<T>,
    /// Column indices for each value
    pub col_indices: Vec<usize>,
    /// Row pointers (start index for each row)
    pub row_ptr: Vec<usize>,
    /// Matrix dimensions
    pub rows: usize,
    pub cols: usize,
}

impl<T: KalmanScalar> SparseMatrix<T> {
    /// Create sparse matrix from dense representation
    pub fn from_dense(dense: &[T], rows: usize, cols: usize) -> Self {
        let mut values = Vec::new();
        let mut col_indices = Vec::new();
        let mut row_ptr = vec![0];
        
        for i in 0..rows {
            for j in 0..cols {
                let val = dense[i * cols + j];
                if val.abs() > <T as KalmanScalar>::epsilon() {
                    values.push(val);
                    col_indices.push(j);
                }
            }
            row_ptr.push(values.len());
        }
        
        Self {
            values,
            col_indices,
            row_ptr,
            rows,
            cols,
        }
    }
    
    /// Convert to dense representation
    pub fn to_dense(&self) -> Vec<T> {
        let mut dense = vec![T::zero(); self.rows * self.cols];
        
        for i in 0..self.rows {
            let start = self.row_ptr[i];
            let end = self.row_ptr[i + 1];
            
            for idx in start..end {
                let j = self.col_indices[idx];
                dense[i * self.cols + j] = self.values[idx];
            }
        }
        
        dense
    }
    
    /// Get sparsity (fraction of zeros)
    pub fn sparsity(&self) -> f64 {
        let total = (self.rows * self.cols) as f64;
        let nonzeros = self.values.len() as f64;
        1.0 - (nonzeros / total)
    }
    
    /// Sparse matrix-vector multiplication
    pub fn multiply_vector(&self, v: &[T]) -> KalmanResult<Vec<T>> {
        if v.len() != self.cols {
            return Err(KalmanError::DimensionMismatch {
                expected: (self.cols, 1),
                actual: (v.len(), 1),
            });
        }
        
        let mut result = vec![T::zero(); self.rows];
        
        for i in 0..self.rows {
            let start = self.row_ptr[i];
            let end = self.row_ptr[i + 1];
            
            for idx in start..end {
                let j = self.col_indices[idx];
                result[i] = result[i] + self.values[idx] * v[j];
            }
        }
        
        Ok(result)
    }
    
    /// Sparse matrix transpose
    pub fn transpose(&self) -> Self {
        // Count non-zeros per column
        let mut col_counts = vec![0; self.cols];
        for &j in &self.col_indices {
            col_counts[j] += 1;
        }
        
        // Build row pointers for transpose
        let mut t_row_ptr = vec![0];
        for count in &col_counts {
            t_row_ptr.push(t_row_ptr.last().unwrap() + count);
        }
        
        // Fill values and column indices
        let mut t_values = vec![T::zero(); self.values.len()];
        let mut t_col_indices = vec![0; self.values.len()];
        let mut col_pos = t_row_ptr[..self.cols].to_vec();
        
        for i in 0..self.rows {
            let start = self.row_ptr[i];
            let end = self.row_ptr[i + 1];
            
            for idx in start..end {
                let j = self.col_indices[idx];
                let pos = col_pos[j];
                t_values[pos] = self.values[idx];
                t_col_indices[pos] = i;
                col_pos[j] += 1;
            }
        }
        
        Self {
            values: t_values,
            col_indices: t_col_indices,
            row_ptr: t_row_ptr,
            rows: self.cols,
            cols: self.rows,
        }
    }
}

/// Information Filter with sparse matrix support
pub struct SparseInformationFilter<T: KalmanScalar> {
    /// Current information state
    pub state: InformationState<T>,
    /// State dimension
    pub state_dim: usize,
    /// Sparse observation matrices by sensor ID
    pub sparse_H: HashMap<usize, SparseMatrix<T>>,
    /// Measurement noise covariances by sensor ID
    pub R: HashMap<usize, Vec<T>>,
    /// State transition matrix (typically dense)
    pub F: Vec<T>,
    /// Process noise covariance (typically dense)
    pub Q: Vec<T>,
}

impl<T: KalmanScalar> SparseInformationFilter<T> {
    /// Create new sparse information filter
    pub fn new(
        state_dim: usize,
        initial_Y: Vec<T>,
        initial_y: Vec<T>,
        F: Vec<T>,
        Q: Vec<T>,
    ) -> KalmanResult<Self> {
        let state = InformationState::from_information(initial_Y, initial_y)?;
        
        Ok(Self {
            state,
            state_dim,
            sparse_H: HashMap::new(),
            R: HashMap::new(),
            F,
            Q,
        })
    }
    
    /// Register a sensor with sparse observation matrix
    pub fn register_sensor(
        &mut self,
        sensor_id: usize,
        H: SparseMatrix<T>,
        R: Vec<T>,
    ) -> KalmanResult<()> {
        if H.cols != self.state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (H.rows, self.state_dim),
                actual: (H.rows, H.cols),
            });
        }
        
        let m = H.rows;
        if R.len() != m * m {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, m),
                actual: (R.len() / m, m),
            });
        }
        
        self.sparse_H.insert(sensor_id, H);
        self.R.insert(sensor_id, R);
        
        Ok(())
    }
    
    /// Update with sparse measurement from specific sensor
    pub fn sparse_update(&mut self, sensor_id: usize, measurement: &[T]) -> KalmanResult<()> {
        let H = self.sparse_H.get(&sensor_id)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown sensor ID: {}", sensor_id)))?;
        let R = self.R.get(&sensor_id)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown sensor ID: {}", sensor_id)))?;
        
        let m = H.rows;
        if measurement.len() != m {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, 1),
                actual: (measurement.len(), 1),
            });
        }
        
        // Efficient sparse update using CSR format
        // Step 1: Compute R^-1 (typically small and dense)
        let R_inv = crate::filter::KalmanFilter::<T>::invert_matrix(R, m)?;
        
        // Step 2: Compute H^T·R^-1·z using sparse operations
        let Ht = H.transpose();
        
        // R^-1·z (dense)
        let mut R_inv_z = vec![T::zero(); m];
        for i in 0..m {
            for j in 0..m {
                R_inv_z[i] = R_inv_z[i] + R_inv[i * m + j] * measurement[j];
            }
        }
        
        // H^T·(R^-1·z) using sparse multiplication
        let delta_y = Ht.multiply_vector(&R_inv_z)?;
        
        // Step 3: Compute H^T·R^-1·H efficiently
        // This is the most expensive operation - exploit sparsity
        let n = self.state_dim;
        let mut delta_Y = vec![T::zero(); n * n];
        
        // For each row of H^T (column of H)
        for i in 0..n {
            // Get sparse column i of H
            let mut h_col = vec![T::zero(); m];
            for j in 0..m {
                let start = H.row_ptr[j];
                let end = H.row_ptr[j + 1];
                for idx in start..end {
                    if H.col_indices[idx] == i {
                        h_col[j] = H.values[idx];
                        break;
                    }
                }
            }
            
            // Compute R^-1·h_col
            let mut R_inv_h = vec![T::zero(); m];
            for k in 0..m {
                for l in 0..m {
                    R_inv_h[k] = R_inv_h[k] + R_inv[k * m + l] * h_col[l];
                }
            }
            
            // Update delta_Y row i
            for j in 0..n {
                // Get sparse column j of H
                let mut h_col_j = vec![T::zero(); m];
                for k in 0..m {
                    let start = H.row_ptr[k];
                    let end = H.row_ptr[k + 1];
                    for idx in start..end {
                        if H.col_indices[idx] == j {
                            h_col_j[k] = H.values[idx];
                            break;
                        }
                    }
                }
                
                // Compute (R^-1·h_i)^T·h_j
                for k in 0..m {
                    delta_Y[i * n + j] = delta_Y[i * n + j] + R_inv_h[k] * h_col_j[k];
                }
            }
        }
        
        // Step 4: Update information state
        self.state.add_information(&delta_y, &delta_Y);
        
        Ok(())
    }
    
    /// Batch update from multiple sparse sensors
    pub fn batch_sparse_update(&mut self, updates: Vec<(usize, &[T])>) -> KalmanResult<()> {
        for (sensor_id, measurement) in updates {
            self.sparse_update(sensor_id, measurement)?;
        }
        Ok(())
    }
    
    /// Get measurement sparsity for a sensor
    pub fn get_sensor_sparsity(&self, sensor_id: usize) -> Option<f64> {
        self.sparse_H.get(&sensor_id).map(|H| H.sparsity())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sparse_matrix() {
        // Create sparse matrix with mostly zeros
        let dense = vec![
            1.0, 0.0, 0.0, 2.0,
            0.0, 3.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 4.0,
        ];
        
        let sparse = SparseMatrix::from_dense(&dense, 3, 4);
        
        // Check sparsity
        assert_eq!(sparse.values.len(), 4);  // Only 4 non-zeros
        assert!(sparse.sparsity() > 0.6);    // >60% sparse
        
        // Check conversion back to dense
        let recovered = sparse.to_dense();
        for i in 0..dense.len() {
            assert_eq!(dense[i], recovered[i]);
        }
        
        // Test multiplication
        let v = vec![1.0, 2.0, 3.0, 4.0];
        let result = sparse.multiply_vector(&v).unwrap();
        
        // Manual computation: [1*1 + 2*4, 3*2, 4*4] = [9, 6, 16]
        assert_eq!(result[0], 9.0);
        assert_eq!(result[1], 6.0);
        assert_eq!(result[2], 16.0);
    }
    
    #[test]
    fn test_sparse_information_filter() {
        // Create filter with sparse sensor
        let mut filter = SparseInformationFilter::new(
            4,
            vec![1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.0],
            vec![1.0, 0.1, 0.0, 0.0,
                 0.0, 1.0, 0.1, 0.0,
                 0.0, 0.0, 1.0, 0.1,
                 0.0, 0.0, 0.0, 1.0],
            vec![0.01, 0.0, 0.0, 0.0,
                 0.0, 0.01, 0.0, 0.0,
                 0.0, 0.0, 0.01, 0.0,
                 0.0, 0.0, 0.0, 0.01],
        ).unwrap();
        
        // Register sparse sensor (only observes first and third states)
        let H_dense = vec![
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ];
        let H_sparse = SparseMatrix::from_dense(&H_dense, 2, 4);
        let R = vec![0.1, 0.0, 0.0, 0.1];
        
        filter.register_sensor(1, H_sparse, R).unwrap();
        
        // Update with sparse measurement
        filter.sparse_update(1, &[1.0, 2.0]).unwrap();
        
        // Check that information increased
        assert!(filter.state.Y[0] > 1.0);
        assert!(filter.state.Y[10] > 1.0);  // Y[2,2]
    }
}