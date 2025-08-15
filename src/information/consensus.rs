//! Consensus algorithms for distributed information filtering

use crate::information::InformationState;
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use num_traits::Zero;
use std::collections::HashMap;

/// Consensus algorithm trait
pub trait ConsensusAlgorithm<T: KalmanScalar> {
    /// Perform one consensus iteration
    fn iterate(
        &mut self,
        local_states: &mut HashMap<usize, InformationState<T>>,
        topology: &HashMap<usize, Vec<usize>>,
    ) -> KalmanResult<()>;

    /// Check convergence
    fn is_converged(&self, epsilon: T) -> bool;

    /// Get consensus value
    fn get_consensus(&self) -> Option<InformationState<T>>;
}

/// Average consensus algorithm
pub struct AverageConsensus<T: KalmanScalar> {
    /// Consensus weights (doubly stochastic matrix)
    pub weights: HashMap<(usize, usize), T>,
    /// Previous states for convergence check
    pub prev_states: HashMap<usize, InformationState<T>>,
    /// Maximum change in last iteration
    pub max_change: T,
    /// Number of iterations performed
    pub iterations: usize,
    /// State dimension
    pub state_dim: usize,
}

impl<T: KalmanScalar> AverageConsensus<T> {
    /// Create new average consensus with Metropolis weights
    pub fn new_metropolis(topology: &HashMap<usize, Vec<usize>>, state_dim: usize) -> Self {
        let mut weights = HashMap::new();

        // Compute degrees
        let degrees: HashMap<usize, usize> = topology
            .iter()
            .map(|(node, neighbors)| (*node, neighbors.len() + 1)) // +1 for self
            .collect();

        // Compute Metropolis weights
        for (node, neighbors) in topology {
            let d_i = degrees[node];

            // Self weight
            let mut self_weight = T::one();

            for neighbor in neighbors {
                let d_j = degrees[neighbor];
                let weight = T::one() / T::from(d_i.max(d_j) + 1).unwrap();
                weights.insert((*node, *neighbor), weight);
                weights.insert((*neighbor, *node), weight);
                self_weight = self_weight - weight;
            }

            weights.insert((*node, *node), self_weight);
        }

        Self {
            weights,
            prev_states: HashMap::new(),
            max_change: T::zero(),
            iterations: 0,
            state_dim,
        }
    }

    /// Create with custom weights
    pub fn new_custom(weights: HashMap<(usize, usize), T>, state_dim: usize) -> Self {
        Self {
            weights,
            prev_states: HashMap::new(),
            max_change: T::zero(),
            iterations: 0,
            state_dim,
        }
    }
}

impl<T: KalmanScalar> ConsensusAlgorithm<T> for AverageConsensus<T> {
    fn iterate(
        &mut self,
        local_states: &mut HashMap<usize, InformationState<T>>,
        topology: &HashMap<usize, Vec<usize>>,
    ) -> KalmanResult<()> {
        let n = self.state_dim;

        // Save current states
        self.prev_states = local_states.clone();

        // Compute weighted averages
        let mut new_states = HashMap::new();
        self.max_change = T::zero();

        for (node_id, _) in local_states.iter() {
            let mut new_Y = vec![T::zero(); n * n];
            let mut new_y = vec![T::zero(); n];

            // Self contribution
            if let Some(weight) = self.weights.get(&(*node_id, *node_id)) {
                let state = &local_states[node_id];
                for i in 0..n * n {
                    new_Y[i] = new_Y[i] + *weight * state.Y[i];
                }
                for i in 0..n {
                    new_y[i] = new_y[i] + *weight * state.y[i];
                }
            }

            // Neighbor contributions
            if let Some(neighbors) = topology.get(node_id) {
                for neighbor_id in neighbors {
                    if let Some(weight) = self.weights.get(&(*node_id, *neighbor_id)) {
                        if let Some(neighbor_state) = local_states.get(neighbor_id) {
                            for i in 0..n * n {
                                new_Y[i] = new_Y[i] + *weight * neighbor_state.Y[i];
                            }
                            for i in 0..n {
                                new_y[i] = new_y[i] + *weight * neighbor_state.y[i];
                            }
                        }
                    }
                }
            }

            // Compute change for convergence check
            if let Some(prev) = self.prev_states.get(node_id) {
                for i in 0..n * n {
                    let change = (new_Y[i] - prev.Y[i]).abs();
                    if change > self.max_change {
                        self.max_change = change;
                    }
                }
                for i in 0..n {
                    let change = (new_y[i] - prev.y[i]).abs();
                    if change > self.max_change {
                        self.max_change = change;
                    }
                }
            }

            new_states.insert(*node_id, InformationState::from_information(new_Y, new_y)?);
        }

        // Update states
        *local_states = new_states;
        self.iterations += 1;

        Ok(())
    }

    fn is_converged(&self, epsilon: T) -> bool {
        self.max_change < epsilon && self.iterations > 0
    }

    fn get_consensus(&self) -> Option<InformationState<T>> {
        // Return average of all states
        if self.prev_states.is_empty() {
            return None;
        }

        let n = self.state_dim;
        let num_nodes = self.prev_states.len();

        let mut avg_Y = vec![T::zero(); n * n];
        let mut avg_y = vec![T::zero(); n];

        for state in self.prev_states.values() {
            for i in 0..n * n {
                avg_Y[i] = avg_Y[i] + state.Y[i] / T::from(num_nodes).unwrap();
            }
            for i in 0..n {
                avg_y[i] = avg_y[i] + state.y[i] / T::from(num_nodes).unwrap();
            }
        }

        InformationState::from_information(avg_Y, avg_y).ok()
    }
}

/// Weighted consensus based on information confidence
pub struct WeightedConsensus<T: KalmanScalar> {
    /// Confidence weights based on information matrix trace
    pub confidence_weights: HashMap<usize, T>,
    /// Previous states
    pub prev_states: HashMap<usize, InformationState<T>>,
    /// Maximum change
    pub max_change: T,
    /// State dimension
    pub state_dim: usize,
}

impl<T: KalmanScalar> WeightedConsensus<T> {
    /// Create new weighted consensus
    pub fn new(state_dim: usize) -> Self {
        Self {
            confidence_weights: HashMap::new(),
            prev_states: HashMap::new(),
            max_change: T::zero(),
            state_dim,
        }
    }

    /// Update confidence weights based on information matrices
    fn update_confidence_weights(&mut self, local_states: &HashMap<usize, InformationState<T>>) {
        self.confidence_weights.clear();

        // Compute trace of each information matrix (measure of confidence)
        let mut total_confidence = T::zero();

        for (node_id, state) in local_states {
            let mut trace = T::zero();
            for i in 0..self.state_dim {
                trace = trace + state.Y[i * self.state_dim + i];
            }
            self.confidence_weights.insert(*node_id, trace);
            total_confidence = total_confidence + trace;
        }

        // Normalize weights
        if total_confidence > T::zero() {
            for weight in self.confidence_weights.values_mut() {
                *weight = *weight / total_confidence;
            }
        }
    }
}

impl<T: KalmanScalar> ConsensusAlgorithm<T> for WeightedConsensus<T> {
    fn iterate(
        &mut self,
        local_states: &mut HashMap<usize, InformationState<T>>,
        _topology: &HashMap<usize, Vec<usize>>,
    ) -> KalmanResult<()> {
        let n = self.state_dim;

        // Update confidence weights
        self.update_confidence_weights(local_states);

        // Save current states
        self.prev_states = local_states.clone();

        // Compute weighted average (all nodes get same result)
        let mut consensus_Y = vec![T::zero(); n * n];
        let mut consensus_y = vec![T::zero(); n];

        for (node_id, state) in self.prev_states.iter() {
            if let Some(weight) = self.confidence_weights.get(node_id) {
                for i in 0..n * n {
                    consensus_Y[i] = consensus_Y[i] + *weight * state.Y[i];
                }
                for i in 0..n {
                    consensus_y[i] = consensus_y[i] + *weight * state.y[i];
                }
            }
        }

        // Update all nodes with consensus
        let consensus_state = InformationState::from_information(consensus_Y, consensus_y)?;

        self.max_change = T::zero();
        for (node_id, prev_state) in self.prev_states.iter() {
            // Compute change
            for i in 0..n * n {
                let change = (consensus_state.Y[i] - prev_state.Y[i]).abs();
                if change > self.max_change {
                    self.max_change = change;
                }
            }

            local_states.insert(*node_id, consensus_state.clone());
        }

        Ok(())
    }

    fn is_converged(&self, epsilon: T) -> bool {
        self.max_change < epsilon
    }

    fn get_consensus(&self) -> Option<InformationState<T>> {
        // All nodes have the same state after weighted consensus
        self.prev_states.values().next().cloned()
    }
}

/// Max consensus for finding maximum information
pub struct MaxConsensus<T: KalmanScalar> {
    /// Node with maximum information
    pub max_node: Option<usize>,
    /// Maximum information state
    pub max_state: Option<InformationState<T>>,
    /// Iterations
    pub iterations: usize,
    /// State dimension
    pub state_dim: usize,
}

impl<T: KalmanScalar> MaxConsensus<T> {
    pub fn new(state_dim: usize) -> Self {
        Self {
            max_node: None,
            max_state: None,
            iterations: 0,
            state_dim,
        }
    }
}

impl<T: KalmanScalar> ConsensusAlgorithm<T> for MaxConsensus<T> {
    fn iterate(
        &mut self,
        local_states: &mut HashMap<usize, InformationState<T>>,
        topology: &HashMap<usize, Vec<usize>>,
    ) -> KalmanResult<()> {
        // Find node with maximum information (trace of Y)
        let mut max_trace = T::zero();
        let mut max_node_id = None;

        for (node_id, state) in local_states.iter() {
            let mut trace = T::zero();
            for i in 0..self.state_dim {
                trace = trace + state.Y[i * self.state_dim + i];
            }

            if trace > max_trace {
                max_trace = trace;
                max_node_id = Some(*node_id);
            }
        }

        if let Some(node_id) = max_node_id {
            self.max_node = Some(node_id);
            self.max_state = local_states.get(&node_id).cloned();

            // Propagate max to neighbors
            let max_state = self.max_state.as_ref().unwrap().clone();

            for (current_node, neighbors) in topology {
                if neighbors.contains(&node_id) || *current_node == node_id {
                    local_states.insert(*current_node, max_state.clone());
                }
            }
        }

        self.iterations += 1;
        Ok(())
    }

    fn is_converged(&self, _epsilon: T) -> bool {
        // Converged when all nodes have same max value
        self.iterations > 0 && self.max_state.is_some()
    }

    fn get_consensus(&self) -> Option<InformationState<T>> {
        self.max_state.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_average_consensus() {
        // Create line topology: 0 -- 1 -- 2
        let mut topology = HashMap::new();
        topology.insert(0, vec![1]);
        topology.insert(1, vec![0, 2]);
        topology.insert(2, vec![1]);

        let mut consensus = AverageConsensus::<f64>::new_metropolis(&topology, 2);

        // Create different initial states
        let mut states = HashMap::new();
        states.insert(
            0,
            InformationState::from_information(vec![2.0, 0.0, 0.0, 2.0], vec![2.0, 0.0]).unwrap(),
        );
        states.insert(
            1,
            InformationState::from_information(vec![1.0, 0.0, 0.0, 1.0], vec![1.0, 0.0]).unwrap(),
        );
        states.insert(
            2,
            InformationState::from_information(vec![0.5, 0.0, 0.0, 0.5], vec![0.5, 0.0]).unwrap(),
        );

        // Run consensus iterations
        for _ in 0..30 {
            consensus.iterate(&mut states, &topology).unwrap();
        }

        // Check convergence toward average
        assert!(consensus.is_converged(0.05));

        let consensus_state = consensus.get_consensus().unwrap();
        // Average of [2, 1, 0.5] ≈ 1.17
        assert!((consensus_state.Y[0] - 1.17).abs() < 0.2);
    }

    #[test]
    fn test_weighted_consensus() {
        let mut consensus = WeightedConsensus::<f64>::new(2);
        let topology = HashMap::new(); // Not used for weighted consensus

        // Create states with different confidence levels
        let mut states = HashMap::new();
        states.insert(
            0,
            InformationState::from_information(
                vec![10.0, 0.0, 0.0, 10.0], // High confidence
                vec![10.0, 0.0],
            )
            .unwrap(),
        );
        states.insert(
            1,
            InformationState::from_information(
                vec![1.0, 0.0, 0.0, 1.0], // Low confidence
                vec![2.0, 0.0],
            )
            .unwrap(),
        );

        consensus.iterate(&mut states, &topology).unwrap();

        // Result should be weighted toward high-confidence node
        let result = states.get(&0).unwrap();
        assert!(result.y[0] > 8.0); // Closer to 10 than 2
    }
}
