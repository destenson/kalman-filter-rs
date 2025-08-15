//! Distributed Information Filter for sensor networks

use crate::information::{InformationState, InformationForm};
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use num_traits::Zero;
use std::collections::{HashMap, VecDeque};

/// Node state in distributed network
#[derive(Clone, Debug)]
pub struct NodeState<T: KalmanScalar> {
    /// Node identifier
    pub node_id: usize,
    /// Local information state
    pub local_state: InformationState<T>,
    /// Neighbors in the network
    pub neighbors: Vec<usize>,
    /// Message queue for received information
    pub message_queue: VecDeque<InformationMessage<T>>,
    /// Track processed messages to avoid duplicates
    pub processed_messages: HashMap<(usize, u64), bool>,
}

/// Information message between nodes
#[derive(Clone, Debug)]
pub struct InformationMessage<T: KalmanScalar> {
    /// Source node ID
    pub source_id: usize,
    /// Timestamp
    pub timestamp: u64,
    /// Information matrix contribution
    pub delta_Y: Vec<T>,
    /// Information vector contribution
    pub delta_y: Vec<T>,
}

/// Distributed Information Filter for decentralized sensor networks
pub struct DistributedInformationFilter<T: KalmanScalar> {
    /// Network nodes by ID
    pub nodes: HashMap<usize, NodeState<T>>,
    /// Network topology (adjacency list)
    pub topology: HashMap<usize, Vec<usize>>,
    /// State dimension
    pub state_dim: usize,
    /// Current timestamp
    pub timestamp: u64,
    /// Channel filter parameters for delayed data
    pub channel_filter_depth: usize,
    /// Communication delay model (node pairs to delay)
    pub delays: HashMap<(usize, usize), u64>,
}

impl<T: KalmanScalar> DistributedInformationFilter<T> {
    /// Create new distributed filter
    pub fn new(state_dim: usize) -> Self {
        Self {
            nodes: HashMap::new(),
            topology: HashMap::new(),
            state_dim,
            timestamp: 0,
            channel_filter_depth: 10,
            delays: HashMap::new(),
        }
    }
    
    /// Add a node to the network
    pub fn add_node(&mut self, node_id: usize, initial_Y: Vec<T>, initial_y: Vec<T>) -> KalmanResult<()> {
        if initial_Y.len() != self.state_dim * self.state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (self.state_dim, self.state_dim),
                actual: (initial_Y.len() / self.state_dim, self.state_dim),
            });
        }
        if initial_y.len() != self.state_dim {
            return Err(KalmanError::DimensionMismatch {
                expected: (self.state_dim, 1),
                actual: (initial_y.len(), 1),
            });
        }
        
        let local_state = InformationState::from_information(initial_Y, initial_y)?;
        
        let node = NodeState {
            node_id,
            local_state,
            neighbors: Vec::new(),
            message_queue: VecDeque::new(),
            processed_messages: HashMap::new(),
        };
        
        self.nodes.insert(node_id, node);
        self.topology.insert(node_id, Vec::new());
        
        Ok(())
    }
    
    /// Connect two nodes in the network
    pub fn connect_nodes(&mut self, node1: usize, node2: usize, delay: u64) -> KalmanResult<()> {
        // Update topology
        self.topology.get_mut(&node1)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node1)))?
            .push(node2);
        self.topology.get_mut(&node2)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node2)))?
            .push(node1);
        
        // Update node neighbors
        self.nodes.get_mut(&node1)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node1)))?
            .neighbors.push(node2);
        self.nodes.get_mut(&node2)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node2)))?
            .neighbors.push(node1);
        
        // Set communication delay
        self.delays.insert((node1, node2), delay);
        self.delays.insert((node2, node1), delay);
        
        Ok(())
    }
    
    /// Local measurement update at a node
    pub fn local_update(
        &mut self,
        node_id: usize,
        measurement: &[T],
        H: &[T],
        R: &[T],
    ) -> KalmanResult<()> {
        let n = self.state_dim;
        let m = measurement.len();
        
        if H.len() != m * n {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, n),
                actual: (H.len() / n, n),
            });
        }
        if R.len() != m * m {
            return Err(KalmanError::DimensionMismatch {
                expected: (m, m),
                actual: (R.len() / m, m),
            });
        }
        
        // Compute information contribution
        let R_inv = crate::filter::KalmanFilter::<T>::invert_matrix(R, m)?;
        
        // H^T·R^-1
        let mut HtR_inv = vec![T::zero(); n * m];
        for i in 0..n {
            for j in 0..m {
                for k in 0..m {
                    HtR_inv[i * m + j] = HtR_inv[i * m + j] + H[k * n + i] * R_inv[k * m + j];
                }
            }
        }
        
        // δY = H^T·R^-1·H
        let mut delta_Y = vec![T::zero(); n * n];
        for i in 0..n {
            for j in 0..n {
                for k in 0..m {
                    delta_Y[i * n + j] = delta_Y[i * n + j] + HtR_inv[i * m + k] * H[k * n + j];
                }
            }
        }
        
        // δy = H^T·R^-1·z
        let mut delta_y = vec![T::zero(); n];
        for i in 0..n {
            for j in 0..m {
                delta_y[i] = delta_y[i] + HtR_inv[i * m + j] * measurement[j];
            }
        }
        
        // Update local state
        let node = self.nodes.get_mut(&node_id)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node_id)))?;
        node.local_state.add_information(&delta_y, &delta_Y);
        
        // Create message for neighbors
        let message = InformationMessage {
            source_id: node_id,
            timestamp: self.timestamp,
            delta_Y,
            delta_y,
        };
        
        // Send to neighbors
        self.broadcast_from_node(node_id, message)?;
        
        Ok(())
    }
    
    /// Broadcast information from a node to its neighbors
    fn broadcast_from_node(&mut self, source_id: usize, message: InformationMessage<T>) -> KalmanResult<()> {
        let neighbors = self.nodes.get(&source_id)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", source_id)))?
            .neighbors.clone();
        
        for neighbor_id in neighbors {
            let delay = self.delays.get(&(source_id, neighbor_id)).unwrap_or(&0);
            let delivery_time = self.timestamp + delay;
            
            // Queue message with delay
            let delayed_message = InformationMessage {
                source_id: message.source_id,
                timestamp: delivery_time,
                delta_Y: message.delta_Y.clone(),
                delta_y: message.delta_y.clone(),
            };
            
            self.nodes.get_mut(&neighbor_id)
                .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", neighbor_id)))?
                .message_queue.push_back(delayed_message);
        }
        
        Ok(())
    }
    
    /// Process queued messages at all nodes
    pub fn process_messages(&mut self) -> KalmanResult<()> {
        self.timestamp += 1;
        
        let node_ids: Vec<usize> = self.nodes.keys().cloned().collect();
        
        for node_id in node_ids {
            self.process_node_messages(node_id)?;
        }
        
        Ok(())
    }
    
    /// Process messages for a specific node
    fn process_node_messages(&mut self, node_id: usize) -> KalmanResult<()> {
        let messages_to_process: Vec<InformationMessage<T>> = {
            let node = self.nodes.get_mut(&node_id)
                .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node_id)))?;
            
            let mut to_process = Vec::new();
            while let Some(msg) = node.message_queue.front() {
                if msg.timestamp <= self.timestamp {
                    to_process.push(node.message_queue.pop_front().unwrap());
                } else {
                    break;
                }
            }
            to_process
        };
        
        for message in messages_to_process {
            // Check if already processed
            let message_id = (message.source_id, message.timestamp);
            
            let node = self.nodes.get_mut(&node_id)
                .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node_id)))?;
            
            if node.processed_messages.contains_key(&message_id) {
                continue;
            }
            
            // Apply information update
            node.local_state.add_information(&message.delta_y, &message.delta_Y);
            node.processed_messages.insert(message_id, true);
            
            // Forward to other neighbors (flooding with duplicate detection)
            let neighbors = node.neighbors.clone();
            for neighbor_id in neighbors {
                if neighbor_id != message.source_id {
                    let forward_message = message.clone();
                    let delay = self.delays.get(&(node_id, neighbor_id)).unwrap_or(&0);
                    
                    self.nodes.get_mut(&neighbor_id)
                        .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", neighbor_id)))?
                        .message_queue.push_back(InformationMessage {
                            source_id: forward_message.source_id,
                            timestamp: self.timestamp + delay,
                            delta_Y: forward_message.delta_Y,
                            delta_y: forward_message.delta_y,
                        });
                }
            }
        }
        
        Ok(())
    }
    
    /// Get consensus state across all nodes
    pub fn get_consensus_state(&self) -> KalmanResult<Vec<T>> {
        if self.nodes.is_empty() {
            return Err(KalmanError::FilterDivergence("No nodes in network".to_string()));
        }
        
        let n = self.state_dim;
        let num_nodes = self.nodes.len();
        
        // Average information matrices and vectors
        let mut avg_Y = vec![T::zero(); n * n];
        let mut avg_y = vec![T::zero(); n];
        
        for node in self.nodes.values() {
            for i in 0..n * n {
                avg_Y[i] = avg_Y[i] + node.local_state.Y[i] / T::from(num_nodes).unwrap();
            }
            for i in 0..n {
                avg_y[i] = avg_y[i] + node.local_state.y[i] / T::from(num_nodes).unwrap();
            }
        }
        
        // Recover state from averaged information
        let avg_state = InformationState::from_information(avg_Y, avg_y)?;
        avg_state.recover_state()
    }
    
    /// Get state estimate from a specific node
    pub fn get_node_state(&self, node_id: usize) -> KalmanResult<Vec<T>> {
        self.nodes.get(&node_id)
            .ok_or_else(|| KalmanError::FilterDivergence(format!("Unknown node: {}", node_id)))?
            .local_state.recover_state()
    }
    
    /// Get network connectivity statistics
    pub fn get_network_stats(&self) -> NetworkStats {
        let num_nodes = self.nodes.len();
        let num_edges = self.topology.values()
            .map(|neighbors| neighbors.len())
            .sum::<usize>() / 2;
        
        let avg_degree = if num_nodes > 0 {
            self.topology.values()
                .map(|neighbors| neighbors.len())
                .sum::<usize>() as f64 / num_nodes as f64
        } else {
            0.0
        };
        
        let max_degree = self.topology.values()
            .map(|neighbors| neighbors.len())
            .max()
            .unwrap_or(0);
        
        NetworkStats {
            num_nodes,
            num_edges,
            avg_degree,
            max_degree,
        }
    }
}

/// Network statistics
#[derive(Debug, Clone)]
pub struct NetworkStats {
    pub num_nodes: usize,
    pub num_edges: usize,
    pub avg_degree: f64,
    pub max_degree: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_distributed_filter() {
        // Create 3-node network
        let mut dif = DistributedInformationFilter::<f64>::new(2);
        
        // Add nodes with initial information
        let Y_init = vec![1.0, 0.0, 0.0, 1.0];
        let y_init = vec![0.0, 0.0];
        
        dif.add_node(0, Y_init.clone(), y_init.clone()).unwrap();
        dif.add_node(1, Y_init.clone(), y_init.clone()).unwrap();
        dif.add_node(2, Y_init.clone(), y_init.clone()).unwrap();
        
        // Connect in a line: 0 -- 1 -- 2
        dif.connect_nodes(0, 1, 1).unwrap();
        dif.connect_nodes(1, 2, 1).unwrap();
        
        // Node 0 makes a measurement
        let H = vec![1.0, 0.0];
        let R = vec![0.1];
        dif.local_update(0, &[1.0], &H, &R).unwrap();
        
        // Process messages (propagate information)
        dif.process_messages().unwrap();
        dif.process_messages().unwrap();  // Need two steps for info to reach node 2
        
        // Check that information propagated
        let state0 = dif.get_node_state(0).unwrap();
        let state2 = dif.get_node_state(2).unwrap();
        
        // Node 2 should have received the information
        assert!(state2[0] > 0.0);
    }
    
    #[test]
    fn test_network_stats() {
        let mut dif = DistributedInformationFilter::<f64>::new(2);
        
        // Create star topology with 4 nodes
        let Y_init = vec![1.0, 0.0, 0.0, 1.0];
        let y_init = vec![0.0, 0.0];
        
        for i in 0..4 {
            dif.add_node(i, Y_init.clone(), y_init.clone()).unwrap();
        }
        
        // Connect as star: 0 is center
        dif.connect_nodes(0, 1, 0).unwrap();
        dif.connect_nodes(0, 2, 0).unwrap();
        dif.connect_nodes(0, 3, 0).unwrap();
        
        let stats = dif.get_network_stats();
        assert_eq!(stats.num_nodes, 4);
        assert_eq!(stats.num_edges, 3);
        assert_eq!(stats.max_degree, 3);  // Center node
    }
}