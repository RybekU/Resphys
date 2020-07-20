use fxhash::FxHashMap;
use petgraph::graph::{NodeIndex, UnGraph};

type HandleNodeMap = FxHashMap<usize, NodeIndex<usize>>;

/// Structure for storing informations about the current collision.  
/// Currently unaware of anything besides the handles that collide or whether the collision started this frame.  
/// Stores result of broadphase that narrowphase should use.
pub struct CollisionGraph {
    // <BodyHandle, whether it was added this update, index_type to match `bodies` struct>
    pub src: UnGraph<usize, bool, usize>,
    // TODO: In the future try to sync Handle with Node weight
    pub binding: HandleNodeMap,
}

impl CollisionGraph {
    pub fn with_capacity(nodes: usize, edges: usize) -> Self {
        Self {
            src: UnGraph::with_capacity(nodes, edges),
            binding: HandleNodeMap::default(),
        }
    }

    pub fn add_node(&mut self, handle: usize) {
        let node_id = self.src.add_node(handle);
        self.binding.insert(handle, node_id);
    }

    pub fn update_edge(&mut self, handle1: usize, handle2: usize) {
        let node_id1 = *self.binding.get(&handle1).unwrap();
        let node_id2 = *self.binding.get(&handle2).unwrap();

        // don't add the edge if it already exists
        if !self.src.contains_edge(node_id1, node_id2) {
            self.src.add_edge(node_id1, node_id2, true);
        }
    }
    #[allow(dead_code)]
    pub fn remove_node(&mut self, handle: usize) {
        let node_id = match self.binding.get(&handle) {
            Some(&node_id) => node_id,
            None => panic!("Trying to remove nonexistent node"),
        };
        self.src.remove_node(node_id);
        // in case graph reallocated some other handle to this node
        if let Some(&new_handle) = self.src.node_weight(node_id) {
            self.binding.insert(new_handle, node_id);
        }
    }
}
