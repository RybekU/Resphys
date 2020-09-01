use crate::collision::aabb::CollisionInfo;
use crate::ColliderHandle;
use fxhash::FxHashMap;
use petgraph::graph::{NodeIndex, UnGraph};
use petgraph::visit::EdgeRef;

type HandleNodeMap = FxHashMap<ColliderHandle, NodeIndex<usize>>;

#[derive(Debug, Clone)]
pub enum Interaction {
    Collision(CollisionInfo),
    Overlap,
}

impl Interaction {
    pub fn collision(&self) -> Option<&CollisionInfo> {
        match self {
            Interaction::Collision(data) => Some(data),
            _ => None,
        }
    }
    pub fn is_overlap(&self) -> bool {
        matches!(self, Interaction::Overlap)
    }
}

/// Structure for storing informations about the active collisions.  
/// Currently unaware of anything besides the handles that collide or whether the collision started this frame.  
/// Stores result of broadphase that narrowphase should use.
pub struct CollisionGraph {
    // <BodyHandle, whether it was added this update, index_type to match `bodies` struct>
    pub src: UnGraph<ColliderHandle, Option<Interaction>, usize>,
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

    pub fn add_node(&mut self, handle: ColliderHandle) {
        let node_id = self.src.add_node(handle);
        self.binding.insert(handle, node_id);
    }

    pub fn get_node_index(&self, handle: ColliderHandle) -> NodeIndex<usize> {
        *self.binding.get(&handle).unwrap()
    }

    pub fn update_edge(&mut self, handle1: ColliderHandle, handle2: ColliderHandle) {
        let node_id1 = &self.binding[&handle1];
        let node_id2 = &self.binding[&handle2];

        // don't add the edge if it already exists
        if !self.src.contains_edge(*node_id1, *node_id2) {
            self.src.add_edge(*node_id1, *node_id2, None);
        }
    }

    pub fn remove_node(&mut self, handle: ColliderHandle) {
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

    pub fn edges(
        &self,
        handle: ColliderHandle,
    ) -> impl Iterator<Item = (crate::ColliderHandle, &Interaction)> {
        let node_id = self.binding[&handle];
        self.src
            .edges(node_id)
            .filter_map(move |edge| Some((self.src[edge.target()], edge.weight().as_ref()?)))
    }
}
