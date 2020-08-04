use super::collision::{CollisionGraph, ContactManifold};
use super::event::PhysicsEvent;
use super::object::{
    collided, collision_info, Body, BodyHandle, BodyStatus, Collider, ColliderHandle, ColliderState,
};
use glam::Vec2;
use slab::Slab;

type ContactInfo = (usize, usize, ContactManifold);

/// T - User supplied type used as a tag, present in all events
pub struct PhysicsWorld<T> {
    pub bodies: Slab<Body>,
    pub colliders: Slab<Collider<T>>,
    pub collision_graph: CollisionGraph,
    pub manifolds: Vec<ContactInfo>,

    pub(crate) events: Vec<PhysicsEvent<T>>,
}

impl<T: Copy> PhysicsWorld<T> {
    //TODO: with_capacity to set slab initial size
    pub fn new() -> Self {
        Self {
            bodies: Slab::with_capacity(16),
            colliders: Slab::with_capacity(128),
            collision_graph: CollisionGraph::with_capacity(128, 16),
            manifolds: Vec::with_capacity(128),
            events: Vec::with_capacity(16),
        }
    }

    /// Adds the new collider to the physics engine and returns it's unique handle.
    pub fn add_collider(&mut self, collider: Collider<T>) -> ColliderHandle {
        let key = self.colliders.insert(collider);
        self.collision_graph.add_node(key);
        ColliderHandle(key)
    }
    /// Panics if there's no collider associated with the handle.  
    /// Currently if a `Collider` gets removed no CollisionEnded event gets sent.  
    /// The behavior might change in the future.
    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        self.colliders.remove(handle.0);
        self.collision_graph.remove_node(handle.0);
        // TODO: Consider collision ended event during next step
        // another consideration is letting the user get a list of colliding colliders
        // and letting them handle it themselves prior to removal
    }
    pub fn get_collider(&self, handle: ColliderHandle) -> Option<&Collider<T>> {
        self.colliders.get(handle.0)
    }
    pub fn mut_collider(&mut self, handle: ColliderHandle) -> Option<&mut Collider<T>> {
        self.colliders.get_mut(handle.0)
    }

    /// Adds the new body to the physics engine and returns it's unique handle.
    pub fn add_body(&mut self, body: Body) -> BodyHandle {
        let key = self.bodies.insert(body);
        BodyHandle(key)
    }

    /// Panics if there's no body associated with the handle.  
    /// Currently if a body gets removed no CollisionEnded event gets sent for the colliders.  
    /// The behavior might change in the future.
    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(handle.0);
        // IMPORTANT TODO: Remove all associated colliders
    }
    pub fn get_body(&self, handle: BodyHandle) -> Option<&Body> {
        self.bodies.get(handle.0)
    }
    pub fn mut_body(&mut self, handle: BodyHandle) -> Option<&mut Body> {
        self.bodies.get_mut(handle.0)
    }
    pub fn events(&self) -> &Vec<PhysicsEvent<T>> {
        &self.events
    }

    pub fn step(&mut self, dt: f32) {
        self.manifolds.clear();
        self.events.clear();
        let bodies = &mut self.bodies;
        let colliders = &mut self.colliders;
        let manifolds = &mut self.manifolds;
        let collision_graph = &mut self.collision_graph;
        let events = &mut self.events;

        // apply velocity for every body
        for (_, body) in bodies.iter_mut() {
            if let BodyStatus::Dynamic = body.status {
                body.position += body.velocity * dt;
            }
        }

        // TODO: Real broad phase
        // Makeshift broad-phase
        for (h1, collider1) in colliders.iter() {
            let body1 = bodies
                .get(collider1.owner.0)
                .expect("Collider without a body");
            if let BodyStatus::Static = body1.status {
                continue;
            }

            for (h2, collider2) in colliders.iter() {
                if h1 == h2 {
                    continue;
                }
                // only bodies with matching masks can collide
                let category_mismatch = ((collider1.category_bits & collider2.mask_bits) == 0)
                    || ((collider2.category_bits & collider1.mask_bits) == 0);
                if category_mismatch {
                    continue;
                }
                let body2 = bodies
                    .get(collider2.owner.0)
                    .expect("Collider without a body");

                if collided(collider1, body1.position, collider2, body2.position) {
                    collision_graph.update_edge(h1, h2);
                }
            }
        }

        let mut removed_edges = vec![];
        // fake narrow-phase replacement
        for edge_id in collision_graph.src.edge_indices() {
            let (node1_id, node2_id) = collision_graph.src.edge_endpoints(edge_id).unwrap();
            let handle1 = collision_graph.src[node1_id];
            let handle2 = collision_graph.src[node2_id];
            let collider1 = &colliders[handle1];
            let collider2 = &colliders[handle2];

            let edge_status = collision_graph.src.edge_weight_mut(edge_id).unwrap();
            let body1 = bodies
                .get(collider1.owner.0)
                .expect("Collider without a body");
            let body2 = bodies
                .get(collider2.owner.0)
                .expect("Collider without a body");
            let remove_edge = detect_collision(
                handle1,
                &collider1,
                body1.position,
                handle2,
                &collider2,
                body2.position,
                edge_status,
                manifolds,
                events,
            );
            if remove_edge {
                removed_edges.push((node1_id, node2_id));
            }
        }

        removed_edges.into_iter().for_each(|(node1_id, node2_id)| {
            if let Some(edge_id) = collision_graph.src.find_edge(node1_id, node2_id) {
                if let None = collision_graph.src.remove_edge(edge_id) {
                    log::debug!("CollisionGraph error: Invalid edge removed")
                }
            } else {
                log::debug!(
                    "CollisionGraph error: No edge between {:?} and {:?}",
                    node1_id,
                    node2_id
                );
            }
        });

        // resolve collisions TODO: resolve multiple collisions for one body
        for (h1, _h2, manifold) in manifolds.iter() {
            let body = bodies.get_mut(*h1).expect("Body missing post collision");
            let contact = manifold.best_contact();
            body.position -= contact.normal * contact.depth;

            *body.velocity.x_mut() *= contact.normal.y().abs();
            *body.velocity.y_mut() *= contact.normal.x().abs();
        }
    }
}

// Makeshift function for collision detection
fn detect_collision<T: Copy>(
    h1: usize,
    collider1: &Collider<T>,
    position1: Vec2,
    h2: usize,
    collider2: &Collider<T>,
    position2: Vec2,
    new_edge: &mut bool,
    manifolds: &mut Vec<ContactInfo>,
    events: &mut Vec<PhysicsEvent<T>>,
) -> bool {
    use ColliderState::*;

    let remove_edge = match (&collider1.state, &collider2.state) {
        (Solid, Solid) => {
            if let Some(manifold) = collision_info(collider1, position1, collider2, position2) {
                if *new_edge {
                    events.push(PhysicsEvent::CollisionStarted(
                        ColliderHandle(h1),
                        ColliderHandle(h2),
                        collider1.user_tag,
                        collider2.user_tag,
                    ));
                }
                manifolds.push((collider1.owner.0, collider2.owner.0, manifold));
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::CollisionEnded(
                        ColliderHandle(h1),
                        ColliderHandle(h2),
                        collider1.user_tag,
                        collider2.user_tag,
                    ));
                }
                true
            }
        }
        (Solid, Sensor) => {
            if collided(collider1, position1, collider2, position2) {
                if *new_edge {
                    events.push(PhysicsEvent::OverlapStarted(
                        ColliderHandle(h1),
                        ColliderHandle(h2),
                        collider1.user_tag,
                        collider2.user_tag,
                    ));
                }
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::OverlapEnded(
                        ColliderHandle(h1),
                        ColliderHandle(h2),
                        collider1.user_tag,
                        collider2.user_tag,
                    ));
                }
                true
            }
        }
        (Sensor, Solid) => {
            if collided(collider1, position1, collider2, position2) {
                if *new_edge {
                    events.push(PhysicsEvent::OverlapStarted(
                        ColliderHandle(h2),
                        ColliderHandle(h1),
                        collider2.user_tag,
                        collider1.user_tag,
                    ));
                }
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::OverlapEnded(
                        ColliderHandle(h2),
                        ColliderHandle(h1),
                        collider2.user_tag,
                        collider1.user_tag,
                    ));
                }
                true
            }
        }
        (Sensor, Sensor) => {
            if collided(collider1, position1, collider2, position2) {
                if *new_edge {
                    events.push(PhysicsEvent::OverlapStarted(
                        ColliderHandle(h1),
                        ColliderHandle(h2),
                        collider1.user_tag,
                        collider2.user_tag,
                    ));
                }
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::OverlapEnded(
                        ColliderHandle(h1),
                        ColliderHandle(h2),
                        collider1.user_tag,
                        collider2.user_tag,
                    ));
                }
                true
            }
        }
    };
    *new_edge = false;
    remove_edge
}
