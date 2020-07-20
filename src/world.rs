use super::collision::{CollisionGraph, ContactManifold};
use super::event::PhysicsEvent;
use super::object::{collided, collision_info, Body, BodyHandle, BodyState, BodyType};
use slab::Slab;

type ContactInfo = (usize, usize, ContactManifold);

/// T - User supplied type used as a tag, present in all events
pub struct PhysicsWorld<T> {
    pub bodies: Slab<Body<T>>,
    pub collision_graph: CollisionGraph,
    pub manifolds: Vec<ContactInfo>,

    pub events: Vec<PhysicsEvent<T>>,
}

impl<T: Copy> PhysicsWorld<T> {
    //TODO: with_capacity to set slab initial size
    pub fn new() -> Self {
        Self {
            bodies: Slab::with_capacity(128),
            collision_graph: CollisionGraph::with_capacity(128, 16),
            manifolds: Vec::with_capacity(128),
            events: Vec::with_capacity(16),
        }
    }
    pub fn add(&mut self, body: Body<T>) -> BodyHandle {
        let key = self.bodies.insert(body);
        self.collision_graph.add_node(key);
        BodyHandle(key)
    }
    pub fn get_body(&self, handle: BodyHandle) -> Option<&Body<T>> {
        self.bodies.get(handle.0)
    }
    pub fn mut_body(&mut self, handle: BodyHandle) -> Option<&mut Body<T>> {
        self.bodies.get_mut(handle.0)
    }
    pub fn events(&self) -> &Vec<PhysicsEvent<T>> {
        &self.events
    }

    pub fn step(&mut self, dt: f32) {
        self.manifolds.clear();
        self.events.clear();
        let bodies = &mut self.bodies;
        let manifolds = &mut self.manifolds;
        let collision_graph = &mut self.collision_graph;
        let events = &mut self.events;

        // apply velocity for every body
        for (_, body) in bodies.iter_mut() {
            if let BodyType::Dynamic = body.btype {
                body.position += body.velocity * dt;
            }
        }

        // TODO: Real broad phase
        // Makeshift broad-phase
        for (h1, body1) in bodies.iter() {
            if let BodyType::Static = body1.btype {
                continue;
            }

            for (h2, body2) in bodies.iter() {
                if h1 == h2 {
                    continue;
                }
                // only bodies with matching masks can collide
                let category_mismatch = ((body1.category_bits & body2.mask_bits) == 0)
                    || ((body2.category_bits & body1.mask_bits) == 0);
                if category_mismatch {
                    continue;
                }

                if collided(body1, body2) {
                    collision_graph.update_edge(h1, h2);
                }
            }
        }

        let mut removed_edges = vec![];
        // fake narrow-phase replacement
        for edge_id in collision_graph.src.edge_indices() {
            let (node_id1, node_id2) = collision_graph.src.edge_endpoints(edge_id).unwrap();
            let handle1 = collision_graph.src[node_id1];
            let handle2 = collision_graph.src[node_id2];
            let body1 = &bodies[handle1];
            let body2 = &bodies[handle2];
            // todo: move "collided" to broad phase, try to do "collision started/ended" thing
            let edge_status = collision_graph.src.edge_weight_mut(edge_id).unwrap();
            let remove_edge = detect_collision(
                handle1,
                &body1,
                handle2,
                &body2,
                edge_status,
                manifolds,
                events,
            );
            if remove_edge {
                removed_edges.push(edge_id);
            }
        }
        removed_edges.into_iter().for_each(|edge| {
            collision_graph.src.remove_edge(edge);
        });

        // resolve collisions TODO: resolve multiple collisions for one body
        for (h1, _h2, manifold) in manifolds.iter() {
            let body = bodies.get_mut(*h1).expect("Body missing post collision");
            let contact = manifold.best_contact();
            body.position -= contact.normal * contact.depth;

            body.velocity.x *= contact.normal.y.abs();
            body.velocity.y *= contact.normal.x.abs();
        }
    }
}

// Makeshift function for collision detection
fn detect_collision<T: Copy>(
    h1: usize,
    body1: &Body<T>,
    h2: usize,
    body2: &Body<T>,
    new_edge: &mut bool,
    manifolds: &mut Vec<ContactInfo>,
    events: &mut Vec<PhysicsEvent<T>>,
) -> bool {
    use BodyState::*;

    let remove_edge = match (&body1.state, &body2.state) {
        (Solid, Solid) => {
            if let Some(manifold) = collision_info(body1, body2) {
                if *new_edge {
                    events.push(PhysicsEvent::CollisionStarted(
                        BodyHandle(h1),
                        BodyHandle(h2),
                        body1.user_tag,
                        body2.user_tag,
                    ));
                }
                manifolds.push((h1, h2, manifold));
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::CollisionEnded(
                        BodyHandle(h1),
                        BodyHandle(h2),
                        body1.user_tag,
                        body2.user_tag,
                    ));
                }
                true
            }
        }
        (Solid, Sensor) => {
            if collided(body1, body2) {
                if *new_edge {
                    events.push(PhysicsEvent::OverlapStarted(
                        BodyHandle(h1),
                        BodyHandle(h2),
                        body1.user_tag,
                        body2.user_tag,
                    ));
                }
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::OverlapEnded(
                        BodyHandle(h1),
                        BodyHandle(h2),
                        body1.user_tag,
                        body2.user_tag,
                    ));
                }
                true
            }
        }
        (Sensor, Solid) => {
            if collided(body1, body2) {
                if *new_edge {
                    events.push(PhysicsEvent::OverlapStarted(
                        BodyHandle(h2),
                        BodyHandle(h1),
                        body2.user_tag,
                        body1.user_tag,
                    ));
                }
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::OverlapEnded(
                        BodyHandle(h2),
                        BodyHandle(h1),
                        body2.user_tag,
                        body1.user_tag,
                    ));
                }
                true
            }
        }
        (Sensor, Sensor) => {
            if collided(body1, body2) {
                if *new_edge {
                    events.push(PhysicsEvent::OverlapStarted(
                        BodyHandle(h1),
                        BodyHandle(h2),
                        body1.user_tag,
                        body2.user_tag,
                    ));
                }
                false
            } else {
                if !*new_edge {
                    events.push(PhysicsEvent::OverlapEnded(
                        BodyHandle(h1),
                        BodyHandle(h2),
                        body1.user_tag,
                        body2.user_tag,
                    ));
                }
                true
            }
        }
    };
    *new_edge = false;
    remove_edge
}
