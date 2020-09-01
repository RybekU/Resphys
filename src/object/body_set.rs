use super::Body;
use generational_arena::Arena;
use std::ops::{Index, IndexMut};

/// Unique identifier of a body stored in the world.
/// If it gets removed the identifier will be reused.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyHandle(generational_arena::Index);

/// Container for bodies, removal is currently performed through `PhysicsWorld`, but access and modification is possible through this structure
pub struct BodySet {
    bodies: Arena<Body>,
}

impl Default for BodySet {
    fn default() -> Self {
        Self::new()
    }
}

impl BodySet {
    pub fn new() -> Self {
        Self {
            bodies: Arena::with_capacity(16),
        }
    }

    /// Inserts a new body into the world and returns it's unique handle.
    pub fn insert(&mut self, body: Body) -> BodyHandle {
        let key = self.bodies.insert(body);
        BodyHandle(key)
    }

    pub fn get(&self, handle: BodyHandle) -> Option<&Body> {
        self.bodies.get(handle.0)
    }
    pub fn get_mut(&mut self, handle: BodyHandle) -> Option<&mut Body> {
        self.bodies.get_mut(handle.0)
    }
    pub fn iter(&self) -> impl Iterator<Item = (crate::BodyHandle, &Body)> {
        self.bodies
            .iter()
            .map(|(index, body)| (BodyHandle(index), body))
    }
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (crate::BodyHandle, &mut Body)> {
        self.bodies
            .iter_mut()
            .map(|(index, body)| (BodyHandle(index), body))
    }
    pub(crate) fn internal_remove(&mut self, handle: BodyHandle) -> Body {
        self.bodies
            .remove(handle.0)
            .expect("Tried to remove nonexistent body")
    }
}

impl Index<BodyHandle> for BodySet {
    type Output = Body;

    fn index(&self, index: BodyHandle) -> &Body {
        &self.bodies[index.0]
    }
}

impl IndexMut<BodyHandle> for BodySet {
    fn index_mut(&mut self, index: BodyHandle) -> &mut Body {
        &mut self.bodies[index.0]
    }
}
