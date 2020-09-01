use super::{BodySet, Collider};
use generational_arena::Arena;
use std::ops::{Index, IndexMut};

/// Unique identifier of a collider stored in the world.
/// If it gets removed the identifier will be reused.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ColliderHandle(generational_arena::Index);

/// Container for colliders, removal is currently performed through `PhysicsWorld`, but access and modification is possible through this structure
pub struct ColliderSet<T> {
    colliders: Arena<Collider<T>>,
}

impl<T> Default for ColliderSet<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> ColliderSet<T> {
    pub fn new() -> Self {
        Self {
            colliders: Arena::with_capacity(128),
        }
    }

    /// Inserts a new collider into the Set if it's associated body exists.  
    /// In the case where body doesn't exist returns `None`.  
    /// Currently requires `PhysicsWorld` as an argument to add a node to `CollisionGraph`.
    pub fn insert(
        &mut self,
        collider: Collider<T>,
        bodies: &mut BodySet,
        world: &mut crate::PhysicsWorld<T>,
    ) -> Option<ColliderHandle> {
        let body = bodies.get_mut(collider.owner)?;
        let key = self.colliders.insert(collider);
        world.collision_graph.add_node(ColliderHandle(key));
        body.colliders.push(ColliderHandle(key));
        Some(ColliderHandle(key))
    }

    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider<T>> {
        self.colliders.get(handle.0)
    }
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider<T>> {
        self.colliders.get_mut(handle.0)
    }
    pub fn iter(&self) -> impl Iterator<Item = (crate::ColliderHandle, &Collider<T>)> {
        self.colliders
            .iter()
            .map(|(index, collider)| (ColliderHandle(index), collider))
    }
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (crate::ColliderHandle, &mut Collider<T>)> {
        self.colliders
            .iter_mut()
            .map(|(index, collider)| (ColliderHandle(index), collider))
    }
    pub(crate) fn internal_remove(&mut self, handle: ColliderHandle) -> Collider<T> {
        self.colliders
            .remove(handle.0)
            .expect("Tried to remove nonexistent collider")
    }
}

impl<T> Index<ColliderHandle> for ColliderSet<T> {
    type Output = Collider<T>;

    fn index(&self, index: ColliderHandle) -> &Collider<T> {
        &self.colliders[index.0]
    }
}

impl<T> IndexMut<ColliderHandle> for ColliderSet<T> {
    fn index_mut(&mut self, index: ColliderHandle) -> &mut Collider<T> {
        &mut self.colliders[index.0]
    }
}
