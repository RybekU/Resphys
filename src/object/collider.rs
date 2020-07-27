use super::super::collision::{self, ContactManifold, Shape};
use super::body::BodyHandle;
use glam::Vec2;

// Body has position, velocity, body type
// Collider has everything else

/// Describes a collider in shape of `Shape`. Attached to a body.
#[derive(Clone, Debug)]
pub struct Collider<T> {
    ///
    pub shape: Shape,
    /// Offset from the body's position, 0 for centered
    pub offset: Vec2,
    /// Whether to treat the body as physical or not
    pub state: ColliderState,
    /// Ideally only one bit should be set
    pub category_bits: u32,
    /// Bodies only collide if both of their masks match
    pub mask_bits: u32,
    /// User supplied tag for identification
    pub user_tag: T,
    /// Body who owns the collider
    pub owner: BodyHandle,
}

impl<T> Collider<T> {
    pub fn new(
        shape: Shape,
        offset: Vec2,
        state: ColliderState,
        category_bits: u32,
        mask_bits: u32,
        user_tag: T,
        owner: BodyHandle,
    ) -> Self {
        Self {
            shape,
            offset,
            state,
            category_bits,
            mask_bits,
            user_tag,
            owner,
        }
    }
}

/// Boolean test whether two bodies collided.
pub fn collided<T>(
    collider1: &Collider<T>,
    position1: Vec2,
    collider2: &Collider<T>,
    position2: Vec2,
) -> bool {
    use Shape::*;
    match (collider1.shape, collider2.shape) {
        (AABB(half_extents1), AABB(half_extents2)) => {
            collision::collision_aabb_aabb(position1, half_extents1, position2, half_extents2)
        }
    }
}

/// Generates a ContactManifold if two bodies collided.
pub fn collision_info<T>(
    collider1: &Collider<T>,
    position1: Vec2,
    collider2: &Collider<T>,
    position2: Vec2,
) -> Option<ContactManifold> {
    use Shape::*;
    match (collider1.shape, collider2.shape) {
        (AABB(half_extents1), AABB(half_extents2)) => collision::collision_aabb_aabb_manifold(
            position1,
            half_extents1,
            position2,
            half_extents2,
        ),
    }
}

/// Unique identifier of an object stored in the world.
/// If object gets removed the identifier will be reused.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ColliderHandle(pub usize);

/// State of the body, determines collision resolution and types of events sent.
#[derive(Copy, Clone, Debug)]
pub enum ColliderState {
    /// Solid body resolves collision.
    Solid,
    /// Sensor sends events about possible overlap.
    Sensor,
}
