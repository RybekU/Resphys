use super::super::collision::{self, ContactManifold, Shape};
// TODO: maybe mint vector?
use quicksilver::geom::Vector;

/// Describes a body in shape of `Shape`.
///
/// Currently there's no "fixture" like in Box2D and body has only 1 shape attached.
#[derive(Clone, Debug)]
pub struct Body<T> {
    pub shape: Shape,
    pub position: Vector,
    /// static body CAN have velocity - it just behaves as if it had infinite mass
    /// and doesn't collide with other static bodies
    pub velocity: Vector,
    /// Type of body - `static` or `dynamic`
    pub btype: BodyType,
    /// Whether to treat the body as physical or not
    pub state: BodyState,
    /// Ideally only one bit should be set
    pub category_bits: u32,
    /// Bodies only collide if both of their masks match
    pub mask_bits: u32,
    /// User supplied tag for identification
    pub user_tag: T,
}

impl<T> Body<T> {
    pub fn new(
        shape: Shape,
        position: Vector,
        velocity: Vector,
        btype: BodyType,
        state: BodyState,
        category_bits: u32,
        mask_bits: u32,
        user_tag: T,
    ) -> Self {
        Self {
            shape,
            position,
            velocity,
            btype,
            state,
            category_bits,
            mask_bits,
            user_tag,
        }
    }
}

/// Boolean test whether two bodies collided.
pub fn collided<T>(body1: &Body<T>, body2: &Body<T>) -> bool {
    use Shape::*;
    match (body1.shape, body2.shape) {
        (AABB(half_extents1), AABB(half_extents2)) => collision::collision_aabb_aabb(
            body1.position,
            half_extents1,
            body2.position,
            half_extents2,
        ),
    }
}

/// Generates a ContactManifold if two bodies collided.
pub fn collision_info<T>(body1: &Body<T>, body2: &Body<T>) -> Option<ContactManifold> {
    use Shape::*;
    match (body1.shape, body2.shape) {
        (AABB(half_extents1), AABB(half_extents2)) => collision::collision_aabb_aabb_manifold(
            body1.position,
            half_extents1,
            body2.position,
            half_extents2,
        ),
    }
}

/// Unique identifier of an object stored in the world
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyHandle(pub usize);

/// Type of the body, determines collision resolution and how it's affected by other bodies.
#[derive(Copy, Clone, Debug)]
pub enum BodyType {
    /// Even when it moves it never collides with anything.
    Static,
    /// Collides with both static and dynamic bodies.
    Dynamic,
}

/// State of the body, determines collision resolution and types of events sent.
#[derive(Copy, Clone, Debug)]
pub enum BodyState {
    /// Solid body resolves collision.
    Solid,
    /// Sensor sends events about possible overlap.
    Sensor,
}
