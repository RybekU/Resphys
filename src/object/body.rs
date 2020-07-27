use glam::Vec2;

// Body has position, velocity, body type
// Collider has everything else

/// Describes a body in shape of `Shape`.
///  
/// Currently there's no "fixture" like in Box2D and body has only 1 shape attached.
#[derive(Clone, Debug)]
pub struct Body {
    pub position: Vec2,
    /// static body CAN have velocity - it just behaves as if it had infinite mass  
    /// (this might change with introduction of kinematic body that pushes other objects)  
    /// and doesn't collide with other static bodies
    pub velocity: Vec2,
    /// Type of body - `static` or `dynamic`
    pub status: BodyStatus,
}

impl Body {
    pub fn new(position: Vec2, velocity: Vec2, status: BodyStatus) -> Self {
        Self {
            position,
            velocity,
            status,
        }
    }
}

/// Unique identifier of an object stored in the world.
/// If object gets removed the identifier will be reused.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyHandle(pub usize);

/// Status of the body, determines collision resolution and how it's affected by other bodies.
#[derive(Copy, Clone, Debug)]
pub enum BodyStatus {
    /// Even when it moves it never collides with anything.
    Static,
    /// Collides with both static and dynamic bodies.
    Dynamic,
}
