use super::collider::ColliderHandle;
use glam::Vec2;

/// Describes a body.
///  
/// It functions as a container for colliders.
#[derive(Clone, Debug)]
pub struct Body {
    pub position: Vec2,
    /// static body CAN have velocity - it just behaves as if it had infinite mass  
    /// (this might change with introduction of kinematic body that pushes other objects)  
    /// and doesn't collide with other static bodies
    pub velocity: Vec2,
    /// Type of body - `static` or `kinematic`
    pub status: BodyStatus,
    /// Whether colliders of the same body should collide
    pub self_collide: bool,
    // cached list of colliders belonging to body
    pub(crate) colliders: Vec<ColliderHandle>,
}

impl Body {
    pub fn new(position: Vec2, velocity: Vec2, status: BodyStatus, self_collide: bool) -> Self {
        Self {
            position,
            velocity,
            status,
            self_collide,
            colliders: Vec::new(),
        }
    }
}

/// Unique identifier of a body stored in the world.
/// If it gets removed the identifier will be reused.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyHandle(pub usize);

/// Status of the body, determines how it's affected by other bodies.
#[derive(Copy, Clone, Debug)]
pub enum BodyStatus {
    /// Even when it moves it never collides with anything.
    Static,
    /// Collides with both static and kinematic bodies.
    Kinematic,
}
