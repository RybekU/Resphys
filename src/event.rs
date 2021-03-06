use super::object::{Collider, ColliderHandle, ColliderState};

/// Event generated by the collision engine.  
/// In case of an overlap between a solid body and sensor the solid body is guaranteed to be the first handle.
#[derive(Debug, Clone, Copy)]
pub enum ContactEvent<T> {
    OverlapStarted(ColliderHandle, ColliderHandle, T, T),
    OverlapEnded(ColliderHandle, ColliderHandle, T, T),
    CollisionStarted(ColliderHandle, ColliderHandle, T, T),
    CollisionEnded(ColliderHandle, ColliderHandle, T, T),
}

impl<T: Copy> ContactEvent<T> {
    pub fn new(
        h1: ColliderHandle,
        collider1: &Collider<T>,
        h2: ColliderHandle,
        collider2: &Collider<T>,
    ) -> ContactEvent<T> {
        use ColliderState::*;
        match (&collider1.state, &collider2.state) {
            (Solid, Solid) => {
                ContactEvent::CollisionStarted(h1, h2, collider1.user_tag, collider2.user_tag)
            }
            (Solid, Sensor) => {
                ContactEvent::OverlapStarted(h1, h2, collider1.user_tag, collider2.user_tag)
            }
            (Sensor, Solid) => {
                ContactEvent::OverlapStarted(h2, h1, collider2.user_tag, collider1.user_tag)
            }
            (Sensor, Sensor) => {
                ContactEvent::OverlapStarted(h1, h2, collider1.user_tag, collider2.user_tag)
            }
        }
    }
    // changes started events into ended
    pub(crate) fn into_finished(self) -> ContactEvent<T> {
        match self {
            Self::OverlapStarted(h1, h2, t1, t2) => Self::OverlapEnded(h1, h2, t1, t2),
            Self::CollisionStarted(h1, h2, t1, t2) => Self::CollisionEnded(h1, h2, t1, t2),
            _ => self,
        }
    }
}
