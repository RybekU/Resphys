pub use super::super::collision::Shape;
pub use super::{Body, BodyHandle, BodyStatus, Collider, ColliderState};
use glam::Vec2;

/// Builder for the `Body`. Start with `new`, finish with `build`.
#[derive(Debug, Clone)]
pub struct BodyBuilder<T> {
    pub shape: Shape,
    pub position: Vec2,

    pub velocity: Vec2,
    pub status: BodyStatus,
    pub self_collide: bool,
    pub state: ColliderState,

    pub category_bits: u32,
    pub mask_bits: u32,

    pub user_tag: T,
}

impl<T: Copy> BodyBuilder<T> {
    pub fn new(shape: Shape, position: Vec2, user_tag: T) -> Self {
        Self {
            shape,
            position,
            velocity: Vec2::zero(),
            status: BodyStatus::Kinematic,
            self_collide: true,
            state: ColliderState::Solid,
            category_bits: 1,
            mask_bits: u32::MAX,
            user_tag,
        }
    }
    pub fn with_position(mut self, position: Vec2) -> Self {
        self.position = position;
        self
    }
    pub fn with_velocity(mut self, velocity: Vec2) -> Self {
        self.velocity = velocity;
        self
    }
    pub fn make_static(mut self) -> Self {
        self.status = BodyStatus::Static;
        self
    }
    pub fn sensor(mut self) -> Self {
        self.state = ColliderState::Sensor;
        self
    }
    pub fn self_collision(mut self, check: bool) -> Self {
        self.self_collide = check;
        self
    }
    pub fn with_category(mut self, category_bits: u32) -> Self {
        self.category_bits = category_bits;
        self
    }
    pub fn with_mask(mut self, mask_bits: u32) -> Self {
        self.mask_bits = mask_bits;
        self
    }
    pub fn with_tag(mut self, user_tag: T) -> Self {
        self.user_tag = user_tag;
        self
    }
    pub fn build(self) -> (Body, Collider<T>) {
        (
            Body::new(self.position, self.velocity, self.status, self.self_collide),
            Collider::new(
                self.shape,
                Vec2::zero(),
                self.state,
                self.category_bits,
                self.mask_bits,
                self.user_tag,
                BodyHandle { 0: 0 },
            ),
        )
    }
}
