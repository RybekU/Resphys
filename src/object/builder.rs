pub use super::super::collision::Shape;
pub use super::{Body, BodyHandle, BodyStatus, Collider, ColliderState};
use glam::Vec2;

/// Builder for the `Body`. Start with `new`, finish with `build`.
#[derive(Debug, Clone)]
pub struct BodyDesc {
    pub position: Vec2,

    pub velocity: Vec2,
    pub status: BodyStatus,
    pub self_collide: bool,
}

impl BodyDesc {
    pub fn new() -> Self {
        Self {
            position: Vec2::zero(),
            velocity: Vec2::zero(),
            status: BodyStatus::Kinematic,
            self_collide: true,
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
    pub fn self_collision(mut self, check: bool) -> Self {
        self.self_collide = check;
        self
    }
    pub fn build(self) -> Body {
        Body::new(self.position, self.velocity, self.status, self.self_collide)
    }
}

// Builder for the `Collider`. Start with `new`, finish with `build`.
#[derive(Debug, Clone)]
pub struct ColliderDesc<T> {
    pub shape: Shape,
    pub offset: Vec2,
    pub state: ColliderState,

    pub category_bits: u32,
    pub mask_bits: u32,

    pub user_tag: T,
}

impl<T: Copy> ColliderDesc<T> {
    pub fn new(shape: Shape, user_tag: T) -> Self {
        Self {
            shape,
            offset: Vec2::zero(),
            state: ColliderState::Solid,
            category_bits: 1,
            mask_bits: u32::MAX,
            user_tag,
        }
    }
    pub fn with_shape(mut self, shape: Shape) -> Self {
        self.shape = shape;
        self
    }
    pub fn with_offset(mut self, offset: Vec2) -> Self {
        self.offset = offset;
        self
    }
    pub fn sensor(mut self) -> Self {
        self.state = ColliderState::Sensor;
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
    pub fn build(self, owner: BodyHandle) -> Collider<T> {
        Collider::new(
            self.shape,
            self.offset,
            self.state,
            self.category_bits,
            self.mask_bits,
            self.user_tag,
            owner,
        )
    }
}
