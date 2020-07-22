pub use super::super::collision::Shape;
pub use super::{Body, BodyHandle, BodyState, BodyType};
use glam::Vec2;

/// Builder for the `Body`. Start with `new`, finish with `build`.
#[derive(Debug, Clone)]
pub struct BodyBuilder<T> {
    pub shape: Shape,
    pub position: Vec2,

    pub velocity: Vec2,
    pub btype: BodyType,
    pub state: BodyState,

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
            btype: BodyType::Dynamic,
            state: BodyState::Solid,
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
        self.btype = BodyType::Static;
        self
    }
    pub fn sensor(mut self) -> Self {
        self.state = BodyState::Sensor;
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
    pub fn build(self) -> Body<T> {
        Body::new(
            self.shape,
            self.position,
            self.velocity,
            self.btype,
            self.state,
            self.category_bits,
            self.mask_bits,
            self.user_tag,
        )
    }
}
