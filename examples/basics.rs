use glam::Vec2;
use macroquad::*;
use resphys::{Collider, ColliderState, Shape};

// Body creation with builder assistance, event iteration and deletion of bodies

extern crate log;

use log::debug;

const FPS_INV: f32 = 1. / 60.;

#[macroquad::main("Emitting events")]
async fn main() {
    simple_logger::init().unwrap();

    let mut physics = resphys::PhysicsWorld::<TagType>::new();

    let rectangle = Shape::AABB(Vec2::new(36., 36.));
    let bar = Shape::AABB(Vec2::new(36., 8.));

    let (body1, mut collider1) =
        resphys::builder::BodyBuilder::new(rectangle, Vec2::new(360., 285.), TagType::Moving)
            .with_velocity(Vec2::new(75., 48.))
            .self_collision(false)
            .build();
    let body1_handle = physics.add_body(body1);
    collider1.owner = body1_handle;
    let mut collider1_2 = collider1.clone();
    collider1_2.shape = bar;
    collider1_2.offset = Vec2::new(0., 36. - 8.);
    collider1_2.state = ColliderState::Sensor;
    collider1_2.user_tag = TagType::MovingSensor;
    physics.add_collider(collider1);
    physics.add_collider(collider1_2);

    let (body2, mut collider2) =
        resphys::builder::BodyBuilder::new(rectangle, Vec2::new(450., 450.), TagType::Collidable)
            .make_static()
            .build();
    let body2_handle = physics.add_body(body2);
    collider2.owner = body2_handle;
    physics.add_collider(collider2);

    let (body3, mut collider3) =
        resphys::builder::BodyBuilder::new(rectangle, Vec2::new(600., 360.), TagType::Collidable)
            .make_static()
            .build();
    let body3_handle = physics.add_body(body3);
    collider3.owner = body3_handle;
    physics.add_collider(collider3);

    let (body4, mut collider4) =
        resphys::builder::BodyBuilder::new(rectangle, Vec2::new(375., 375.), TagType::Sensor)
            .make_static()
            .sensor()
            .build();
    let body4_handle = physics.add_body(body4);
    collider4.owner = body4_handle;
    physics.add_collider(collider4);

    let mut remaining_time = 0.;
    loop {
        remaining_time += get_frame_time();
        while remaining_time >= FPS_INV {
            physics.step(FPS_INV);

            for event in physics.events().iter() {
                debug!("{:?}", event);
            }

            remaining_time -= FPS_INV;
        }

        clear_background(Color::new(0., 1., 1., 1.));
        for (_, collider) in physics.colliders.iter() {
            let body = physics.get_body(collider.owner).unwrap();
            draw_collider(&collider, body.position);
        }

        next_frame().await
    }
}

fn draw_collider(collider: &Collider<TagType>, position: Vec2) {
    let mut color = match collider.state {
        ColliderState::Solid => BLUE,
        ColliderState::Sensor => YELLOW,
    };
    // Quickly change color's alpha
    let fill_color = color;

    color.0[3] = (0.3 * 255.) as u8;
    // This works because there's currently only AABB shape. Half extents.
    let Shape::AABB(wh) = collider.shape;
    let x_pos = position.x() - wh.x() + collider.offset.x();
    let y_pos = position.y() - wh.y() + collider.offset.y();
    draw_rectangle(x_pos, y_pos, wh.x() * 2., wh.y() * 2., color);
    draw_rectangle_lines(x_pos, y_pos, wh.x() * 2., wh.y() * 2., 3., fill_color);
}
#[derive(Clone, Copy, Debug)]
enum TagType {
    Moving,
    MovingSensor,
    Collidable,
    Sensor,
}
