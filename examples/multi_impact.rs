use glam::Vec2;
use macroquad::*;
use resphys::{Collider, ColliderState, AABB};

// A test if collision gets resolved properly even if multiple impacts happen

const FPS_INV: f32 = 1. / 60.;

#[macroquad::main("Hitting multiple colliders in a single step")]
async fn main() {
    let mut physics = resphys::PhysicsWorld::<TagType>::new();
    let mut bodies = resphys::BodySet::new();
    let mut colliders = resphys::ColliderSet::new();

    let rectangle = AABB {
        half_exts: Vec2::new(36., 36.),
    };

    let body1 = resphys::builder::BodyDesc::new()
        .with_position(Vec2::new(360., 0.))
        .with_velocity(Vec2::new(0., 1600.))
        .build();
    let collider1 = resphys::builder::ColliderDesc::new(rectangle, TagType::Moving);

    let handle1 = bodies.insert(body1);
    colliders.insert(collider1.build(handle1), &mut bodies, &mut physics);

    let body2 = resphys::builder::BodyDesc::new()
        .with_position(Vec2::new(340., 450.))
        .make_static()
        .build();
    let collider2 = resphys::builder::ColliderDesc::new(rectangle, TagType::Collidable);
    let handle2 = bodies.insert(body2);
    colliders.insert(collider2.build(handle2), &mut bodies, &mut physics);

    let body3 = resphys::builder::BodyDesc::new()
        .with_position(Vec2::new(360., 450.))
        .make_static()
        .build();
    let collider3 = resphys::builder::ColliderDesc::new(rectangle, TagType::Collidable);

    let handle3 = bodies.insert(body3);
    colliders.insert(collider3.build(handle3), &mut bodies, &mut physics);

    let mut remaining_time = 0.;
    loop {
        remaining_time += get_frame_time();
        while remaining_time >= FPS_INV {
            physics.step(FPS_INV, &mut bodies, &mut colliders);

            for event in physics.events().iter() {
                println!("{:?}", event);
            }

            physics
                .interactions_of(resphys::ColliderHandle { 0: 0 })
                .for_each(|(first, second)| {
                    println!("Collider: {:?}, Weight: {:?}", first, second)
                });

            remaining_time -= FPS_INV;
        }

        clear_background(Color::new(0., 1., 1., 1.));
        for (_, collider) in colliders.iter() {
            let body = &bodies[collider.owner];
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
    let wh = collider.shape.half_exts;
    let x_pos = position.x() - wh.x() + collider.offset.x();
    let y_pos = position.y() - wh.y() + collider.offset.y();
    draw_rectangle(x_pos, y_pos, wh.x() * 2., wh.y() * 2., color);
    draw_rectangle_lines(x_pos, y_pos, wh.x() * 2., wh.y() * 2., 3., fill_color);
}

#[derive(Clone, Copy, Debug)]
enum TagType {
    Moving,
    Collidable,
    // Sensor,
}
