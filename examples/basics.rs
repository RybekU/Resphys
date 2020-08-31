use glam::Vec2;
use macroquad::*;
use resphys::{Collider, ColliderState, AABB};

// Body creation with builder assistance, event iteration and deletion of bodies

extern crate log;

use log::debug;

const FPS_INV: f32 = 1. / 60.;

#[macroquad::main("Basic usage")]
async fn main() {
    simple_logger::init().unwrap();

    let mut physics = resphys::PhysicsWorld::<TagType>::new();
    let mut bodies = resphys::BodySet::new();
    let mut colliders = resphys::ColliderSet::new();

    let rectangle = AABB {
        half_exts: Vec2::new(36., 36.),
    };
    let bottom_bar = AABB {
        half_exts: Vec2::new(36., 8.),
    };

    let body1 = resphys::builder::BodyDesc::new()
        .with_position(Vec2::new(360., 285.)) // x 360 y 285.
        .with_velocity(Vec2::new(75., 48.)) // x 75
        .self_collision(false)
        .build();
    let collider1 = resphys::builder::ColliderDesc::new(rectangle, TagType::Moving)
        .with_offset(Vec2::new(0., 0.));

    let collider1_2 = resphys::builder::ColliderDesc::new(bottom_bar, TagType::MovingSensor)
        .with_offset(Vec2::new(0., 36. - 8.))
        .sensor();

    let body1_handle = bodies.insert(body1);

    let player = colliders
        .insert(collider1.build(body1_handle), &mut bodies, &mut physics)
        .unwrap();
    colliders.insert(collider1_2.build(body1_handle), &mut bodies, &mut physics);

    let body2 = resphys::builder::BodyDesc::new()
        .with_position(Vec2::new(450., 450.))
        .make_static()
        .build();
    let collider2 = resphys::builder::ColliderDesc::new(rectangle, TagType::Collidable);
    let collider2_2 = resphys::builder::ColliderDesc::new(rectangle, TagType::Collidable)
        .with_offset(Vec2::new(0., 80.))
        .sensor();

    let body2_handle = bodies.insert(body2);
    colliders.insert(collider2.build(body2_handle), &mut bodies, &mut physics);
    colliders.insert(collider2_2.build(body2_handle), &mut bodies, &mut physics);

    let body3 = resphys::builder::BodyDesc::new()
        .with_position(Vec2::new(600., 360.))
        .make_static()
        .build();
    let collider3 = resphys::builder::ColliderDesc::new(rectangle, TagType::Collidable)
        .with_offset(Vec2::new(0., -15.));
    let body3_handle = bodies.insert(body3);
    colliders.insert(collider3.build(body3_handle), &mut bodies, &mut physics);

    let body4 = resphys::builder::BodyDesc::new()
        .with_position(Vec2::new(375., 375.))
        .make_static()
        .build();
    let collider4 = resphys::builder::ColliderDesc::new(rectangle, TagType::Sensor).sensor();
    let body4_handle = bodies.insert(body4);
    colliders.insert(collider4.build(body4_handle), &mut bodies, &mut physics);

    let mut remaining_time = 0.;
    let mut counter = 0;
    loop {
        remaining_time += get_frame_time();
        while remaining_time >= FPS_INV {
            physics.step(FPS_INV, &mut bodies, &mut colliders);

            let mut to_remove = Vec::new();
            for event in physics.events().iter() {
                counter += 1;
                debug!("{}: {:?}", counter, event);
                if let resphys::ContactEvent::CollisionStarted(
                    _moving,
                    other,
                    TagType::MovingSensor,
                    _any,
                ) = event
                {
                    to_remove.push(*other);
                }
                if let resphys::ContactEvent::OverlapStarted(
                    other,
                    _moving,
                    _any,
                    TagType::MovingSensor,
                ) = event
                {
                    to_remove.push(*other);
                }
            }

            // Reset velocity on axis with collision
            let vel_mask = {
                let mut vel_mask = Vec2::one();
                for (_, info) in physics.collisions_of(player) {
                    println!("info: {:?}", info);
                    if info.normal.x() == 0. {
                        vel_mask.set_y(0.);
                    } else {
                        vel_mask.set_x(0.);
                    }
                }
                vel_mask
            };
            let player_body_handle = colliders[player].owner;
            let player_body = &mut bodies[player_body_handle];
            player_body.velocity *= vel_mask;

            // TODO: Make interactions etc work fine even if a body is supposed to be removed
            to_remove.into_iter().for_each(|collision_handle| {
                let collider_owner = colliders[collision_handle].owner;
                // physics.remove_collider(collision_handle);
                physics.remove_body(collider_owner, &mut bodies, &mut colliders);
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
    MovingSensor,
    Collidable,
    Sensor,
}
