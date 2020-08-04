use glam::Vec2;
use macroquad::*;
use resphys::{Body, BodyState, Shape};

// A test if collision gets resolved properly even if multiple impacts happen

const FPS_INV: f32 = 1. / 60.;

#[macroquad::main("Emitting events")]
async fn main() {
    let mut physics = resphys::PhysicsWorld::<TagType>::new();

    let rectangle = Shape::AABB(Vec2::new(36., 36.));

    let body1 =
        resphys::builder::BodyBuilder::new(rectangle, Vec2::new(360., 285.), TagType::Moving)
            .with_velocity(Vec2::new(0., 1600.))
            .build();
    physics.add(body1);

    let body2 =
        resphys::builder::BodyBuilder::new(rectangle, Vec2::new(340., 450.), TagType::Collidable)
            .make_static()
            .build();
    physics.add(body2);

    let body3 =
        resphys::builder::BodyBuilder::new(rectangle, Vec2::new(360., 450.), TagType::Collidable)
            .make_static()
            .build();
    physics.add(body3);

    let mut remaining_time = 0.;
    loop {
        remaining_time += get_frame_time();
        while remaining_time >= FPS_INV {
            physics.step(FPS_INV);
            remaining_time -= FPS_INV;
        }

        for event in physics.events().iter() {
            println!("{:?}", event);
        }

        clear_background(Color::new(0., 1., 1., 1.));
        for (_, body) in physics.bodies.iter() {
            draw_body(&body);
        }

        next_frame().await
    }
}

fn draw_body(body: &Body<TagType>) {
    let mut color = match body.state {
        BodyState::Solid => BLUE,
        BodyState::Sensor => YELLOW,
    };
    // Quickly change color's alpha
    let fill_color = color;

    color.0[3] = (0.3 * 255.) as u8;
    // This works because there's currently only AABB shape.
    let Shape::AABB(wh) = body.shape;
    draw_rectangle(
        body.position.x(),
        body.position.y(),
        wh.x() * 2.,
        wh.y() * 2.,
        color,
    );
    draw_rectangle_lines(
        body.position.x(),
        body.position.y(),
        wh.x() * 2.,
        wh.y() * 2.,
        3.,
        fill_color,
    );
}
#[derive(Clone, Copy, Debug)]
enum TagType {
    Moving,
    Collidable,
    // Sensor,
}