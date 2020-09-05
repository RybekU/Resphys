use super::intersection_aabb_aabb;
use glam::Vec2;

#[derive(Debug, Clone)]
pub struct Ray {
    pub origin: Vec2,
    pub dir: Vec2,
    pub toi: f32,
}

#[derive(Debug, Clone)]
pub struct Raycast {
    pub toi: f32,
    pub normal: Vec2,
}

// ported https://github.com/RandyGaul/cute_headers/blob/master/cute_c2.h#L1427
#[allow(clippy::float_cmp)]
pub fn contact_ray_aabb(ray: &Ray, aabb_pos: Vec2, aabb_half_exts: Vec2) -> Option<Raycast> {
    // test if collision with ray is possible by simplifying to aabb2aabb test
    let dest = ray.origin + ray.dir * ray.toi;
    let ray_box_min = ray.origin.min(dest);
    let ray_box_max = ray.origin.max(dest);
    let ray_box_dim = ray_box_max - ray_box_min;

    if !intersection_aabb_aabb(
        ray_box_min + ray_box_dim * 0.5,
        ray_box_dim * 0.5,
        aabb_pos,
        aabb_half_exts,
    ) {
        return None;
    }

    let ab = dest - ray.origin;
    let n = clockwise_90_turn(ab);
    let abs_n = n.abs();
    let d = n.dot(ray.origin - aabb_pos).abs() - abs_n.dot(aabb_half_exts);
    if d > 0. {
        return None;
    }

    let t_left = ray_plane_1d(
        Vec2::new(ray.origin.x(), dest.x()),
        -1.,
        aabb_pos.x() - aabb_half_exts.x(),
    );
    let t_right = ray_plane_1d(
        Vec2::new(ray.origin.x(), dest.x()),
        1.,
        aabb_pos.x() + aabb_half_exts.x(),
    );
    let t_top = ray_plane_1d(
        Vec2::new(ray.origin.y(), dest.y()),
        -1.,
        aabb_pos.y() - aabb_half_exts.y(),
    );
    let t_bottom = ray_plane_1d(
        Vec2::new(ray.origin.y(), dest.y()),
        1.,
        aabb_pos.y() + aabb_half_exts.y(),
    );

    // Calculate hit predicate
    let hit_left = t_left < 1.;
    let hit_right = t_right < 1.;
    let hit_top = t_top < 1.;
    let hit_bottom = t_bottom < 1.;
    let hit = hit_left | hit_right | hit_top | hit_bottom;

    if hit {
        // Remap t's within 0-1 range, where >= 1 is treated as 0
        let t_left = hit_left as u8 as f32 * t_left;
        let t_right = hit_right as u8 as f32 * t_right;
        let t_top = hit_top as u8 as f32 * t_top;
        let t_bottom = hit_bottom as u8 as f32 * t_bottom;

        let t_max = t_left.max(t_right).max(t_top).max(t_bottom);
        let toi = ray.toi * t_max;

        // result of multiple calls to `max` so there shouldn't be any issue with floating point error
        let normal = if t_left == t_max {
            Vec2::unit_x() * -1.
        } else if t_right == t_max {
            Vec2::unit_x()
        } else if t_top == t_max {
            Vec2::unit_y() * -1.
        } else {
            Vec2::unit_y()
        };
        Some(Raycast { toi, normal })
    } else {
        None
    }
}

fn clockwise_90_turn(vec: Vec2) -> Vec2 {
    Vec2::new(-vec.y(), vec.x())
}

// both ray and plane are one dimensional
// returns value between 0 to 1 describing time of impact, 1 means no impact
fn ray_plane_1d(ray_1d: Vec2, normal: f32, plane_1d: f32) -> f32 {
    let d = (ray_1d - Vec2::splat(plane_1d)) * normal;

    ray_plane_1d_time(d.x(), d.y())
}

fn ray_plane_1d_time(da: f32, db: f32) -> f32 {
    // Ray started behind plane
    if da < 0. {
        0.
    }
    // Ray doesn't intersect the plane
    else if da * db >= 0. {
        1.
    }
    // Ray is too tiny
    else if (da - db) < f32::EPSILON {
        0.
    } else {
        da / (da - db)
    }
}
