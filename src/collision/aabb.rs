use glam::Vec2;

#[derive(Default, Copy, Clone, Debug)]
pub struct AABB {
    /// half width, half height
    pub half_exts: Vec2,
}

#[derive(Debug, Clone)]
pub struct Contact {
    pub depth: f32,
    pub normal: Vec2,
    pub contact_point: Vec2,
}

impl Contact {
    pub fn new(depth: f32, normal: Vec2, contact_point: Vec2) -> Self {
        Self {
            depth,
            normal,
            contact_point,
        }
    }
}

#[derive(Debug)]
pub struct ContactManifold {
    pub contact_x: Contact,
    pub contact_y: Contact,
}

impl ContactManifold {
    pub fn best_contact(&self) -> &Contact {
        if self.contact_x.depth < self.contact_y.depth {
            &self.contact_x
        } else {
            &self.contact_y
        }
    }
}

// ported https://github.com/RandyGaul/cute_headers/blob/master/cute_c2.h#L1193
pub fn collision_aabb_aabb(a_loc: Vec2, a_half_exts: Vec2, b_loc: Vec2, b_half_exts: Vec2) -> bool {
    let a_min = a_loc - a_half_exts;
    let a_max = a_loc + a_half_exts;
    let b_min = b_loc - b_half_exts;
    let b_max = b_loc + b_half_exts;

    let d0 = b_max.x() < a_min.x();
    let d1 = a_max.x() < b_min.x();
    let d2 = b_max.y() < a_min.y();
    let d3 = a_max.y() < b_min.y();

    !(d0 || d1 || d2 || d3)
}

// contact points aren't precise - they are currently in the middle on the correct side
pub fn collision_aabb_aabb_manifold(
    a_loc: Vec2,
    a_half_exts: Vec2,
    b_loc: Vec2,
    b_half_exts: Vec2,
) -> Option<ContactManifold> {
    let distance = b_loc - a_loc;

    let overlap = a_half_exts + b_half_exts - distance.abs();

    if overlap.x() < 0. || overlap.y() < 0. {
        return None;
    }

    let depth1 = overlap.x();
    let normal1 = Vec2::new(distance.x().signum(), 0.);
    let contact_point_x = Vec2::new(a_loc.x() + a_half_exts.x() * normal1.x(), a_loc.y());
    let contact1 = Contact::new(depth1, normal1, contact_point_x);

    let depth2 = overlap.y();
    let normal2 = Vec2::new(0., distance.y().signum());
    let contact_point_y = Vec2::new(a_loc.x(), a_loc.y() + a_half_exts.y() * normal2.y());
    let contact2 = Contact::new(depth2, normal2, contact_point_y);

    Some(ContactManifold {
        contact_x: contact1,
        contact_y: contact2,
    })
}
