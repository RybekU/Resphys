mod collision;
mod event;
mod object;
mod world;

pub use self::collision::*;
pub use self::event::PhysicsEvent;
pub use self::object::*;
pub use self::world::*;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
