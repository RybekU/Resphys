mod collision;
mod event;
mod object;
mod world;

pub use self::collision::*;
pub use self::event::ContactEvent;
pub use self::object::*;
pub use self::world::*;

// TODO: tests once public API is more defined...
#[cfg(test)]
mod tests {
    #[test]
    fn todo() {
        assert_eq!(2 + 2, 4);
    }
}
