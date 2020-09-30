# Resphys

Simple 2D collision detection/resolution library supporting **only** AABB. Developed primarily to be used in my private projects and as learning opportunity.

The library currently only depenetrates the shapes, without modifying their velocities. Its goal isn't to provide a complete physics simulation, but rather to provide collision detection and resolution for simple games.

API breaking changes definitely will happen. The library is in no way production ready. It might be good enough for a gamejam, but *may* have some bugs. All issues are highly appreciated, including feedback and bug reports.
<details>
<summary><b>Physics solver</b></summary>

A physics solver is the component of physics engine that determines how collision resolution is performed. It's what determines most of the engine's 'feel'.

The solver used is very, very simple. The bodies move on each axis separately, first on x axis, later on y. 

##### Downsides:
- "skipping corners" when velocity is high (because the movements are done separately), but this behavior requires very high velocities to begin with and can be alleviated by substepping very easily
- individual steps aren't very precise, but in general that's still precise enough for many games and like the previous issue can be alleviated by substepping

##### Upsides:
- easy elimination of so called "ghost collisions"
- efficiently solving the "bullet through paper" problem (currently only partially, with a tolerance up to two times as high as regular discrete collision detection)
- no need to use any real-life units
</details>

### Features
- [x] Body and collider separation
(allows building bodies out of multiple AABBs)
- [x] Solid colliders and sensors
- [x] **[QoL]** Builders for `Body` (`BodyDesc`) and `Collider` (`ColliderDesc`)
- [x] User supplied metadata
- [x] Iteration over `Collider`'s contacts
- [x] `Collision`/`Overlap` event generation
- [x] Collision mask for `Collider`'s
- [ ] **[Optimization]** Broadphase
- [x] Querying the `World` for overlap with arbitrary AABB
- [x] Querying the `World` for overlap with ray (Raycast)
- [ ] **[QoL]** "Simple" version of the interface
- [ ] Tilemap integration (possibly from different crate)

### Inspiration
This library's API is inspired by [nphysics](https://nphysics.org/), [Box2D](https://box2d.org/) and to a lesser extent by the currently emerging [rapier](https://rapier.rs).

The implementations of most intersection and collision algorithms are heavily based on [cute_c2](https://github.com/RandyGaul/cute_headers/blob/master/cute_c2.h).

##### Pull requests
All pull requests are welcome, however I want to treat your work with due respect so please file an issue first if it introduces any significant changes to existing functionality.
