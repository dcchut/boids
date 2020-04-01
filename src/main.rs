use dcc_boids::boid::{update_boid_velocity, Boid, BoidNeighbours, Position, Velocity, ID};
use ggez::conf::{FullscreenType, WindowMode, WindowSetup};
use ggez::event::EventHandler;
use ggez::{event, nalgebra as na};
use ggez::{graphics, Context, ContextBuilder, GameResult};
use legion::prelude::*;
use rand::Rng;
use std::time::{Duration, Instant};

const UPDATES_PER_SECOND: f32 = 60.0;
// And we get the milliseconds of delay that this update rate corresponds to.
const MILLIS_PER_UPDATE: u64 = (1.0 / UPDATES_PER_SECOND * 1000.0) as u64;

// How long we expect each tick to last for.
const TICK_DURATION: f64 = 1.0 / (UPDATES_PER_SECOND as f64);

/// The distance between two Boids at which they are considered in the same group
const DISTANCE_THRESHOLD: f64 = 800.0;

/// The constant (square magnitude) velocity for each boid
const VELOCITY: f64 = 100.0;

fn main() {
    // Create our boid universe
    let universe = Universe::new();
    let mut world = universe.create_world();

    // Function to create a boid with random position / velocity.
    let boid_maker = |i| {
        let mut rng = rand::thread_rng();
        let x = rng.gen_range(10.0, 1900.0);
        let y = rng.gen_range(10.0, 1000.0);
        let dx = rng.gen_range(-30.0, 30.0);
        let dy = rng.gen_range(-30.0, 30.0);
        (Position::new(x, y), Velocity::new(dx, dy), ID::new(i))
    };

    world.insert((Boid,), (0..1500).map(boid_maker));

    let (mut ctx, mut event_loop) = ContextBuilder::new("dcc-boids", "Robert Usher")
        .window_setup(WindowSetup::default().title("dcc-boid"))
        .window_mode(
            WindowMode::default()
                .dimensions(1920.0, 1080.0)
                .maximized(true)
                .fullscreen_type(FullscreenType::Desktop),
        )
        .build()
        .expect("Failed to create context!");

    let mut runner = WorldRunner::new(world);

    match event::run(&mut ctx, &mut event_loop, &mut runner) {
        Ok(_) => println!("Exited cleanly!"),
        Err(e) => println!("Error occurred: {}", e),
    }
}

struct WorldRunner {
    world: World,
    last_update: Instant,
}

impl WorldRunner {
    pub fn new(world: World) -> Self {
        Self {
            world,
            last_update: Instant::now(),
        }
    }

    /// Updates the current position of every object in the world based on its velocity.
    pub fn update_positions(&mut self) {
        // Get every object that has both a Position and a Velocity.
        let query = <(Write<Position>, Read<Velocity>)>::query();

        for (mut position, velocity) in query.iter(&mut self.world) {
            // Update position based on the velocity
            position.x += velocity.x * TICK_DURATION;
            position.y += velocity.y * TICK_DURATION;
        }
    }

    /// Updates the current velocities of each boid in the world
    pub fn update_velocities(&mut self) {
        let boid_query = <(Read<Position>, Write<Velocity>)>::query().filter(tag::<Boid>());

        let mut neighbours = vec![];
        let mut pairs = vec![];
        let mut v_refs = vec![];

        for (position, velocity) in boid_query.iter(&mut self.world) {
            neighbours.push(BoidNeighbours::new(position.as_vec(), velocity.as_vec()));
            pairs.push((position.as_vec(), velocity.as_vec()));
            v_refs.push(velocity);
        }

        for (i, (position, velocity)) in pairs.iter().enumerate() {
            for (j, (other_position, other_velocity)) in pairs.iter().skip(i + 1).enumerate() {
                // compute the distance between these two boids
                let distance = (*position - *other_position).square_magnitude();

                if distance <= DISTANCE_THRESHOLD {
                    neighbours[i].update(*other_position, *other_velocity);
                    neighbours[i + j + 1].update(*position, *velocity);
                }
            }

            let v_direction = *velocity + update_boid_velocity(&neighbours[i]);
            let new_velocity = v_direction.normalize().unwrap() * VELOCITY;
            *v_refs[i] = Velocity::new(new_velocity.x, new_velocity.y);
        }
    }
}

impl EventHandler for WorldRunner {
    fn update(&mut self, _ctx: &mut Context) -> GameResult<()> {
        // Only update the game state if enough frames have elapsed
        if Instant::now() - self.last_update >= Duration::from_millis(MILLIS_PER_UPDATE) {
            self.update_velocities();
            self.update_positions();
            self.last_update = Instant::now();
        }

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx, graphics::WHITE);

        let circle = graphics::Mesh::new_circle(
            ctx,
            graphics::DrawMode::fill(),
            na::Point2::new(0.0, 0.0),
            5.0,
            2.0,
            graphics::BLACK,
        )?;

        // Iterate over the positions of all boids
        let query = Read::<Position>::query().filter(tag::<Boid>());

        // Draw each boid at its current position
        for position in query.iter(&mut self.world) {
            graphics::draw(ctx, &circle, ([position.x as f32, position.y as f32],))?;
        }

        graphics::present(ctx)?;

        ggez::timer::yield_now();

        Ok(())
    }
}
