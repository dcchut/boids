use dcc_boids::{Boid, BoidCage, Vector};
use ggez::event::EventHandler;
use ggez::{nalgebra as na, event};
use ggez::{graphics, Context, ContextBuilder, GameResult};
use rand::Rng;
use std::time::{Instant, Duration};
use ggez::conf::{WindowMode, FullscreenType, WindowSetup};

const UPDATES_PER_SECOND: f32 = 60.0;
// And we get the milliseconds of delay that this update rate corresponds to.
const MILLIS_PER_UPDATE: u64 = (1.0 / UPDATES_PER_SECOND * 1000.0) as u64;


fn main() {
    let mut rng = rand::thread_rng();

    let mut boids = vec![];

    for _ in 0..750 {
        let x = rng.gen_range(10.0, 1900.0);
        let y = rng.gen_range(10.0, 1000.0);
        let v_x = rng.gen_range(-30.0, 30.0);
        let v_y = rng.gen_range(-30.0, 30.0);

        boids.push(Boid::new(Vector::new(x, y), Vector::new(v_x, v_y)));
    }

    let cage = BoidCage::new(boids);

    let (mut ctx, mut event_loop) = ContextBuilder::new("dcc-boids", "Robert Usher")
        .window_setup( WindowSetup::default().title("dcc-boid"))
        .window_mode(WindowMode::default().dimensions(1920.0, 1080.0).maximized(true).fullscreen_type(FullscreenType::Desktop))
        .build()
        .expect("Failed to create context!");

    let mut runner = BoidRunner::new(cage);

    match event::run(&mut ctx, &mut event_loop, &mut runner) {
        Ok(_) => println!("Exited cleanly!"),
        Err(e) => println!("Error occurred: {}", e),
    }
}

struct BoidRunner {
    cage: BoidCage,
    last_update: Instant,
}

impl BoidRunner {
    pub fn new(cage: BoidCage) -> Self {
        Self {
            cage,
            last_update: Instant::now(),
        }
    }
}

impl EventHandler for BoidRunner {
    fn update(&mut self, _ctx: &mut Context) -> GameResult<()> {
        // Only update the game state if enough frames have elapsed
        if Instant::now() - self.last_update >= Duration::from_millis(MILLIS_PER_UPDATE) {
            self.cage.update_velocities();
            self.cage.update_positions();
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
            3.0,
            2.0,
            graphics::BLACK,
        )?;

        for boid in self.cage.boids.iter() {
            graphics::draw(ctx, &circle, (na::Point2::new(boid.x() as f32, boid.y() as f32), ))?;
        }

        graphics::present(ctx)?;

        ggez::timer::yield_now();

        Ok(())
    }
}
