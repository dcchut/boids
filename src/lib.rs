use std::iter::Sum;
use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct Vector {
    x: f64,
    y: f64,
}

impl Vector {
    /// Returns a new [`Vector`] with the given `x` and `y` coordinate.
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Returns the zero [`Vector`]
    pub fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    pub fn square_magnitude(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub fn normalize(self) -> Self {
        self * (1.0/(self.square_magnitude().sqrt()))
    }
}

impl Add for Vector {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl AddAssign for Vector {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vector {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl SubAssign for Vector {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Sum for Vector {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::zero(), Vector::add)
    }
}

impl Mul<f64> for Vector {
    type Output = Vector;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Mul<Vector> for f64 {
    type Output = Vector;

    fn mul(self, rhs: Vector) -> Self::Output {
        rhs * self
    }
}

#[derive(Debug, Clone)]
pub struct Boid {
    position: Vector,
    velocity: Vector,
}

const SEPARATION_FACTOR: f64 = -0.20;
const ALIGNMENT_FACTOR: f64 = 0.05;
const COHESION_FACTOR: f64 = 0.05;
const WALL_FACTOR: f64 = 30.0;
const DISTANCE_THRESHOLD: f64 = 500.0;
const VELOCITY:f64 = 100.0;
const TICK_RATE: f64 = 1.0/100.0;

fn lower_threshold(x: f64) -> f64 {
    if x > 0.0 {
        1.0/x
    } else {
        20.0
    }
}

fn upper_threshold(x: f64, limit: f64) -> f64 {
    if x < limit {
        1.0/(limit - x)
    } else {
        20.0
    }
}

impl Boid {
    /// Returns a new [`Boid`] instance.
    pub fn new(position: Vector, velocity: Vector) -> Self {
        Self { position, velocity }
    }

    /// Return the new velocity vector for this boid, given a slice containing all of the other
    /// boids in range of this one.
    fn next_velocity(&self, boids: &[&Boid]) -> Vector {
        // Compute the sum of our position and velocity vectors
        let sum_of_positions: Vector = boids.iter().map(|b| b.position - self.position).sum();

        let sum_of_velocities: Vector = boids.iter().map(|b| b.velocity - self.velocity).sum();

        let centre_of_mass = if !boids.is_empty() {
            let centre_of_mass_x: f64 = boids
                .iter()
                .map(|b| b.position.x)
                .sum::<f64>()
                / (boids.len() as f64);

            let centre_of_mass_y: f64 = boids
                .iter()
                .map(|b| b.position.y)
                .sum::<f64>()
                / (boids.len() as f64);

            Vector::new(centre_of_mass_x - self.position.x, centre_of_mass_y - self.position.y)
        } else {
            Vector::zero()
        };

        // compute wall deltas
        let left_wall = Vector::new(1.0, 0.0) * lower_threshold(self.position.x);
        let right_wall = Vector::new(-1.0, 0.0) * upper_threshold(self.position.x, 1920.0);
        let top_wall = Vector::new(0.0, 1.0) * lower_threshold(self.position.y);
        let bottom_wall = Vector::new(0.0, -1.0) * upper_threshold(self.position.y, 1080.0);
        let wall = left_wall + right_wall + top_wall + bottom_wall;

        SEPARATION_FACTOR * sum_of_positions
            + ALIGNMENT_FACTOR * sum_of_velocities
            + COHESION_FACTOR * centre_of_mass
            + WALL_FACTOR * wall
    }

    #[inline(always)]
    pub fn x(&self) -> f64 {
        self.position.x
    }

    #[inline(always)]
    pub fn y(&self) -> f64 {
        self.position.y
    }
}

#[derive(Clone, Debug)]
pub struct BoidCage {
    pub boids: Vec<Boid>,
}

impl BoidCage {
    pub fn new(boids: Vec<Boid>) -> Self {
        Self { boids }
    }

    pub fn update_positions(&mut self) {
        for boid in self.boids.iter_mut() {
            boid.position += TICK_RATE * boid.velocity;
        }
    }

    pub fn update_velocities(&mut self) {
        let mut boids_within_range = vec![vec![]; self.boids.len()];

        // First, figure out which boids are in range of which other boids
        for (i, boid) in self.boids.iter().enumerate() {
            for (j, other_boid) in self.boids.iter().skip(i + 1).enumerate() {
                let distance = (boid.position - other_boid.position).square_magnitude();
                if distance <= DISTANCE_THRESHOLD {
                    boids_within_range[i].push(j + i + 1);
                    boids_within_range[j + i + 1].push(i);
                }
            }
        }

        // Now figure out the new velocities of each boid
        let velocity_deltas: Vec<Vector> = boids_within_range
            .into_iter()
            .enumerate()
            .map(|(index, other_boid_indexes)| {
                self.boids[index].next_velocity(
                    &other_boid_indexes
                        .into_iter()
                        .map(|idx| &self.boids[idx])
                        .collect::<Vec<_>>(),
                )
            })
            .collect();

        // Now modify each boid's velocity
        for (boid, velocity_delta) in self.boids.iter_mut().zip(velocity_deltas) {
            boid.velocity += velocity_delta;
            boid.velocity = boid.velocity.normalize() * VELOCITY;
        }
    }
}
