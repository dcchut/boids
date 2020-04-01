use crate::vector::Vector;
use derive_more::{Add, AddAssign, Mul, Sub, SubAssign};

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ID {
    pub id: usize,
}

impl ID {
    pub fn new(id: usize) -> Self {
        Self { id }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Add, Sub, AddAssign, SubAssign, Mul)]
pub struct Position {
    pub x: f64,
    pub y: f64,
}

impl Position {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn as_vec(self) -> Vector {
        Vector::new(self.x, self.y)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Add, Sub, AddAssign, SubAssign)]
pub struct Velocity {
    pub x: f64,
    pub y: f64,
}

impl Velocity {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn as_vec(self) -> Vector {
        Vector::new(self.x, self.y)
    }
}

/// A `Boid`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Boid;

pub struct BoidNeighbours {
    pub position: Vector,
    pub velocity: Vector,
    pub position_sum: Vector,
    pub velocity_sum: Vector,
    pub x_sum: f64,
    pub y_sum: f64,
    pub neighbour_count: usize,
}

impl BoidNeighbours {
    pub fn new(position: Vector, velocity: Vector) -> Self {
        Self {
            position,
            velocity,
            position_sum: Vector::zero(),
            velocity_sum: Vector::zero(),
            x_sum: 0.0,
            y_sum: 0.0,
            neighbour_count: 0,
        }
    }

    pub fn update(&mut self, other_position: Vector, other_velocity: Vector) {
        self.neighbour_count += 1;
        self.x_sum += other_position.x;
        self.y_sum += other_position.y;
        self.position_sum += other_position - self.position;
        self.velocity_sum += other_velocity - self.velocity;
    }
}

const SEPARATION_FACTOR: f64 = -0.20;
const ALIGNMENT_FACTOR: f64 = 0.05;
const COHESION_FACTOR: f64 = 0.05;
const WALL_FACTOR: f64 = 10.0;

fn lower_threshold(x: f64) -> f64 {
    if x <= 0.0 {
        0.5
    } else {
        (0.5 - 0.005 * x).max(0.0)
    }
}

fn upper_threshold(x: f64, limit: f64) -> f64 {
    if x >= limit {
        0.5
    } else {
        (0.5 - (limit - 0.005 * x)).max(0.0)
    }
}

pub fn update_boid_velocity(bn: &BoidNeighbours) -> Vector {
    // Compute the sum of our position and velocity vectors
    let centre_of_mass = if bn.neighbour_count > 0 {
        let com_x = bn.x_sum / (bn.neighbour_count as f64);
        let com_y = bn.y_sum / (bn.neighbour_count as f64);

        Vector::new(com_x - bn.position.x, com_y - bn.position.y)
    } else {
        Vector::zero()
    };

    // compute wall deltas
    let left_wall = Vector::new(1.0, 0.0) * lower_threshold(bn.position.x);
    let right_wall = Vector::new(-1.0, 0.0) * upper_threshold(bn.position.x, 1900.0);
    let top_wall = Vector::new(0.0, 1.0) * lower_threshold(bn.position.y);
    let bottom_wall = Vector::new(0.0, -1.0) * upper_threshold(bn.position.y, 1060.0);
    let wall = left_wall + right_wall + top_wall + bottom_wall;

    SEPARATION_FACTOR * bn.position_sum
        + ALIGNMENT_FACTOR * bn.velocity_sum
        + COHESION_FACTOR * centre_of_mass
        + WALL_FACTOR * wall
}

// impl Boid {
//     /// Returns a new [`Boid`] instance.
//     pub fn new(position: Vector, velocity: Vector) -> Self {
//         Self { position, velocity }
//     }
//
//     /// Return the new velocity vector for this boid, given a slice containing all of the other
//     /// boids in range of this one.
//     fn next_velocity(&self, boids: &[&Boid]) -> Vector {
//         // Compute the sum of our position and velocity vectors
//         let sum_of_positions: Vector = boids.iter().map(|b| b.position - self.position).sum();
//
//         let sum_of_velocities: Vector = boids.iter().map(|b| b.velocity - self.velocity).sum();
//
//         let centre_of_mass = if !boids.is_empty() {
//             let centre_of_mass_x: f64 =
//                 boids.iter().map(|b| b.position.x).sum::<f64>() / (boids.len() as f64);
//
//             let centre_of_mass_y: f64 =
//                 boids.iter().map(|b| b.position.y).sum::<f64>() / (boids.len() as f64);
//
//             Vector::new(
//                 centre_of_mass_x - self.position.x,
//                 centre_of_mass_y - self.position.y,
//             )
//         } else {
//             Vector::zero()
//         };
//
//         // compute wall deltas
//         let left_wall = Vector::new(1.0, 0.0) * lower_threshold(self.position.x);
//         let right_wall = Vector::new(-1.0, 0.0) * upper_threshold(self.position.x, 1900.0);
//         let top_wall = Vector::new(0.0, 1.0) * lower_threshold(self.position.y);
//         let bottom_wall = Vector::new(0.0, -1.0) * upper_threshold(self.position.y, 1060.0);
//         let wall = left_wall + right_wall + top_wall + bottom_wall;
//
//         SEPARATION_FACTOR * sum_of_positions
//             + ALIGNMENT_FACTOR * sum_of_velocities
//             + COHESION_FACTOR * centre_of_mass
//             + WALL_FACTOR * wall
//     }
//
//     #[inline(always)]
//     pub fn x(&self) -> f64 {
//         self.position.x
//     }
//
//     #[inline(always)]
//     pub fn y(&self) -> f64 {
//         self.position.y
//     }
// }
