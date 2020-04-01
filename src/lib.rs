pub mod boid;
pub mod vector;

//
// #[derive(Clone, Debug)]
// pub struct BoidCage {
//     pub boids: Vec<Boid>,
// }
//
// impl BoidCage {
//     pub fn new(boids: Vec<Boid>) -> Self {
//         Self { boids }
//     }
//
//     pub fn update_positions(&mut self) {
//         for boid in self.boids.iter_mut() {
//             boid.position += boid.velocity * TICK_RATE;
//         }
//     }
//
//     pub fn update_velocities(&mut self) {
//         let mut boids_within_range = vec![vec![]; self.boids.len()];
//
//         // First, figure out which boids are in range of which other boids
//         for (i, boid) in self.boids.iter().enumerate() {
//             for (j, other_boid) in self.boids.iter().skip(i + 1).enumerate() {
//                 let distance = (boid.position - other_boid.position).square_magnitude();
//                 if distance <= DISTANCE_THRESHOLD {
//                     boids_within_range[i].push(j + i + 1);
//                     boids_within_range[j + i + 1].push(i);
//                 }
//             }
//         }
//
//         // Now figure out the new velocities of each boid
//         let velocity_deltas: Vec<Vector> = boids_within_range
//             .into_iter()
//             .enumerate()
//             .map(|(index, other_boid_indexes)| {
//                 self.boids[index].next_velocity(
//                     &other_boid_indexes
//                         .into_iter()
//                         .map(|idx| &self.boids[idx])
//                         .collect::<Vec<_>>(),
//                 )
//             })
//             .collect();
//
//         // Now modify each boid's velocity
//         for (boid, velocity_delta) in self.boids.iter_mut().zip(velocity_deltas) {
//             boid.velocity += velocity_delta;
//             boid.velocity = boid.velocity.normalize().unwrap() * VELOCITY;
//         }
//     }
// }
