use dcc_boids::{Boid, BoidCage, Vector};
use gifski::progress::NoProgress;
use gifski::{CatResult, Collector};
use std::fs::File;
use std::path::PathBuf;
use std::thread;
use imageproc::drawing::draw_filled_circle_mut;
use image::{ImageBuffer, Rgb};
use rand::Rng;

struct Lodecoder {
    frames: Vec<PathBuf>,
    fps: usize,
}

impl Lodecoder {
    pub fn new(frames: Vec<PathBuf>, fps: usize) -> Self {
        Self { frames, fps }
    }

    fn collect(&mut self, mut dest: Collector) -> CatResult<()> {
        for (i, frame) in self.frames.drain(..).enumerate() {
            let delay = ((i + 1) * 100 / self.fps) - (i * 100 / self.fps); // See telecine/pulldown.
            dest.add_frame_png_file(i, frame, delay as f64)?;
        }
        Ok(())
    }
}

fn main() {
    let mut rng = rand::thread_rng();

    let mut boids = vec![];

    for _ in 0..750 {
        let x = rng.gen_range(10.0, 1900.0);
        let y = rng.gen_range(10.0, 1000.0);
        let v_x = rng.gen_range(-30.0, 30.0);
        let v_y = rng.gen_range(-30.0, 30.0);

        boids.push(
            Boid::new(Vector::new(x, y), Vector::new(v_x, v_y))
        );
    }

    let mut cage = BoidCage::new(boids);
    let mut files = Vec::new();
    let dir = tempfile::tempdir().unwrap();


    for tick in 0..1000 {
        let mut buffer = ImageBuffer::new(1920, 1080);

        for boid in cage.boids.iter() {
            draw_filled_circle_mut(&mut buffer, (boid.x() as i32, boid.y() as i32), 1, Rgb([255u8, 0u8, 100u8]));
        }

        let filename = dir
            .path()
            .join(format!("frame-{:08}.png", tick));

        buffer.save(&filename).unwrap();
        files.push(filename);
        cage.update_velocities();
        cage.update_positions();
    }

    let settings = gifski::Settings {
        width: None,
        height: None,
        quality: 100,
        once: false,
        fast: false,
    };

    let mut decoder = Box::new(Lodecoder::new(files, 100));
    let (collector, writer) = gifski::new(settings).expect("Failed to initialise gifski");
    let decode_thread = thread::spawn(move || decoder.collect(collector));
    let file = File::create("zz.gif").expect("Couldn't open output file");
    writer
        .write(file, &mut NoProgress {})
        .expect("Failed to write");
    let _ = decode_thread.join().expect("Failed to decode");


    drop(dir);
}
