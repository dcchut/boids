use std::iter::Sum;
use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

/// `Vector` is a simple 2D Vector type.
#[derive(Debug, PartialEq, Copy, Clone)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
}

impl Vector {
    /// Returns a new `Vector` with the given `x` and `y` coordinates.
    ///
    /// # Example
    /// ```rust
    /// # use dcc_boids::vector::Vector;
    /// let v = Vector::new(3.0, 4.0);
    /// let w = Vector::new(1.0, 2.0);
    ///
    /// assert_eq!(v + w, Vector::new(4.0, 6.0));
    /// assert_eq!(2.0 * v, Vector::new(6.0, 8.0));
    /// ```
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Returns the zero `Vector`.`
    ///
    /// # Example
    /// ```rust
    /// # use dcc_boids::vector::Vector;
    /// assert_eq!(Vector::zero(), Vector::new(0.0, 0.0));
    /// ```
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    /// Returns the square magnitude of the current `Vector`.
    ///
    /// # Examples
    /// ```rust
    /// # use dcc_boids::vector::Vector;
    /// let v = Vector::new(3.0, 4.0);
    ///
    /// // should have square magnitude 3^2 + 4^2 = 25
    /// assert_eq!(v.square_magnitude(), 25.0);
    ///
    /// // should have zero square magnitude
    /// assert_eq!(Vector::zero().square_magnitude(), 0.0);
    /// ```
    pub fn square_magnitude(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    /// Returns the magnitude of the current `Vector`.
    ///
    /// # Examples
    /// ```rust
    /// # use dcc_boids::vector::Vector;
    /// let v = Vector::new(3.0, 4.0);
    ///
    /// // should have magnitude sqrt(3^2 + 4^2) = 5
    /// assert_eq!(v.magnitude(), 5.0);
    /// ```
    pub fn magnitude(self) -> f64 {
        self.square_magnitude().sqrt()
    }

    /// Returns a unit vector parallel to the current `Vector`, or `None` if this vector
    /// is the unit vector.
    ///
    /// # Examples
    /// ```rust
    /// # use dcc_boids::vector::Vector;
    /// // Zero vector cannot be normalized
    /// assert_eq!(Vector::zero().normalize(), None);
    ///
    /// // Any other vector can
    /// let v = Vector::new(5.0, 0.0);
    /// assert_eq!(v.normalize(), Some(Vector::new(1.0, 0.0)));
    /// ```
    pub fn normalize(self) -> Option<Self> {
        let square_magnitude = self.square_magnitude().sqrt();

        if square_magnitude == 0.0 {
            // cannot normalize zero vector
            None
        } else {
            Some(self * (1.0 / square_magnitude))
        }
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
