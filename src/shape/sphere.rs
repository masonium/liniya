use crate::common::*;
use crate::shape::{Camera, Shape, Path, Paths};
use ncollide3d::query::RayCast;

/// Sphere with lattitude and longitude lines oriented around the y-axis.
#[derive(Clone)]
pub struct Sphere {
    transform: Isometry3<f64>,

    pos: Point3<f64>,
    radius: f64,
    /// internal use of a ncollide3d Sphere object for ray-intersection
    shape: ncollide3d::shape::Ball<f64>,

    /// angle spacing for latitude lines
    lat_angle: Option<f64>,

    /// angle spancing for longitude lines.
    long_angle: Option<f64>,
}

impl Sphere {
    pub fn new(
        pos: &Point3<f64>,
        radius: f64,
        lat_angle: Option<f64>,
        long_angle: Option<f64>,
    ) -> Sphere {
        let ball = ncollide3d::shape::Ball::new(radius * 0.99);
        let transform = na::Isometry3::translation(pos.x, pos.y, pos.z);
        Sphere {
            transform,
            pos: *pos,
            shape: ball,
            radius,
            lat_angle,
            long_angle,
        }
    }

    /// Return a path trading a latitutde line at a sepcific latitude.
    fn latitude_path(&self, angle: f64) -> Path {
        const N_SEGMENTS: usize = 100;
        let (s, c) = angle.sin_cos();
        let radius_to_axis = c * self.radius;
        let y = s * self.radius;
        (0..=N_SEGMENTS)
            .map(|i| {
                let theta = std::f64::consts::TAU / (N_SEGMENTS as f64) * i as f64;
                let (s, c) = theta.sin_cos();
                self.pos + Vector3::new(radius_to_axis * s, y, radius_to_axis * c)
            })
            .collect()
    }
}

impl Shape for Sphere {
    fn name(&self) -> String {
        format!("Sphere")
    }
    fn intersect(&self, ray: &Ray<f64>, max_toi: f64) -> Option<f64> {
        self.shape
            .toi_with_ray(&self.transform, ray, max_toi, true)
    }

    fn paths(&self, _camera: &Camera) -> Paths {
	let mut paths = vec![];
	if let Some(lat_angle) = self.lat_angle {
            paths.push(self.latitude_path(0.0));
            let mut rising_angle = 0.0;
            while rising_angle < std::f64::consts::FRAC_PI_2 {
		paths.push(self.latitude_path(rising_angle));
		paths.push(self.latitude_path(-rising_angle));
		rising_angle += lat_angle;
            }
	}

        paths
    }
    fn bounding_box(&self) -> AABB<f64> {
        let half_extents = Vector3::new(self.radius, self.radius, self.radius);
        AABB::from_half_extents(self.pos, half_extents)
    }
}
