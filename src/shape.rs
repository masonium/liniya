//! Trace-able shapes to render
use super::common::*;
use ncollide3d::{bounding_volume::AABB, query::{Ray, RayCast}, shape::Cuboid};

pub type Path = Vec<Point3<f64>>;
pub type Paths = Vec<Path>;

/// NOTE: Not sure if this will actually be needed.
/// Intersection 'at' t-value and point.
pub struct Intersection {
    /// 'time' along direction where intersection occurs
    pub t: f64,

    /// point of intersection with shape
    pub point: Point3<f64>,
}

/// Renderable 3D with a colllection of paths on the 'surface' of the
/// shape.
///
/// The underlying shape is use to determine visibility of the
/// paths. Rendering is thus only guaranteed if the points of the path
/// lie 'on' the shape within some tolerance.
pub trait Shape {
    /// Ray-intersection
    fn intersect(&self, ray: &Ray<f64>, max_toi: f64) -> Option<Intersection>;

    /// Return the set of paths that lie on the shape to render.
    fn paths(&self) -> Paths;

    /// Return the bounding volume for this shape.
    fn bounding_box(&self) -> AABB<f64>;

    fn name(&self) -> String {
	"Shape".to_string()
    }
}

pub trait Textureable: Shape {
    /// Transformation from 2-D texture coordinates to on-shape point.
    fn uv_to_point(&self, uv: &Point2<f64>) -> Option<Point3<f64>>;
}

/// Sphere with lattitude-line paths at equal-angle spacing.
#[derive(Clone)]
pub struct LatSphere {
    transform: Isometry3<f64>,

    pos: Point3<f64>,
    radius: f64,
    /// internal use of a ncollide3d Sphere object for ray-intersection
    shape: ncollide3d::shape::Ball<f64>,

    /// Angle spacing for lines
    spacing_angle: f64,
}

impl LatSphere {
    pub fn new(pos: &Point3<f64>, radius: f64, spacing_angle: f64) -> LatSphere {
        let ball = ncollide3d::shape::Ball::new(radius*0.99);
        let transform = na::Isometry3::translation(pos.x, pos.y, pos.z);
        LatSphere {
            transform,
            pos: *pos,
            shape: ball,
            radius,
            spacing_angle,
        }
    }

    fn circle_path(&self, angle: f64) -> Path {
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

impl Shape for LatSphere {
    fn name(&self) -> String {
	format!("Sphere")
    }
    fn intersect(&self, ray: &Ray<f64>, max_toi: f64) -> Option<Intersection> {
        self.shape
            .toi_with_ray(&self.transform, ray, max_toi, true)
            .map(|t| Intersection {
                t,
                point: ray.point_at(t),
            })
    }

    /// Latitude circles with centers on the y-axis, spaced by `spacing_angle`.
    fn paths(&self) -> Paths {
        // center cirlce
        let mut paths = vec![self.circle_path(0.0)];
        let mut rising_angle = 0.0;
        while rising_angle < std::f64::consts::FRAC_PI_2 {
            paths.push(self.circle_path(rising_angle));
            paths.push(self.circle_path(-rising_angle));
            rising_angle += self.spacing_angle;
        }
        paths
    }
    fn bounding_box(&self) -> AABB<f64> {
        let half_extents = Vector3::new(self.radius, self.radius, self.radius);
        AABB::from_half_extents(self.pos, half_extents)
    }
}

/// Box with paths on all of the edges.
pub struct BoxOutline {
    pos: Point3<f64>,
    half_extents: Vector3<f64>,
    aabb: AABB<f64>
}

impl BoxOutline {
    pub fn new(pos: Point3<f64>, half_extents: Vector3<f64>) -> Self {
	let aabb = AABB::from_half_extents(pos, half_extents);
	BoxOutline {
	    pos,
	    half_extents,
	    aabb,
	}
    }
}

impl Shape for BoxOutline {
    fn name(&self) -> String {
	format!("Box {}", self.pos)
    }
	
    fn intersect(&self, ray: &Ray<f64>, max_toi: f64) -> Option<Intersection> {
        self.aabb.toi_with_ray(&Isometry3::identity(), ray, max_toi, true)
	    .map(|t| { 
		eprintln!("Ray intersects at {}", t);
		Intersection {
                    t,
                    point: ray.point_at(t),
		}
	    })

    }

    fn paths(&self) -> Paths {
	let mut corners = Vec::with_capacity(8);
	for i in &[-1.0, 1.0] {
	    for j in  &[-1.0, 1.0] {
		for k in &[-1.0, 1.0] {
		    corners.push(Point3::new(self.pos.x + i * self.half_extents.x,
					     self.pos.y + j * self.half_extents.y,
					     self.pos.z + k * self.half_extents.z));
		}
	    }
	}
	vec![vec![corners[0], corners[1]],
	     vec![corners[0], corners[2]],
	     vec![corners[1], corners[3]],
	     vec![corners[2], corners[3]],
	     vec![corners[4], corners[5]],
	     vec![corners[4], corners[6]],
	     vec![corners[5], corners[7]],
	     vec![corners[6], corners[7]],
	     vec![corners[0], corners[4]],
	     vec![corners[1], corners[5]],
	     vec![corners[2], corners[6]],
	     vec![corners[3], corners[7]]
	]
    }

    fn bounding_box(&self) -> AABB<f64> {
        self.aabb
    }
}
