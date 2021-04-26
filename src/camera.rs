//! Perspective or orthographic cameras for rendering scenes
pub use super::common::*;

pub struct Camera {
    world_camera_iso: na::Isometry3<f64>,
    // world_camera_matrix: Matrix4<f64>,
    projection: na::Perspective3<f64>,
}

impl Camera {
    /// Create a new camera with perepctive projection.
    pub fn look_at_perspective(origin: &Point3<f64>,
			   target: &Point3<f64>,
			   up: &Vector3<f64>,
			   fov: f64,
			   aspect: f64,
			   znear: f64,
			   zfar: f64
    ) -> Camera {
	let iso = na::geometry::Isometry3::look_at_rh(origin, target, up);
	let proj = na::geometry::Perspective3::new(aspect, fov, znear, zfar);
	Camera {
	    world_camera_iso: iso,
	    // world_camera_matrix: iso.to_matrix(),
	    projection: proj,
	}
    }

    /// Project a point into device coordinates.
    pub fn project(&self, world_point: Point3<f64>) -> Option<Point2<f64>> {
	// transform the point into camera space
	let camera_point = self.world_camera_iso.transform_point(&world_point);

	// transform the camera space point to NDC
	let ndc = self.projection.project_point(&camera_point);

	// only return the point if it is within the frustum
	if ndc.z >= 0.0 && ndc.z <= 1.0{
	    Some(Point2::new(ndc.x, ndc.y))
	} else {
	    None
	}
    }

}
