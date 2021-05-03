//! Perspective or orthographic cameras for rendering scenes
use super::common::*;
use crate::shape::Path;
use art_util::{frustum::ClipResult, Frustum};

/// `Camera` determines the view and projection of a scene during
/// rendering.
#[derive(Clone, Debug)]
pub struct Camera {
    view_iso: na::Isometry3<f64>,

    /// projection matrix, usually either perspective or orthographic
    projection: na::Projective3<f64>,

    /// frustum derived from view-projection matrix
    frustum: Frustum<f64>,

    /// maximum screen space distance for line segments to be
    /// rendered.
    resolution: f64,
}

impl Camera {
    /// Returns a default camera that can be modified.
    pub fn new() -> Camera {
        let proj =
            *na::Perspective3::new(1.0, std::f64::consts::FRAC_PI_2, 1.0, 10.0).as_projective();
        Camera {
            view_iso: na::Isometry3::identity(),
            frustum: Frustum::from_clip_matrix(proj.matrix()),
            projection: proj,
            resolution: 0.001,
        }
    }
    /// Update the internal frustum.
    ///
    /// Must be called whenver the view or projection matrices are
    /// update.
    fn update_frustum(self) -> Self {
        let clip_matrix = self.projection.matrix() * self.view_iso.to_matrix();
        let frustum = Frustum::from_clip_matrix(&clip_matrix);
        Self { frustum, ..self }
    }

    /// Return a modified version of the camera with new look_at
    /// parameters.
    pub fn look_at(self, origin: &Point3<f64>, target: &Point3<f64>, up: &Vector3<f64>) -> Camera {
        let new_wc = Isometry3::look_at_rh(origin, target, up);
        Camera {
            view_iso: new_wc,
            ..self
        }
        .update_frustum()
    }

    /// Return a modified version of the camera with perepctive projection.
    pub fn perspective(self, fov: f64, aspect: f64, znear: f64, zfar: f64) -> Camera {
        let projection = *na::geometry::Perspective3::new(aspect, fov, znear, zfar).as_projective();
        Camera { projection, ..self }.update_frustum()
    }

    /// Return a modified version of the camera with orthographic projection.
    pub fn ortho(self, half_width: f64, half_height: f64, znear: f64, zfar: f64) -> Camera {
        let projection = *na::geometry::Orthographic3::new(
            -half_width,
            half_width,
            -half_height,
            half_height,
            znear,
            zfar,
        )
        .as_projective();
        Camera { projection, ..self }.update_frustum()
    }

    pub fn set_resolution(self, res: f64) -> Camera {
        Camera {
            resolution: res,
            ..self
        }
    }

    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Clip a path into separate paths within the cameras view.
    pub(crate) fn clip_path(&self, path: &Path) -> Vec<Path> {
        let mut clipped_paths: Vec<Path> = vec![];
        if path.len() < 2 {
            return vec![];
        }

        let mut current_path: Path = vec![];
        for i in 1..path.len() {
            let (p0, p1) = (path[i - 1], path[i]);
            let clip_res = self.frustum.clip_line(&p0, &p1);
            match clip_res {
                ClipResult::Outside => {
                    // Necessarily, there is no current path.
                    assert!(current_path.is_empty());

                    // No need to do anything else.
                }
                ClipResult::Inside(_, p1) => {
                    // We must already have been building a previous path.
                    if current_path.is_empty() {
                        current_path.push(p0);
                    }

                    // just append the lastest point onto it.
                    current_path.push(p1);
                }
                ClipResult::Partial(c0_in, c0, c1_in, c1) => {
                    match (c0_in, c1_in) {
                        // We're begining and ending a path, as the
                        // clip is strictly internal to the segment.
                        // Don't bother creating an intermediate current_path.
                        (false, false) => {
                            assert!(current_path.is_empty());
                            clipped_paths.push(vec![c0, c1]);
                        }
                        // We're starting a path.
                        //
                        (false, true) => {
                            assert!(current_path.is_empty());
                            current_path.push(c0);
                        }
                        // We're ending a path.
                        // Add it to the list of clipped paths.
                        (true, false) => {
                            assert!(!current_path.is_empty());
                            current_path.push(c1);
                            clipped_paths.push(current_path);
                            current_path = vec![];
                        }
                        // This is not allowed by Partial.
                        (true, true) => {
                            panic!("Frustum::clip_line returned an unexpected Partial result.")
                        }
                    }
                }
            }
        }
        // If we have a path still in construction, it must be
        // finished, and we can add it to our clipped paths
        if current_path.len() > 0 {
            clipped_paths.push(current_path);
        }

        clipped_paths
    }

    /// Return true if the point is contained within the camera's frustum.
    pub fn is_point_visible(&self, p: &Point3<f64>) -> bool {
        self.frustum.is_point_in(p)
    }

    /// Return true iff the bounding box has any intersection with the
    /// camera's frustum.
    pub fn is_aabb_visible(&self, bb: &AABB<f64>) -> bool {
        // TODO: This implementation is technically work, but it will
        // definitely handle most cases.
        self.is_point_visible(&bb.center())
    }

    /// Project a point into device coordinates.
    pub fn project(&self, world_point: &Point3<f64>) -> Point2<f64> {
        self.project_3d(world_point).xy()
    }

    /// Unproject a point from NDC to world coordinates.
    pub fn unproject(&self, ndc_point: &Point3<f64>) -> Point3<f64> {
        self.view_iso
            .inverse_transform_point(&self.projection.inverse_transform_point(&ndc_point))
    }

    /// Project a point into device coordinates, including the 3d coordinate.
    pub fn project_3d(&self, world_point: &Point3<f64>) -> Point3<f64> {
        // transform the point into camera space
        let camera_point = self.view_iso.transform_point(world_point);

        // transform the camera space point to NDC
        self.projection.transform_point(&camera_point)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // use nalgebra as na;
    // use approx::{abs_diff_eq, AbsDiff, AbsDiffEq};

    #[test]
    fn test_visiblity() {
        let cam = Camera::new().ortho(2.0, 2.0, 1.0, 10.0).look_at(
            &Point3::new(0.0, 0.0, 5.0),
            &Point3::new(0.0, 0.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
        );

        assert!(cam.is_point_visible(&Point3::new(0.0, 0.0, 0.0)));
        assert!(cam.is_point_visible(&Point3::new(0.0, 0.0, 4.0)));
        assert!(!cam.is_point_visible(&Point3::new(0.0, 0.0, 6.0)));
    }
}
