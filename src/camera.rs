//! Perspective or orthographic cameras for rendering scenes
use super::common::*;
use crate::shape::Path;
use art_util::{frustum::ClipResult, Frustum};

/// `Camera` determines the view and projection of a scene during
/// rendering.
#[derive(Clone, Debug)]
pub struct Camera {
    world_camera_iso: na::Isometry3<f64>,
    // world_camera_matrix: Matrix4<f64>,
    projection: na::Perspective3<f64>,

    /// frustum derived from view-projection matrix
    frustum: Frustum<f64>,
}

impl Camera {
    /// Create a new camera with perepctive projection.
    pub fn look_at_perspective(
        origin: &Point3<f64>,
        target: &Point3<f64>,
        up: &Vector3<f64>,
        fov: f64,
        aspect: f64,
        znear: f64,
        zfar: f64,
    ) -> Camera {
        let iso = na::geometry::Isometry3::look_at_rh(origin, target, up);
        let proj = na::geometry::Perspective3::new(aspect, fov, znear, zfar);
        let clip = iso.to_matrix() * proj.as_matrix();
        let frustum = Frustum::from_clip_matrix(&clip);
        Camera {
            world_camera_iso: iso,
            projection: proj,
            frustum,
        }
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
                    assert!(!current_path.is_empty());

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
        todo!("implement occlusion")
    }

    /// Project a point into device coordinates.
    pub fn project(&self, world_point: &Point3<f64>) -> Point2<f64> {
        self.project_3d(world_point).xy()
    }

    /// Unproject a point from NDC to world coordinates.
    pub fn unproject(&self, ndc_point: &Point3<f64>) -> Point3<f64> {
        self.world_camera_iso
            .inverse_transform_point(&self.projection.unproject_point(ndc_point))
    }

    /// Project a point into device coordinates, including the 3d coordinate.
    pub fn project_3d(&self, world_point: &Point3<f64>) -> Point3<f64> {
        // transform the point into camera space
        let camera_point = self.world_camera_iso.transform_point(world_point);

        // transform the camera space point to NDC
        let ndc = self.projection.project_point(&camera_point);
        ndc
    }
}
