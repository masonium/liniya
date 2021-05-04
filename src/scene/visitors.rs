//! Define visitors for use when rendering scenes.
use super::{scene::RenderPath, Scene};
use crate::{camera::Camera, common::*, shape::Shape};
use na::Isometry;
use ncollide3d::{
    partitioning::{VisitStatus, Visitor},
    query::{Ray, RayCast},
};

/// Visitor for rendering paths within view of the camera
pub struct CameraVisiblePathCollector<'a> {
    scene: &'a Scene,

    /// camera space to render from
    camera: Camera,

    /// Final paths rendered by this visitor
    pub rendered_paths: Vec<RenderPath>,
}

impl<'a> CameraVisiblePathCollector<'a> {
    pub fn new(scene: &'a Scene, camera: Camera) -> Self {
        CameraVisiblePathCollector {
            camera,
            scene,
            rendered_paths: vec![],
        }
    }
}

impl<'a> Visitor<Box<dyn Shape>, AABB<f64>> for CameraVisiblePathCollector<'a> {
    fn visit(&mut self, bv: &AABB<f64>, data: Option<&Box<dyn Shape>>) -> VisitStatus {
        if self.camera.is_aabb_visible(bv) {
            if let Some(shape) = data {
                for path in shape.paths() {
                    self.rendered_paths
                        .extend(self.scene.render_path(&path, &self.camera));
                }
            }
            VisitStatus::Continue
        } else {
            VisitStatus::Stop
        }
    }
}

/// Visitor for determining point occlusion.
///
/// Yields true if the point is *occluded* by another object.
pub struct SceneOcclusionVisitor<'b> {
    /// Ray to be tested.
    ray: &'b Ray<f64>,

    /// target time-of-impact of the original point
    target_toi: f64,

    /// Maximum time-of-impact of the ray with the objects.
    max_toi: f64,

    /// Final value dictating whether the point was occluded.
    is_occluded: bool,
}

impl<'b> SceneOcclusionVisitor<'b> {
    /// Creates a new `RayIntersectionCostFnVisitor`.
    pub fn new(
        ray: &'b Ray<f64>,
        target_toi: f64,
        max_toi: f64,
        //broad_phase: &'a dyn BroadPhase<f64, AABB<f64>, Box<dyn Shape>>,
    ) -> SceneOcclusionVisitor<'b> {
        SceneOcclusionVisitor {
            ray,
            target_toi,
            max_toi,
	    is_occluded: false,
        }
    }

    pub fn is_occluded(&self) -> bool {
	self.is_occluded
    }
}

impl<'b> Visitor<Box<dyn Shape>, AABB<f64>> for SceneOcclusionVisitor<'b> {
    fn visit(
        &mut self,
        bv: &AABB<f64>,
        data: Option<&Box<dyn Shape>>,
    ) -> VisitStatus {
        if bv.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true).is_none() {
	    return VisitStatus::Stop;
	}

        // If the node has data in it, check against 
        if let Some(shape) = data {
            if let Some(result) = shape.intersect(self.ray, self.max_toi) {
                if result.t < self.target_toi - 1e-5 {
                    self.is_occluded = true;
		    return VisitStatus::ExitEarly;
		}
            }
        }
	VisitStatus::Continue
    }
}
