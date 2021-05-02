//! A `Scene` is a collection of objects to render.
use crate::common::*;
use crate::shape::Shape;
use crate::{camera::Camera, shape::Path};
use ncollide3d::bounding_volume::AABB;
use ncollide3d::{
    partitioning::{VisitStatus, Visitor, BVH, BVT},
    query::Ray,
};

/// A scene is a collection of shapes in space that can be rendered.
pub struct Scene {
    /// bounded-volume tree for containing objects
    bvt: BVT<Box<dyn Shape>, AABB<f64>>,
}

/// Convenience class for incrementally building a scene.
#[derive(Default)]
pub struct SceneBuilder {
    shapes: Vec<Box<dyn Shape>>,
}

impl SceneBuilder {
    pub fn new() -> SceneBuilder {
	SceneBuilder::default()
    }

    pub fn add<S: Shape + 'static>(mut self, shape: S) -> Self {
	self.shapes.push(Box::new(shape));
	self
    }

    pub fn build(self) -> Scene {
	Scene::new(self.shapes)
    }
}

type RenderPath = Vec<Point2<f64>>;

impl Scene {
    pub fn new(shapes: Vec<Box<dyn Shape>>) -> Scene {
	let shapes_and_bounds = shapes
	    .into_iter()
	    .map(|s| {
		let bb = s.bounding_box();
		(s, bb)
	    })
	    .collect();
	let bvt = BVT::new_balanced(shapes_and_bounds);
	Scene { bvt }
    }

    /// Render a line segment adaptive based on the desired screen resolution.
    ///
    /// A line segment can end up being an arbitrary long number of
    /// paths, depending on what lies in front of it and how much it
    /// needs to be adaptively modified.
    ///
    /// Returns an in-progress path, representing
    /// a (non-proper) prefix of the the current segment.
    fn render_segment_adaptive(
	self,
	camera: &Camera,
	p0: &Point3<f64>,
	p1: &Point3<f64>,
	screen_res: f64,
	paths: &mut Vec<Path>,
	// The Path currently being built, if one exists.
	curr_path: &Option<Path>,
    ) -> Option<Path> {
	// Project the segment, split it to the desired resolution,
	// unproject them, and perform the render.
	let proj_p0 = camera.project_3d(&p0);
	let proj_p1 = camera.project_3d(&p1);

	let dist_2d_sq = (proj_p0.xy() - proj_p1.xy()).norm_squared();

	// poly_proj contains the projected segments
	let segments: Vec<Point3<f64>> = {
	    let mut proj_segments = if dist_2d_sq > screen_res * screen_res {
		let dist_2d = dist_2d_sq.sqrt();
		let n = (dist_2d / screen_res).ceil() as usize;
		let v = proj_p1 - proj_p0;
		(0..=n).map(|i| proj_p0 + v * i as f64 / n as f64).collect()
	    } else {
		vec![proj_p0, proj_p1]
	    };
	    // unproject the segments so they are back in world space.
	    for p in proj_segments.iter_mut() {
		*p = camera.unproject(p);
	    }
	    proj_segments
	};
	let mut curr_path: Path = curr_path.as_ref().cloned().unwrap_or_else(|| vec![]);

	None
    }

    /// Given a point that we otherwise expect to be able to render
    /// (in the camera frustum, etc.), return true if the point is not
    /// occluded by any other shapes.
    fn is_point_occluded(
	&self,
	camera: &Camera,
	point: &Point3<f64>,
	proj_point: impl Into<Option<Point3<f64>>>,
    ) -> bool {
	// project the point into 2d
	let proj_point = proj_point
	    .into()
	    .unwrap_or_else(|| camera.project_3d(&point));

	// find the equivalent point projected behind slightly on the
	// near plane.
	let proj_origin = Point3::new(proj_point.x, proj_point.y, 0.0);
	let origin = camera.unproject(&proj_origin);
	let proj_far = Point3::new(proj_point.x, proj_point.y, 1.0);
	let far = camera.unproject(&proj_far);
    }

    /// Render a 3d-path onto one or more 2d paths in normalized
    /// coordinates.
    pub(crate) fn render_polyline(
	&self,
	path: &Vec<Point3<f64>>,
	camera: &Camera,
    ) -> Vec<Vec<Point2<f64>>> {
	let clipped_paths = camera.clip_path(path);

	let mut prev_point = path[0];
	let mut prev_project = camera.project(&prev_point);

	//let mut curr_path = vec![];

	for i in 1..path.len() {
	    let point = path[i];
	    let project = camera.project(&point);

	    prev_point = point;
	}

	vec![]
    }

    /// Return a collection of paths that visible from the provided
    /// camera, at the specific resolution.
    ///
    /// Each path segment is automatically until the segment, as
    /// projected into screen space, meets the desired resolution.
    pub fn render(&self, camera: &Camera, screen_res: f64) -> Vec<RenderPath> {
	let mut visitor = OnShapePathVisitor::new(self, camera.clone());
	self.bvt.visit(&mut visitor);
	visitor.rendered_paths
    }
}
