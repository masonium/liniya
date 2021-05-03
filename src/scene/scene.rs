//! A `Scene` is a collection of objects to render.
use super::visitors::{CameraVisiblePathCollector, SceneOcclusionVisitor};
use crate::common::*;
use crate::shape::Shape;
use crate::{camera::Camera, shape::Path};
use ncollide3d::bounding_volume::AABB;
use ncollide3d::{
    partitioning::{BVH, BVT},
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

pub(crate) type RenderPath = Vec<Point2<f64>>;

/// Encapsulate the state of the current path being built in
/// `Scene::render_segment_adaptive`.
#[derive(Clone, Debug)]
enum SegmentPathState {
    /// This line has is not represented on the current path.
    Empty,
    /// The last point on the path represents the first
    /// endpoint of a segment.  The next point should add onto
    /// this one.
    Started(RenderPath),
    /// The last point on the path represents the second
    /// endpoint of a segment.  Any new endpoints shoudl
    /// update the last point, rather than changing the
    /// current point.
    Continuing(RenderPath),
}

impl SegmentPathState {
    /// Update `self` in response to a new point in the path.
    ///
    /// When the point provided is `None`, this is interpreted that a
    /// point that is not visible.
    fn update(self, point: Option<Point2<f64>>) -> (Self, Option<RenderPath>) {
        use SegmentPathState::*;
        match (self, point) {
            (Empty, Some(p)) => (Started(vec![p]), None),
            (Empty, None) => (Empty, None),
            (Started(mut path), Some(p)) => {
                path.push(p);
                (Continuing(path), None)
            }
            (Started(path), None) => (Empty, Some(path)),
            (Continuing(mut path), Some(p)) => {
                *path.last_mut().unwrap() = p;
                (Continuing(path), None)
            }
            (Continuing(path), None) => (Empty, Some(path)),
        }
    }

    /// Return the current path if one exists.
    fn into_path(self) -> Option<RenderPath> {
        use SegmentPathState::*;
        match self {
            Started(path) | Continuing(path) => Some(path),
            Empty => None,
        }
    }
}

pub fn split_segment_adaptive(camera: &Camera, p0: &Point3<f64>, p1: &Point3<f64>) -> (Path, Path) {
    // Project the segment, split it to the desired resolution,
    // unproject them, and perform the render.
    let proj_p0 = camera.project_3d(&p0);
    let proj_p1 = camera.project_3d(&p1);

    let dist_2d_sq = (proj_p0.xy() - proj_p1.xy()).norm_squared();
    let sres = camera.resolution();

    // poly_proj contains the projected segments
    let proj_segments = if dist_2d_sq > sres * sres {
        let dist_2d = dist_2d_sq.sqrt();
        let n = (dist_2d / sres).ceil() as usize;
        let v = proj_p1 - proj_p0;
        (0..=n).map(|i| proj_p0 + v * i as f64 / n as f64).collect()
    } else {
        vec![proj_p0, proj_p1]
    };
    let segments = proj_segments.iter().map(|p| camera.unproject(p)).collect();

    (segments, proj_segments)
}

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
        &self,
        camera: &Camera,
        p0: &Point3<f64>,
        p1: &Point3<f64>,
        paths: &mut Vec<RenderPath>,
        // The Path currently being built, if one exists.
        curr_path: Option<RenderPath>,
    ) -> Option<RenderPath> {
        let (points, proj_points) = split_segment_adaptive(camera, p0, p1);

        let (first_point, mut path_state) = match curr_path {
            Some(p) if !p.is_empty() => (1, SegmentPathState::Started(p)),
            _ => (0, SegmentPathState::Empty),
        };

        for i in first_point..points.len() {
            let is_visible = self.is_point_visible(camera, &points[i], proj_points[i]);

            let (new_path_state, finished_path) = path_state.update(if is_visible {
                Some(proj_points[i].xy())
            } else {
                None
            });
            if let Some(fp) = finished_path {
                paths.push(fp);
            }
            path_state = new_path_state;
        }

        path_state.into_path()
    }

    /// Given a point that we otherwise expect to be able to render
    /// (in the camera frustum, etc.), return true if the point is not
    /// occluded by any other shapes.
    fn is_point_visible(
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
        let proj_origin = Point3::new(proj_point.x, proj_point.y, -1.0);
        let origin = camera.unproject(&proj_origin);
        let proj_far = Point3::new(proj_point.x, proj_point.y, 1.0);
        let far = camera.unproject(&proj_far);
        let unnorm_dir = point - origin;
        let target_toi = unnorm_dir.norm();

        let ray = Ray::new(origin, unnorm_dir / target_toi);
        let max_toi = (far - origin).norm();

        let mut sov = SceneOcclusionVisitor::new(&ray, target_toi, max_toi);
        let res = self.bvt.best_first_search(&mut sov);

        res.map(|(_, is_visible)| is_visible).unwrap_or(true)
    }

    /// Render a 3d-path onto one or more 2d paths in normalized
    /// coordinates.
    pub fn render_path(&self, path: &Vec<Point3<f64>>, camera: &Camera) -> Vec<Vec<Point2<f64>>> {
        let clipped_paths = camera.clip_path(path);

        let mut all_paths = vec![];

        for clipped_path in clipped_paths {
            let mut curr_render_path = None;
            for i in 1..clipped_path.len() {
                let prev_point = &clipped_path[i - 1];
                let point = &clipped_path[i];

                curr_render_path = self.render_segment_adaptive(
                    camera,
                    prev_point,
                    point,
                    &mut all_paths,
                    curr_render_path,
                );
            }
            if let Some(p) = curr_render_path {
                if p.len() >= 2 {
                    all_paths.push(p)
                }
            }
        }

        all_paths
    }

    /// Return a collection of paths that visible from the provided
    /// camera, at the specific resolution.
    ///
    /// Each path segment is automatically until the segment, as
    /// projected into screen space, meets the desired resolution.
    pub fn render(&self, camera: &Camera) -> Vec<RenderPath> {
        let mut visitor = CameraVisiblePathCollector::new(self, camera.clone());
        self.bvt.visit(&mut visitor);

        visitor.rendered_paths
    }
}
