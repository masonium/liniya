mod box_outline;
mod sphere;

use super::common::*;
use ncollide3d::{
    bounding_volume::AABB,
    query::Ray,
};

pub type Path = Vec<Point3<f64>>;
pub type Paths = Vec<Path>;

/// Renderable 3D with a colllection of paths on the 'surface' of the
/// shape.
///
/// The underlying shape is use to determine visibility of the
/// paths. Rendering is thus only guaranteed if the points of the path
/// lie 'on' the shape within some tolerance.
pub trait Shape {
    /// Ray-intersection
    fn intersect(&self, ray: &Ray<f64>, max_toi: f64) -> Option<f64>;

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

pub use box_outline::BoxOutline;
pub use sphere::Sphere;
