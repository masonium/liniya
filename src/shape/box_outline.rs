//! Box with paths as outlines..
use crate::common::*;
use crate::shape::{Shape, Path};

/// Box with paths on all of the edges.
pub struct BoxOutline {
    pos: Point3<f64>,
    half_extents: Vector3<f64>,
    aabb: AABB<f64>,
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

    fn intersect(&self, ray: &Ray<f64>, max_toi: f64) -> Option<f64> {
        self.aabb
            .toi_with_ray(&Isometry3::identity(), ray, max_toi, true)
    }

    fn paths(&self) -> Vec<Path> {
        let mut corners = Vec::with_capacity(8);
        for i in &[-1.0, 1.0] {
            for j in &[-1.0, 1.0] {
                for k in &[-1.0, 1.0] {
                    corners.push(Point3::new(
                        self.pos.x + i * self.half_extents.x,
                        self.pos.y + j * self.half_extents.y,
                        self.pos.z + k * self.half_extents.z,
                    ));
                }
            }
        }
        vec![
            vec![corners[0], corners[1]],
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
            vec![corners[3], corners[7]],
        ]
    }

    fn bounding_box(&self) -> AABB<f64> {
        self.aabb
    }
}
