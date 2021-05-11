use crate::common::*;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum BoxPlaneTest {
    Inside,
    Intersects,
    Outside,
}

/// Return the position of the bounding box relative to `bb`.  The
/// bounding box is `Inside` if it is completely on the side facing
/// the normal.
///
/// # Remarks
///
/// Assumes the plane normal is a unit vector.
pub fn box_plane_intersection(bb: &AABB<f64>, plane: &Vector4<f64>) -> BoxPlaneTest {
    let n = plane.xyz();
    let pos = bb.center();
    let he = bb.half_extents();

    let r = n.abs().dot(&he);
    let s = n.dot(&pos.coords) + plane.w;

    if s < -r {
        BoxPlaneTest::Outside
    } else if s > r {
        BoxPlaneTest::Inside
    } else {
        BoxPlaneTest::Intersects
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ncollide3d::bounding_volume::AABB;
    #[test]
    fn box_plane_test() {
        let aabb = AABB::from_half_extents(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        assert_eq!(
            box_plane_intersection(&aabb, &Vector4::new(0.0, 1.0, 0.0, -2.0)),
            BoxPlaneTest::Outside
        );
        assert_eq!(
            box_plane_intersection(&aabb, &Vector4::new(0.0, 1.0, 0.0, 0.0)),
            BoxPlaneTest::Intersects
        );
        assert_eq!(
            box_plane_intersection(&aabb, &Vector4::new(0.0, 1.0, 0.0, 2.0)),
            BoxPlaneTest::Inside
        );
    }
}
