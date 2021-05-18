use na::{Point3, Vector4};
use nalgebra as na;
use std::cmp::Ordering;

#[derive(Clone, Copy)]
pub enum FrustumPlane {
    Left = 0,
    Right = 1,
    Bottom = 2,
    Top = 3,
    Near = 4,
    Far = 5,
}

/// The collection of planes forming the convex region bounded by a
/// projection matrix or, more generally, a clip matrix.
#[derive(Clone, Debug)]
pub struct Frustum<F: na::RealField> {
    pub planes: [na::Vector4<F>; 6],
}

#[derive(Debug, Clone)]
pub enum ClipResultPartial {
    /// The first point is in, the frustum, but the second point is
    /// outside.
    Prefix,

    /// The first point is outside the frustum, but the second point
    /// is in.
    Suffix,

    /// Neither point is in, but the segment intersects.
    Infix,
}

#[derive(Debug, Clone)]
/// Result from clipping a line segment against a `Frustum`.
pub enum ClipResult<F: na::RealField> {
    /// The line segment is completely outside of the frustum.
    Outside,

    /// The line segment is completely inside the frustum.
    Inside(Point3<F>, Point3<F>),

    /// The line segment is partially within the frustum.
    ///
    /// The last value indicators whether at least one end-point is
    /// within the frustum.
    Partial(ClipResultPartial, Point3<F>, Point3<F>),
}

/// Return the value t along the line p0 + p1 * t that intersects with
/// the plane, if it exists.
fn plane_segment_directed_intersection<F: na::RealField>(
    plane: &Vector4<F>,
    p0: &Point3<F>,
    p1: &Point3<F>,
) -> Option<F> {
    let v = p1 - p0;
    let pn = plane.xyz();
    let v_dot_pn = v.dot(&pn);
    let eps: F = na::convert(1e-5);
    if v_dot_pn.abs() / v.norm() < eps {
        return None;
    }
    Some((plane[3] + p0.coords.dot(&pn)) / -v_dot_pn)
}

impl<F: na::RealField> Frustum<F> {
    /// Compute the 6 planes of the frustum defined by a projection matrix
    /// the following order:
    ///
    /// Left, Right, Bottom, Top, Near, Far
    ///
    /// Planes all face inside the frustum.
    pub fn from_clip_matrix(m: &na::Matrix4<F>) -> Frustum<F> {
        let mt = m.transpose();
        let mut planes = [
            mt.column(3) + mt.column(0),
            mt.column(3) - mt.column(0),
            mt.column(3) + mt.column(1),
            mt.column(3) - mt.column(1),
            mt.column(3) + mt.column(2),
            mt.column(3) - mt.column(2),
        ];

        for p in planes.iter_mut() {
            // normalize the plane direction
            let n_norm = p.xyz().norm();
            *p /= n_norm;
        }

        Frustum { planes }
    }

    /// Return true iff the point lines within the frustum.
    pub fn is_point_in(&self, v: &na::Point3<F>) -> bool {
        let ext = na::Vector4::new(v[0], v[1], v[2], F::one());
        self.planes.iter().all(|p| p.dot(&ext) >= F::zero())
    }

    /// Return true iff the point lines within or on the frustum.
    pub fn is_point_in_or_on(&self, v: &na::Point3<F>, eps: F) -> bool {
        let ext = na::Vector4::new(v[0], v[1], v[2], F::one());
        self.planes.iter().all(|p| p.dot(&ext) >= -eps)
    }

    /// Clip a 3d line segment against the frustum.
    pub fn clip_line(&self, p0: &na::Point3<F>, p1: &na::Point3<F>) -> ClipResult<F> {
        let in0 = self.is_point_in(p0);
        let in1 = self.is_point_in(p1);

        // frustums are convex, so if both points are inside the frustum, we're done.
        if in0 && in1 {
            return ClipResult::Inside(*p0, *p1);
        }
        // Get the intersections for the line segment across all planes.
        let mut isects: Vec<F> = (0..6)
            .filter_map(|i| plane_segment_directed_intersection(&self.planes[i], p0, p1))
            .filter_map(|t| {
                if t >= F::zero() && t <= F::one() {
                    Some(t)
                } else {
                    None
                }
            })
            .collect();

        let cmp = |a: &F, b: &F| {
            if a < b {
                Ordering::Less
            } else if a > b {
                Ordering::Greater
            } else {
                Ordering::Equal
            }
        };
        isects.sort_by(cmp);

        // If the first point is inside, we can find the first
        // intersection and use that as the other clip point.
        if in0 {
            let min_t: F = isects[0];
            return ClipResult::Partial(ClipResultPartial::Prefix, *p0, p0 + (p1 - p0) * min_t);
        }

        // If the second point inside, we can find the last
        // intersection and use that as the other clip point.
        if in1 {
            let max_t: F = *isects.last().unwrap();
            return ClipResult::Partial(ClipResultPartial::Suffix, p0 + (p1 - p0) * max_t, *p1);
        }

        // If both points are outside, then we need to find the
        // intersection points that are on the frustum.
        let eps = na::convert(1e-5);
        let points_on_frustum: Vec<_> = isects
            .iter()
            .filter_map(|t| {
                let p = p0 + (p1 - p0) * *t;
                if self.is_point_in_or_on(&p, eps) {
                    Some(p)
                } else {
                    None
                }
            })
            .collect();

        match points_on_frustum.len() {
            2 => ClipResult::Partial(
                ClipResultPartial::Infix,
                points_on_frustum[0],
                points_on_frustum[1],
            ),
            0 | 1 => ClipResult::Outside,
            _ => panic!("Too many on-frustum intersections"),
        }
    }

    /// Return the desired frustum plane.
    pub fn get_plane(&self, idx: FrustumPlane) -> na::Vector4<F> {
        self.planes[idx as usize]
    }
}
