//! Define visitors for use when rendering scenes.
use crate::{shape::Shape, common::*};
use ncollide3d::{query::{RayCast, Ray}, partitioning::{BestFirstVisitor, BestFirstVisitStatus}};
use na::Isometry;

/// Visitor for determining point occlusion.
///
/// Ultimate yields a true value for the `visible` field if the
/// earliest intersection is coincident with a shape (probably the
/// shape the path is coincident with.
pub struct SceneOcclusionVisitor<'b>
where
{
    /// Ray to be tested.
    ray: &'b Ray<f64>,

    /// target time-of-impact of the original point
    target_toi: f64,

    /// Maximum time-of-impact of the ray with the objects.
    max_toi: f64,
}

impl<'b> SceneOcclusionVisitor<'b>
{
    /// Creates a new `RayIntersectionCostFnVisitor`.
    #[inline]
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
	}
    }
}

impl<'b> BestFirstVisitor<f64, Box<dyn Shape>, AABB<f64>>
    for SceneOcclusionVisitor<'b>
{
    type Result = bool;

    #[inline]
    fn visit(
	&mut self,
	best_cost_so_far: f64,
	bv: &AABB<f64>,
	data: Option<&Box<dyn Shape>>,
    ) -> BestFirstVisitStatus<f64, Self::Result> {
	if let Some(rough_toi) =
	    bv.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true)
	{
	    let mut res = BestFirstVisitStatus::Continue {
		cost: rough_toi,
		result: Some(true),
	    };

	    // If the node has data then it is a leaf
	    if let Some(shape) = data {
		// rough_toi is less than or equal the cost of any subnode.
		// Either: The ray origin is outside the bv, and so no point in the bv
		//   could have a lower cost than rough_toi.
		// Or: The ray origin is inside the bv, and rough_toi is 0
		// We only check the data if it may be better than best_cost_so_far
		if rough_toi < best_cost_so_far {
		    if let Some(result) = shape.intersect(self.ray, self.max_toi)
		    {
			if result.t < self.target_toi {
			    res = BestFirstVisitStatus::ExitEarly(Some(false));
			} else {
			    res = BestFirstVisitStatus::Continue {
				cost: result.t,
				result: Some(true)
			    };
			}
		    }
		}
	    }

	    res
	} else {
	    // No intersection so we can ignore all children
	    BestFirstVisitStatus::Stop
	}
    }
}
