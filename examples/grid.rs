extern crate liniya;

use liniya::{camera::Camera, scene::SceneBuilder, shape::BoxOutline};
use nalgebra::{Point3, Vector3};

fn main() {
    let mut scene_builder = SceneBuilder::new();
    let w = 800.0;
    let h = 600.0;
    const GRID_SIZE: isize = 10;
    let gs = GRID_SIZE as f64;
    let target = Point3::new(0.0, 0.0, 0.0);
    const PI: f64 = std::f64::consts::PI;
    let diff = nalgebra::geometry::Rotation::from_euler_angles(-PI / 8.0, PI / 4.0 + 0.2, 0.0)
        * Vector3::new(0.0, 0.0, gs * 1.4);
    let camera = Camera::new()
        .look_at(&(target + diff), &target, &Vector3::new(0.0, 1.0, 0.0))
        //.ortho(gs * w / h, gs, 0.01, 100.0)
        .perspective(std::f64::consts::FRAC_PI_2, w / h, 0.1, 50.0)
        .set_resolution(0.001);

    let unit = Vector3::new(1.0, 1.0, 1.0);
    for i in -GRID_SIZE..=GRID_SIZE {
        for j in -GRID_SIZE..=GRID_SIZE {
            let height = fastrand::usize(1..=if i % 2 != 0 && j % 2 != 0 { 6 } else { 1 });
            //let height = 1;
            for h in 0..height {
                let c = Point3::new(i as f64, h as f64, j as f64);
                scene_builder = scene_builder.add(BoxOutline::from_extents(c, c + unit));
            }
        }
    }
    let scene = scene_builder.build();

    let mut doc =
        svg::Document::new()
            .set("width", w)
            .set("height", h)
            .add(svg::node::element::Style::new(
                "polyline { fill: none; stroke-width: 1px; }",
            ));
    doc = doc.add(scene.render_to_svg(&camera, (w, h)));

    println!("{}", doc.to_string())
}
