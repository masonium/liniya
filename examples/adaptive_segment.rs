extern crate liniya;

use liniya::{
    camera::Camera,
    scene::SceneBuilder,
    shape::{BoxOutline, Sphere},
};
use nalgebra::{Point2, Point3, Vector3};

fn main() {
    let unit_size = Vector3::new(0.5, 0.5, 0.5);
    let b = BoxOutline::new(Point3::new(0.0, 0.0, 0.0), unit_size);
    let b2 = BoxOutline::new(Point3::new(-2.0, 0.0, 0.0), unit_size);
    let b3 = BoxOutline::new(Point3::new(2.0, 0.0, 0.0), unit_size);

    let sphere2 = Sphere::new(
        &Point3::new(2.0, 2.0, 0.0),
        2.0,
        Some(std::f64::consts::PI / 10.0),
        None,
    );

    let scene = SceneBuilder::new()
        .add(b)
        .add(b2)
        .add(b3)
        .add(sphere2)
        .build();

    let w = 800.0;
    let h = 600.0;
    let camera = Camera::new()
        .look_at(
            &Point3::new(0.0, 0.0, 5.0),
            &Point3::new(0.0, 0.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
        )
        .perspective(std::f64::consts::FRAC_PI_2, w / h, 1.0, 10.0)
        .set_resolution(0.01);

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
