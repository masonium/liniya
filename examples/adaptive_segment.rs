extern crate liniya;

use liniya::{
    camera::Camera,
    scene::{Scene, SceneBuilder},
    shape::{BoxOutline, LatSphere},
};
use nalgebra::{Point2, Point3, Vector3};

fn format_polyline(p: &Vec<Point2<f64>>, w: f64, h: f64) -> String {
    let strs: Vec<String> = p
        .iter()
        .map(|p| format!("{:.4},{:.4}", (p.x + 1.0) * w / 2.0, (1.0 - p.y) * h / 2.0))
        .collect();
    strs.join(" ")
}

fn main() {
    let unit_size = Vector3::new(0.5, 0.5, 0.5);
    let b = BoxOutline::new(Point3::new(0.0, 0.0, 0.0), unit_size);
    let b2 = BoxOutline::new(Point3::new(-2.0, 0.0, 0.0), unit_size);
    let b3 = BoxOutline::new(Point3::new(2.0, 0.0, 0.0), unit_size);

    let sphere2 = LatSphere::new(
        &Point3::new(2.0, 2.0, 0.0),
        2.0,
        std::f64::consts::PI / 10.0,
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

    let r = scene.render(&camera);

    let mut doc =
        svg::Document::new()
            .set("width", w)
            .set("height", h)
            .add(svg::node::element::Style::new(
                "polyline { fill: none; stroke-width: 1px; }",
            ));
    for path in r {
        doc = doc.add(svg::node::element::Polyline::new().set("d", format_polyline(&path, w, h)))
    }
    println!("{}", doc.to_string())
}
