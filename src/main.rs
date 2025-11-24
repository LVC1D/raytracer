use raytracer::{BVHNode, Sphere, Vec3, render_bvh, render_naive};

fn create_many_spheres(count: usize) -> Vec<Sphere> {
    let mut spheres = Vec::new();
    let mut rng_x = 0.0;
    let mut rng_y = 0.0;
    let mut rng_z = 5.0;

    for i in 0..count {
        spheres.push(Sphere {
            center: Vec3 {
                x: (rng_x * 31.0) % 20.0 - 10.0, // Spread across X: -10 to 10
                y: (rng_y * 17.0) % 20.0 - 10.0, // Spread across Y: -10 to 10
                z: rng_z + (i as f64 * 0.1),     // Depth: 5 to 5 + count*0.1
            },
            radius: 0.5,
        });

        // Simple pseudo-random updates
        rng_x += 1.7;
        rng_y += 2.3;
        rng_z += 0.05;
    }

    spheres
}

fn main() {
    use std::time::Instant;

    println!("Creating 1000 spheres...");
    let spheres = create_many_spheres(1000);

    let bvh = BVHNode::build(spheres.clone());

    println!("\nRendering with naive approach...");
    let start = Instant::now();
    let _ = render_naive(400, 300, &spheres);
    let naive_time = start.elapsed();
    println!("Naive: {:?}", naive_time);

    println!("\nRendering with BVH...");
    let start = Instant::now();
    let _ = render_bvh(400, 300, &bvh);
    let bvh_time = start.elapsed();
    println!("BVH: {:?}", bvh_time);

    let speedup = naive_time.as_secs_f64() / bvh_time.as_secs_f64();
    println!("\nSpeedup: {:.2}x faster with BVH!", speedup);
}
