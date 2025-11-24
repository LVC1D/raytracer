use std::cmp::{self, Ordering};

// A coordinate struct
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
}

// Naive Approach
// Check if the ray hits a given sphere - using the quadratic formula
pub fn ray_sphere_intersect(ray: &Ray, sphere: &Sphere) -> bool {
    let oc_x = ray.origin.x - sphere.center.x;
    let oc_y = ray.origin.y - sphere.center.y;
    let oc_z = ray.origin.z - sphere.center.z;

    let a = ray.direction.x.powi(2) + ray.direction.y.powi(2) + ray.direction.z.powi(2);
    let b = 2 as f64 * (oc_x * ray.direction.x + oc_y * ray.direction.y + oc_z * ray.direction.z);
    let c = oc_x.powi(2) + oc_y.powi(2) + oc_z.powi(2) - sphere.radius.powi(2);

    let disc_value = b.powf(2.0) - 4.0 as f64 * a * c;
    if disc_value < 0.0 {
        return false;
    }

    let discriminant = f64::sqrt(disc_value);
    let t_1 = (-b - discriminant) / (2 as f64 * a);
    let t_2 = (-b + discriminant) / (2 as f64 * a);

    t_1 > 0.0 || t_2 > 0.0
}

pub fn normalized_direction(width: usize, height: usize, x: f64, y: f64) -> Vec3 {
    let x_norm = (x / (width as f64 / 2.0)) - 1.0;
    let y_norm = -((y / (height as f64 / 2.0)) - 1.0);

    Vec3 {
        x: x_norm,
        y: y_norm,
        z: 1.00,
    }
}

pub fn hit_or_miss(spheres: &[Sphere], ray: &Ray) -> Vec3 {
    for sphere in spheres.iter() {
        if ray_sphere_intersect(&ray, sphere) {
            return Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            };
        }
    }

    Vec3 {
        x: 0.5,
        y: 0.7,
        z: 1.0,
    }
}

pub fn render_naive(width: usize, height: usize, spheres: &[Sphere]) -> Image {
    let mut image = Image::new(width, height);
    let camera = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    for h in 0..height {
        let hf64 = h as f64;
        for w in 0..width {
            let wf64 = w as f64;
            let direction = normalized_direction(width, height, wf64, hf64);
            let ray = Ray {
                origin: camera,
                direction,
            };

            let pixel = hit_or_miss(spheres, &ray);
            let pixel_idx = h * width + w;
            image.pixels[pixel_idx] = pixel;
        }
    }

    image
}

// The BVH-Optimized Approach

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3,
}

impl AABB {
    pub fn intersect(&self, ray: &Ray) -> bool {
        // Calculate t_enter and t_exit for each axis
        // Return true if the slabs overlap
        let (tx_smaller, tx_larger) = if ray.direction.x.abs() < 0.0001 {
            // Ray is parallel to X axis
            if ray.origin.x >= self.min.x && ray.origin.x <= self.max.x {
                (f64::NEG_INFINITY, f64::INFINITY) // Inside for all t
            } else {
                (f64::INFINITY, f64::NEG_INFINITY) // Never inside (impossible range)
            }
        } else {
            // Normal case: calculate intersection times
            let t1 = (self.min.x - ray.origin.x) / ray.direction.x;
            let t2 = (self.max.x - ray.origin.x) / ray.direction.x;
            (t1.min(t2), t1.max(t2))
        };

        let (ty_smaller, ty_larger) = if ray.direction.y.abs() < 0.0001 {
            // Ray is parallel to X axis
            if ray.origin.y >= self.min.y && ray.origin.y <= self.max.y {
                (f64::NEG_INFINITY, f64::INFINITY) // Inside for all t
            } else {
                (f64::INFINITY, f64::NEG_INFINITY) // Never inside (impossible range)
            }
        } else {
            // Normal case: calculate intersection times
            let t1 = (self.min.y - ray.origin.y) / ray.direction.y;
            let t2 = (self.max.y - ray.origin.y) / ray.direction.y;
            (t1.min(t2), t1.max(t2))
        };

        let (tz_smaller, tz_larger) = if ray.direction.z.abs() < 0.0001 {
            // Ray is parallel to X axis
            if ray.origin.z >= self.min.z && ray.origin.z <= self.max.z {
                (f64::NEG_INFINITY, f64::INFINITY) // Inside for all t
            } else {
                (f64::INFINITY, f64::NEG_INFINITY) // Never inside (impossible range)
            }
        } else {
            // Normal case: calculate intersection times
            let t1 = (self.min.z - ray.origin.z) / ray.direction.z;
            let t2 = (self.max.z - ray.origin.z) / ray.direction.z;
            (t1.min(t2), t1.max(t2))
        };

        let t_entry = [tx_smaller, ty_smaller, tz_smaller]
            .into_iter()
            .reduce(f64::max)
            .unwrap_or(0.00);
        let t_exit = [tx_larger, ty_larger, tz_larger]
            .into_iter()
            .reduce(f64::min)
            .unwrap_or(0.00);

        t_entry <= t_exit
    }
}

#[derive(Debug, PartialEq)]
pub enum BVHNode {
    Internal {
        aabb: AABB,
        left: Box<BVHNode>,
        right: Box<BVHNode>,
    },
    Leaf {
        aabb: AABB,
        spheres: Vec<Sphere>,
    },
}

pub fn compute_aabb(spheres: &[Sphere]) -> AABB {
    let mut x_min = spheres[0].center.x - spheres[0].radius;
    let mut x_max = spheres[0].center.x + spheres[0].radius;
    let mut y_min = spheres[0].center.y - spheres[0].radius;
    let mut y_max = spheres[0].center.y + spheres[0].radius;
    let mut z_min = spheres[0].center.z - spheres[0].radius;
    let mut z_max = spheres[0].center.z + spheres[0].radius;

    for &sphere in spheres.iter() {
        let this_x_min = sphere.center.x - sphere.radius;
        let this_x_max = sphere.center.x + sphere.radius;

        let this_y_min = sphere.center.y - sphere.radius;
        let this_y_max = sphere.center.y + sphere.radius;

        let this_z_min = sphere.center.z - sphere.radius;
        let this_z_max = sphere.center.z + sphere.radius;

        if this_x_min < x_min {
            x_min = this_x_min;
        }

        if this_x_max > x_max {
            x_max = this_x_max;
        }

        if this_y_min < y_min {
            y_min = this_y_min;
        }

        if this_y_max > y_max {
            y_max = this_y_max;
        }

        if this_z_min < z_min {
            z_min = this_z_min;
        }

        if this_z_max > z_max {
            z_max = this_z_max;
        }
    }

    AABB {
        min: Vec3 {
            x: x_min,
            y: y_min,
            z: z_min,
        },
        max: Vec3 {
            x: x_max,
            y: y_max,
            z: z_max,
        },
    }
}

// helper funciton to compare f64 values
fn cmp_f64(a: &f64, b: &f64) -> Ordering {
    if a.is_nan() {
        return Ordering::Greater;
    }
    if b.is_nan() {
        return Ordering::Less;
    }
    if a < b {
        return Ordering::Less;
    } else if a > b {
        return Ordering::Greater;
    }
    return Ordering::Equal;
}

impl BVHNode {
    pub fn build(mut spheres: Vec<Sphere>) -> Self {
        if spheres.len() <= 4 {
            let aabb = compute_aabb(&spheres);
            return BVHNode::Leaf { aabb, spheres };
        }

        let aabb = compute_aabb(&spheres);

        let extent_x = aabb.max.x - aabb.min.x;
        let extent_y = aabb.max.y - aabb.min.y;
        let extent_z = aabb.max.z - aabb.min.z;

        let split_axis = if extent_x >= extent_y && extent_x >= extent_z {
            0 // X-axis
        } else if extent_y >= extent_z {
            1 // Y-axis
        } else {
            2 // Z-axis
        };

        match split_axis {
            0 => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.x, &b.center.x));
            }
            1 => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.y, &b.center.y));
            }
            _ => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.z, &b.center.z));
            }
        }

        let mid_idx = spheres.len() / 2;
        let right = spheres.split_off(mid_idx);

        BVHNode::Internal {
            aabb,
            left: Box::new(BVHNode::build(spheres)),
            right: Box::new(BVHNode::build(right)),
        }
    }

    pub fn intersect_bvh(&self, ray: &Ray) -> bool {
        match self {
            BVHNode::Internal { aabb, left, right } => {
                if !aabb.intersect(ray) {
                    return false;
                }

                let left_hit = left.intersect_bvh(ray);
                let right_hit = right.intersect_bvh(ray);

                left_hit || right_hit
            }
            BVHNode::Leaf { aabb, spheres } => {
                if !aabb.intersect(ray) {
                    return false;
                }

                spheres
                    .iter()
                    .any(|sphere| ray_sphere_intersect(ray, &sphere))
            }
        }
    }
}

pub fn render_bvh(width: usize, height: usize, bvh: &BVHNode) -> Image {
    let mut image = Image::new(width, height);
    let camera = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    for h in 0..height {
        let hf64 = h as f64;
        for w in 0..width {
            let wf64 = w as f64;
            let direction = normalized_direction(width, height, wf64, hf64);
            let ray = Ray {
                origin: camera,
                direction,
            };

            let pixel = if bvh.intersect_bvh(&ray) {
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                }
            } else {
                Vec3 {
                    x: 0.5,
                    y: 0.7,
                    z: 1.0,
                }
            };

            let pixel_idx = h * width + w;
            image.pixels[pixel_idx] = pixel;
        }
    }

    image
}

// The Image rendering

#[derive(Debug, Clone)]
pub struct Image {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<Vec3>, // RGB colors, one per pixel
}

impl Image {
    pub fn new(width: usize, height: usize) -> Self {
        Image {
            width,
            height,
            pixels: vec![
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0
                };
                width * height
            ],
        }
    }

    pub fn save_ppm(&self, filename: &str) -> std::io::Result<()> {
        use std::fs::File;
        use std::io::Write;

        let mut file = File::create(filename)?;
        writeln!(file, "P3")?;
        writeln!(file, "{} {}", self.width, self.height)?;
        writeln!(file, "255")?;

        for pixel in &self.pixels {
            let r = (pixel.x.clamp(0.0, 1.0) * 255.0) as u8;
            let g = (pixel.y.clamp(0.0, 1.0) * 255.0) as u8;
            let b = (pixel.z.clamp(0.0, 1.0) * 255.0) as u8;
            writeln!(file, "{} {} {}", r, g, b)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::time::Instant;

    #[test]
    fn test_ray_hits_sphere() {
        let ray = Ray {
            origin: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            direction: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            }, // pointing forward
        };
        let sphere = Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 5.0,
            }, // 5 units ahead
            radius: 1.0,
        };

        assert!(ray_sphere_intersect(&ray, &sphere)); // Should hit!
    }

    #[test]
    fn test_ray_misses_sphere() {
        let ray = Ray {
            origin: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            direction: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            }, // pointing right
        };
        let sphere = Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 5.0,
            }, // ahead, not right
            radius: 1.0,
        };

        assert!(!ray_sphere_intersect(&ray, &sphere)); // Should miss!
    }

    #[test]
    fn test_ray_hits_aabb() {
        let aabb = AABB {
            min: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 4.0,
            },
            max: Vec3 {
                x: 3.0,
                y: 3.0,
                z: 6.0,
            },
        };

        let ray = Ray {
            origin: Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
            direction: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            }, // Shooting forward through box
        };

        assert!(aabb.intersect(&ray));
    }

    #[test]
    fn test_ray_misses_aabb() {
        let aabb = AABB {
            min: Vec3 {
                x: 1.0,
                y: 1.0,
                z: 4.0,
            },
            max: Vec3 {
                x: 3.0,
                y: 3.0,
                z: 6.0,
            },
        };

        let ray = Ray {
            origin: Vec3 {
                x: 5.0,
                y: 2.0,
                z: 0.0,
            }, // Starting to the right of box
            direction: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            }, // Shooting forward, will miss
        };

        assert!(!aabb.intersect(&ray));
    }

    #[test]
    fn test_compute_aabb() {
        let spheres = vec![
            Sphere {
                center: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 5.0,
                    y: 5.0,
                    z: 5.0,
                },
                radius: 1.0,
            },
        ];

        let aabb = compute_aabb(&spheres);

        // What should min and max be?
        // Add assertions here
        assert_eq!(
            aabb.min,
            Vec3 {
                x: -1.0,
                y: -1.0,
                z: -1.0
            }
        );
        assert_eq!(
            aabb.max,
            Vec3 {
                x: 6.0,
                y: 6.0,
                z: 6.0
            }
        );
    }

    #[test]
    fn test_bvh_node_less_than_4_spheres() {
        let mut spheres = vec![
            Sphere {
                center: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 5.0,
                    y: 5.0,
                    z: 5.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 2.0,
                    y: -3.0,
                    z: 1.0,
                },
                radius: 1.0,
            },
        ];

        let bvhnode = BVHNode::build(spheres.clone());

        assert_eq!(
            bvhnode,
            BVHNode::Leaf {
                aabb: compute_aabb(&spheres),
                spheres
            }
        );
    }

    #[test]
    fn test_bvh_node_more_than_4_spheres() {
        let mut spheres = vec![
            Sphere {
                center: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 5.0,
                    y: 5.0,
                    z: 5.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 2.0,
                    y: -3.0,
                    z: 1.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 5.0,
                    y: 5.0,
                    z: 5.0,
                },
                radius: 1.0,
            },
            Sphere {
                center: Vec3 {
                    x: 2.0,
                    y: -3.0,
                    z: 1.0,
                },
                radius: 1.0,
            },
        ];

        let bvhnode = BVHNode::build(spheres.clone());

        let aabb = compute_aabb(&spheres);

        let extent_x = aabb.max.x - aabb.min.x;
        let extent_y = aabb.max.y - aabb.min.y;
        let extent_z = aabb.max.z - aabb.min.z;

        let split_axis = if extent_x >= extent_y && extent_x >= extent_z {
            0 // X-axis
        } else if extent_y >= extent_z {
            1 // Y-axis
        } else {
            2 // Z-axis
        };

        match split_axis {
            0 => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.x, &b.center.x));
            }
            1 => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.y, &b.center.y));
            }
            _ => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.z, &b.center.z));
            }
        }

        let mid_idx = spheres.len() / 2;
        let right_half = spheres.split_off(mid_idx);

        assert_eq!(
            bvhnode,
            BVHNode::Internal {
                aabb,
                left: Box::new(BVHNode::build(spheres)),
                right: Box::new(BVHNode::build(right_half)),
            }
        );
    }

    #[test]
    fn test_bvh_intersect() {
        let spheres = vec![
            Sphere {
                center: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 5.0,
                },
                radius: 1.0,
            },
            // ... a few more spheres
        ];

        let bvh = BVHNode::build(spheres);
        let ray = Ray {
            origin: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            direction: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        };

        assert!(bvh.intersect_bvh(&ray)); // Should hit the sphere at z=5
    }

    #[test]
    fn test_render_naive_produces_correct_size() {
        let spheres = vec![Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 5.0,
            },
            radius: 1.0,
        }];
        let image = render_naive(10, 10, &spheres);
        assert_eq!(image.pixels.len(), 100);
    }

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

    /*
     * This test interferes with the GH Actions workflow
    #[test]
    fn test_render_bvh_500_below_600_ms() {
        let spheres = create_many_spheres(500);
        let bvh = BVHNode::build(spheres);

        let now = Instant::now();
        let image = render_bvh(400, 300, &bvh);
        let elapsed = now.elapsed().as_millis();
        println!("Elapsed: {elapsed}");
        assert!(elapsed < 600);
    }
    */

    proptest! {
        #[test]
        fn property_aabb_contains_all_spheres(
            sphere_cap in 1usize..500
            ) {
            // Generate 100 random spheres
                let spheres = create_many_spheres(sphere_cap);
                let aabb = compute_aabb(&spheres);

                // Property: EVERY sphere must fit inside the AABB
                for sphere in &spheres {
                    prop_assert!(sphere.center.x - sphere.radius >= aabb.min.x);
                    prop_assert!(sphere.center.x + sphere.radius <= aabb.max.x);
                    // ... same for y and z
                    prop_assert!(sphere.center.y - sphere.radius >= aabb.min.y);
                    prop_assert!(sphere.center.y + sphere.radius <= aabb.max.y);

                    prop_assert!(sphere.center.z - sphere.radius >= aabb.min.z);
                    prop_assert!(sphere.center.z + sphere.radius <= aabb.max.z);
                }

        }
    }
}
