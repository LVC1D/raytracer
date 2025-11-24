# Raytracer: Learnings

## What is BVH and why does it matter?

BVH (or Bounding Volume Hierarchy) is a type of an algorithm that allows us to search through sections of the box (i.e.: 3D space)
that are split by the density of the grouped objects in them, compared to just splitting in exact halves.

This idea is synonymous to the binary search because we organize the data in a tree-like form to iterate over, and the 
time complexity is O(log N) compared to O(N) in a standard naive approach.

This is useful when we need to perform certain calculaitons to process 2D or 3D scenes and / or render those
several times faster and more efficiently than with standard iteration-like approach.

## Key technical concepts I learned:

- AABB (Axis-Aligned Bounding Box): 

Instead of relying on the center position, width or height - we simply get two coordinates:
The one with the most minimal coordinates (closest to the origin) and the one with the largest coordinates (the farthest from the origin)

- Spatial Splitting:

This is the core of the BVH algorithm construction. As we define the initial AABB box, we see which
extent (distance from the minimal coord-point to the maximal coord-point) is the largest, based on which we
dictate along which axis to split - that ensures we get the splitting into sub-areas with the most tightly-grouped objects 
we then perform checks / operations against.

- BVH Tree Traversal:

Here we rely on recursive method calling (just like we would in binary searching) across the tree until we reach the leaf nodes 
(the nodes without children - but that have the object(s) grouped in).

The key advantage to note here is the efficiency represented by how many iterations the algorithm requiers until we find the obejct in question,
whcih defines the exact strength of it as the tree scales.

## Implementation challenges I solved:

1. Division by zero for parallel rays

In the `ray_sphere_intersect()` function, we used a quadratic formula to determine whether the ray hits the sphere
or not.

As the quadratic formula involves a discriminant, there were possibilites that the produced value could be a negative floating number.
And taking a square root of the negative floating number returns a `NaN` so that had to be checked first

2. Determining split axis

As explained in the Spatial Splitting section among the key concepts I learned, by obtaining the values of the extents for 
all 3 axes, followed up by establishing which one is the largest, we then split along that axis to ensure each sub-AABB contains the most 
tightly-grouped objects to check for (in our case - if the ray hits that AABB area on the entry point and (potentially) the exit point).

3. Other challenges

Related to splitting spatially, in order to efficiently split the spheres collection into the 2 logically-sound halves,
we had to to compare the 2 floating-point values of each 2 adjacent spheres' centers.

Take a look at this part of the BVHNode's `.build()` method snippet:

```rust
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

// the .build method()
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
            0  // X-axis
        } else if extent_y >= extent_z {
            1  // Y-axis
        } else {
            2  // Z-axis
        };

        match split_axis {
            0 => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.x, &b.center.x));
            },
            1 => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.y, &b.center.y));
            },
            _ => {
                spheres.sort_by(|a, b| cmp_f64(&a.center.z, &b.center.z));
            }, 
        }

        let mid_idx = spheres.len() / 2;
        let right = spheres.split_off(mid_idx);

        BVHNode::Internal {
            aabb,
            left: Box::new(BVHNode::build(spheres)),
            right: Box::new(BVHNode::build(right)),
        }
    }

```

Rust's `cmp` method doesn't seem to have the floating-point numbers to implement the Ordering trait, so we had to go about using a helper function
to be able to use it directly as a closure for `.sort_by()`

## The 12x speedup story:

We tested the naive and BVH-optimized algorithms to render a 400x300 image that simulates 100, 200, 500 and 1000 using the `cargo run`
command, and only the 1000-sphere sample test was run with `--release` flag to illustrate the effect of additional compiler-enforced optimizations.

We noticed that with a 100-sphere sample test run ,the BVH approach was 0.96x faster than the naive approach (meaning, slower)./
That is because Rust's compiler has deemed the sample size so small that further BVH optimizations are merely additional overhead-adding operations.

From 200 samples onwards, the performance improvement was increasing geometrically, finally reaching up to 12x improvement on a 1000-sample test run with the 
`--release` flag. Without that flag, the result for BFH-optimized approach yielded a rough estimate of 9.5x improvement (still pretty good).
