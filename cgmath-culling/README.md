# cgmath frustum culling

[![Build Status](https://travis-ci.org/germangb/cgmath-culling.svg?branch=master)](https://travis-ci.org/germangb/cgmath-culling)
[![Package](https://img.shields.io/crates/v/cgmath-culling.svg)](https://crates.io/crates/cgmath-culling)
[![Downloads](https://img.shields.io/crates/d/cgmath-culling.svg)](https://crates.io/crates/cgmath-culling)
![LICENSE](https://img.shields.io/crates/l/cgmath-culling.svg)


Small [frustum culling](https://en.wikipedia.org/wiki/Hidden_surface_determination#Viewing_frustum_culling) crate meant to be used alongside the `cgmath` crate.

## Usage

```rust
extern crate cgmath;
extern crate cgmath_culling;

use cgmath::{PerspectiveFov, Rad};
use cgmath_culling::{FrustumCuller, BoundingBox, Intersection as Int};

let per = PerspectiveFov { fovy: Rad(3.1415_f32 / 2.0),
                           aspect: 1.0,
                           near: 0.1,
                           far: 100.0 };

let culling = FrustumCuller::from_perspective_fov(per);
let bounding_box = BoundingBox::from_params(Vector3::new(0.0, 0.0, -7.0), Vector3::new(1.0, 1.0, -5.0));

match culling.test_bounding_box(bounding_box) {
    Intersection::Inside | Intersection::Partial => println!("I'm Inside!!!"),
    Intersection::Outside => println!("I'm Outside"),
}
```

## License

[MIT](LICENSE.md)

## Disclaimer

* Reference implementation: [JOML](https://github.com/JOML-CI/JOML)
