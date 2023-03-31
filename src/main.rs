#![cfg_attr(feature = "cargo-clippy", allow(cast_lossless))]
#![allow(unused_must_use)]
#![allow(dead_code)]

extern crate cgmath;
extern crate gl;
extern crate image;
extern crate tobj;

mod camera;
mod common;
mod macros;
mod mesh;
mod model;
mod profiler;
mod quadtree;
mod shader;
mod utils;
mod vehicle;

mod game;
use game::*;

fn main() {
    start();
}
