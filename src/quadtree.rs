extern crate ndarray;
use self::ndarray::Array2;
use crate::game::NODE_COUNTER;
use crate::game::TRIANGLE_COUNTER;
extern crate rand;
use self::rand::Rng;

use std::ffi::CString;

use crate::model::Model;
use crate::shader::Shader;
use cgmath::prelude::*;
use cgmath::{vec2, vec3, Deg, Matrix4, Vector2, Vector3, Vector4};
use cgmath_culling::{BoundingBox, FrustumCuller, Intersection};
use gl::types::*;

use std::mem;
use std::os::raw::c_void;
use std::ptr;

const COLOUR_SCALE: f32 = 0.5;
const maxheight: f32 = 140.0;
const SIZE_MAT4: i32 = mem::size_of::<Matrix4<f32>>() as i32;
const SIZE_VEC4: i32 = mem::size_of::<Vector4<f32>>() as i32;
const LOD2_DISTANCE: f32 = 14.0 * (40.0 / 10.0);
const LOD3_DISTANCE: f32 = 28.0 * (40.0 / 10.0);
const LOD4_DISTANCE: f32 = 29.0 * (40.0 / 10.0);
const LOD5_DISTANCE: f32 = 30.0 * (40.0 / 10.0); // const LOD2_DISTANCE: f32 = 10.0;
                                                 // const LOD3_DISTANCE: f32 = 25.0;
                                                 // const LOD4_DISTANCE: f32 = 35.0;
                                                 // const LOD5_DISTANCE: f32 = 50.0;
const ZOOM_LOD: f32 = 1.04;
const TREE_ZOOM_LOD: f32 = 1.035;
const SIDE1: bool = true;
const SIDE2: bool = true;
const SIDE3: bool = true;
const SIDE4: bool = true;
const EDGE_HEIGHT: f32 = 8.0;

#[derive(Debug)]
pub struct Quadtree {
    pub level: u32,
    one: Option<Box<Quadtree>>,
    two: Option<Box<Quadtree>>,
    three: Option<Box<Quadtree>>,
    four: Option<Box<Quadtree>>,
    leaf: bool,
    centre: Vector3<f32>,
    width: f32,
    vertices_size: u32,
    vertices: Vec<f32>,
    indices_size: u32,
    indices1: Vec<i32>,
    indices2: Vec<i32>,
    indices3: Vec<i32>,
    indices4: Vec<i32>,
    indices5: Vec<i32>,
    edge_vertices_size: u32,
    edge_vertices: Vec<f32>,
    edge_indices_size: u32,
    edge_indices1: Vec<i32>,
    edge_indices2: Vec<i32>,
    edge_indices3: Vec<i32>,
    edge_indices4: Vec<i32>,
    edge_indices5: Vec<i32>,
    min_height: f32,
    max_height: f32,
    vbo: gl::types::GLuint,
    ebo1: gl::types::GLuint,
    ebo2: gl::types::GLuint,
    ebo3: gl::types::GLuint,
    ebo4: gl::types::GLuint,
    ebo5: gl::types::GLuint,
    edge_vbo: gl::types::GLuint,
    edge_ebo1: gl::types::GLuint,
    edge_ebo2: gl::types::GLuint,
    edge_ebo3: gl::types::GLuint,
    edge_ebo4: gl::types::GLuint,
    edge_ebo5: gl::types::GLuint,
    tree_count: u32,
    model_matrices: Vec<Matrix4<f32>>,
    // model_matrices: Vec<f32>,
    tree_buffer: gl::types::GLuint,
    index_draw1: i32,
    index_draw2: i32,
    index_draw3: i32,
    index_draw4: i32,
    index_draw5: i32,
    index_edge_draw1: i32,
    index_edge_draw2: i32,
    index_edge_draw3: i32,
    index_edge_draw4: i32,
    index_edge_draw5: i32,
}
impl Quadtree {
    pub fn new(level: u32) -> Quadtree {
        Quadtree {
            level,
            one: None,
            two: None,
            three: None,
            four: None,
            leaf: false,
            centre: vec3(0.0, 0.0, 0.0),
            width: 0.0,
            vertices_size: 0,
            vertices: vec![],
            indices_size: 0,
            indices1: vec![],
            indices2: vec![],
            indices3: vec![],
            indices4: vec![],
            indices5: vec![],
            edge_vertices_size: 0,
            edge_vertices: vec![],
            edge_indices_size: 0,
            edge_indices1: vec![],
            edge_indices2: vec![],
            edge_indices3: vec![],
            edge_indices4: vec![],
            edge_indices5: vec![],
            min_height: 10000.0,
            max_height: -10000.0,
            vbo: 0,
            ebo1: 0,
            ebo2: 0,
            ebo3: 0,
            ebo4: 0,
            ebo5: 0,
            edge_vbo: 0,
            edge_ebo1: 0,
            edge_ebo2: 0,
            edge_ebo3: 0,
            edge_ebo4: 0,
            edge_ebo5: 0,
            tree_count: 0,
            model_matrices: Vec::new(),
            // model_matrices: vec![],
            tree_buffer: 0,
            index_draw1: 0,
            index_draw2: 0,
            index_draw3: 0,
            index_draw4: 0,
            index_draw5: 0,
            index_edge_draw1: 0,
            index_edge_draw2: 0,
            index_edge_draw3: 0,
            index_edge_draw4: 0,
            index_edge_draw5: 0,
        }
    }
    pub fn create_tree(
        &mut self,
        root: &mut Quadtree,
        level: u32,
        centre: Vector3<f32>,
        width: u16,
        heights: &mut Array2<f32>,
        tree_shadows: &mut Array2<f32>,
        world_size: i32,
        leaf_size: i32,
        tree_heights: &mut Array2<f32>,
        tree_centres_x: &mut Array2<f32>,
        tree_centres_z: &mut Array2<f32>,
        tree_sizes: &mut Array2<f32>,
    ) {
        unsafe {
            NODE_COUNTER = NODE_COUNTER + 1;
            // println!("{}", NODE_COUNTER);
        }
        root.level = level;
        root.width = width as f32;
        root.centre = centre;
        // let vertex_count: u32 = (root.width + 6.0 + ((LOD * 2) - 1) as f32).powf(2.0) as u32;
        let vertex_count: u32 = (root.width + 32.0).powf(2.0) as u32;
        let edge_vertex_count: u32 = ((root.width + 32.0) * 2.0 * 8.0 * 2.0) as u32;
        if vertex_count >= leaf_size as u32 {
            root.leaf = false;
            let mut one: Quadtree = Quadtree::new(level + 1);
            let mut two: Quadtree = Quadtree::new(level + 1);
            let mut three: Quadtree = Quadtree::new(level + 1);
            let mut four: Quadtree = Quadtree::new(level + 1);
            let mut newcentre: Vector3<f32> =
                vec3(root.centre.x - root.width / 4.0, 0.0, root.centre.z - root.width / 4.0);
            root.create_tree(
                &mut one,
                level + 1,
                newcentre,
                (root.width / 2.0) as u16,
                heights,
                tree_shadows,
                world_size,
                leaf_size,
                tree_heights,
                tree_centres_x,
                tree_centres_z,
                tree_sizes,
            );
            root.one = Some(Box::new(one));
            newcentre = vec3(root.centre.x + root.width / 4.0, 0.0, root.centre.z - root.width / 4.0);
            root.create_tree(
                &mut two,
                level + 1,
                newcentre,
                (root.width / 2.0) as u16,
                heights,
                tree_shadows,
                world_size,
                leaf_size,
                tree_heights,
                tree_centres_x,
                tree_centres_z,
                tree_sizes,
            );
            root.two = Some(Box::new(two));
            newcentre = vec3(root.centre.x + root.width / 4.0, 0.0, root.centre.z + root.width / 4.0);
            root.create_tree(
                &mut three,
                level + 1,
                newcentre,
                (root.width / 2.0) as u16,
                heights,
                tree_shadows,
                world_size,
                leaf_size,
                tree_heights,
                tree_centres_x,
                tree_centres_z,
                tree_sizes,
            );
            root.three = Some(Box::new(three));
            newcentre = vec3(root.centre.x - root.width / 4.0, 0.0, root.centre.z + root.width / 4.0);
            root.create_tree(
                &mut four,
                level + 1,
                newcentre,
                (root.width / 2.0) as u16,
                heights,
                tree_shadows,
                world_size,
                leaf_size,
                tree_heights,
                tree_centres_x,
                tree_centres_z,
                tree_sizes,
            );
            root.four = Some(Box::new(four));
        } else {
            root.leaf = true;
            root.vertices_size = vertex_count * 34;
            root.vertices = vec![0.0; root.vertices_size as usize];
            root.indices_size = ((root.width + 26.0).powf(2.0) * 6.0) as u32;
            root.indices1 = vec![0; root.indices_size as usize];
            root.indices2 = vec![0; (root.indices_size / 2) as usize];
            root.indices3 = vec![0; (root.indices_size / 4) as usize];
            root.indices4 = vec![0; (root.indices_size / 8) as usize];
            root.indices5 = vec![0; (root.indices_size / 16) as usize];
            root.edge_vertices_size = edge_vertex_count * 34;
            root.edge_vertices = vec![0.0; root.edge_vertices_size as usize];
            root.edge_indices_size = ((root.width + 26.0) * 6.0 * 4.0) as u32;
            root.edge_indices1 = vec![0; root.edge_indices_size as usize];
            root.edge_indices2 = vec![0; (root.edge_indices_size / 2) as usize];
            root.edge_indices3 = vec![0; (root.edge_indices_size / 4) as usize];
            root.edge_indices4 = vec![0; (root.edge_indices_size / 8) as usize];
            root.edge_indices5 = vec![0; (root.edge_indices_size / 16) as usize];
            let mut vertex_index = 0;
            root.min_height = -500.0;
            root.max_height = 500.0;
            let mut edge_index_count1 = 0;
            let mut edge_index_count2 = 0;
            let mut edge_index_count3 = 0;
            let mut edge_index_count4 = 0;
            let mut edge_index_count5 = 0;
            let vertex_index_offset = vertex_index / 34;
            if SIDE1 {
                let z = (root.centre.z - (root.width / 2.0)) as i32;
                for x in
                    (root.centre.x - (root.width / 2.0)) as i32..(root.centre.x + (root.width / 2.0) + (32.0)) as i32
                {
                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;

                    // Vertex positions
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0) + 0.5;
                    root.edge_vertices[vertex_index + 1] = xz_height;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0) + 0.5;

                    // Shadows
                    let slope = get_average_slope(heights, x, z, world_size - 1);
                    let height = xz_height;
                    let shadow = tree_shadows[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]];
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    let mut normal: Vector3<f32> = vec3(0.0, 0.0, 0.0);
                    if x > 0 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, xz_height, z as f32),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                        );
                    }
                    if x < world_size - 1 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, heights[[x as usize, z as usize]], z as f32),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                        );
                    }
                    if x < world_size - 1 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                        );
                    }
                    if x > 0 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                        );
                    }
                    normal = normalise_normal(normal);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;

                    //grass
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);

                    //grassrock
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);

                    //rock
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);

                    //snow
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);

                    //grass2
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);

                    //grassrock2
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);

                    //rock2
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);

                    //snow2
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    let grassrock = clamp(slope / (4.0 / COLOUR_SCALE)) * (1.0) * 1.0;
                    let mut grass = clamp((1.0 - clamp(height / (270.0 / COLOUR_SCALE))) - grassrock);
                    let mut rock = (clamp((height - (180.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE))
                        * clamp(slope / (12.0 / COLOUR_SCALE)))
                        * 2.5;
                    let snow = clamp(clamp((height - (260.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE)) - rock);
                    let grassrock2 = grassrock / 2.0;
                    let mut grass2 = grass / 2.0;
                    let mut rock2 = rock / 2.0;
                    let snow2 = snow / 2.0;
                    let total = grass + grass2 + grassrock + grassrock2;
                    grass /= total;
                    grass2 /= total;
                    rock /= total;
                    rock2 /= total;
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;

                    //--------------------------------------------------------------------------------------------

                    //--------------------------------------------------------------------------------------------

                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 1] = xz_height - EDGE_HEIGHT;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;
                }
                //--------------------------------------------------------------------------------------------

                //--------------------------------------------------------------------------------------------
                for w in (0..=(root.width - 1.0) as i32).step_by(1 as usize) {
                    root.edge_indices1[edge_index_count1 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices1[edge_index_count1 + 1] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 4] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 5] = vertex_index_offset as i32 + (w * 2) + 3;
                    edge_index_count1 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(2 as usize) {
                    root.edge_indices2[edge_index_count2 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices2[edge_index_count2 + 1] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 4] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 5] = vertex_index_offset as i32 + (w * 2) + 5;
                    edge_index_count2 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(4 as usize) {
                    root.edge_indices3[edge_index_count3 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices3[edge_index_count3 + 1] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 4] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 5] = vertex_index_offset as i32 + (w * 2) + 9;
                    edge_index_count3 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(8 as usize) {
                    root.edge_indices4[edge_index_count4 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices4[edge_index_count4 + 1] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 4] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 5] = vertex_index_offset as i32 + (w * 2) + 17;
                    edge_index_count4 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(16 as usize) {
                    root.edge_indices5[edge_index_count5 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices5[edge_index_count5 + 1] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 4] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 5] = vertex_index_offset as i32 + (w * 2) + 33;
                    edge_index_count5 += 6;
                }
            }
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            if SIDE2 {
                let vertex_index_offset = vertex_index / 34; //((root.width + 32.0) * 34.0) as i32;
                let x = (root.centre.x + (root.width / 2.0)) as i32;
                for z in
                    (root.centre.z - (root.width / 2.0)) as i32..(root.centre.z + (root.width / 2.0) + (32.0)) as i32
                {
                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;

                    // Vertex positions
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0) + 0.5;
                    root.edge_vertices[vertex_index + 1] = xz_height;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0) + 0.5;

                    // Shadows
                    let slope = get_average_slope(heights, x, z, world_size - 1);
                    let height = xz_height;
                    let shadow = tree_shadows[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]];
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    let mut normal: Vector3<f32> = vec3(0.0, 0.0, 0.0);
                    if x > 0 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, xz_height, z as f32),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                        );
                    }
                    if x < world_size - 1 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, heights[[x as usize, z as usize]], z as f32),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                        );
                    }
                    if x < world_size - 1 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                        );
                    }
                    if x > 0 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                        );
                    }
                    normal = normalise_normal(normal);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;

                    //grass
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);

                    //grassrock
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);

                    //rock
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);

                    //snow
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);

                    //grass2
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);

                    //grassrock2
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);

                    //rock2
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);

                    //snow2
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    let grassrock = clamp(slope / (4.0 / COLOUR_SCALE)) * (1.0) * 1.0;
                    let mut grass = clamp((1.0 - clamp(height / (170.0 / COLOUR_SCALE))) - grassrock);
                    let mut rock = (clamp((height - (80.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE))
                        * clamp(slope / (12.0 / COLOUR_SCALE)))
                        * 2.5;
                    let snow = clamp(clamp((height - (160.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE)) - rock);
                    let grassrock2 = grassrock / 2.0;
                    let mut grass2 = grass / 2.0;
                    let mut rock2 = rock / 2.0;
                    let snow2 = snow / 2.0;
                    let total = grass + grass2 + grassrock + grassrock2;
                    grass /= total;
                    grass2 /= total;
                    rock /= total;
                    rock2 /= total;
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;

                    //--------------------------------------------------------------------------------------------

                    //--------------------------------------------------------------------------------------------

                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 1] = xz_height - EDGE_HEIGHT;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;
                }
                //--------------------------------------------------------------------------------------------

                //--------------------------------------------------------------------------------------------
                for w in (0..=(root.width - 1.0) as i32).step_by(1 as usize) {
                    root.edge_indices1[edge_index_count1 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices1[edge_index_count1 + 1] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 4] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 5] = vertex_index_offset as i32 + (w * 2) + 3;
                    edge_index_count1 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(2 as usize) {
                    root.edge_indices2[edge_index_count2 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices2[edge_index_count2 + 1] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 4] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 5] = vertex_index_offset as i32 + (w * 2) + 5;
                    edge_index_count2 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(4 as usize) {
                    root.edge_indices3[edge_index_count3 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices3[edge_index_count3 + 1] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 4] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 5] = vertex_index_offset as i32 + (w * 2) + 9;
                    edge_index_count3 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(8 as usize) {
                    root.edge_indices4[edge_index_count4 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices4[edge_index_count4 + 1] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 4] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 5] = vertex_index_offset as i32 + (w * 2) + 17;
                    edge_index_count4 += 6;
                }
                for w in (0..=(root.width - 1.0) as i32).step_by(16 as usize) {
                    root.edge_indices5[edge_index_count5 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices5[edge_index_count5 + 1] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 4] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 5] = vertex_index_offset as i32 + (w * 2) + 33;
                    edge_index_count5 += 6;
                }
            }
            if SIDE3 {
                let vertex_index_offset = vertex_index / 34; //((root.width + 32.0) * 34.0) as i32;
                let z = (root.centre.z + (root.width / 2.0)) as i32;
                for x in
                    (root.centre.x - (root.width / 2.0)) as i32..(root.centre.x + (root.width / 2.0) + (32.0)) as i32
                {
                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;

                    // Vertex positions
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0) + 0.5;
                    root.edge_vertices[vertex_index + 1] = xz_height;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0) + 0.5;

                    // Shadows
                    let slope = get_average_slope(heights, x, z, world_size - 1);
                    let height = xz_height;
                    let shadow = tree_shadows[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]];
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    let mut normal: Vector3<f32> = vec3(0.0, 0.0, 0.0);
                    if x > 0 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, xz_height, z as f32),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                        );
                    }
                    if x < world_size - 1 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, heights[[x as usize, z as usize]], z as f32),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                        );
                    }
                    if x < world_size - 1 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                        );
                    }
                    if x > 0 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                        );
                    }
                    normal = normalise_normal(normal);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;

                    //grass
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);

                    //grassrock
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);

                    //rock
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);

                    //snow
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);

                    //grass2
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);

                    //grassrock2
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);

                    //rock2
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);

                    //snow2
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    let grassrock = clamp(slope / (4.0 / COLOUR_SCALE)) * (1.0) * 1.0;
                    let mut grass = clamp((1.0 - clamp(height / (170.0 / COLOUR_SCALE))) - grassrock);
                    let mut rock = (clamp((height - (80.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE))
                        * clamp(slope / (12.0 / COLOUR_SCALE)))
                        * 2.5;
                    let snow = clamp(clamp((height - (160.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE)) - rock);
                    let grassrock2 = grassrock / 2.0;
                    let mut grass2 = grass / 2.0;
                    let mut rock2 = rock / 2.0;
                    let snow2 = snow / 2.0;
                    let total = grass + grass2 + grassrock + grassrock2;
                    grass /= total;
                    grass2 /= total;
                    rock /= total;
                    rock2 /= total;
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;

                    //--------------------------------------------------------------------------------------------

                    //--------------------------------------------------------------------------------------------

                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 1] = xz_height - EDGE_HEIGHT;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;
                }

                //--------------------------------------------------------------------------------------------

                //--------------------------------------------------------------------------------------------
                for w in (0..=(root.width - 1.0) as i32).step_by(1 as usize) {
                    root.edge_indices1[edge_index_count1 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices1[edge_index_count1 + 1] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 4] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 5] = vertex_index_offset as i32 + (w * 2) + 3;
                    edge_index_count1 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(2 as usize) {
                    root.edge_indices2[edge_index_count2 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices2[edge_index_count2 + 1] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 4] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 5] = vertex_index_offset as i32 + (w * 2) + 5;
                    edge_index_count2 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(4 as usize) {
                    root.edge_indices3[edge_index_count3 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices3[edge_index_count3 + 1] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 4] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 5] = vertex_index_offset as i32 + (w * 2) + 9;
                    edge_index_count3 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(8 as usize) {
                    root.edge_indices4[edge_index_count4 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices4[edge_index_count4 + 1] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 4] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 5] = vertex_index_offset as i32 + (w * 2) + 17;
                    edge_index_count4 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(16 as usize) {
                    root.edge_indices5[edge_index_count5 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices5[edge_index_count5 + 1] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 4] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 5] = vertex_index_offset as i32 + (w * 2) + 33;
                    edge_index_count5 += 6;
                }
            }
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            if SIDE4 {
                let vertex_index_offset = vertex_index / 34; //((root.width + 32.0) * 34.0) as i32;
                let x = (root.centre.x - (root.width / 2.0)) as i32;
                for z in
                    (root.centre.z - (root.width / 2.0)) as i32..(root.centre.z + (root.width / 2.0) + (32.0)) as i32
                {
                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;

                    // Vertex positions
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0) + 0.5;
                    root.edge_vertices[vertex_index + 1] = xz_height;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0) + 0.5;

                    // Shadows
                    let slope = get_average_slope(heights, x, z, world_size - 1);
                    let height = xz_height;
                    let shadow = tree_shadows[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]];
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    let mut normal: Vector3<f32> = vec3(0.0, 0.0, 0.0);
                    if x > 0 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, xz_height, z as f32),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                        );
                    }
                    if x < world_size - 1 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, heights[[x as usize, z as usize]], z as f32),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                        );
                    }
                    if x < world_size - 1 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                        );
                    }
                    if x > 0 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                        );
                    }
                    normal = normalise_normal(normal);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;

                    //grass
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);

                    //grassrock
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);

                    //rock
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);

                    //snow
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);

                    //grass2
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);

                    //grassrock2
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);

                    //rock2
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);

                    //snow2
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    let grassrock = clamp(slope / (4.0 / COLOUR_SCALE)) * (1.0) * 1.0;
                    let mut grass = clamp((1.0 - clamp(height / (170.0 / COLOUR_SCALE))) - grassrock);
                    let mut rock = (clamp((height - (80.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE))
                        * clamp(slope / (12.0 / COLOUR_SCALE)))
                        * 2.5;
                    let snow = clamp(clamp((height - (160.0 / COLOUR_SCALE)) / (140.0 / COLOUR_SCALE)) - rock);
                    let grassrock2 = grassrock / 2.0;
                    let mut grass2 = grass / 2.0;
                    let mut rock2 = rock / 2.0;
                    let snow2 = snow / 2.0;
                    let total = grass + grass2 + grassrock + grassrock2;
                    grass /= total;
                    grass2 /= total;
                    rock /= total;
                    rock2 /= total;
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;

                    //--------------------------------------------------------------------------------------------

                    //--------------------------------------------------------------------------------------------

                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;
                    root.edge_vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 1] = xz_height - EDGE_HEIGHT;
                    root.edge_vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0);
                    root.edge_vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    root.edge_vertices[vertex_index + 6] = normal.x;
                    root.edge_vertices[vertex_index + 7] = normal.y;
                    root.edge_vertices[vertex_index + 8] = normal.z;
                    root.edge_vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.edge_vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);
                    root.edge_vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.edge_vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);
                    root.edge_vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.edge_vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);
                    root.edge_vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.edge_vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);
                    root.edge_vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.edge_vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);
                    root.edge_vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.edge_vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);
                    root.edge_vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.edge_vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);
                    root.edge_vertices[vertex_index + 25] = grass;
                    root.edge_vertices[vertex_index + 26] = grassrock;
                    root.edge_vertices[vertex_index + 27] = rock;
                    root.edge_vertices[vertex_index + 28] = snow;
                    root.edge_vertices[vertex_index + 29] = grass2;
                    root.edge_vertices[vertex_index + 30] = grassrock2;
                    root.edge_vertices[vertex_index + 31] = rock2;
                    root.edge_vertices[vertex_index + 32] = snow2;
                    root.edge_vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;
                }
                //--------------------------------------------------------------------------------------------

                //--------------------------------------------------------------------------------------------
                for w in (0..=(root.width - 1.0) as i32).step_by(1 as usize) {
                    root.edge_indices1[edge_index_count1 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices1[edge_index_count1 + 1] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices1[edge_index_count1 + 4] = vertex_index_offset as i32 + (w * 2) + 2;
                    root.edge_indices1[edge_index_count1 + 5] = vertex_index_offset as i32 + (w * 2) + 3;
                    edge_index_count1 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(2 as usize) {
                    root.edge_indices2[edge_index_count2 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices2[edge_index_count2 + 1] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices2[edge_index_count2 + 4] = vertex_index_offset as i32 + (w * 2) + 4;
                    root.edge_indices2[edge_index_count2 + 5] = vertex_index_offset as i32 + (w * 2) + 5;
                    edge_index_count2 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(4 as usize) {
                    root.edge_indices3[edge_index_count3 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices3[edge_index_count3 + 1] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices3[edge_index_count3 + 4] = vertex_index_offset as i32 + (w * 2) + 8;
                    root.edge_indices3[edge_index_count3 + 5] = vertex_index_offset as i32 + (w * 2) + 9;
                    edge_index_count3 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(8 as usize) {
                    root.edge_indices4[edge_index_count4 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices4[edge_index_count4 + 1] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices4[edge_index_count4 + 4] = vertex_index_offset as i32 + (w * 2) + 16;
                    root.edge_indices4[edge_index_count4 + 5] = vertex_index_offset as i32 + (w * 2) + 17;
                    edge_index_count4 += 6;
                }

                for w in (0..=(root.width - 1.0) as i32).step_by(16 as usize) {
                    root.edge_indices5[edge_index_count5 + 0] = vertex_index_offset as i32 + (w * 2);
                    root.edge_indices5[edge_index_count5 + 1] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 2] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 3] = vertex_index_offset as i32 + (w * 2) + 1;
                    root.edge_indices5[edge_index_count5 + 4] = vertex_index_offset as i32 + (w * 2) + 32;
                    root.edge_indices5[edge_index_count5 + 5] = vertex_index_offset as i32 + (w * 2) + 33;
                    edge_index_count5 += 6;
                }
            }
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------

            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            //---------------------------------------------------------------------------
            root.index_edge_draw1 = edge_index_count1 as i32;
            root.index_edge_draw2 = edge_index_count2 as i32;
            root.index_edge_draw3 = edge_index_count3 as i32;
            root.index_edge_draw4 = edge_index_count4 as i32;
            root.index_edge_draw5 = edge_index_count5 as i32;
            let mut index_count1 = 0;
            let mut index_count2 = 0;
            let mut index_count3 = 0;
            let mut index_count4 = 0;
            let mut index_count5 = 0;

            //--------------------------------------------------------------------------------------------

            //--------------------------------------------------------------------------------------------
            let mut vertex_index = 0;

            for z in (root.centre.z - (root.width / 2.0)) as i32..(root.centre.z + (root.width / 2.0) + (32.0)) as i32 {
                for x in
                    (root.centre.x - (root.width / 2.0)) as i32..(root.centre.x + (root.width / 2.0) + (32.0)) as i32
                {
                    let xz_height = heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] as f32;

                    // Vertex positions
                    root.vertices[vertex_index] = x as f32 - (world_size as f32 / 2.0) + 0.5;
                    root.vertices[vertex_index + 1] = xz_height;
                    root.vertices[vertex_index + 2] = z as f32 - (world_size as f32 / 2.0) + 0.5;

                    // Shadows
                    let slope = get_average_slope(heights, x, z, world_size - 1);
                    let height = xz_height;
                    let shadow = tree_shadows[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]];
                    root.vertices[vertex_index + 3] = 1.0 - (shadow * 0.5);
                    root.vertices[vertex_index + 4] = 1.0 - (shadow * 0.5);
                    root.vertices[vertex_index + 5] = 1.0 - (shadow * 0.5);
                    let mut normal: Vector3<f32> = vec3(0.0, 0.0, 0.0);
                    if x > 0 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, xz_height, z as f32),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[
                                    (x - 1).min(world_size - 1) as usize,
                                    (z + 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                        );
                    }
                    if x < world_size - 1 && z < world_size - 1 {
                        normal += calc_surface_normal(
                            vec3(x as f32, heights[[x as usize, z as usize]], z as f32),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z + 1).min(world_size - 1) as usize]],
                                z as f32 + 1.0,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z as usize]],
                                z as f32,
                            ),
                        );
                    }
                    if x < world_size - 1 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[(x + 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                        );
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32 + 1.0,
                                heights[[
                                    (x + 1).min(world_size - 1) as usize,
                                    (z - 1).min(world_size - 1) as usize,
                                ]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32,
                                heights[[x as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                        );
                    }
                    if x > 0 && z > 0 {
                        normal += calc_surface_normal(
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                            vec3(
                                x as f32,
                                heights[[x.min(world_size - 1) as usize, (z - 1).min(world_size - 1) as usize]],
                                z as f32 - 1.0,
                            ),
                            vec3(
                                x as f32 - 1.0,
                                heights[[(x - 1).min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                z as f32,
                            ),
                        );
                    }
                    normal = normalise_normal(normal);
                    root.vertices[vertex_index + 6] = normal.x;
                    root.vertices[vertex_index + 7] = normal.y;
                    root.vertices[vertex_index + 8] = normal.z;

                    //grass
                    root.vertices[vertex_index + 9] = 0.03 * x as f32 + ((z as f32 / 7.7).sin() * 0.15);
                    root.vertices[vertex_index + 10] = 0.028 * z as f32 + ((x as f32 / 6.7).sin() * 0.15);

                    //grassrock
                    root.vertices[vertex_index + 11] = 0.031 * x as f32 + ((z as f32 / 6.5).sin() * 0.15);
                    root.vertices[vertex_index + 12] = 0.027 * z as f32 + ((x as f32 / 7.1).sin() * 0.15);

                    //rock
                    root.vertices[vertex_index + 13] = 0.033 * x as f32 + ((z as f32 / 6.9).sin() * 0.15);
                    root.vertices[vertex_index + 14] = 0.026 * z as f32 + ((x as f32 / 7.3).sin() * 0.15);

                    //snow
                    root.vertices[vertex_index + 15] = 0.042 * x as f32 + ((z as f32 / 6.7).sin() * 0.15);
                    root.vertices[vertex_index + 16] = 0.033 * z as f32 + ((x as f32 / 7.2).sin() * 0.15);

                    //grass2
                    root.vertices[vertex_index + 17] = 2.4 * x as f32 + ((z as f32 / 0.44).sin() * 0.35);
                    root.vertices[vertex_index + 18] = 2.36 * z as f32 + ((x as f32 / 0.34).sin() * 0.35);

                    //grassrock2
                    root.vertices[vertex_index + 19] = 0.31 * x as f32 + ((z as f32 / 1.65).sin() * 0.15);
                    root.vertices[vertex_index + 20] = 0.27 * z as f32 + ((x as f32 / 1.71).sin() * 0.15);

                    //rock2
                    root.vertices[vertex_index + 21] = 0.32 * x as f32 + ((z as f32 / 1.35).sin() * 0.15);
                    root.vertices[vertex_index + 22] = 0.29 * z as f32 + ((x as f32 / 1.41).sin() * 0.15);

                    //snow2
                    root.vertices[vertex_index + 23] = 0.42 * x as f32 + ((z as f32 / 1.55).sin() * 0.15);
                    root.vertices[vertex_index + 24] = 0.33 * z as f32 + ((x as f32 / 1.61).sin() * 0.15);

                    // let mut grass = 1.0 - clamp((height - 0.0).abs() / 100.0);

                    let mut grassrock =
                        (1.0 - clamp((height - 0.0).abs() / (maxheight * 1.5))) * clamp(slope / (1.0 / COLOUR_SCALE));
                    let mut grass = 1.0 - clamp((height - 0.0).abs() / (maxheight * 1.5)) - grassrock;
                    // let mut grass = clamp((1.0 - clamp(height / (100.0 / COLOUR_SCALE))) - grassrock);
                    let mut rock = clamp((clamp((height - maxheight) / (maxheight * 1.2))) * (slope * 2.0));
                    let mut snow = clamp((clamp((height - maxheight) / (maxheight * 1.2))) / (slope * 1.0));

                    // let mut rock = (clamp((height - (80.0 / COLOUR_SCALE)) / (20.0 / COLOUR_SCALE))); //
                    // let mut snow = clamp(clamp((height - (100.0 / COLOUR_SCALE)) / (35.0 / COLOUR_SCALE)));
                    let mut grassrock2 = grassrock / 2.0;
                    let mut grass2 = grass / 2.0;
                    let rock2 = 0.0; //rock / 2.0;
                    let snow2 = 0.0; //snow / 2.0;
                                     // let total = grass + grassrock + rock + snow + grass2 + grassrock2;
                    let total = grass + grass2 + grassrock + grassrock2 + rock + snow;
                    // grass /= total;
                    // grassrock /= total;
                    // rock /= total;
                    // snow /= total;
                    // grass2 /= total;
                    // grassrock2 /= total;
                    grass /= total;
                    grassrock /= total;
                    rock /= total;
                    snow /= total;
                    grass2 /= total;
                    grassrock2 /= total;
                    root.vertices[vertex_index + 25] = grass;
                    root.vertices[vertex_index + 26] = grassrock;
                    root.vertices[vertex_index + 27] = rock;
                    root.vertices[vertex_index + 28] = snow;
                    root.vertices[vertex_index + 29] = grass2;
                    root.vertices[vertex_index + 30] = grassrock2;
                    root.vertices[vertex_index + 31] = rock2;
                    root.vertices[vertex_index + 32] = snow2;
                    root.vertices[vertex_index + 33] = 1.0;
                    vertex_index += 34;
                    if x < (root.centre.x + (root.width / 2.0)) as i32
                        && z < (root.centre.z + (root.width / 2.0)) as i32
                    {
                        if tree_heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] > -600.0 {
                            let mut rng = rand::thread_rng();
                            let tree_x = x as f32 - (world_size as f32 / 2.0)
                                + tree_centres_x[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]];
                            let tree_z = z as f32 - (world_size as f32 / 2.0)
                                + tree_centres_z[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]];
                            let mut model = Matrix4::<f32>::from_translation(vec3(
                                tree_x,
                                tree_heights[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]],
                                tree_z,
                            ));
                            let scale =
                                tree_sizes[[x.min(world_size - 1) as usize, z.min(world_size - 1) as usize]] * 0.5; //(rng.gen::<u32>() % 10) as f32 / 100.0 + 0.125;
                            model = model * Matrix4::from_scale(scale * 1.2);
                            // // 3. rotation: add random rotation around a (semi)randomly picked rotation axis vector
                            let rot_angle = (rng.gen::<u32>() % 360) as f32;
                            model = model * Matrix4::from_angle_y(Deg(rot_angle));
                            root.model_matrices.push(model);
                            root.tree_count += 1;
                        }
                    }
                }
            }
            let index_width = root.width + 32.0;
            for z in (0..=(root.width - 1.0) as u16).step_by(1 as usize) {
                for x in (0..=(root.width - 1.0) as u16).step_by(1 as usize) {
                    root.indices1[index_count1 + 0] =
                        vertex_index_offset as i32 + (((z + 1) * (index_width) as u16) + x) as i32;
                    root.indices1[index_count1 + 1] =
                        vertex_index_offset as i32 + (((z + 1) * (index_width) as u16) + x + 1) as i32;
                    root.indices1[index_count1 + 2] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x) as i32;
                    root.indices1[index_count1 + 3] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x) as i32;
                    root.indices1[index_count1 + 4] =
                        vertex_index_offset as i32 + (((z + 1) * (index_width) as u16) + x + 1) as i32;
                    root.indices1[index_count1 + 5] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 1) as i32;
                    index_count1 += 6;
                }
            }

            for z in (0..=(root.width - 1.0) as u16).step_by(2 as usize) {
                for x in (0..=(root.width - 1.0) as u16).step_by(2 as usize) {
                    root.indices2[index_count2 + 0] =
                        vertex_index_offset as i32 + (((z + 2) * (index_width) as u16) + x) as i32;
                    root.indices2[index_count2 + 1] =
                        vertex_index_offset as i32 + (((z + 2) * (index_width) as u16) + x + 2) as i32;
                    root.indices2[index_count2 + 2] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 2) as i32;
                    root.indices2[index_count2 + 3] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x) as i32;
                    root.indices2[index_count2 + 4] =
                        vertex_index_offset as i32 + (((z + 2) * (index_width) as u16) + x) as i32;
                    root.indices2[index_count2 + 5] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 2) as i32;
                    index_count2 += 6;
                }
            }

            for z in (0..=(root.width - 1.0) as u16).step_by(4 as usize) {
                for x in (0..=(root.width - 1.0) as u16).step_by(4 as usize) {
                    root.indices3[index_count3 + 0] =
                        vertex_index_offset as i32 + (((z + 4) * (index_width) as u16) + x) as i32;
                    root.indices3[index_count3 + 1] =
                        vertex_index_offset as i32 + (((z + 4) * (index_width) as u16) + x + 4) as i32;
                    root.indices3[index_count3 + 2] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 4) as i32;
                    root.indices3[index_count3 + 3] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x) as i32;
                    root.indices3[index_count3 + 4] =
                        vertex_index_offset as i32 + (((z + 4) * (index_width) as u16) + x) as i32;
                    root.indices3[index_count3 + 5] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 4) as i32;
                    index_count3 += 6;
                }
            }

            for z in (0..=(root.width - 1.0) as u16).step_by(8 as usize) {
                for x in (0..=(root.width - 1.0) as u16).step_by(8 as usize) {
                    root.indices4[index_count4 + 0] =
                        vertex_index_offset as i32 + (((z + 8) * (index_width) as u16) + x) as i32;
                    root.indices4[index_count4 + 1] =
                        vertex_index_offset as i32 + (((z + 8) * (index_width) as u16) + x + 8) as i32;
                    root.indices4[index_count4 + 2] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 8) as i32;
                    root.indices4[index_count4 + 3] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x) as i32;
                    root.indices4[index_count4 + 4] =
                        vertex_index_offset as i32 + (((z + 8) * (index_width) as u16) + x) as i32;
                    root.indices4[index_count4 + 5] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 8) as i32;
                    index_count4 += 6;
                }
            }

            for z in (0..=(root.width - 1.0) as u16).step_by(16 as usize) {
                for x in (0..=(root.width - 1.0) as u16).step_by(16 as usize) {
                    root.indices5[index_count5 + 0] =
                        vertex_index_offset as i32 + (((z + 16) * (index_width) as u16) + x) as i32;
                    root.indices5[index_count5 + 1] =
                        vertex_index_offset as i32 + (((z + 16) * (index_width) as u16) + x + 16) as i32;
                    root.indices5[index_count5 + 2] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 16) as i32;
                    root.indices5[index_count5 + 3] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x) as i32;
                    root.indices5[index_count5 + 4] =
                        vertex_index_offset as i32 + (((z + 16) * (index_width) as u16) + x) as i32;
                    root.indices5[index_count5 + 5] =
                        vertex_index_offset as i32 + ((z * (index_width) as u16) + x + 16) as i32;
                    index_count5 += 6;
                }
            }
            root.index_draw1 = index_count1 as i32;
            root.index_draw2 = index_count2 as i32;
            root.index_draw3 = index_count3 as i32;
            root.index_draw4 = index_count4 as i32;
            root.index_draw5 = index_count5 as i32;
            unsafe {
                gl::GenBuffers(1, &mut root.vbo);
                gl::BindBuffer(gl::ARRAY_BUFFER, root.vbo);
                gl::BufferData(
                    gl::ARRAY_BUFFER,
                    (root.vertices_size as usize * mem::size_of::<GLfloat>()) as GLsizeiptr,
                    &root.vertices[0] as *const f32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.ebo1);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo1);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    (root.indices_size as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.indices1[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.ebo2);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo2);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.indices_size / 4) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.indices2[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.ebo3);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo3);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.indices_size / 16) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.indices3[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.ebo4);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo4);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.indices_size / 64) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.indices4[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.ebo5);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo5);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.indices_size / 256) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.indices5[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.tree_buffer);

                gl::GenBuffers(1, &mut root.edge_vbo);
                gl::BindBuffer(gl::ARRAY_BUFFER, root.edge_vbo);
                gl::BufferData(
                    gl::ARRAY_BUFFER,
                    (root.edge_vertices_size as usize * mem::size_of::<GLfloat>()) as GLsizeiptr,
                    &root.edge_vertices[0] as *const f32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.edge_ebo1);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo1);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.edge_indices_size / 1) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.edge_indices1[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.edge_ebo2);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo2);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.edge_indices_size / 2) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.edge_indices2[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.edge_ebo3);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo3);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.edge_indices_size / 4) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.edge_indices3[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.edge_ebo4);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo4);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.edge_indices_size / 8) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.edge_indices4[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
                gl::GenBuffers(1, &mut root.edge_ebo5);
                gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo5);
                gl::BufferData(
                    gl::ELEMENT_ARRAY_BUFFER,
                    ((root.edge_indices_size / 16) as usize * mem::size_of::<GLuint>()) as GLsizeiptr,
                    &root.edge_indices5[0] as *const i32 as *const c_void,
                    gl::DYNAMIC_DRAW,
                );
            }
        }
    }
    pub fn update(
        &mut self,
        root: &Quadtree,
        vao: gl::types::GLuint,
        u1: Vector2<f32>,
        u2: Vector2<f32>,
        heights: &mut Array2<f32>,
        world_size: i32,
    ) {
        if root.leaf {
            unsafe {
                let nodexmin = root.centre.x - root.width / 2.0;
                let nodexmax = root.centre.x + root.width / 2.0;
                let nodezmin = root.centre.z - root.width / 2.0;
                let nodezmax = root.centre.z + root.width / 2.0;
                let t1 = vec2(root.centre.x - root.width / 2.0, root.centre.z - root.width / 2.0);
                let t2 = vec2(root.centre.x + root.width / 2.0, root.centre.z + root.width / 2.0);
                let outside = false;

                // If one rectangle is on left side of other
                if u1.x >= t2.x || u2.x >= t1.x {
                    outside == true;
                }

                // If one rectangle is above other
                if u1.y <= t2.y || u2.y <= t1.y {
                    outside == true;
                }
                if !outside {
                    let nodewidth = root.width + 1.0;
                    gl::BindBuffer(gl::ARRAY_BUFFER, root.vbo);
                    let vbo_data = gl::MapBuffer(gl::ARRAY_BUFFER, gl::WRITE_ONLY).offset(0 as isize) as *mut f32;
                    for z in u1.y as usize..u2.y as usize {
                        for x in u1.x as usize..u2.x as usize {
                            if x as f32 <= nodexmax
                                && x as f32 >= nodexmin
                                && z as f32 <= nodezmax
                                && z as f32 >= nodezmin
                            {
                                let xz_height = heights[[
                                    x.min(world_size as usize - 1) as usize,
                                    z.min(world_size as usize - 1) as usize,
                                ]] as f32;
                                let zcount = z - nodezmin as usize;
                                let xcount = x - nodexmin as usize;
                                let zoffset = nodewidth * zcount as f32;
                                let xoffset = xcount as f32;
                                let vertex_index = ((zoffset + xoffset) * 34.0) as usize;
                                let mut normal: Vector3<f32> = vec3(0.0, 0.0, 0.0);
                                if x > 0 && z < world_size as usize - 1 {
                                    normal += calc_surface_normal(
                                        vec3(x as f32, xz_height, z as f32),
                                        vec3(
                                            x as f32 - 1.0,
                                            heights[[(x - 1).min(world_size as usize - 1) as usize, z as usize]],
                                            z as f32,
                                        ),
                                        vec3(
                                            x as f32 - 1.0,
                                            heights[[
                                                (x - 1).min(world_size as usize - 1) as usize,
                                                (z + 1).min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32 + 1.0,
                                        ),
                                    );
                                    normal += calc_surface_normal(
                                        vec3(
                                            x as f32,
                                            heights[[
                                                x.min(world_size as usize - 1) as usize,
                                                z.min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32,
                                        ),
                                        vec3(
                                            x as f32 - 1.0,
                                            heights[[
                                                (x - 1).min(world_size as usize - 1) as usize,
                                                (z + 1).min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32 + 1.0,
                                        ),
                                        vec3(
                                            x as f32,
                                            heights[[
                                                x.min(world_size as usize - 1) as usize,
                                                (z + 1).min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32 + 1.0,
                                        ),
                                    );
                                }
                                if x < world_size as usize - 1 && z < world_size as usize - 1 {
                                    normal += calc_surface_normal(
                                        vec3(x as f32, heights[[x as usize, z as usize]], z as f32),
                                        vec3(
                                            x as f32,
                                            heights[[x as usize, (z + 1).min(world_size as usize - 1) as usize]],
                                            z as f32 + 1.0,
                                        ),
                                        vec3(
                                            x as f32 + 1.0,
                                            heights[[(x + 1).min(world_size as usize - 1) as usize, z as usize]],
                                            z as f32,
                                        ),
                                    );
                                }
                                if x < world_size as usize - 1 && z > 0 {
                                    normal += calc_surface_normal(
                                        vec3(
                                            x as f32,
                                            heights[[x as usize, z.min(world_size as usize - 1) as usize]],
                                            z as f32,
                                        ),
                                        vec3(
                                            x as f32 + 1.0,
                                            heights[[
                                                (x + 1).min(world_size as usize - 1) as usize,
                                                z.min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32,
                                        ),
                                        vec3(
                                            x as f32 + 1.0,
                                            heights[[
                                                (x + 1).min(world_size as usize - 1) as usize,
                                                (z - 1).min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32 - 1.0,
                                        ),
                                    );
                                    normal += calc_surface_normal(
                                        vec3(
                                            x as f32,
                                            heights[[x as usize, z.min(world_size as usize - 1) as usize]],
                                            z as f32,
                                        ),
                                        vec3(
                                            x as f32 + 1.0,
                                            heights[[
                                                (x + 1).min(world_size as usize - 1) as usize,
                                                (z - 1).min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32 - 1.0,
                                        ),
                                        vec3(
                                            x as f32,
                                            heights[[x as usize, (z - 1).min(world_size as usize - 1) as usize]],
                                            z as f32 - 1.0,
                                        ),
                                    );
                                }
                                if x > 0 && z > 0 {
                                    normal += calc_surface_normal(
                                        vec3(
                                            x as f32,
                                            heights[[
                                                x.min(world_size as usize - 1) as usize,
                                                z.min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32,
                                        ),
                                        vec3(
                                            x as f32,
                                            heights[[
                                                x.min(world_size as usize - 1) as usize,
                                                (z - 1).min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32 - 1.0,
                                        ),
                                        vec3(
                                            x as f32 - 1.0,
                                            heights[[
                                                (x - 1).min(world_size as usize - 1) as usize,
                                                z.min(world_size as usize - 1) as usize,
                                            ]],
                                            z as f32,
                                        ),
                                    );
                                }
                                normal = normalise_normal(normal);
                                let slope = get_average_slope(heights, x as i32, z as i32, world_size - 1);
                                let height = xz_height;
                                let mut grassrock = clamp(slope / (4.0 / COLOUR_SCALE))
                                    * (1.0 - clamp(height / (120.0 / COLOUR_SCALE)))
                                    * 2.0;
                                let mut grass = clamp((1.0 - clamp(height / (170.0 / COLOUR_SCALE))) - grassrock);
                                let mut rock = 0.0;
                                let mut snow = 0.0;
                                let mut grassrock2 = grassrock * 1.0;
                                let mut grass2 = grass * 1.0;
                                let mut rock2 = 0.0;
                                let mut snow2 = 0.0;
                                let total = grass + grass2 + grassrock + grassrock2 + rock + rock2 + snow + snow2;
                                grass /= total;
                                grass2 /= total;
                                grassrock /= total;
                                grassrock2 /= total;
                                rock /= total;
                                rock2 /= total;
                                snow /= total;
                                snow2 /= total;
                                set(vbo_data, vertex_index + 1, xz_height);
                                set(vbo_data, vertex_index + 7, normal.x);
                                set(vbo_data, vertex_index + 7, normal.y);
                                set(vbo_data, vertex_index + 8, normal.z);
                                set(vbo_data, vertex_index + 25, grass);
                                set(vbo_data, vertex_index + 26, grassrock);
                                set(vbo_data, vertex_index + 27, rock);
                                set(vbo_data, vertex_index + 28, snow);
                                set(vbo_data, vertex_index + 29, grass2);
                                set(vbo_data, vertex_index + 30, grassrock2);
                                set(vbo_data, vertex_index + 31, rock2);
                                set(vbo_data, vertex_index + 32, snow2);
                            }
                        }
                    }
                    gl::UnmapBuffer(gl::ARRAY_BUFFER);
                }
            }
        } else {
            match &root.one {
                Some(_q) => {
                    self.update(&**root.one.as_ref().unwrap(), vao, u1, u2, heights, world_size);
                }
                None => (),
            }
            match &root.two {
                Some(_q) => {
                    self.update(&**root.two.as_ref().unwrap(), vao, u1, u2, heights, world_size);
                }
                None => (),
            }
            match &root.three {
                Some(_q) => {
                    self.update(&**root.three.as_ref().unwrap(), vao, u1, u2, heights, world_size);
                }
                None => (),
            }
            match &root.four {
                Some(_q) => {
                    self.update(&**root.four.as_ref().unwrap(), vao, u1, u2, heights, world_size);
                }
                None => (),
            }
        }
    }

    pub fn draw_terrain_depth(
        &mut self,
        root: &Quadtree,
        world_size: usize,
        frustum_matrix: Matrix4<f32>,
        vao: gl::types::GLuint,
        view_position: Vector3<f32>,
        view_dist: f32,
        zoom: f32,
    ) {
        if root.leaf {
            let box_min: Vector3<f32> = vec3(
                root.centre.x - root.width / 2.0 - (world_size / 2) as f32,
                root.min_height,
                root.centre.z - root.width / 2.0 - (world_size / 2) as f32,
            );
            let box_max: Vector3<f32> = vec3(
                root.centre.x + root.width / 2.0 - (world_size / 2) as f32,
                root.max_height,
                root.centre.z + root.width / 2.0 - (world_size / 2) as f32,
            );
            let culling = FrustumCuller::from_matrix(frustum_matrix);
            let bounding_box = BoundingBox::from_params(box_min, box_max);

            match culling.test_bounding_box(bounding_box) {
                Intersection::Inside | Intersection::Partial => {
                    unsafe {
                        let mut distance_to_node = ((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt();

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);
                        gl::BindVertexArray(vao);
                        gl::BindBuffer(gl::ARRAY_BUFFER, root.vbo);
                        let mut draw_count = 0;
                        let mut edge_draw_count = 0;

                        // let zoom_scale = ((((46.0 - zoom) / 44.0) * 0.25) + 1.0).powf(5.0);
                        let zoom_scale = ZOOM_LOD.powf(51.0 - zoom);
                        // println!("{}", zoom_scale);
                        if distance_to_node > ((LOD5_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo5);
                            draw_count += root.index_draw5;
                        } else if distance_to_node > ((LOD4_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo4);
                            draw_count += root.index_draw4;
                        } else if distance_to_node > ((LOD3_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo3);
                            draw_count += root.index_draw3;
                        } else if distance_to_node > ((LOD2_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo2);
                            draw_count += root.index_draw2;
                        } else {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo1);
                            draw_count += root.index_draw1;
                        }
                        // position attribute
                        gl::VertexAttribPointer(
                            0,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            ptr::null(),
                        );
                        gl::EnableVertexAttribArray(0);

                        gl::Enable(gl::CULL_FACE);

                        let draw_indices = draw_count;
                        gl::DrawElements(gl::TRIANGLES, draw_indices, gl::UNSIGNED_INT, ptr::null());

                        gl::BindBuffer(gl::ARRAY_BUFFER, root.edge_vbo);
                        if distance_to_node > ((LOD5_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo5);
                            edge_draw_count += root.index_edge_draw5;
                        } else if distance_to_node > ((LOD4_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo4);
                            edge_draw_count += root.index_edge_draw4;
                        } else if distance_to_node > ((LOD3_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo3);
                            edge_draw_count += root.index_edge_draw3;
                        } else if distance_to_node > ((LOD2_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo2);
                            edge_draw_count += root.index_edge_draw2;
                        } else {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo1);
                            edge_draw_count += root.index_draw1;
                        }
                        // position attribute
                        gl::VertexAttribPointer(
                            0,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            ptr::null(),
                        );
                        gl::EnableVertexAttribArray(0);

                        let draw_indices = edge_draw_count;
                        gl::Disable(gl::CULL_FACE);
                        NODE_COUNTER = NODE_COUNTER + 1;
                        TRIANGLE_COUNTER = TRIANGLE_COUNTER + (draw_indices as f32 / 3.0) as u32;
                        // println!("{}", NODE_COUNTER);
                        gl::DrawElements(gl::TRIANGLES, draw_indices, gl::UNSIGNED_INT, ptr::null());
                    }
                }
                Intersection::Outside => {}
            }
        } else {
            match &root.one {
                Some(_q) => {
                    self.draw_terrain_depth(
                        &**root.one.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
            match &root.two {
                Some(_q) => {
                    self.draw_terrain_depth(
                        &**root.two.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
            match &root.three {
                Some(_q) => {
                    self.draw_terrain_depth(
                        &**root.three.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
            match &root.four {
                Some(_q) => {
                    self.draw_terrain_depth(
                        &**root.four.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
        }
    }

    pub fn draw_terrain(
        &mut self,
        root: &Quadtree,
        world_size: usize,
        frustum_matrix: Matrix4<f32>,
        vao: gl::types::GLuint,
        view_position: Vector3<f32>,
        view_dist: f32,
        zoom: f32,
        basil: i32,
    ) {
        if root.leaf {
            let box_min: Vector3<f32> = vec3(
                root.centre.x - root.width / 2.0 - (world_size / 2) as f32,
                root.min_height,
                root.centre.z - root.width / 2.0 - (world_size / 2) as f32,
            );
            let box_max: Vector3<f32> = vec3(
                root.centre.x + root.width / 2.0 - (world_size / 2) as f32,
                root.max_height,
                root.centre.z + root.width / 2.0 - (world_size / 2) as f32,
            );
            let culling = FrustumCuller::from_matrix(frustum_matrix);
            let bounding_box = BoundingBox::from_params(box_min, box_max);

            match culling.test_bounding_box(bounding_box) {
                Intersection::Inside | Intersection::Partial => {
                    unsafe {
                        let mut distance_to_node = ((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt();

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);
                        // println!("view position {} {}", view_position.x, view_position.z);
                        // println!("node centre   {} {}", root.centre.x, root.centre.z);
                        // println!("d             {}", distance_to_node);
                        gl::BindVertexArray(vao);
                        gl::BindBuffer(gl::ARRAY_BUFFER, root.vbo);
                        let mut draw_count = 0;
                        let mut edge_draw_count = 0;

                        // let zoom_scale = ((((46.0 - zoom) / 44.0) * 0.25) + 1.0).powf(5.0);
                        let zoom_scale = ZOOM_LOD.powf(51.0 - zoom);
                        // println!("{}", zoom_scale);
                        if distance_to_node > ((LOD5_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo5);
                            draw_count += root.index_draw5;
                        } else if distance_to_node > ((LOD4_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo4);
                            draw_count += root.index_draw4;
                        } else if distance_to_node > ((LOD3_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo3);
                            draw_count += root.index_draw3;
                        } else if distance_to_node > ((LOD2_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo2);
                            draw_count += root.index_draw2;
                        } else {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.ebo1);
                            draw_count += root.index_draw1;
                        }
                        // position attribute
                        gl::VertexAttribPointer(
                            0,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            ptr::null(),
                        );
                        gl::EnableVertexAttribArray(0);

                        // colour coord attribute
                        gl::VertexAttribPointer(
                            1,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (3 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(1);

                        // normal attribute
                        gl::VertexAttribPointer(
                            2,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (6 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(2);

                        // texture coord 1 attribute
                        gl::VertexAttribPointer(
                            3,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (9 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(3);

                        // texture coord 2 attribute
                        gl::VertexAttribPointer(
                            4,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (11 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(4);

                        // texture coord 3 attribute
                        gl::VertexAttribPointer(
                            5,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (13 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(5);

                        // texture coord 4 attribute
                        gl::VertexAttribPointer(
                            6,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (15 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(6);

                        // texture coord 5 attribute
                        gl::VertexAttribPointer(
                            7,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (17 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(7);

                        // texture coord 6 attribute
                        gl::VertexAttribPointer(
                            8,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (19 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(8);

                        // texture coord 7 attribute
                        gl::VertexAttribPointer(
                            9,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (21 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(9);

                        // texture coord 8 attribute
                        gl::VertexAttribPointer(
                            10,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (23 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(10);

                        // texture mix 123
                        gl::VertexAttribPointer(
                            11,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (25 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(11);

                        // texture mix 456
                        gl::VertexAttribPointer(
                            12,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (28 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(12);

                        // texture mix 78
                        gl::VertexAttribPointer(
                            13,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (31 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(13);
                        gl::Enable(gl::CULL_FACE);

                        let draw_indices = draw_count;
                        gl::DrawElements(gl::TRIANGLES, draw_indices, gl::UNSIGNED_INT, ptr::null());

                        gl::BindBuffer(gl::ARRAY_BUFFER, root.edge_vbo);
                        if distance_to_node > ((LOD5_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo5);
                            edge_draw_count += root.index_edge_draw5;
                        } else if distance_to_node > ((LOD4_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo4);
                            edge_draw_count += root.index_edge_draw4;
                        } else if distance_to_node > ((LOD3_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo3);
                            edge_draw_count += root.index_edge_draw3;
                        } else if distance_to_node > ((LOD2_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo2);
                            edge_draw_count += root.index_edge_draw2;
                        } else {
                            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, root.edge_ebo1);
                            edge_draw_count += root.index_draw1;
                        }
                        // position attribute
                        gl::VertexAttribPointer(
                            0,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            ptr::null(),
                        );
                        gl::EnableVertexAttribArray(0);

                        // colour coord attribute
                        gl::VertexAttribPointer(
                            1,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (3 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(1);

                        // normal attribute
                        gl::VertexAttribPointer(
                            2,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (6 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(2);

                        // texture coord 1 attribute
                        gl::VertexAttribPointer(
                            3,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (9 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(3);

                        // texture coord 2 attribute
                        gl::VertexAttribPointer(
                            4,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (11 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(4);

                        // texture coord 3 attribute
                        gl::VertexAttribPointer(
                            5,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (13 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(5);

                        // texture coord 4 attribute
                        gl::VertexAttribPointer(
                            6,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (15 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(6);

                        // texture coord 5 attribute
                        gl::VertexAttribPointer(
                            7,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (17 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(7);

                        // texture coord 6 attribute
                        gl::VertexAttribPointer(
                            8,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (19 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(8);

                        // texture coord 7 attribute
                        gl::VertexAttribPointer(
                            9,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (21 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(9);

                        // texture coord 8 attribute
                        gl::VertexAttribPointer(
                            10,
                            2,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (23 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(10);

                        // texture mix 123
                        gl::VertexAttribPointer(
                            11,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (25 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(11);

                        // texture mix 456
                        gl::VertexAttribPointer(
                            12,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (28 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        gl::EnableVertexAttribArray(12);

                        // texture mix 78
                        gl::VertexAttribPointer(
                            13,
                            3,
                            gl::FLOAT,
                            gl::FALSE,
                            34 * 1 as i32 * mem::size_of::<GLfloat>() as GLsizei,
                            (31 * mem::size_of::<GLfloat>()) as *const c_void,
                        );
                        let draw_indices = edge_draw_count;
                        gl::Disable(gl::CULL_FACE);
                        NODE_COUNTER = NODE_COUNTER + 1;
                        TRIANGLE_COUNTER = TRIANGLE_COUNTER + (draw_count as f32 / 3.0) as u32;
                        // println!("{}", TRIANGLE_COUNTER);
                        gl::DrawElements(gl::TRIANGLES, draw_indices, gl::UNSIGNED_INT, ptr::null());
                    }
                }
                Intersection::Outside => {}
            }
        } else {
            match &root.one {
                Some(_q) => {
                    self.draw_terrain(
                        &**root.one.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                        basil,
                    );
                }
                None => (),
            }
            match &root.two {
                Some(_q) => {
                    self.draw_terrain(
                        &**root.two.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                        basil,
                    );
                }
                None => (),
            }
            match &root.three {
                Some(_q) => {
                    self.draw_terrain(
                        &**root.three.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                        basil,
                    );
                }
                None => (),
            }
            match &root.four {
                Some(_q) => {
                    self.draw_terrain(
                        &**root.four.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        vao,
                        view_position,
                        view_dist,
                        zoom,
                        basil,
                    );
                }
                None => (),
            }
        }
    }

    pub fn draw_trees(
        &mut self,
        root: &Quadtree,
        world_size: usize,
        frustum_matrix: Matrix4<f32>,
        model_1: &Model,
        model_2: &Model,
        model_3: &Model,
        model_4: &Model,
        shader: &Shader,
        view_position: Vector3<f32>,
        view_dist: f32,
        zoom: f32,
    ) {
        if root.leaf {
            let box_min: Vector3<f32> = vec3(
                root.centre.x - root.width / 2.0 - (world_size / 2) as f32 + 1.0,
                root.min_height,
                root.centre.z - root.width / 2.0 - (world_size / 2) as f32 + 1.0,
            );
            let box_max: Vector3<f32> = vec3(
                root.centre.x + root.width / 2.0 - (world_size / 2) as f32 - 1.0,
                root.max_height,
                root.centre.z + root.width / 2.0 - (world_size / 2) as f32 - 1.0,
            );
            let culling = FrustumCuller::from_matrix(frustum_matrix);
            let bounding_box = BoundingBox::from_params(box_min, box_max);
            if root.tree_count > 0 {
                // println!("Tree count: {}", root.tree_count);
            }
            match culling.test_bounding_box(bounding_box) {
                Intersection::Inside | Intersection::Partial => unsafe {
                    // NODE_COUNTER = 5;
                    if root.tree_count > 0 {
                        let mut distance_to_node = ((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt();

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z - (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x - (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        distance_to_node = (((view_position.x - root.centre.x + (root.width / 2.0)).powf(2.0)
                            + (view_position.z - root.centre.z + (root.width / 2.0)).powf(2.0))
                        .sqrt())
                        .min(distance_to_node);

                        let size_mat4 = mem::size_of::<Matrix4<f32>>() as i32;
                        let size_vec4 = mem::size_of::<Vector4<f32>>() as i32;
                        gl::BindBuffer(gl::ARRAY_BUFFER, root.tree_buffer);
                        gl::BufferData(
                            gl::ARRAY_BUFFER,
                            (root.tree_count as usize * mem::size_of::<Matrix4<f32>>()) as isize,
                            &root.model_matrices[0] as *const Matrix4<f32> as *const c_void,
                            gl::STATIC_DRAW,
                        ); //
                        let zoom_scale = TREE_ZOOM_LOD.powf(41.0 - zoom);
                        let mut node_model = model_4;
                        // println!(
                        //     "dist {} limit {}",
                        //     distance_to_node,
                        //     ((LOD2_DISTANCE * zoom_scale) / 100.0) * view_dist
                        // );
                        if distance_to_node < ((LOD2_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            node_model = model_1;
                        // println!("1");
                        } else if distance_to_node < ((LOD3_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            node_model = model_2;
                        // println!(" 2");
                        } else if distance_to_node < ((LOD4_DISTANCE * zoom_scale) / 100.0) * view_dist {
                            node_model = model_3;
                        // println!("  3");
                        } else {
                            // println!("   4");
                        }
                        for mesh in &node_model.meshes {
                            let vao = mesh.VAO;
                            gl::BindVertexArray(vao);
                            gl::EnableVertexAttribArray(3);
                            gl::VertexAttribPointer(3, 4, gl::FLOAT, gl::FALSE, size_mat4, ptr::null());
                            gl::EnableVertexAttribArray(4);
                            gl::VertexAttribPointer(4, 4, gl::FLOAT, gl::FALSE, size_mat4, size_vec4 as *const c_void);
                            gl::EnableVertexAttribArray(5);
                            gl::VertexAttribPointer(
                                5,
                                4,
                                gl::FLOAT,
                                gl::FALSE,
                                size_mat4,
                                (2 * size_vec4) as *const c_void,
                            );
                            gl::EnableVertexAttribArray(6);
                            gl::VertexAttribPointer(
                                6,
                                4,
                                gl::FLOAT,
                                gl::FALSE,
                                size_mat4,
                                (3 * size_vec4) as *const c_void,
                            );
                            gl::VertexAttribDivisor(3, 1);
                            gl::VertexAttribDivisor(4, 1);
                            gl::VertexAttribDivisor(5, 1);
                            gl::VertexAttribDivisor(6, 1);
                            let mut diffuse_nr = 0;
                            for (i, texture) in mesh.textures.iter().enumerate() {
                                gl::ActiveTexture(gl::TEXTURE0 + i as u32); // active proper texture unit before binding
                                                                            // retrieve texture number (the N in diffuse_textureN)
                                let name = &texture.type_;
                                let number = match name.as_str() {
                                    "texture_diffuse" => {
                                        diffuse_nr += 1;
                                        diffuse_nr
                                    }
                                    _ => panic!("unknown texture type"),
                                };
                                // now set the sampler to the correct texture unit
                                let sampler = CString::new(format!("{}{}", name, number)).unwrap();
                                gl::Uniform1i(gl::GetUniformLocation(shader.ID, sampler.as_ptr()), i as i32);
                                // and finally bind the texture
                                gl::BindTexture(gl::TEXTURE_2D, texture.id);
                            }
                            // TRIANGLE_COUNTER += mesh.indices.len() as i32;
                            gl::DrawElementsInstanced(
                                gl::TRIANGLES,
                                mesh.indices.len() as i32,
                                gl::UNSIGNED_INT,
                                ptr::null(),
                                root.tree_count as i32,
                            );
                            gl::BindVertexArray(0);
                        }
                        // println!("vbo {} - Tree count: {}", root.vbo, root.tree_count);
                    }
                },
                Intersection::Outside => {}
            }
        } else {
            match &root.one {
                Some(_q) => {
                    self.draw_trees(
                        &**root.one.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        &model_1,
                        &model_2,
                        &model_3,
                        &model_4,
                        &shader,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
            match &root.two {
                Some(_q) => {
                    self.draw_trees(
                        &**root.two.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        &model_1,
                        &model_2,
                        &model_3,
                        &model_4,
                        &shader,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
            match &root.three {
                Some(_q) => {
                    self.draw_trees(
                        &**root.three.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        &model_1,
                        &model_2,
                        &model_3,
                        &model_4,
                        &shader,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
            match &root.four {
                Some(_q) => {
                    self.draw_trees(
                        &**root.four.as_ref().unwrap(),
                        world_size,
                        frustum_matrix,
                        &model_1,
                        &model_2,
                        &model_3,
                        &model_4,
                        &shader,
                        view_position,
                        view_dist,
                        zoom,
                    );
                }
                None => (),
            }
        }
    }
}

fn calc_surface_normal(tri1: Vector3<f32>, tri2: Vector3<f32>, tri3: Vector3<f32>) -> Vector3<f32> {
    let u = tri2 - tri1;
    let v = tri3 - tri1;
    u.cross(v).normalize()
}
fn normalise_normal(normal: Vector3<f32>) -> Vector3<f32> {
    normal.normalize()
}
fn get_average_slope(heights: &mut Array2<f32>, x: i32, z: i32, max: i32) -> f32 {
    if x < max && z < max {
        let n = heights[[x as usize, z as usize]];
        let mut slope = 0.0;
        let min = 0;
        let mut count = 0;
        if x > min {
            let d = heights[[x as usize - 1, z as usize]];
            slope = (n - d).abs();
            count += 1;
        }
        if x < max {
            let e = heights[[x as usize + 1, z as usize]];
            slope += (n - e).abs();
            count += 1;
        }
        if z < max {
            let b = heights[[x as usize, z as usize + 1]];
            slope += (n - b).abs();
            count += 1;
            if x > min {
                let a = heights[[x as usize - 1, z as usize + 1]];
                slope += (n - a).abs();
                count += 1;
            }
            if x < max {
                let c = heights[[x as usize + 1, z as usize + 1]];
                slope += (n - c).abs();
                count += 1;
            }
        }
        if z > min {
            let g = heights[[x as usize, z as usize - 1]];
            slope += (n - g).abs();
            count += 1;
            if x > min {
                let f = heights[[x as usize - 1, z as usize - 1]];
                slope += (n - f).abs();
                count += 1;
            }
            if x < max {
                let h = heights[[x as usize + 1, z as usize - 1]];
                slope += (n - h).abs();
                count += 1;
            }
        }
        return slope / count as f32;
    }
    0.0
}
fn clamp(value: f32) -> f32 {
    value.max(0.0).min(1.0)
}
unsafe fn set(data: *mut f32, offset: usize, value: f32) {
    let ptr = data.offset(offset as isize) as *mut f32;
    *ptr = value;
}

fn get_height(heights: &Array2<f32>, mut pos_x: f32, mut pos_z: f32, world_size: i32) -> f32 {
    pos_x = pos_x.min((world_size - 2) as f32);
    pos_z = pos_z.min((world_size - 2) as f32);
    pos_x = pos_x.max(1.0);
    pos_z = pos_z.max(1.0);
    let tx = pos_x.trunc() as usize;
    let tz = pos_z.trunc() as usize;
    let h1 = heights[[tx, tz]];
    let h2 = heights[[tx + 1, tz]];
    let h3 = heights[[tx, tz + 1]];
    let h4 = heights[[tx + 1, tz + 1]];
    let dx = pos_x - tx as f32;
    let dz = pos_z - tz as f32;
    if dx < dz {
        let slopex = h4 - h3;
        let slopez = h1 - h3;
        return h3 + (slopex * (dx)) + (slopez * (1.0 - dz));
    } else {
        let slopex = h1 - h2;
        let slopez = h4 - h2;
        return h2 + (slopex * (1.0 - dx)) + (slopez * (dz));
    }
}
