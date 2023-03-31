#![allow(non_snake_case)]
extern crate cgmath;
extern crate fps_counter;
extern crate gl;
extern crate glfw;
extern crate math;
extern crate nalgebra as na;
extern crate ndarray;
extern crate noise;
extern crate nphysics3d;
extern crate rand;
use self::glfw::Context;
use self::glfw::{Action, Key};
use self::ndarray::Array2;
use crate::camera::Camera;
use crate::common::{processInput, process_events as proc_events};
use crate::model::Model;
use crate::profiler::Profiler;
use crate::quadtree::Quadtree;
use crate::shader::Shader;
use crate::vehicle::Vehicle;
use cgmath::{perspective, vec2, vec3, Deg, Matrix4, Point3 as cPoint3, Vector2, Vector3 as cVector3, Vector4};
use gl::types::*;
use image;
use image::GenericImage;
use ndarray::Array;
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::material::{BasicMaterial, MaterialHandle};
use nphysics3d::nalgebra::Point3 as p3;
use nphysics3d::nalgebra::{DMatrix, Vector3 as v3};
use nphysics3d::ncollide3d::shape::{HeightField, ShapeHandle};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet, Ground,
};
use nphysics3d::world::GeometricalWorld;
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use rand::Rng;
use std::f32;
use std::ffi::{CStr, CString};
use std::mem;
use std::os::raw::c_void;
use std::path::Path;
use std::ptr;
use std::time::Instant;

type Vc = cgmath::Vector3<f32>;

const DRAW_TREES: bool = true;
const DRAW_TERRAIN: bool = true;
const DRAW_DEPTH: bool = false;
const SCR_WIDTH: u32 = 1024;
const SCR_HEIGHT: u32 = 768;
const WORLDSIZE: i32 = 1792 / 1;
const VIEWDIST: f32 = 1200.0;
const F1: f32 = 2.0;
const F2: f32 = 0.375;
const D1: f32 = 56.0;
const D2: f32 = 3.0;
const SKYHEIGHT: f32 = 250.0;
const SKYRADIUS: f32 = VIEWDIST * 1.7;
const STAT_TIME: u128 = 1000;
const FOG_DENSITY: f32 = 0.0;
const QUADS_PER_NODE: i32 = 80000;
const NIGHT_RED: f32 = 0.7;
const NIGHT_GREEN: f32 = 0.2;
const NIGHT_BLUE: f32 = 0.0;
const TREE_LINE: f32 = 85.0;
const TREE_MAX_SLOPE: f32 = 0.25;
const HEIGHT_RANGE: f32 = 180.1;
const AVERAGE_SIZE: i32 = 10;
const STARTX: f32 = 0.0;
const STARTZ: f32 = 0.0;
const SCALE: f32 = 0.3333;
const SHADOW_BUFFER_SCALE: u32 = 3; // ^2 0=1, 1=2, 2=4, 3=8, 4=16
const GRAVITY: f32 = 9.81;
const CAMERA_SPACING: f32 = 896.0 / 8.0;
#[cfg(target_os = "windows")]
const ROOT_PATH: &str = "C:/users/user/code/game-rs/";
#[cfg(target_os = "linux")]
const ROOT_PATH: &str = "/home/dad/Code/game-rs/";
const SIZE_MAT4: i32 = mem::size_of::<Matrix4<f32>>() as i32;
const SIZE_VEC4: i32 = mem::size_of::<Vector4<f32>>() as i32;

pub static mut NODE_COUNTER: u32 = 0;
pub static mut TRIANGLE_COUNTER: u32 = 0;
static mut QUAD_VAO: u32 = 0;
static mut QUAD_VBO: u32 = 0;

pub fn start() {
    let mut lighton = false;
    let mut lighttime = false;
    let mut REAR_AXLE = false;
    let mut MIDDLE_AXLE = false;
    let mut FRONT_AXLE = false;
    let mut INSIDE = false;
    let mut BACK = false;
    let mut FRONT = false;
    let mut BUMPER = true;
    let mut MOUSE = false;
    let mut fly = 5.0;
    let FF = 8.56183;
    let UU = -3.0909584;
    let RR = 0.06899999;
    let mut time = 4.7;
    let mut light = cVector3::new(1.0, 1.0, 1.0);
    let mut night_mode = false;
    let mut angle: f32 = 0.0;
    let mut motor: f32 = 0.0;
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    glfw.window_hint(glfw::WindowHint::ContextVersion(3, 3));
    glfw.window_hint(glfw::WindowHint::OpenGlProfile(glfw::OpenGlProfileHint::Core));
    #[cfg(target_os = "macos")]
    glfw.window_hint(glfw::WindowHint::OpenGlForwardCompat(true));
    let (mut window, events) = glfw
        .create_window(SCR_WIDTH, SCR_HEIGHT, "", glfw::WindowMode::Windowed)
        .expect("Failed to create GLFW window");
    window.set_pos(0, 0);
    window.make_current();
    window.set_framebuffer_size_polling(true);
    window.set_cursor_pos_polling(true);
    window.set_scroll_polling(true);
    window.set_cursor_mode(glfw::CursorMode::Disabled);
    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);
    let mut vao: gl::types::GLuint = 0;
    unsafe {
        gl::GenVertexArrays(1, &mut vao);
        gl::ClearColor(0.08, 0.0, 0.08, 1.0);
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        gl::Enable(gl::DEPTH_TEST);
        gl::CullFace(gl::BACK);
        window.swap_buffers();
    }
    let mut tree_centres_x = Array::<f32, _>::zeros((WORLDSIZE as usize, WORLDSIZE as usize));
    let mut tree_centres_z = Array::<f32, _>::zeros((WORLDSIZE as usize, WORLDSIZE as usize));
    let mut tree_sizes = Array::<f32, _>::zeros((WORLDSIZE as usize, WORLDSIZE as usize));
    let mut tree_heights = Array::<f32, _>::zeros((WORLDSIZE as usize, WORLDSIZE as usize));
    let mut tree_shadows = Array::<f32, _>::zeros((WORLDSIZE as usize, WORLDSIZE as usize));
    let mut heights = Array::<f32, _>::zeros((WORLDSIZE as usize, WORLDSIZE as usize));
    for x in 0..WORLDSIZE {
        for z in 0..WORLDSIZE {
            tree_heights[[x as usize, z as usize]] = -600.0;
        }
    }
    {
        let img = image::open(&Path::new("maps/dm2.jpg")).expect("Failed to load image");
        for x in 0..WORLDSIZE - 2 {
            for z in 0..WORLDSIZE - 2 {
                let byte: u8 = img.get_pixel(x as u32, z as u32)[0];
                heights[[x as usize, z as usize]] = byte as f32;
            }
        }
    }
    normalise_heightmap(&mut heights, HEIGHT_RANGE);
    if AVERAGE_SIZE > 0 {
        // average_height_map(&mut heights, AVERAGE_SIZE, false, 0.3);
    }
    average_height_map(&mut heights, 2, false, 0.0);
    normalise_heightmap(&mut heights, HEIGHT_RANGE);

    for _hat in 0..100 {
        // add_bump(&mut heights, 130.0, false);
    }
    // normalise_heightmap(&mut heights, 200.0);

    let mut mechanical_world = DefaultMechanicalWorld::new(nphysics3d::nalgebra::Vector3::new(0.0, -GRAVITY, 0.0));
    let mut geometrical_world: GeometricalWorld<f32, DefaultBodyHandle, DefaultColliderHandle> =
        DefaultGeometricalWorld::new();
    let mut bodies: DefaultBodySet<f32> = DefaultBodySet::new();
    let mut colliders: DefaultColliderSet<f32> = DefaultColliderSet::new();
    let mut joint_constraints: DefaultJointConstraintSet<f32> = DefaultJointConstraintSet::new();
    let mut force_generators: DefaultForceGeneratorSet<f32, generational_arena::Index> =
        DefaultForceGeneratorSet::new();

    let _ground_size = nphysics3d::nalgebra::Vector3::new(WORLDSIZE as f32, 1.0, WORLDSIZE as f32);
    let mut phys_heights = DMatrix::<f32>::zeros(WORLDSIZE as usize, WORLDSIZE as usize);
    for x in 0..WORLDSIZE {
        for z in 0..WORLDSIZE {
            phys_heights[(x as usize, z as usize)] = heights[[z as usize, x as usize]];
        }
    }
    let heightfield: HeightField<f32> =
        HeightField::new(phys_heights, v3::new(WORLDSIZE as f32, 1.0, WORLDSIZE as f32));
    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ShapeHandle::new(heightfield))
        .material(MaterialHandle::new(BasicMaterial::new(0.1, 5.2)))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);
    let mut cars: Vec<Vehicle> = Vec::new();
    let _rng = rand::thread_rng();
    for i in 0..0 {
        let mut car = Vehicle::new();
        let centre: Vector2<f32> = vec2(STARTX + 20.0 * (i + 1) as f32, STARTZ + 30.0);
        car.build(
            &mut bodies,
            &mut colliders,
            &mut joint_constraints,
            centre.x,
            get_height(
                &mut heights,
                centre.x + (WORLDSIZE as f32 / 2.0),
                centre.y + (WORLDSIZE as f32 / 2.0),
            ) + 5.0
                + (i as f32 * 2.01),
            centre.y + (i as f32 * 0.01),
        );
        cars.push(car);
    }
    let mut first_car = Vehicle::new();
    let centre: Vector2<f32> = vec2(STARTX, STARTZ);
    first_car.build(
        &mut bodies,
        &mut colliders,
        &mut joint_constraints,
        centre.x,
        get_height(
            &mut heights,
            centre.x + (WORLDSIZE as f32 / 2.0),
            centre.y + (WORLDSIZE as f32 / 2.0),
        ) + 5.0,
        centre.y,
    );
    mechanical_world.step(
        &mut geometrical_world,
        &mut bodies,
        &mut colliders,
        &mut joint_constraints,
        &mut force_generators,
    );
    let mut root_node = Quadtree::new(1);
    let mut the_tree = Quadtree::new(1);
    let centre: cVector3<f32> = vec3(WORLDSIZE as f32 / 2.0, 0.0, WORLDSIZE as f32 / 2.0);
    let width: u16 = WORLDSIZE as u16;
    let tree_model_1 = Model::new(&("objects/tree3.obj"));
    let tree_model_2 = Model::new(&("objects/test1.obj"));
    let tree_model_3 = Model::new(&("objects/test1.obj"));
    let tree_model_4 = Model::new(&("objects/test1.obj"));
    let body_model = Model::new(&("objects/armytruck3.obj"));
    let rr_wheel_model = Model::new(&("objects/rear_right_wheel2.obj"));
    let rl_wheel_model = Model::new(&("objects/rear_left_wheel2.obj"));
    let fr_wheel_model = Model::new(&("objects/front_right_wheel2.obj"));
    let fl_wheel_model = Model::new(&("objects/front_left_wheel2.obj"));
    if DRAW_TREES {
        for _i in 1..60 {
            add_trees(
                &mut tree_centres_x,
                &mut tree_centres_z,
                &mut tree_sizes,
                &mut tree_heights,
                &mut heights,
                80.0,
                false,
                1.0,
                1.0,
            );
        }
        add_tree_shadows(
            &mut tree_heights,
            &mut tree_centres_x,
            &mut tree_centres_z,
            &mut tree_shadows,
        );
        unsafe {
            NODE_COUNTER = 0;
        }
    }
    the_tree.create_tree(
        &mut root_node,
        1,
        centre,
        width,
        &mut heights,
        &mut tree_shadows,
        WORLDSIZE,
        QUADS_PER_NODE,
        &mut tree_heights,
        &mut tree_centres_x,
        &mut tree_centres_z,
        &mut tree_sizes,
    );
    unsafe {
        println!("t {}", TRIANGLE_COUNTER);
    }
    let mut texture1: u32 = 0;
    let mut texture2: u32 = 0;
    let mut texture3: u32 = 0;
    let mut texture4: u32 = 0;
    let mut texture5: u32 = 0;
    let mut texture6: u32 = 0;
    let mut texture7: u32 = 0;
    let mut texture8: u32 = 0;
    let mut sky_texture: u32 = 0;
    unsafe {
        load_texture("textures/grass.jpg", &mut texture1, false);
        load_texture("textures/grass4.jpg", &mut texture2, false);
        load_texture("textures/rock.jpg", &mut texture3, false);
        load_texture("textures/snow.jpg", &mut texture4, false);
        load_texture("textures/ground.jpg", &mut texture5, false);
        load_texture("textures/grass4.jpg", &mut texture6, false);
        load_texture("textures/grass.jpg", &mut texture7, false);
        load_texture("textures/mud.jpg", &mut texture8, false);
        load_texture("textures/sky.jpg", &mut sky_texture, false);
        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, texture1);
        gl::ActiveTexture(gl::TEXTURE1);
        gl::BindTexture(gl::TEXTURE_2D, texture2);
        gl::ActiveTexture(gl::TEXTURE2);
        gl::BindTexture(gl::TEXTURE_2D, texture3);
        gl::ActiveTexture(gl::TEXTURE3);
        gl::BindTexture(gl::TEXTURE_2D, texture4);
        gl::ActiveTexture(gl::TEXTURE4);
        gl::BindTexture(gl::TEXTURE_2D, texture5);
        gl::ActiveTexture(gl::TEXTURE5);
        gl::BindTexture(gl::TEXTURE_2D, texture6);
        gl::ActiveTexture(gl::TEXTURE6);
        gl::BindTexture(gl::TEXTURE_2D, texture7);
        gl::ActiveTexture(gl::TEXTURE7);
        gl::BindTexture(gl::TEXTURE_2D, texture8);
    }
    let shadow_buffer_shader = Shader::new("shaders/shadow_vert.c", "shaders/shadow_frag.c");
    let tree_shadow_shader = Shader::new("shaders/tree_shadow_vert.c", "shaders/tree_shadow_frag.c");
    let sky_shader = Shader::new("shaders/sky_vert.c", "shaders/sky_frag.c");
    let terrain_shader = Shader::new("shaders/terrain_vert.c", "shaders/terrain_frag.c");
    let trees_shader = Shader::new("shaders/tree_model_vert.c", "shaders/tree_model_frag.c");
    let object_shader = Shader::new("shaders/object_vert.c", "shaders/object_frag.c");
    let ssao_shader = Shader::new("shaders/buffer_vert.c", "shaders/ssao_frag.c");
    let ssao_blur_shader = Shader::new("shaders/buffer_vert.c", "shaders/ssao_blur_frag.c");
    let lighting_shader = Shader::new("shaders/buffer_vert.c", "shaders/lighting_frag.c");
    let shadow_fog_bloom_shader = Shader::new("shaders/buffer_vert.c", "shaders/shadow_fog_bloom_frag.c");
    let output_shader = Shader::new("shaders/buffer_vert.c", "shaders/post_frag.c");

    unsafe {
        terrain_shader.useProgram();
        terrain_shader.setInt(c_str!("texture1"), 0);
        terrain_shader.setInt(c_str!("texture2"), 1);
        terrain_shader.setInt(c_str!("texture3"), 2);
        terrain_shader.setInt(c_str!("texture4"), 3);
        terrain_shader.setInt(c_str!("texture5"), 4);
        terrain_shader.setInt(c_str!("texture6"), 5);
        terrain_shader.setInt(c_str!("texture7"), 6);
        terrain_shader.setInt(c_str!("texture8"), 7);
    }
    let mut sky_vao: gl::types::GLuint = 0;
    let mut sky_vbo: gl::types::GLuint = 0;
    let mut sky_ebo: gl::types::GLuint = 0;
    let levels = 20.0;
    let _levelgap = 180.0 / levels;
    let steps: f32 = 20.0;
    let stepgap = (std::f64::consts::PI * 2.0) / steps as f64;
    let sky_vertices_size = (levels as i32 + 1) * (steps as i32 + 1) * 5;
    let sky_indices_size = (levels as i32 + 1) * (steps as i32 + 1) * 6;
    let mut sky_vertices: Vec<f32> = vec![0.0; sky_vertices_size as usize];
    let mut sky_indices: Vec<i32> = vec![0; sky_indices_size as usize];
    let mut sky_vertex_index = 0;
    unsafe {
        sky_shader.useProgram();
        sky_shader.setVec3(c_str!("fogcolour"), light.x, light.y, light.z);
        for level in 0..(levels * 0.6) as i32 {
            let mut radius = ((level as f32 / levels) * std::f64::consts::PI as f32 * 1.0).sin() * SKYRADIUS;

            let mut height = ((level as f32 / levels) * std::f64::consts::PI as f32 * 1.0).cos() * SKYHEIGHT;
            if level as f32 / levels > 0.5 {
                radius = SKYRADIUS;
                height = (level as f32 / levels) * -800.0;
            }
            for step in 0..steps as i32 {
                let angle = step as f32 * stepgap as f32;
                sky_vertices[sky_vertex_index] = angle.sin() * radius;
                sky_vertices[sky_vertex_index + 1] = height;
                sky_vertices[sky_vertex_index + 2] = angle.cos() * radius;
                sky_vertices[sky_vertex_index + 3] = clamp(0.5 + angle.sin() * (radius / SKYRADIUS) * 0.5);
                sky_vertices[sky_vertex_index + 4] = clamp(0.5 + angle.cos() * (radius / SKYRADIUS) * 0.5);
                sky_vertex_index += 5;
            }
        }
        let mut sky_index_index = 0;
        for level in 0..levels as i32 {
            for step in 0..steps as i32 {
                sky_indices[sky_index_index] = (level * (steps as i32)) + step;
                sky_indices[sky_index_index + 1] = ((level + 1) * (steps as i32)) + step;
                sky_indices[sky_index_index + 2] = ((level) * (steps as i32)) + step + 1;

                sky_indices[sky_index_index + 3] = ((level + 1) * (steps as i32)) + step;
                sky_indices[sky_index_index + 4] = ((level + 1) * (steps as i32)) + step + 1;
                sky_indices[sky_index_index + 5] = (level * (steps as i32)) + step + 1;
                sky_index_index += 6;
            }
        }
        gl::GenBuffers(1, &mut sky_vbo);
        gl::BindBuffer(gl::ARRAY_BUFFER, sky_vbo);
        gl::BufferData(
            gl::ARRAY_BUFFER,
            (sky_vertices_size as usize * mem::size_of::<GLfloat>()) as GLsizeiptr,
            &sky_vertices[0] as *const f32 as *const c_void,
            gl::STATIC_DRAW,
        );
        gl::GenBuffers(1, &mut sky_ebo);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, sky_ebo);
        gl::BufferData(
            gl::ELEMENT_ARRAY_BUFFER,
            (sky_indices_size as usize * mem::size_of::<GLfloat>()) as GLsizeiptr,
            &sky_indices[0] as *const i32 as *const c_void,
            gl::STATIC_DRAW,
        );
        gl::GenVertexArrays(1, &mut sky_vao);
    }
    let mut camera = Camera {
        Position: cPoint3::new(0.0, 0.0, 0.0),
        ..Camera::default()
    };
    let mut first_mouse = true;
    let mut last_x: f32 = SCR_WIDTH as f32 / 2.0;
    let mut last_y: f32 = SCR_HEIGHT as f32 / 2.0;
    let mut delta_time: f32;
    let mut last_frame: f32 = 0.0;
    let _now = Instant::now();
    let mut fps_display = Instant::now();
    let mut frame_count = 0;
    let mut tframe_count = 0;
    let mut _fbo = 0;
    let mut _depth_vao = 0;
    let mut _depth_texture = 0;
    let mut g_buffer = 0;
    let mut g_position = 0;
    let mut g_normal = 0;
    let mut g_albedo = 0;
    let mut g_fog_position = 0;
    let mut rbo_depth = 0;
    let mut ssaoFBO = 0;
    let mut ssaoBlurFBO = 0;
    let mut ssaoColorBuffer = 0;
    let mut ssaoColorBufferBlur = 0;
    let mut lightBuffer = 0;
    let mut colourBuffer = 0;
    let mut shadowBuffer = 0;
    let mut shadow = 0;
    let mut ground_colour = 0;
    let mut colours: [u32; 2] = [0, 0];
    let mut lightTextures: [u32; 2] = [0, 0];
    let mut ssaoKernel: Vec<v3<f32>> = Vec::new();
    let mut ssaoNoise: Vec<v3<f32>> = Vec::new();
    let mut noiseTexture = 0;
    let mut light_positions: Vec<v3<f32>> = Vec::new();
    let mut light_colors: Vec<v3<f32>> = Vec::new();

    unsafe {
        fn lerp(a: f32, b: f32, f: f32) -> f32 {
            return a + f * (b - a);
        }

        // ------------------------------
        gl::GenFramebuffers(1, &mut g_buffer);
        gl::BindFramebuffer(gl::FRAMEBUFFER, g_buffer);

        // position color buffer
        gl::GenTextures(1, &mut g_position);
        gl::BindTexture(gl::TEXTURE_2D, g_position);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RGBA32F as i32,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
            0,
            gl::RGBA,
            gl::FLOAT,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::CLAMP_TO_EDGE as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::CLAMP_TO_EDGE as i32);
        gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT0, gl::TEXTURE_2D, g_position, 0);

        // normal color buffer
        gl::GenTextures(1, &mut g_normal);
        gl::BindTexture(gl::TEXTURE_2D, g_normal);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RGB16F as i32,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
            0,
            gl::RGB,
            gl::FLOAT,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT1, gl::TEXTURE_2D, g_normal, 0);

        // color + specular color buffer
        gl::GenTextures(1, &mut g_albedo);
        gl::BindTexture(gl::TEXTURE_2D, g_albedo);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RGB32F as i32,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
            0,
            gl::RGB,
            gl::UNSIGNED_BYTE,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT2, gl::TEXTURE_2D, g_albedo, 0);

        // fog position color buffer
        gl::GenTextures(1, &mut g_fog_position);
        gl::BindTexture(gl::TEXTURE_2D, g_fog_position);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RGB32F as i32,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
            0,
            gl::RGB,
            gl::FLOAT,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::CLAMP_TO_EDGE as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::CLAMP_TO_EDGE as i32);
        gl::FramebufferTexture2D(
            gl::FRAMEBUFFER,
            gl::COLOR_ATTACHMENT3,
            gl::TEXTURE_2D,
            g_fog_position,
            0,
        );

        // ground colour buffer
        gl::GenTextures(1, &mut ground_colour);
        gl::BindTexture(gl::TEXTURE_2D, ground_colour);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RGB16F as i32,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
            0,
            gl::RGB,
            gl::UNSIGNED_BYTE,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::COLOR_ATTACHMENT4, gl::TEXTURE_2D, ground_colour, 0);

        // tell OpenGL which color attachments we'll use (of this framebuffer) for rendering
        let attachments = [
            gl::COLOR_ATTACHMENT0,
            gl::COLOR_ATTACHMENT1,
            gl::COLOR_ATTACHMENT2,
            gl::COLOR_ATTACHMENT3,
            gl::COLOR_ATTACHMENT4,
        ];
        gl::DrawBuffers(5, &attachments[0]);

        // create and attach depth buffer (renderbuffer)
        gl::GenRenderbuffers(1, &mut rbo_depth);
        gl::BindRenderbuffer(gl::RENDERBUFFER, rbo_depth);
        gl::RenderbufferStorage(
            gl::RENDERBUFFER,
            gl::DEPTH_COMPONENT,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
        );
        gl::FramebufferRenderbuffer(gl::FRAMEBUFFER, gl::DEPTH_ATTACHMENT, gl::RENDERBUFFER, rbo_depth);

        // finally check if framebuffer is complete
        if gl::CheckFramebufferStatus(gl::FRAMEBUFFER) != gl::FRAMEBUFFER_COMPLETE {
            println!("Framebuffer not complete!");
        }
        gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
        gl::ClearColor(0.0, 0.0, 0.0, 1.0);
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // SSAO
        gl::GenFramebuffers(1, &mut ssaoFBO);
        gl::GenFramebuffers(1, &mut ssaoBlurFBO);
        gl::BindFramebuffer(gl::FRAMEBUFFER, ssaoFBO);

        // SSAO color buffer
        gl::GenTextures(1, &mut ssaoColorBuffer);
        gl::BindTexture(gl::TEXTURE_2D, ssaoColorBuffer);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RED as i32,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
            0,
            gl::RED as u32,
            gl::FLOAT,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::FramebufferTexture2D(
            gl::FRAMEBUFFER,
            gl::COLOR_ATTACHMENT0,
            gl::TEXTURE_2D,
            ssaoColorBuffer,
            0,
        );
        if gl::CheckFramebufferStatus(gl::FRAMEBUFFER) != gl::FRAMEBUFFER_COMPLETE {
            println!("SSAO Framebuffer not complete!");
        }

        // blur buffer
        gl::BindFramebuffer(gl::FRAMEBUFFER, ssaoBlurFBO);
        gl::GenTextures(1, &mut ssaoColorBufferBlur);
        gl::BindTexture(gl::TEXTURE_2D, ssaoColorBufferBlur);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RED as i32,
            SCR_WIDTH as i32,
            SCR_HEIGHT as i32,
            0,
            gl::RED,
            gl::FLOAT,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::FramebufferTexture2D(
            gl::FRAMEBUFFER,
            gl::COLOR_ATTACHMENT0,
            gl::TEXTURE_2D,
            ssaoColorBufferBlur,
            0,
        );
        if gl::CheckFramebufferStatus(gl::FRAMEBUFFER) != gl::FRAMEBUFFER_COMPLETE {
            println!("SSAO Blur Framebuffer not complete!");
        }
        gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

        gl::GenFramebuffers(1, &mut lightBuffer);
        gl::BindFramebuffer(gl::FRAMEBUFFER, lightBuffer);
        gl::GenTextures(2, &mut lightTextures[0]);
        for tex in 0..2 {
            gl::BindTexture(gl::TEXTURE_2D, lightTextures[tex]);
            gl::TexImage2D(
                gl::TEXTURE_2D,
                0,
                gl::RGB32F as i32,
                SCR_WIDTH as i32,
                SCR_HEIGHT as i32,
                0,
                gl::RGB,
                gl::FLOAT,
                ptr::null(),
            );
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
            gl::FramebufferTexture2D(
                gl::FRAMEBUFFER,
                gl::COLOR_ATTACHMENT0 + tex as u32,
                gl::TEXTURE_2D,
                lightTextures[tex],
                0,
            );
        }
        if gl::CheckFramebufferStatus(gl::FRAMEBUFFER) != gl::FRAMEBUFFER_COMPLETE {
            println!("Light Framebuffer not complete!");
        }
        let attachments = [gl::COLOR_ATTACHMENT0, gl::COLOR_ATTACHMENT1];
        gl::DrawBuffers(2, &attachments[0]);
        gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

        let mut rng = rand::thread_rng();
        for i in 0..64 as u32 {
            let mut sample = v3::new(
                rng.gen::<f32>() * 2.0 - 1.0,
                rng.gen::<f32>() * 2.0 - 1.0,
                rng.gen::<f32>(),
            )
            .normalize();
            sample *= rng.gen::<f32>();
            let mut scale = i as f32 / 64.0;
            scale = lerp(0.1, 1.0, scale.powf(2.0));
            sample *= scale;
            ssaoKernel.push(sample);
        }

        for _i in 0..64 as u32 {
            let noise = v3::new(rng.gen::<f32>() * 2.0 - 1.0, rng.gen::<f32>() * 2.0 - 1.0, 0.0); // rotate around z-axis (in tangent space)
            ssaoNoise.push(noise);
        }
        gl::GenTextures(1, &mut noiseTexture);
        gl::BindTexture(gl::TEXTURE_2D, noiseTexture);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RGBA32F as i32,
            8,
            8,
            0,
            gl::RGB,
            gl::FLOAT,
            &ssaoNoise[0] as *const _ as *const c_void,
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::REPEAT as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::REPEAT as i32);

        // colour buffer
        gl::GenFramebuffers(1, &mut colourBuffer);
        gl::BindFramebuffer(gl::FRAMEBUFFER, colourBuffer);
        gl::GenTextures(2, &mut colours[0]);
        for tex in 0..2 {
            gl::BindTexture(gl::TEXTURE_2D, colours[tex]);
            gl::TexImage2D(
                gl::TEXTURE_2D,
                0,
                gl::RGBA32F as i32,
                SCR_WIDTH as i32,
                SCR_HEIGHT as i32,
                0,
                gl::RGBA,
                gl::FLOAT,
                ptr::null(),
            );
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::REPEAT as i32);
            gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::REPEAT as i32);
            gl::FramebufferTexture2D(
                gl::FRAMEBUFFER,
                gl::COLOR_ATTACHMENT0 + tex as u32,
                gl::TEXTURE_2D,
                colours[tex],
                0,
            );
        }
        let attachments = [gl::COLOR_ATTACHMENT0, gl::COLOR_ATTACHMENT1];
        gl::DrawBuffers(2, &attachments[0]);
        gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

        // shadow buffer
        gl::GenFramebuffers(1, &mut shadowBuffer);
        gl::BindFramebuffer(gl::FRAMEBUFFER, shadowBuffer);
        gl::GenTextures(1, &mut shadow);
        gl::BindTexture(gl::TEXTURE_2D, shadow);
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::DEPTH_COMPONENT32F as i32,
            (SCR_WIDTH * 2_u32.pow(SHADOW_BUFFER_SCALE)) as i32,
            (SCR_HEIGHT * 2_u32.pow(SHADOW_BUFFER_SCALE)) as i32,
            0,
            gl::DEPTH_COMPONENT,
            gl::FLOAT,
            ptr::null(),
        );
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::REPEAT as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::REPEAT as i32);
        gl::FramebufferTexture2D(gl::FRAMEBUFFER, gl::DEPTH_ATTACHMENT as u32, gl::TEXTURE_2D, shadow, 0);
        gl::DrawBuffer(gl::COLOR_ATTACHMENT0);
        gl::ReadBuffer(gl::NONE);
        gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
        let _nr_lights = 1024;
        for i in 0..2 {
            for j in 0..2 {
                let x = STARTX + j as f32 * 60.0 + rng.gen::<f32>() * 5.0 - 450.0;
                let z = STARTZ - i as f32 * 60.0 + rng.gen::<f32>();
                light_positions.push(v3::new(
                    x,
                    get_height(&mut heights, x + (WORLDSIZE as f32 / 2.0), z + (WORLDSIZE as f32 / 2.0)) + 8.5,
                    z,
                ));
                let rColor = rng.gen::<f32>() * 1.0;
                let gColor = rng.gen::<f32>() * 1.0;
                let bColor = rng.gen::<f32>() * 1.0;
                let mut _Color = v3::new(18.0 * rColor, 18.0 * gColor, 18.0 * bColor);

                let Color = v3::new(18.0, 0.0, 0.0);
                light_colors.push(Color);
            }
        }

        // shader configuration
        output_shader.useProgram();
        output_shader.setInt(c_str!("colours"), 0);
        output_shader.setInt(c_str!("bloom"), 1);
        output_shader.setInt(c_str!("light"), 2);
        lighting_shader.useProgram();
        lighting_shader.setVec3(c_str!("rightspot.colour"), 0.0, 0.0, 0.0);
        lighting_shader.setInt(c_str!("g_position"), 0);
        lighting_shader.setInt(c_str!("g_normal"), 1);
        lighting_shader.setInt(c_str!("g_albedo"), 2);
        lighting_shader.setInt(c_str!("g_fog_position"), 3);
        lighting_shader.setInt(c_str!("ground_colour"), 4);
        lighting_shader.setInt(c_str!("ssao"), 6);
        lighting_shader.setInt(c_str!("light_depth"), 7);
        ssao_shader.useProgram();
        ssao_shader.setInt(c_str!("g_position"), 0);
        ssao_shader.setInt(c_str!("g_normal"), 1);
        ssao_shader.setInt(c_str!("texNoise"), 2);
        ssao_shader.setVec2(c_str!("screen_size"), SCR_WIDTH as f32, SCR_HEIGHT as f32);
        ssao_blur_shader.useProgram();
        ssao_blur_shader.setInt(c_str!("ssaoInput"), 0);
        lighting_shader.useProgram();
        for i in 0..2 {
            lighting_shader.setVec3(
                &(CString::new(format!("{}{}{}", "lights[", i, "].Position")).unwrap()),
                light_positions[i].x,
                light_positions[i].y,
                light_positions[i].z,
            );
            lighting_shader.setVec3(
                &(CString::new(format!("{}{}{}", "lights[", i, "].Color")).unwrap()),
                light_colors[i].x,
                light_colors[i].y,
                light_colors[i].z,
            );
            let linear = 0.7;
            let quadratic = 1.8;
            lighting_shader.setFloat(
                &(CString::new(format!("{}{}{}", "lights[", i, "].Linear")).unwrap()),
                linear,
            );

            lighting_shader.setFloat(
                &(CString::new(format!("{}{}{}", "lights[", i, "].Quadratic")).unwrap()),
                quadratic,
            );
            lighting_shader.setFloat(
                &(CString::new(format!("{}{}{}", "lights[", i, "].Radius")).unwrap()),
                22.0,
            );
        }
        ssao_shader.useProgram();
        for i in 0..64 {
            ssao_shader.setVec3(
                &(CString::new(format!("{}{}{}", "samples[", i, "]")).unwrap()),
                ssaoKernel[i].x,
                ssaoKernel[i].y,
                ssaoKernel[i].z,
            );
        }
    }
    let model: Matrix4<f32> = Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );
    camera.Position.x = STARTX;
    camera.Position.z = STARTZ;
    let mut timer = Instant::now();
    unsafe {
        sky_shader.setInt(c_str!("texture_diffuse_1"), 0);
    }
    let tfps_display = Instant::now();
    let mut phys = Instant::now();
    let mut lightstime = Instant::now();
    let start_phys = Instant::now();
    let mut camera_rot: f32 = 0.0;
    let mut camera_rot_2: f32 = 1.7;
    let mut donkey = Profiler::new();
    let mut rabbit = Profiler::new();
    let mut ferret = Profiler::new();
    let mut badger = Profiler::new();
    let mut marmot = Profiler::new();
    let mut beaver = Profiler::new();
    let mut monkey = Profiler::new();

    while !window.should_close() {
        donkey.start();
        if window.get_key(Key::Num1) == Action::Press {
            REAR_AXLE = false;
            MIDDLE_AXLE = false;
            FRONT_AXLE = false;
            INSIDE = false;
            BACK = false;
            FRONT = false;
            BUMPER = true;
            MOUSE = false;
        }
        if window.get_key(Key::Num2) == Action::Press {
            REAR_AXLE = false;
            MIDDLE_AXLE = false;
            FRONT_AXLE = false;
            INSIDE = false;
            BACK = false;
            FRONT = true;
            BUMPER = false;
            MOUSE = false;
        }
        if window.get_key(Key::Num3) == Action::Press {
            REAR_AXLE = false;
            MIDDLE_AXLE = false;
            FRONT_AXLE = false;
            INSIDE = true;
            BACK = false;
            FRONT = false;
            BUMPER = false;
            MOUSE = false;
        }
        if window.get_key(Key::Num4) == Action::Press {
            REAR_AXLE = false;
            MIDDLE_AXLE = false;
            FRONT_AXLE = false;
            INSIDE = false;
            BACK = true;
            FRONT = false;
            BUMPER = false;
            MOUSE = false;
        }
        if window.get_key(Key::Num5) == Action::Press {
            REAR_AXLE = false;
            MIDDLE_AXLE = false;
            FRONT_AXLE = true;
            INSIDE = false;
            BACK = false;
            FRONT = false;
            BUMPER = false;
            MOUSE = false;
        }
        if window.get_key(Key::Num6) == Action::Press {
            REAR_AXLE = false;
            MIDDLE_AXLE = true;
            FRONT_AXLE = false;
            INSIDE = true;
            BACK = false;
            FRONT = false;
            BUMPER = false;
            MOUSE = false;
        }
        if window.get_key(Key::Num7) == Action::Press {
            REAR_AXLE = true;
            MIDDLE_AXLE = false;
            FRONT_AXLE = false;
            INSIDE = true;
            BACK = false;
            FRONT = false;
            BUMPER = false;
            MOUSE = false;
        }
        if window.get_key(Key::Num8) == Action::Press {
            REAR_AXLE = false;
            MIDDLE_AXLE = false;
            FRONT_AXLE = false;
            INSIDE = true;
            BACK = false;
            FRONT = false;
            BUMPER = false;
            MOUSE = true;
        }
        unsafe {
            lighting_shader.useProgram();
            if window.get_key(Key::P) == Action::Press {
                lighton = true;
            }
            if window.get_key(Key::O) == Action::Press {
                lighton = false;
            }
            lighttime = time <= 12.0 || time >= 12.0;
            if lighttime && lighton {
                lighting_shader.setVec3(c_str!("rightspot.colour"), 1.5, 1.5, 1.5);
            } else {
                lighting_shader.setVec3(c_str!("rightspot.colour"), 0.0, 0.0, 0.0);
            }
        }
        time += 0.0002;
        if window.get_key(Key::Num9) == Action::Press {
            time = time - 0.04;
        }
        if window.get_key(Key::Num0) == Action::Press {
            time = time + 0.04;
        }
        // time = (time as f32).max(4.0);
        if time >= 24.0 {
            time = 0.0;
        }
        if window.get_key(Key::I) == Action::Press {
            fly += 5.0;
        }
        if window.get_key(Key::K) == Action::Press {
            fly -= 5.0;
        }
        fly = (fly as f32).max(1.0);
        // println!("{}", fly);

        let skyred = (((((3.14159 * 2.0) * (time / 24.0) - 1.5707) as f32).sin()) * 0.4 + 0.2)
            .max(0.0)
            .min(1.0);

        let skygreen = (((((3.14159 * 2.0) * (time / 24.0) - 1.5707) as f32).sin()) * 0.8 + 0.8)
            .max(0.00)
            .min(1.0);

        let skyblue = (((((3.14159 * 2.0) * (time / 24.0) - 1.5707) as f32).sin() * 0.4) + 0.5)
            .max(0.00)
            .min(1.0);

        light = vec3(skygreen, skygreen, skygreen);
        if window.get_key(Key::R) == Action::Press {
            let car_body_handle = first_car.collider_handles.first().unwrap();
            let car_body_collider = colliders.get_mut(*car_body_handle).unwrap();
            let car_position = car_body_collider.position().translation.vector;
            first_car.remove(&mut bodies, &mut colliders);
            first_car.build(
                &mut bodies,
                &mut colliders,
                &mut joint_constraints,
                car_position.x,
                get_height(
                    &mut heights,
                    car_position.x + (WORLDSIZE as f32 / 2.0),
                    car_position.z + (WORLDSIZE as f32 / 2.0),
                ) + 4.0,
                car_position.z,
            );
            mechanical_world.step(
                &mut geometrical_world,
                &mut bodies,
                &mut colliders,
                &mut joint_constraints,
                &mut force_generators,
            );
        }
        let force = 0.51 * (GRAVITY / 9.81);
        if start_phys.elapsed().as_secs() > 0 {
            let elapsed = phys.elapsed().as_millis();
            if elapsed > 16 {
                rabbit.start();
                let steps = (elapsed / 4) as u16;
                for _phys_step in 0..steps {
                    phys = Instant::now();
                    for the_car in cars.iter() {
                        the_car.springs(&mut bodies, force);
                        the_car.motors(&mut bodies, -1.0);
                    }
                    first_car.springs(&mut bodies, force);
                    first_car.motors(&mut bodies, motor);
                    first_car.steering(&mut bodies, &mut angle);
                    mechanical_world.step(
                        &mut geometrical_world,
                        &mut bodies,
                        &mut colliders,
                        &mut joint_constraints,
                        &mut force_generators,
                    );
                }
                rabbit.stop();
            }
        } else {
            phys = Instant::now();
        }
        let current_frame = glfw.get_time() as f32;
        delta_time = current_frame - last_frame;
        last_frame = current_frame;
        proc_events(
            &events,
            &mut first_mouse,
            &mut last_x,
            &mut last_y,
            &mut camera,
            &mut camera_rot,
            &mut camera_rot_2,
        );
        night_mode = processInput(
            &mut window,
            delta_time,
            &mut camera,
            night_mode,
            &mut angle,
            &mut motor,
            &mut camera_rot,
        );
        unsafe {
            let car_body_handle = first_car.collider_handles.last().unwrap();
            let car_body_collider = colliders.get_mut(*car_body_handle).unwrap();
            let car_position = car_body_collider.position().translation.vector;
            let rotation = car_body_collider.position().rotation;
            let forward = rotation.transform_point(&p3::new(0.0, 0.0, 1.0));
            let up = rotation.transform_point(&p3::new(0.0, 1.0, 0.0));
            let up = cVector3::new(up.x, up.y, up.z);
            let right = rotation.transform_point(&p3::new(-1.0, 0.0, 0.0));
            let right = cVector3::new(right.x, right.y, right.z);
            camera.Position.x = car_position.x;
            camera.Position.y = car_position.y + fly;
            camera.Position.z = car_position.z;
            let mut vect = v3::new(0.0, 0.0, 0.0);
            let mut vect2 = v3::new(0.0, 0.0, 0.0);
            if REAR_AXLE {
                vect = v3::new(
                    car_position.x - forward.x * (-8.5 * SCALE) - (4.7 * SCALE) * up.x,
                    car_position.y - forward.y * (-8.5 * SCALE) - (4.7 * SCALE) * up.y,
                    car_position.z - forward.z * (-8.5 * SCALE) - (4.7 * SCALE) * up.z,
                );
                vect2 = v3::new(
                    car_position.x - forward.x * (200.0 * SCALE) - (5.0 * SCALE) * up.x,
                    car_position.y - forward.y * (200.0 * SCALE) - (5.0 * SCALE) * up.y,
                    car_position.z - forward.z * (200.0 * SCALE) - (5.0 * SCALE) * up.z,
                );
            } else if MIDDLE_AXLE {
                vect = v3::new(
                    car_position.x - forward.x * (-6.0 * SCALE) - (5.0 * SCALE) * up.x,
                    car_position.y - forward.y * (-6.0 * SCALE) - (5.0 * SCALE) * up.y,
                    car_position.z - forward.z * (-6.0 * SCALE) - (5.0 * SCALE) * up.z,
                );
                vect2 = v3::new(
                    car_position.x - forward.x * (200.0 * SCALE) - (5.0 * SCALE) * up.x,
                    car_position.y - forward.y * (200.0 * SCALE) - (5.0 * SCALE) * up.y,
                    car_position.z - forward.z * (200.0 * SCALE) - (5.0 * SCALE) * up.z,
                );
            } else if FRONT_AXLE {
                vect = v3::new(
                    car_position.x - forward.x * (2.0 * SCALE) - (5.0 * SCALE) * up.x,
                    car_position.y - forward.y * (2.0 * SCALE) - (5.0 * SCALE) * up.y,
                    car_position.z - forward.z * (2.0 * SCALE) - (5.0 * SCALE) * up.z,
                );
                vect2 = v3::new(
                    car_position.x - forward.x * (200.0 * SCALE) - (5.0 * SCALE) * up.x,
                    car_position.y - forward.y * (200.0 * SCALE) - (5.0 * SCALE) * up.y,
                    car_position.z - forward.z * (200.0 * SCALE) - (5.0 * SCALE) * up.z,
                );
            } else if INSIDE {
                vect = v3::new(
                    car_position.x - forward.x * (2.2 * SCALE) + (1.0 * SCALE) * right.x - (0.2 * SCALE) * up.x,
                    car_position.y - forward.y * (2.2 * SCALE) + (1.0 * SCALE) * right.y - (0.2 * SCALE) * up.y,
                    car_position.z - forward.z * (2.2 * SCALE) + (1.0 * SCALE) * right.z - (0.2 * SCALE) * up.z,
                );
                vect2 = v3::new(
                    car_position.x - forward.x * (20.0 * SCALE) + (1.0 * SCALE) * right.x - (0.2 * SCALE) * up.x,
                    car_position.y - forward.y * (20.0 * SCALE) + (1.0 * SCALE) * right.y - (0.2 * SCALE) * up.y,
                    car_position.z - forward.z * (20.0 * SCALE) + (1.0 * SCALE) * right.z - (0.2 * SCALE) * up.z,
                );
            } else if BACK {
                vect = v3::new(
                    car_position.x + forward.x * (2.0 * SCALE) - (0.5 * SCALE) * up.x,
                    car_position.y + forward.y * (2.0 * SCALE) - (0.5 * SCALE) * up.y,
                    car_position.z + forward.z * (2.0 * SCALE) - (0.5 * SCALE) * up.z,
                );
                vect2 = v3::new(
                    car_position.x + forward.x * (200.0 * SCALE) - (0.5 * SCALE) * up.x,
                    car_position.y + forward.y * (200.0 * SCALE) - (0.5 * SCALE) * up.y,
                    car_position.z + forward.z * (200.0 * SCALE) - (0.5 * SCALE) * up.z,
                );
            } else if FRONT {
                vect = v3::new(
                    car_position.x - forward.x * (5.0 * SCALE) - (0.7 * SCALE) * up.x,
                    car_position.y - forward.y * (5.0 * SCALE) - (0.7 * SCALE) * up.y,
                    car_position.z - forward.z * (5.0 * SCALE) - (0.7 * SCALE) * up.z,
                );
                vect2 = v3::new(
                    car_position.x - forward.x * (200.0 * SCALE) + (1.0 * SCALE) * up.x,
                    car_position.y - forward.y * (200.0 * SCALE) + (1.0 * SCALE) * up.y,
                    car_position.z - forward.z * (200.0 * SCALE) + (1.0 * SCALE) * up.z,
                )
            } else if BUMPER {
                vect = v3::new(
                    car_position.x - forward.x * (FF * SCALE) + (UU * SCALE) * up.x + (RR * SCALE) * right.x,
                    car_position.y - forward.y * (FF * SCALE) + (UU * SCALE) * up.y + (RR * SCALE) * right.y,
                    car_position.z - forward.z * (FF * SCALE) + (UU * SCALE) * up.z + (RR * SCALE) * right.z,
                );
                vect2 = v3::new(
                    car_position.x - forward.x * (20.0 * SCALE) + (UU * SCALE) * up.x + (RR * SCALE) * right.x,
                    car_position.y - forward.y * (20.0 * SCALE) + (UU * SCALE) * up.y + (RR * SCALE) * right.y,
                    car_position.z - forward.z * (20.0 * SCALE) + (UU * SCALE) * up.z + (RR * SCALE) * right.z,
                );
            }
            let mut view = cgmath::Matrix4::look_at_rh(
                cPoint3::new(vect.x, vect.y, vect.z),
                cPoint3::new(vect2.x, vect2.y, vect2.z),
                up,
            );
            if MOUSE {
                view = camera.GetViewMatrix();
            }
            let mut lightview = cgmath::Matrix4::look_at_rh(
                cPoint3::new(
                    car_position.x - forward.x * ((FF - 1.25) * SCALE)
                        + ((UU - 1.0) * SCALE) * up.x
                        + ((RR - 0.0) * SCALE) * right.x,
                    car_position.y - forward.y * ((FF - 1.25) * SCALE)
                        + ((UU - 1.0) * SCALE) * up.y
                        + ((RR - 0.0) * SCALE) * right.y,
                    car_position.z - forward.z * ((FF - 1.25) * SCALE)
                        + ((UU - 1.0) * SCALE) * up.z
                        + ((RR - 0.0) * SCALE) * right.z,
                ),
                cPoint3::new(
                    car_position.x - forward.x * 60.0 - 0.0 * up.x,
                    car_position.y - forward.y * 60.0 - 0.0 * up.y,
                    car_position.z - forward.z * 60.0 - 0.0 * up.z,
                ),
                up,
            );
            lighting_shader.useProgram();
            lighting_shader.setVec3(
                &(CString::new(format!("{}{}{}", "lights[", 0, "].Position")).unwrap()),
                car_position.x - forward.x * 30.0 - 0.0 * up.x,
                car_position.y - forward.y * 30.0 - 0.0 * up.y,
                car_position.z - forward.z * 30.0 - 0.0 * up.z,
            );
            let posright1: v3<f32> = v3::new(
                car_position.x - forward.x * ((FF - 1.25) * SCALE)
                    + ((UU - 1.0) * SCALE) * up.x
                    + ((RR + 0.0) * SCALE) * right.x,
                car_position.y - forward.y * ((FF - 1.25) * SCALE)
                    + ((UU - 1.0) * SCALE) * up.y
                    + ((RR + 0.0) * SCALE) * right.y,
                car_position.z - forward.z * ((FF - 1.25) * SCALE)
                    + ((UU - 1.0) * SCALE) * up.z
                    + ((RR + 0.0) * SCALE) * right.z,
            );
            let posright2: v3<f32> = v3::new(
                car_position.x - forward.x * (40.0 * SCALE) + (UU * SCALE) * up.x + ((RR - 0.0) * SCALE) * right.x,
                car_position.y - forward.y * (40.0 * SCALE)
                    + ((UU - 1.0) * SCALE) * up.y
                    + ((RR - 0.0) * SCALE) * right.y,
                car_position.z - forward.z * (40.0 * SCALE)
                    + ((UU - 1.0) * SCALE) * up.z
                    + ((RR - 0.0) * SCALE) * right.z,
            );
            let rightlightdir: v3<f32> = posright2 - posright1;
            lighting_shader.setVec3(
                c_str!("rightspot.direction"),
                rightlightdir.x,
                rightlightdir.y,
                rightlightdir.z,
            );
            lighting_shader.setFloat(c_str!("rightspot.cutOff"), 0.4);
            lighting_shader.setFloat(c_str!("rightspot.outerCutOff"), 0.6);
            lighting_shader.setVec3(c_str!("rightspot.position"), posright1.x, posright1.y, posright1.z);
            let posleft1: v3<f32> = v3::new(
                car_position.x - forward.x * ((FF - 1.85) * SCALE)
                    + ((UU - 1.0) * SCALE) * up.x
                    + ((RR + 3.0) * SCALE) * right.x,
                car_position.y - forward.y * ((FF - 1.85) * SCALE)
                    + ((UU - 1.0) * SCALE) * up.y
                    + ((RR + 3.0) * SCALE) * right.y,
                car_position.z - forward.z * ((FF - 1.85) * SCALE)
                    + ((UU - 1.0) * SCALE) * up.z
                    + ((RR + 3.0) * SCALE) * right.z,
            );
            let posleft2: v3<f32> = v3::new(
                car_position.x - forward.x * (40.0 * SCALE) + (UU * SCALE) * up.x + ((RR + 3.0) * SCALE) * right.x,
                car_position.y - forward.y * (40.0 * SCALE)
                    + ((UU - 1.0) * SCALE) * up.y
                    + ((RR + 3.0) * SCALE) * right.y,
                car_position.z - forward.z * (40.0 * SCALE)
                    + ((UU - 1.0) * SCALE) * up.z
                    + ((RR + 3.0) * SCALE) * right.z,
            );
            let leftlightdir: v3<f32> = posleft2 - posleft1;
            lighting_shader.setVec3(
                c_str!("leftspot.direction"),
                leftlightdir.x,
                leftlightdir.y,
                leftlightdir.z,
            );
            lighting_shader.setFloat(c_str!("leftspot.cutOff"), 0.4);
            lighting_shader.setFloat(c_str!("leftspot.outerCutOff"), 0.8);
            lighting_shader.setVec3(c_str!("leftspot.colour"), 0.7, 0.7, 0.7);
            lighting_shader.setVec3(c_str!("leftspot.position"), posleft1.x, posleft1.y, posleft1.z);
            let mut sky_view = cgmath::Matrix4::look_at_rh(
                cPoint3::new(0.0, 0.0, 0.0),
                cPoint3::new(vect2.x - vect.x, vect2.y - vect.y, vect2.z - vect.z),
                up,
            );
            if MOUSE {
                sky_view = camera.GetRotationViewMatrix();
            }
            let centre_view = cgmath::Matrix4::look_at_rh(
                cPoint3::new(-car_position.x, -car_position.y, -car_position.z),
                cPoint3::new(
                    -car_position.x + vect2.x - vect.x,
                    -car_position.y + vect2.y - vect.y,
                    -car_position.z + vect2.z - vect.z,
                ),
                up,
            );
            let projection: Matrix4<f32> =
                perspective(Deg(camera.Zoom), SCR_WIDTH as f32 / SCR_HEIGHT as f32, 0.05, VIEWDIST);
            let sky_projection: Matrix4<f32> = perspective(
                Deg(camera.Zoom),
                SCR_WIDTH as f32 / SCR_HEIGHT as f32,
                0.05,
                VIEWDIST * 1.82,
            );
            let _frustum = projection * view;
            let projectiond: Matrix4<f32> =
                perspective(Deg(camera.Zoom), SCR_WIDTH as f32 / SCR_HEIGHT as f32, 0.05, VIEWDIST);
            let frustumd = projectiond * view;
            let duration = timer.elapsed();
            if duration.as_millis() > 10000 {
                timer = Instant::now();
            }
            let _nanos: u128 = duration.as_millis() as u128;
            gl::Viewport(
                0,
                0,
                (SCR_WIDTH * 2_u32.pow(SHADOW_BUFFER_SCALE)) as i32,
                (SCR_HEIGHT * 2_u32.pow(SHADOW_BUFFER_SCALE)) as i32,
            );

            // ************************* SHADOW BUFFER

            gl::BindFramebuffer(gl::FRAMEBUFFER, shadowBuffer);
            gl::ClearColor(light.x, light.y, light.z, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
            let projectionm: Matrix4<f32> = perspective(Deg(70.0), SCR_WIDTH as f32 / SCR_HEIGHT as f32, 1.0, 200.0);
            shadow_buffer_shader.useProgram();
            shadow_buffer_shader.setMat4(c_str!("projection"), &projectionm);
            shadow_buffer_shader.setMat4(c_str!("lightSpaceMatrix"), &lightview);
            shadow_buffer_shader.setMat4(c_str!("model"), &model);
            let frustumm = projectionm * lightview;
            the_tree.draw_terrain(
                &root_node,
                WORLDSIZE as usize,
                frustumm,
                vao,
                cVector3::new(
                    camera.Position.x + (WORLDSIZE / 2) as f32,
                    camera.Position.y,
                    camera.Position.z + (WORLDSIZE / 2) as f32,
                ),
                200.0,
                camera.Zoom,
                0,
            );
            if DRAW_TREES {
                tree_shadow_shader.useProgram();
                tree_shadow_shader.setMat4(c_str!("projection"), &projectionm);
                tree_shadow_shader.setMat4(c_str!("lightSpaceMatrix"), &lightview);
                the_tree.draw_trees(
                    &root_node,
                    WORLDSIZE as usize,
                    frustumm,
                    &tree_model_1,
                    &tree_model_2,
                    &tree_model_3,
                    &tree_model_4,
                    &tree_shadow_shader,
                    cVector3::new(
                        camera.Position.x + (WORLDSIZE / 2) as f32,
                        camera.Position.y,
                        camera.Position.z + (WORLDSIZE / 2) as f32,
                    ),
                    200.0,
                    camera.Zoom,
                );
                gl::Disable(gl::CULL_FACE);
            }
            shadow_buffer_shader.useProgram();
            shadow_buffer_shader.setMat4(c_str!("projection"), &projectionm);
            shadow_buffer_shader.setMat4(c_str!("view"), &lightview);
            shadow_buffer_shader.setVec3(c_str!("car"), car_position.x, car_position.y, car_position.z);
            shadow_buffer_shader.setVec3(
                c_str!("view_position"),
                camera.Position.x,
                camera.Position.y,
                camera.Position.z,
            );
            for the_car in cars.iter() {
                the_car.render(
                    &shadow_buffer_shader,
                    &body_model,
                    &fr_wheel_model,
                    &fl_wheel_model,
                    &rr_wheel_model,
                    &rl_wheel_model,
                    &bodies,
                    &colliders,
                );
            }
            first_car.render(
                &shadow_buffer_shader,
                &body_model,
                &fr_wheel_model,
                &fl_wheel_model,
                &rr_wheel_model,
                &rl_wheel_model,
                &bodies,
                &colliders,
            );
            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
            gl::Viewport(0, 0, SCR_WIDTH as i32, SCR_HEIGHT as i32);

            // ************************* COLOUR BUFFER

            gl::BindFramebuffer(gl::FRAMEBUFFER, g_buffer);
            gl::ClearColor(light.x, 0.5, light.z, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
            sky_shader.useProgram();
            let mut sky_model: Matrix4<f32> = Matrix4::new(
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            );
            sky_model = sky_model * Matrix4::from_translation(vec3(0.0, (-camera.Position.y / 2.0) - 100.0, 0.0));
            sky_shader.setVec2(c_str!("screen_size"), SCR_WIDTH as f32, SCR_HEIGHT as f32);
            sky_shader.setMat4(c_str!("projection"), &sky_projection);
            sky_shader.setMat4(c_str!("view"), &sky_view);
            sky_shader.setMat4(c_str!("centre_view"), &centre_view);
            sky_shader.setMat4(c_str!("model"), &sky_model);
            sky_shader.setVec3(c_str!("viewPos"), 0.0, 0.0, 0.0);
            sky_shader.setVec3(c_str!("car"), car_position.x, car_position.y, car_position.z);
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D, sky_texture);
            gl::BindVertexArray(sky_vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, sky_vbo);
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, sky_ebo);
            gl::VertexAttribPointer(
                0,
                3,
                gl::FLOAT,
                gl::FALSE,
                5 * mem::size_of::<GLfloat>() as GLsizei,
                ptr::null(),
            );
            gl::EnableVertexAttribArray(0);
            gl::VertexAttribPointer(
                1,
                2,
                gl::FLOAT,
                gl::FALSE,
                5 * mem::size_of::<GLfloat>() as GLsizei,
                (3 * mem::size_of::<GLfloat>()) as *const c_void,
            );
            gl::EnableVertexAttribArray(1);
            gl::Disable(gl::CULL_FACE);
            gl::DepthMask(0);
            gl::DrawElements(gl::TRIANGLE_STRIP, sky_indices_size, gl::UNSIGNED_INT, ptr::null());
            gl::DepthMask(1);
            gl::Enable(gl::CULL_FACE);
            terrain_shader.useProgram();
            terrain_shader.setMat4(c_str!("projection"), &projection);
            terrain_shader.setMat4(c_str!("view"), &view);
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D, texture1);
            gl::ActiveTexture(gl::TEXTURE1);
            gl::BindTexture(gl::TEXTURE_2D, texture2);
            gl::ActiveTexture(gl::TEXTURE2);
            gl::BindTexture(gl::TEXTURE_2D, texture3);
            gl::ActiveTexture(gl::TEXTURE3);
            gl::BindTexture(gl::TEXTURE_2D, texture4);
            gl::ActiveTexture(gl::TEXTURE4);
            gl::BindTexture(gl::TEXTURE_2D, texture5);
            gl::ActiveTexture(gl::TEXTURE5);
            gl::BindTexture(gl::TEXTURE_2D, texture6);
            gl::ActiveTexture(gl::TEXTURE6);
            gl::BindTexture(gl::TEXTURE_2D, texture7);
            gl::ActiveTexture(gl::TEXTURE7);
            gl::BindTexture(gl::TEXTURE_2D, texture8);
            gl::ActiveTexture(gl::TEXTURE8);
            terrain_shader.setVec3(
                c_str!("view_position"),
                camera.Position.x,
                camera.Position.y,
                camera.Position.z,
            );
            terrain_shader.setMat4(c_str!("model"), &model);
            NODE_COUNTER = 0;
            TRIANGLE_COUNTER = 0;
            ferret.start();
            the_tree.draw_terrain(
                &root_node,
                WORLDSIZE as usize,
                frustumd,
                vao,
                cVector3::new(
                    camera.Position.x + (WORLDSIZE / 2) as f32,
                    camera.Position.y,
                    camera.Position.z + (WORLDSIZE / 2) as f32,
                ),
                VIEWDIST,
                camera.Zoom,
                0,
            );
            ferret.stop();
            if DRAW_TREES {
                trees_shader.useProgram();
                trees_shader.setMat4(c_str!("projection"), &projection);
                trees_shader.setMat4(c_str!("view"), &view);
                gl::Enable(gl::CULL_FACE);
                the_tree.draw_trees(
                    &root_node,
                    WORLDSIZE as usize,
                    frustumd,
                    &tree_model_1,
                    &tree_model_2,
                    &tree_model_3,
                    &tree_model_4,
                    &trees_shader,
                    cVector3::new(
                        camera.Position.x + (WORLDSIZE / 2) as f32,
                        camera.Position.y,
                        camera.Position.z + (WORLDSIZE / 2) as f32,
                    ),
                    VIEWDIST,
                    camera.Zoom,
                );
                gl::Disable(gl::CULL_FACE);
            }
            gl::ActiveTexture(gl::TEXTURE0);
            object_shader.useProgram();
            object_shader.setMat4(c_str!("projection"), &projection);
            object_shader.setMat4(c_str!("view"), &view);
            object_shader.setVec3(c_str!("car"), car_position.x, car_position.y, car_position.z);
            object_shader.setVec3(
                c_str!("view_position"),
                camera.Position.x,
                camera.Position.y,
                camera.Position.z,
            );
            for the_car in cars.iter() {
                the_car.render(
                    &object_shader,
                    &body_model,
                    &fr_wheel_model,
                    &fl_wheel_model,
                    &rr_wheel_model,
                    &rl_wheel_model,
                    &bodies,
                    &colliders,
                );
            }

            first_car.render(
                &object_shader,
                &body_model,
                &fr_wheel_model,
                &fl_wheel_model,
                &rr_wheel_model,
                &rl_wheel_model,
                &bodies,
                &colliders,
            );
            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

            // ************************* SSAO BUFFER

            gl::BindFramebuffer(gl::FRAMEBUFFER, ssaoFBO);
            gl::Clear(gl::COLOR_BUFFER_BIT);
            ssao_shader.useProgram();
            ssao_shader.setVec3(
                c_str!("camera_position"),
                camera.Position.x,
                camera.Position.y,
                camera.Position.z,
            );
            ssao_shader.setMat4(c_str!("projection"), &projection);
            ssao_shader.setMat4(c_str!("view"), &view);
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D, g_position);
            gl::ActiveTexture(gl::TEXTURE1);
            gl::BindTexture(gl::TEXTURE_2D, g_normal);
            gl::ActiveTexture(gl::TEXTURE2);
            gl::BindTexture(gl::TEXTURE_2D, noiseTexture);
            badger.start();
            renderQuad();
            badger.stop();
            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

            // ************************* SSAO BLUR BUFFER

            gl::BindFramebuffer(gl::FRAMEBUFFER, ssaoBlurFBO);
            gl::Clear(gl::COLOR_BUFFER_BIT);
            ssao_blur_shader.useProgram();
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D, ssaoColorBuffer);
            marmot.start();
            renderQuad();
            marmot.stop();
            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

            // ************************* LIGHTING BUFFER

            gl::BindFramebuffer(gl::FRAMEBUFFER, colourBuffer);
            gl::ClearColor(light.x, light.y, light.z, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
            lighting_shader.useProgram();
            let projectionview = projection * view;
            lighting_shader.setMat4(c_str!("projectionview"), &projectionview);
            lighting_shader.setMat4(c_str!("projection"), &projection);
            lighting_shader.setMat4(c_str!("view"), &view);
            lighting_shader.setMat4(c_str!("lightprojection"), &projectionm);
            lighting_shader.setMat4(c_str!("lightview"), &lightview);
            lighting_shader.setVec3(
                c_str!("viewPos"),
                camera.Position.x,
                camera.Position.y,
                camera.Position.z,
            );
            if lightstime.elapsed().as_secs() % 3 == 1 {
                lighting_shader.setVec3(c_str!("lcol"), 38.0, 2.0, 2.0);
            } else if lightstime.elapsed().as_secs() % 3 == 2 {
                lighting_shader.setVec3(c_str!("lcol"), 20.0, 2.0, 38.0);
            } else {
                lighting_shader.setVec3(c_str!("lcol"), 20.0, 38.0, 2.0);
            }
            lighting_shader.setVec3(c_str!("suncolour"), light.x, light.y, light.z);
            let sun = v3::new(
                ((((3.14159 * 2.0) * (time / 24.0) - 1.5707) as f32).cos()) * 1.0,
                ((((3.14159 * 2.0) * (time / 24.0) - 1.5707) as f32).sin()) * 0.5 + 0.4,
                ((((3.14159 * 2.0) * (time / 24.0) - 1.5707) as f32).sin()) * 1.0,
            );
            // println!("Time:{:.3} sun x {:.2} y {:.2} z {:.2}", time, sun.x, sun.y, sun.z);
            // println!(
            //     "Time:{:.1} red {:.1} green {:.1} blue {:.1}",
            //     time, skyred, skygreen, skyblue
            // );
            lighting_shader.setVec3(c_str!("sunpos"), sun.x, sun.y, sun.z);
            lighting_shader.setFloat(c_str!("near"), 0.05);
            lighting_shader.setFloat(c_str!("far"), VIEWDIST);
            lighting_shader.setFloat(c_str!("f1"), F1);
            lighting_shader.setFloat(c_str!("f2"), F2);
            lighting_shader.setFloat(c_str!("d1"), F1);
            lighting_shader.setInt(c_str!("SSAO"), 1);
            lighting_shader.setFloat(c_str!("d2"), D2);
            lighting_shader.setVec3(c_str!("fogcolour"), light.x, light.y, light.z);
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D, g_position);
            gl::ActiveTexture(gl::TEXTURE1);
            gl::BindTexture(gl::TEXTURE_2D, g_normal);
            gl::ActiveTexture(gl::TEXTURE2);
            gl::BindTexture(gl::TEXTURE_2D, g_albedo);
            gl::ActiveTexture(gl::TEXTURE3);
            gl::BindTexture(gl::TEXTURE_2D, g_fog_position);
            gl::ActiveTexture(gl::TEXTURE4);
            gl::BindTexture(gl::TEXTURE_2D, ground_colour);
            gl::ActiveTexture(gl::TEXTURE6);
            gl::BindTexture(gl::TEXTURE_2D, ssaoColorBufferBlur);
            gl::ActiveTexture(gl::TEXTURE7);
            gl::BindTexture(gl::TEXTURE_2D, shadow);
            beaver.start();
            renderQuad();
            beaver.stop();
            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

            // SHADOW, FOG AND BLOOM BUFFER

            gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
            gl::ClearColor(light.x, light.y, light.z, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
            shadow_fog_bloom_shader.useProgram();
            shadow_fog_bloom_shader.setInt(c_str!("colours"), 0);
            shadow_fog_bloom_shader.setInt(c_str!("shadow"), 1);
            shadow_fog_bloom_shader.setInt(c_str!("g_fog_position"), 2);
            shadow_fog_bloom_shader.setInt(c_str!("g_albedo"), 3);
            shadow_fog_bloom_shader.setInt(c_str!("ssao"), 4);
            shadow_fog_bloom_shader.setVec3(c_str!("fogcolour"), light.x, light.y, light.z);
            shadow_fog_bloom_shader.setVec3(c_str!("suncolour"), light.x, light.y, light.z);
            shadow_fog_bloom_shader.setVec3(c_str!("carpos"), car_position.x, car_position.y, car_position.z);
            shadow_fog_bloom_shader.setFloat(c_str!("near"), 0.05);
            shadow_fog_bloom_shader.setFloat(c_str!("far"), VIEWDIST);
            shadow_fog_bloom_shader.setFloat(c_str!("f1"), F1);
            shadow_fog_bloom_shader.setFloat(c_str!("f2"), F2);
            shadow_fog_bloom_shader.setFloat(c_str!("d1"), F1);
            shadow_fog_bloom_shader.setInt(c_str!("SSAO"), 1);
            shadow_fog_bloom_shader.setFloat(c_str!("d2"), D2);
            let sky_brightness = 0.25 + ((0.299 * skyred + 0.587 * skygreen + 0.114 * skyblue) * 0.75);
            shadow_fog_bloom_shader.setFloat(c_str!("bloom_limit"), sky_brightness);

            shadow_fog_bloom_shader.setVec3(
                c_str!("viewPos"),
                camera.Position.x,
                camera.Position.y,
                camera.Position.z,
            );
            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D, colours[0]);
            gl::ActiveTexture(gl::TEXTURE1);
            gl::BindTexture(gl::TEXTURE_2D, colours[1]);
            gl::ActiveTexture(gl::TEXTURE2);
            gl::BindTexture(gl::TEXTURE_2D, g_fog_position);
            gl::ActiveTexture(gl::TEXTURE3);
            gl::BindTexture(gl::TEXTURE_2D, g_albedo);
            gl::ActiveTexture(gl::TEXTURE4);
            gl::BindTexture(gl::TEXTURE_2D, ssaoColorBufferBlur);

            renderQuad();

            // OUTPUT TO SCREEN

            // gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

            // let attachments = [gl::COLOR_ATTACHMENT0, gl::COLOR_ATTACHMENT1];
            // gl::DrawBuffers(2, &attachments[0]);
            // gl::BindFramebuffer(gl::FRAMEBUFFER, 0);

            // gl::ClearColor(light.x, light.y, light.z, 1.0);
            // gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
            // output_shader.useProgram();
            // output_shader.setInt(c_str!("bloom"), 1);
            // output_shader.setInt(c_str!("light"), 0);
            // gl::ActiveTexture(gl::TEXTURE0);
            // gl::BindTexture(gl::TEXTURE_2D, lightTextures[0]);
            // gl::ActiveTexture(gl::TEXTURE1);
            // gl::BindTexture(gl::TEXTURE_2D, lightTextures[1]);
            // renderQuad();
            // gl::BindFramebuffer(gl::FRAMEBUFFER, 0);
        }
        monkey.start();
        window.swap_buffers();
        monkey.stop();
        donkey.stop();
        frame_count += 1;
        tframe_count += 1;
        if fps_display.elapsed().as_millis() > 500 {
            frame_count = 0;
            fps_display = Instant::now();
            let seconds = tfps_display.elapsed().as_millis();
            if seconds < 5000 {
                donkey.reset();
                rabbit.reset();
                ferret.reset();
                badger.reset();
                marmot.reset();
                beaver.reset();
                monkey.reset();
            }
            if seconds > 0 {
                println!(
                    "x {:.0} z{:.0} time {:.1} {} afps", // {} seconds, {} frames",
                    camera.Position.x,
                    camera.Position.z,
                    time,
                    tframe_count as f32 / (seconds as f32 / 1000.0) as f32,
                    // seconds as f32 / 1000.0,
                    // tframe_count
                );
            }
        }
        glfw.poll_events();
    }
}

fn normalise_heightmap(heights: &mut Array2<f32>, max_height: f32) {
    let mut min_y: f32 = 10000.0;
    let mut max_y: f32 = -10000.0;
    for x in 0..WORLDSIZE as u16 {
        for z in 0..WORLDSIZE as u16 {
            let height: f32 = heights[[x as usize, z as usize]];
            min_y = height.min(min_y);
            max_y = height.max(max_y);
        }
    }
    let range = max_y - min_y;
    let mut n_min_y: f32 = 10000.0;
    let mut n_max_y: f32 = -10000.0;
    for x in 0..(WORLDSIZE as i16) {
        for z in 0..(WORLDSIZE as i16) {
            heights[[x as usize, z as usize]] = ((heights[[x as usize, z as usize]] - min_y) / range) * max_height;
            n_min_y = n_min_y.min(heights[[x as usize, z as usize]]);
            n_max_y = n_max_y.max(heights[[x as usize, z as usize]]);
        }
    }
}

fn get_height(heights: &mut Array2<f32>, mut pos_x: f32, mut pos_z: f32) -> f32 {
    pos_x = pos_x.min((WORLDSIZE - 2) as f32);
    pos_z = pos_z.min((WORLDSIZE - 2) as f32);
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

fn load_texture(name: &str, texture: &mut u32, alpha: bool) {
    unsafe {
        println!("Loading texture {}", name);
        gl::GenTextures(1, &mut *texture);
        gl::BindTexture(gl::TEXTURE_2D, *texture);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::REPEAT as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::REPEAT as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::LINEAR_MIPMAP_LINEAR as i32);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::LINEAR as i32);
        println!("{}", name);
        let img = image::open(&Path::new(name)).expect("Failed to load texture");
        let data = img.raw_pixels();
        if alpha {
            gl::TexImage2D(
                gl::TEXTURE_2D,
                0,
                gl::RGB as i32,
                img.width() as i32,
                img.height() as i32,
                0,
                gl::RGBA,
                gl::UNSIGNED_BYTE,
                &data[0] as *const u8 as *const c_void,
            );
        } else {
            gl::TexImage2D(
                gl::TEXTURE_2D,
                0,
                gl::RGB as i32,
                img.width() as i32,
                img.height() as i32,
                0,
                gl::RGB,
                gl::UNSIGNED_BYTE,
                &data[0] as *const u8 as *const c_void,
            );
        }
        gl::GenerateMipmap(gl::TEXTURE_2D);
    }
}

fn clamp(value: f32) -> f32 {
    value.max(0.0).min(1.0)
}

fn add_trees(
    tree_centres_x: &mut Array2<f32>,
    tree_centres_z: &mut Array2<f32>,
    tree_sizes: &mut Array2<f32>,
    tree_heights: &mut Array2<f32>,
    mut heights: &mut Array2<f32>,
    size: f32,
    subtract: bool,
    x_scale: f32,
    z_scale: f32,
) {
    let mut rng = rand::thread_rng();
    let max_distance = size * x_scale.max(z_scale);
    let mut _tree_count = 0;
    let centre: Vector2<f32> = vec2(rng.gen::<f32>() * WORLDSIZE as f32, rng.gen::<f32>() * WORLDSIZE as f32);
    for x in ((centre.x - size * x_scale) as i32..(centre.x + size * x_scale) as i32).step_by(3) {
        for z in ((centre.y - size * z_scale) as i32..(centre.y + size * z_scale) as i32).step_by(3) {
            if x >= 0 && x < WORLDSIZE && z >= 0 && z < WORLDSIZE {
                if rng.gen::<f32>() > 0.9 {
                    let distance = ((x as f32 - centre.x).powf(2.0) + (z as f32 - centre.y).powf(2.0)).sqrt();
                    if distance <= max_distance {
                        _tree_count += 1;
                        if subtract {
                            tree_heights[[x as usize, z as usize]] = -600.0;
                        } else {
                            let tree_centre_x = rng.gen::<f32>() * 2.5;
                            let tree_centre_z = rng.gen::<f32>() * 2.5;
                            let slope = get_average_slope(heights, x, z, WORLDSIZE - 1);
                            tree_centres_x[[x as usize, z as usize]] = tree_centre_x;
                            tree_centres_z[[x as usize, z as usize]] = tree_centre_z;
                            tree_sizes[[x as usize, z as usize]] = 0.24 + rng.gen::<f32>() * 0.36;
                            let height = get_height(
                                &mut heights,
                                x as f32 + tree_centres_x[[x as usize, z as usize]],
                                z as f32 + tree_centres_z[[x as usize, z as usize]],
                            );
                            if height < TREE_LINE && slope < TREE_MAX_SLOPE {
                                tree_heights[[x as usize, z as usize]] = height;
                            } else {
                                tree_heights[[x as usize, z as usize]] = -600.0;
                            }
                        }
                    }
                }
            }
        }
    }
}

fn add_tree_shadows(
    tree_heights: &mut Array2<f32>,
    tree_centres_x: &mut Array2<f32>,
    tree_centres_z: &mut Array2<f32>,
    tree_shadows: &mut Array2<f32>,
) {
    let max_distance = 5.5;
    for x in 0..WORLDSIZE - 1 {
        for z in 0..WORLDSIZE - 1 {
            if tree_heights[[x as usize, z as usize]] > -600.0 {
                let centre = vec2(
                    x as f32 + tree_centres_x[[x as usize, z as usize]],
                    z as f32 + tree_centres_z[[x as usize, z as usize]],
                );
                for m in (centre.x - max_distance + 1.0) as i32..(centre.x + max_distance + 1.0) as i32 {
                    for n in (centre.y - max_distance + 1.0) as i32..(centre.y + max_distance + 1.0) as i32 {
                        if m >= 0 && m < WORLDSIZE - 1 && n >= 0 && n <= WORLDSIZE - 1 {
                            let distance = ((m as f32 - centre.x).powf(2.0) + (n as f32 - centre.y).powf(2.0)).sqrt();

                            if distance <= max_distance {
                                let shade_amount = distance / max_distance;
                                let tree_shadow = 1.0 - clamp(shade_amount);
                                tree_shadows[[m as usize, n as usize]] =
                                    tree_shadows[[m as usize, n as usize]].max((tree_shadow) as f32);
                            }
                        }
                    }
                }
            }
        }
    }
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

fn add_bump(heights: &mut Array2<f32>, size: f32, sub: bool) {
    let width: f32 = size;
    let height: f32 = width / 10.0;
    let mut rng = rand::thread_rng();

    let centre: cVector3<f64> = vec3(
        rng.gen::<f64>() * (size as f64 + WORLDSIZE as f64),
        rng.gen::<f64>() * (size as f64 + WORLDSIZE as f64),
        rng.gen::<f64>() * (size as f64 + WORLDSIZE as f64),
    );
    let maxdistance: f32 = width / 2.0;
    for x in centre.x as i16 - ((width / 2.0) as i16)..centre.x as i16 + ((width / 2.0) as i16) {
        for z in centre.y as i16 - ((width / 2.0) as i16)..centre.y as i16 + ((width / 2.0) as i16) {
            if x >= 0 && x < WORLDSIZE as i16 && z >= 0 && z < WORLDSIZE as i16 {
                let distance: f32 =
                    ((x as f32 - centre.x as f32).powf(2.0) + (z as f32 - centre.y as f32).powf(2.0)).sqrt();
                if distance <= maxdistance {
                    let pdistance = (distance / maxdistance).powf(2.0);
                    if sub {
                        heights[[x as usize, z as usize]] -= height - (height * pdistance);
                    } else {
                        heights[[x as usize, z as usize]] += height - (height * pdistance);
                    }
                }
            }
        }
    }
}

fn average_height_map(heights: &mut Array2<f32>, average_size: i32, bump_test: bool, add_actual: f32) {
    let mut temp_heights = Array::<f32, _>::zeros((WORLDSIZE as usize, WORLDSIZE as usize));
    let mut rng = rand::thread_rng();

    for z in 0..WORLDSIZE - 1 {
        for x in 0..WORLDSIZE - 1 {
            temp_heights[[x as usize, z as usize]] = heights[[x as usize, z as usize]];
        }
    }

    for x in 0..WORLDSIZE - 1 {
        for z in 0..WORLDSIZE - 1 {
            let mut total_height = 0.0;
            let mut height_count = 0;
            for m in (z as i32 - average_size).max(0)..(z as i32 + average_size).min(WORLDSIZE - 1) {
                for n in (x as i32 - average_size).max(0)..(x as i32 + average_size).min(WORLDSIZE - 1) {
                    total_height += temp_heights[[n as usize, m as usize]];
                    height_count += 1;
                }
            }
            if bump_test {
                heights[[x as usize, z as usize]] = 0.0;
                if rng.gen::<f32>() < 0.001 {
                    heights[[x as usize, z as usize]] = rng.gen::<f32>() * 5.0;
                } else {
                }
            } else {
                heights[[x as usize, z as usize]] = total_height / height_count as f32;
                if add_actual > 0.0 {
                    heights[[x as usize, z as usize]] += (temp_heights[[x as usize, z as usize]] as f32 * add_actual);
                }
            }
        }
        println!("{:.0}%", x as f32 / (WORLDSIZE as f32 - 1.0) * 100.0);
    }
}

fn renderQuad() {
    unsafe {
        if QUAD_VAO == 0 {
            println!("Generate quad buffers");
            let quad_vertices: [f32; 20] = [
                -1.0, 1.0, 0.0, 0.0, 1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, -1.0, 0.0, 1.0, 0.0,
            ];
            // setup plane VAO
            gl::GenVertexArrays(1, &mut QUAD_VAO);
            gl::GenBuffers(1, &mut QUAD_VBO);
            gl::BindVertexArray(QUAD_VAO);
            gl::BindBuffer(gl::ARRAY_BUFFER, QUAD_VBO);
            gl::BufferData(
                gl::ARRAY_BUFFER,
                (quad_vertices.len() as isize * mem::size_of::<GLfloat>() as isize) as GLsizeiptr,
                &quad_vertices[0] as *const f32 as *const c_void,
                gl::STATIC_DRAW,
            );
            gl::EnableVertexAttribArray(0);
            gl::VertexAttribPointer(
                0,
                3,
                gl::FLOAT,
                gl::FALSE,
                5 * mem::size_of::<GLfloat>() as GLsizei,
                ptr::null(),
            );
            gl::EnableVertexAttribArray(1);
            gl::VertexAttribPointer(
                1,
                2,
                gl::FLOAT,
                gl::FALSE,
                5 * mem::size_of::<GLfloat>() as i32,
                (3 * mem::size_of::<GLfloat>()) as *const c_void,
            );
        }
        gl::BindVertexArray(QUAD_VAO);
        gl::DrawArrays(gl::TRIANGLE_STRIP, 0, 4);
        gl::BindVertexArray(0);
    }
}
