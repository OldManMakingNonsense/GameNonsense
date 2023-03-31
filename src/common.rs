#![allow(non_snake_case)]
#![allow(dead_code)]
/// Common code that the original tutorials repeat over and over and over and over
use std::os::raw::c_void;

use std::path::Path;
use std::sync::mpsc::Receiver;

use crate::gl;
extern crate glfw;
use self::glfw::{Action, Key};

use crate::image;
use image::DynamicImage::*;
use image::GenericImage;

use crate::camera::Camera;
// use crate::camera::Camera_Movement::*;
// use crate::vehicle::Vehicle;

// use nphysics3d::object::DefaultBodySet;
/// Event processing function as introduced in 1.7.4 (Camera Class) and used in
/// most later tutorials
pub fn process_events(
    events: &Receiver<(f64, glfw::WindowEvent)>,
    firstMouse: &mut bool,
    lastX: &mut f32,
    lastY: &mut f32,
    camera: &mut Camera,
    camera_rot: &mut f32,
    camera_rot_2: &mut f32,
) {
    for (_, event) in glfw::flush_messages(events) {
        match event {
            glfw::WindowEvent::FramebufferSize(width, height) => {
                // make sure the viewport matches the new window dimensions; note that width and
                // height will be significantly larger than specified on retina displays.
                unsafe { gl::Viewport(0, 0, width, height) }
            }
            glfw::WindowEvent::CursorPos(xpos, ypos) => {
                let (xpos, ypos) = (xpos as f32, ypos as f32);
                if *firstMouse {
                    *lastX = xpos;
                    *lastY = ypos;
                    *firstMouse = false;
                }

                let xoffset = xpos - *lastX;
                let yoffset = *lastY - ypos; // reversed since y-coordinates go from bottom to top

                *lastX = xpos;
                *lastY = ypos;

                *camera_rot -= xoffset / 150.0;
                *camera_rot_2 -= yoffset / 150.0;
                *camera_rot_2 = (camera_rot_2.min(1.7)).max(0.0);

                camera.ProcessMouseMovement(xoffset, yoffset, true);
            }
            glfw::WindowEvent::Scroll(_xoffset, yoffset) => {
                camera.ProcessMouseScroll(yoffset as f32);
            }
            _ => {}
        }
    }
}

/// Input processing function as introduced in 1.7.4 (Camera Class) and used in
/// most later tutorials
pub fn processInput(
    window: &mut glfw::Window,
    deltaTime: f32,
    camera: &mut Camera,
    night_mode: bool,
    turny: &mut f32,
    spinny: &mut f32,
    camera_rot: &mut f32,
) -> bool {
    if window.get_key(Key::Escape) == Action::Press {
        window.set_should_close(true)
    }
    // if window.get_key(Key::W) == Action::Press {
    //     camera.ProcessKeyboard(FORWARD, deltaTime);
    // }
    // if window.get_key(Key::S) == Action::Press {
    //     camera.ProcessKeyboard(BACKWARD, deltaTime);
    // }
    // if window.get_key(Key::A) == Action::Press {
    //     camera.ProcessKeyboard(LEFT, deltaTime);
    // }
    // if window.get_key(Key::D) == Action::Press {
    //     camera.ProcessKeyboard(RIGHT, deltaTime);
    // }
    let mut mode = night_mode;
    if window.get_key(Key::N) == Action::Press {
        mode = true; // - nightmode;
    }
    if window.get_key(Key::M) == Action::Press {
        mode = false;
    }
    if window.get_key(Key::T) == Action::Press {
        camera.ChangeElevation(deltaTime);
    }
    if window.get_key(Key::G) == Action::Press {
        camera.ChangeElevation(-deltaTime);
    }

    let turny_force = 2.5 * ((1.0 - (*spinny / 12.5).abs()).max(0.0)) + 0.2;
    if window.get_key(Key::D) == Action::Press {
        *turny -= turny_force * deltaTime;
        *turny = turny.max(-0.30);
    } else if window.get_key(Key::A) == Action::Press {
        *turny += turny_force * deltaTime;
        *turny = turny.min(0.30);
    } else {
        *turny *= 1.0 - 2.0 * deltaTime;
    }
    if window.get_key(Key::W) == Action::Press {
        *spinny -= 20.0 * deltaTime;
        *spinny = spinny.max(-15.2);
    } else if window.get_key(Key::S) == Action::Press {
        *spinny += 20.0 * deltaTime;
        *spinny = spinny.min(15.2);
    }
    *spinny *= 1.0 - 1.0 * deltaTime;

    if window.get_key(Key::K) == Action::Press {
        *camera_rot -= deltaTime;
    }
    if window.get_key(Key::L) == Action::Press {
        *camera_rot += deltaTime;
    }
    // println!("m {}", spinny);
    mode
}
/// utility function for loading a 2D texture from file
/// ---------------------------------------------------
#[allow(dead_code)]
pub unsafe fn loadTexture(path: &str) -> u32 {
    let mut textureID = 0;

    gl::GenTextures(1, &mut textureID);
    let img = image::open(&Path::new(path)).expect("Texture failed to load");
    let format = match img {
        ImageLuma8(_) => gl::RED,
        ImageLumaA8(_) => gl::RG,
        ImageRgb8(_) => gl::RGB,
        ImageRgba8(_) => gl::RGBA,
    };

    let data = img.raw_pixels();

    gl::BindTexture(gl::TEXTURE_2D, textureID);
    gl::TexImage2D(
        gl::TEXTURE_2D,
        0,
        format as i32,
        img.width() as i32,
        img.height() as i32,
        0,
        format,
        gl::UNSIGNED_BYTE,
        &data[0] as *const u8 as *const c_void,
    );
    gl::GenerateMipmap(gl::TEXTURE_2D);

    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::REPEAT as i32);
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::REPEAT as i32);
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::LINEAR as i32);
    gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::LINEAR_MIPMAP_LINEAR as i32);

    textureID
}
