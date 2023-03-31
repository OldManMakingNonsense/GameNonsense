#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(dead_code)]
extern crate nphysics3d;

use cgmath;
use cgmath::prelude::*;
use cgmath::vec3;

type Point3 = cgmath::Point3<f32>;
type Vector3 = cgmath::Vector3<f32>;
type Matris4 = cgmath::Matrix4<f32>;

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
#[derive(PartialEq, Clone, Copy)]
pub enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
}
use self::Camera_Movement::*;

// Default camera values
const YAW: f32 = -90.0;
const PITCH: f32 = 0.0;
const SPEED: f32 = 3.0;
const SENSITIVTY: f32 = 0.1;
const ZOOM: f32 = 40.0;

pub struct Camera {
    // Camera Attributes
    pub Position: Point3,
    pub Front: Vector3,
    pub Up: Vector3,
    pub Right: Vector3,
    pub WorldUp: Vector3,
    // Euler Angles
    pub Yaw: f32,
    pub Pitch: f32,
    // Camera options
    pub MovementSpeed: f32,
    pub MouseSensitivity: f32,
    pub Zoom: f32,
    pub Elevation: f32,
}

impl Default for Camera {
    fn default() -> Camera {
        let mut camera = Camera {
            Position: Point3::new(0.0, 0.0, 0.0),
            Front: vec3(0.0, 0.0, -1.0),
            Up: Vector3::zero(),    // initialized later
            Right: Vector3::zero(), // initialized later
            WorldUp: Vector3::unit_y(),
            Yaw: YAW,
            Pitch: PITCH,
            MovementSpeed: SPEED,
            MouseSensitivity: SENSITIVTY,
            Zoom: ZOOM,
            Elevation: 2.25,
        };
        camera.updateCameraVectors();
        camera
    }
}

impl Camera {
    /// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
    pub fn GetViewMatrix(&self) -> Matris4 {
        Matris4::look_at_rh(
            self.Position + vec3(0.0, self.Elevation, 0.0),
            self.Position + vec3(0.0, self.Elevation, 0.0) + self.Front,
            self.Up,
        )
    }

    pub fn GetRotationViewMatrix(&self) -> Matris4 {
        Matris4::look_at_rh(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0) + self.Front,
            self.Up,
        )
    }

    /// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    pub fn ProcessKeyboard(&mut self, direction: Camera_Movement, deltaTime: f32) {
        let velocity = self.MovementSpeed * deltaTime;
        if direction == FORWARD {
            self.Position += self.Front * velocity;
        }
        if direction == BACKWARD {
            self.Position += -(self.Front * velocity);
        }
        if direction == LEFT {
            self.Position += -(self.Right * velocity);
        }
        if direction == RIGHT {
            self.Position += self.Right * velocity;
        }
    }

    pub fn ChangeElevation(&mut self, deltaTime: f32) {
        self.Elevation += deltaTime * SPEED;
        self.Elevation = self.Elevation.min(1000.0);
        self.Elevation = self.Elevation.max(0.25);
    }

    /// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    pub fn ProcessMouseMovement(&mut self, mut xoffset: f32, mut yoffset: f32, constrainPitch: bool) {
        xoffset *= self.MouseSensitivity;
        yoffset *= self.MouseSensitivity;

        self.Yaw += xoffset;
        self.Pitch += yoffset;

        // Make sure that when pitch is out of bounds, screen doesn't get flipped
        if constrainPitch {
            if self.Pitch > 89.0 {
                self.Pitch = 89.0;
            }
            if self.Pitch < -89.0 {
                self.Pitch = -89.0;
            }
        }

        // Update Front, Right and Up Vectors using the updated Eular angles
        self.updateCameraVectors();
    }

    // Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    pub fn ProcessMouseScroll(&mut self, yoffset: f32) {
        if self.Zoom >= 0.1 && self.Zoom <= 40.0 {
            self.Zoom -= yoffset;
        }
        if self.Zoom <= 0.1 {
            self.Zoom = 0.1;
        }
        if self.Zoom >= 40.0 {
            self.Zoom = 40.0;
        }
    }

    /// Calculates the front vector from the Camera's (updated) Euler Angles
    fn updateCameraVectors(&mut self) {
        // Calculate the new Front vector
        let front = Vector3 {
            x: self.Yaw.to_radians().cos() * self.Pitch.to_radians().cos(),
            y: self.Pitch.to_radians().sin(),
            z: self.Yaw.to_radians().sin() * self.Pitch.to_radians().cos(),
        };
        self.Front = front.normalize();
        // Also re-calculate the Right and Up vector
        self.Right = self.Front.cross(self.WorldUp).normalize(); // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
        self.Up = self.Right.cross(self.Front).normalize();
    }

    pub fn setPitchYaw(&mut self, p: f32, y: f32) {
        self.Pitch = p;
        self.Yaw = y;
        self.updateCameraVectors();
    }

    pub fn Front(&mut self, v: Vector3) {
        self.Front = v.normalize();
        // Also re-calculate the Right and Up vector
        self.Right = self.Front.cross(self.WorldUp).normalize(); // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
        self.Up = self.Right.cross(self.Front).normalize();
        self.updateCameraVectors();
    }
}
