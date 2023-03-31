extern crate nalgebra as na;

use crate::model::Model;
use crate::shader::Shader;
use std::f32;

use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::joint::{FixedJoint, FreeJoint, PrismaticJoint, RevoluteJoint};
use nphysics3d::nalgebra::Isometry3 as i3;
use nphysics3d::nalgebra::Point3 as p3;
use nphysics3d::nalgebra::Vector3 as v3;
use nphysics3d::ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::object::{
    Body, BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet,
    Multibody, MultibodyDesc, MultibodyLink,
};
// use nphysics3d::volumetric::Volumetric;
use std::ffi::CStr;

const SCALE: f32 = 0.3333;
const WHEEL_RADIUS: f32 = 1.3;
const MOTOR_SPEED: f32 = -30.0;
const MOTOR_TORQUE: f32 = 10.0;
const STEER_SPEED: f32 = 0.2;
const STEER_TORQUE: f32 = 88.0;
const BRAKING_TORQUE: f32 = 0.0000001;
const SPRING_SPEED: f32 = -20.0;
const SPRING_FORCE: f32 = 1.0;
const SPRING_TOP: f32 = 0.0;
const SPRING_BOTTOM: f32 = -0.6;
const AXIS_OFFSET: f32 = 0.75;
const AXIS_WIDTH: f32 = 2.6;
const SPRING_MULT: f32 = 1.2;
const COMPRESSION_FORCE: f32 = 0.0;
const OFFSET_SCALE: f32 = 5.0;
const FRONT_AXIS_POSITION: f32 = 6.5;
const MIDDLE_AXIS_POSITION: f32 = -2.0;
const REAR_AXIS_POSITION: f32 = -5.5;
pub struct Vehicle {
    pub multibody_handle: Option<DefaultBodyHandle>,
    pub ch_body_handle: Option<DefaultBodyHandle>,
    pub nfw_body_handle: Option<DefaultBodyHandle>,
    pub ofw_body_handle: Option<DefaultBodyHandle>,
    pub nrw_body_handle: Option<DefaultBodyHandle>,
    pub orw_body_handle: Option<DefaultBodyHandle>,
    pub collider_handles: Vec<DefaultColliderHandle>,
    pub w1_steer: Option<MultibodyLink<f32>>,
    pub w2_steer: Option<MultibodyLink<f32>>,
    pub w3_steer: Option<MultibodyLink<f32>>,
    pub w4_steer: Option<MultibodyLink<f32>>,
    pub multibody: Option<Multibody<f32>>,
    pub steer_angle: f32,
}
impl Vehicle {
    pub fn new() -> Vehicle {
        Vehicle {
            multibody_handle: None,
            ch_body_handle: None,
            nfw_body_handle: None,
            ofw_body_handle: None,
            nrw_body_handle: None,
            orw_body_handle: None,
            collider_handles: vec![],
            w1_steer: None,
            w2_steer: None,
            w3_steer: None,
            w4_steer: None,
            multibody: None,
            steer_angle: 0.0,
        }
    }
    pub fn remove(&mut self, bodies: &mut DefaultBodySet<f32>, colliders: &mut DefaultColliderSet<f32>) {
        bodies.remove(self.multibody_handle.unwrap());
        for collider_handle in &self.collider_handles {
            colliders.remove(*collider_handle);
        }
        self.collider_handles.drain(..);
    }
    pub fn build(
        &mut self,
        bodies: &mut DefaultBodySet<f32>,
        colliders: &mut DefaultColliderSet<f32>,
        _joints: &mut DefaultJointConstraintSet<f32>,
        start_x: f32,
        start_y: f32,
        start_z: f32,
    ) {
        let body_shape = Cuboid::new(nphysics3d::nalgebra::Vector3::new(
            1.25 * SCALE,
            1.0 * SCALE,
            9.0 * SCALE,
        ));
        let body = ShapeHandle::new(body_shape);
        let body_desc = ColliderDesc::new(body.clone()).density(0.0001);
        let chassis = ShapeHandle::new(Cuboid::new(nphysics3d::nalgebra::Vector3::new(
            1.05 * SCALE,
            0.5 * SCALE,
            9.0 * SCALE,
        )));
        let chassis_desc = ColliderDesc::new(chassis.clone()).density(35.0);

        let wheel = ShapeHandle::new(Ball::new(WHEEL_RADIUS * SCALE));
        let wheel_desc = ColliderDesc::new(wheel.clone()).density(1.0);

        let fixed: FixedJoint<f32> = FixedJoint::new(i3::translation(0.0, -3.25 * SCALE, 0.0));
        let revo = RevoluteJoint::new(v3::x_axis(), 0.0);
        let mut front = RevoluteJoint::new(v3::y_axis(), 0.02 * SCALE);
        front.enable_max_angle(0.30);
        front.enable_min_angle(-0.30);

        let mut prism = PrismaticJoint::new(v3::y_axis(), 0.0);
        prism.enable_max_offset(SPRING_TOP * SCALE);
        prism.enable_min_offset(SPRING_BOTTOM * SCALE);
        let free = FreeJoint::new(i3::translation(start_x, start_y, start_z));
        // let body_shift = v3::new(0.0, 0.0, 0.0);

        let mut multibody_desc = MultibodyDesc::new(free);
        // multibody_desc
        // .body_shift(body_shift)
        // .set_local_center_of_mass(p3::new(50.0, 12.25, 0.0));
        // .set_local_center_of_mass(p3::new(0.0, 5.0, 0.0));

        let w1_spring = multibody_desc.add_child(prism).set_body_shift(v3::new(
            AXIS_WIDTH * SCALE,
            AXIS_OFFSET * SCALE,
            FRONT_AXIS_POSITION * SCALE,
        ));
        let w1_steer = w1_spring.add_child(front).set_body_shift(v3::new(0.0, 0.0, 0.0));
        let _w1_hub = w1_steer.add_child(revo).set_body_shift(v3::new(0.0, 0.0, 0.0));

        let w2_spring = multibody_desc.add_child(prism).set_body_shift(v3::new(
            -AXIS_WIDTH * SCALE,
            AXIS_OFFSET * SCALE,
            FRONT_AXIS_POSITION * SCALE,
        ));
        let w2_steer = w2_spring.add_child(front).set_body_shift(v3::new(0.0, 0.0, 0.0));
        let _w2_hub = w2_steer.add_child(revo).set_body_shift(v3::new(0.0, 0.0, 0.0));

        let w3_spring = multibody_desc.add_child(prism).set_body_shift(v3::new(
            AXIS_WIDTH * SCALE,
            AXIS_OFFSET * SCALE,
            REAR_AXIS_POSITION * SCALE,
        ));
        let w3_steer = w3_spring.add_child(front).set_body_shift(v3::new(0.0, 0.0, 0.0));
        let _w3_hub = w3_steer.add_child(revo).set_body_shift(v3::new(0.0, 0.0, 0.0));

        let w4_spring = multibody_desc.add_child(prism).set_body_shift(v3::new(
            -AXIS_WIDTH * SCALE,
            AXIS_OFFSET * SCALE,
            REAR_AXIS_POSITION * SCALE,
        ));
        let w4_steer = w4_spring.add_child(front).set_body_shift(v3::new(0.0, 0.0, 0.0));
        let _w4_hub = w4_steer.add_child(revo).set_body_shift(v3::new(0.0, 0.0, 0.0));

        let w5_spring = multibody_desc.add_child(prism).set_body_shift(v3::new(
            AXIS_WIDTH * SCALE,
            AXIS_OFFSET * SCALE,
            MIDDLE_AXIS_POSITION * SCALE,
        ));
        let _w5_hub = w5_spring.add_child(revo).set_body_shift(v3::new(0.0, 0.0, 0.0));

        let w6_spring = multibody_desc.add_child(prism).set_body_shift(v3::new(
            -AXIS_WIDTH * SCALE,
            AXIS_OFFSET * SCALE,
            MIDDLE_AXIS_POSITION * SCALE,
        ));
        let _w6_hub = w6_spring.add_child(revo).set_body_shift(v3::new(0.0, 0.0, 0.0));

        let _fixed_body = multibody_desc
            .add_child(fixed)
            .set_body_shift(v3::new(0.0, 0.0, 0.0))
            .set_local_center_of_mass(p3::new(0.0, 12.25 * SCALE, 0.0));

        let mut multibody: Multibody<f32> = multibody_desc.build();
        multibody.damping_mut().fill(682.0 / 9.0);
        self.multibody_handle = Some(bodies.insert(multibody));

        let body_collider_handle = chassis_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 0));
        self.collider_handles.push(colliders.insert(body_collider_handle));

        let co = wheel_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 3));
        self.collider_handles.push(colliders.insert(co));

        let co = wheel_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 6));
        self.collider_handles.push(colliders.insert(co));

        let co = wheel_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 9));
        self.collider_handles.push(colliders.insert(co));

        let co = wheel_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 12));
        self.collider_handles.push(colliders.insert(co));

        let co = wheel_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 14));
        self.collider_handles.push(colliders.insert(co));

        let co = wheel_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 16));
        self.collider_handles.push(colliders.insert(co));

        let co = body_desc.build(BodyPartHandle(self.multibody_handle.unwrap(), 17));
        self.collider_handles.push(colliders.insert(co));

        // let co = body_desc.build(Bo
        // dyPartHandle(self.multibody_handle.unwrap(), 17));
        // self.collider_handles.push(colliders.insert(co));

        // let collider = colliders.get(body_collider_handle).unwrap();
        // let body_handle = body_collider_handle.body();

        // bodies.get()
    }

    pub fn render(
        &self,
        shader: &Shader,
        body_model: &Model,
        fr_wheel_model: &Model,
        fl_wheel_model: &Model,
        rr_wheel_model: &Model,
        rl_wheel_model: &Model,
        _bodies: &DefaultBodySet<f32>,
        colliders: &DefaultColliderSet<f32>,
    ) {
        unsafe {
            let mut count = 0;
            for collider_handle in &self.collider_handles {
                let collider = colliders.get(*collider_handle).unwrap();
                let position = collider.position().translation.vector;
                let position = cgmath::Vector3::new(position.x, position.y, position.z);
                let q = collider.position().rotation;
                let nq = cgmath::Quaternion::new(q.w, q.i, q.j, q.k);
                let rot_matrix = cgmath::Matrix4::from(nq);
                let trans_matrix = cgmath::Matrix4::from_translation(position);
                // let raise_body_matrix = cgmath::Matrix4::from_translation(cgmath::Vector3::new(0.0, 5.0, 0.0));
                let player_matrix = trans_matrix * rot_matrix;
                shader.setMat4(c_str!("model"), &player_matrix);
                count += 1;
                if count % 8 == 1 {
                    body_model.Draw(&shader);
                } else if count % 8 == 2 {
                    fr_wheel_model.Draw(&shader);
                } else if count % 8 == 3 {
                    fl_wheel_model.Draw(&shader);
                } else if count % 8 == 5 {
                    rl_wheel_model.Draw(&shader);
                } else if count % 8 == 4 {
                    rr_wheel_model.Draw(&shader);
                } else if count % 8 == 7 {
                    rl_wheel_model.Draw(&shader);
                } else if count % 8 == 6 {
                    rr_wheel_model.Draw(&shader);
                    // } else if count % 8 == 0 {
                    //     rr_wheel_model.Draw(&shader);
                }
            }
        }
    }

    pub fn springs(&self, bodies: &mut DefaultBodySet<f32>, force: f32) {
        bodies.multibody_mut(self.multibody_handle.unwrap()).unwrap().activate();
        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(1));
        if let Some(prismatic) = link {
            let joint = prismatic.joint_mut().downcast_mut::<PrismaticJoint<f32>>().unwrap();
            joint.enable_linear_motor();
            joint.set_desired_linear_motor_velocity(SPRING_SPEED);
            let joint_position =
                1.0 + ((joint.offset() - SPRING_BOTTOM * SCALE) / (SPRING_TOP * SCALE - SPRING_BOTTOM * SCALE));
            joint.set_max_linear_motor_force(joint_position.powf(3.0) * force);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(4));
        if let Some(prismatic) = link {
            let joint = prismatic.joint_mut().downcast_mut::<PrismaticJoint<f32>>().unwrap();
            joint.enable_linear_motor();
            joint.set_desired_linear_motor_velocity(SPRING_SPEED);
            let joint_position =
                1.0 + ((joint.offset() - SPRING_BOTTOM * SCALE) / (SPRING_TOP * SCALE - SPRING_BOTTOM * SCALE));
            joint.set_max_linear_motor_force(joint_position.powf(3.0) * force);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(7));
        if let Some(prismatic) = link {
            let joint = prismatic.joint_mut().downcast_mut::<PrismaticJoint<f32>>().unwrap();
            joint.enable_linear_motor();
            joint.set_desired_linear_motor_velocity(SPRING_SPEED);
            let joint_position =
                1.0 + ((joint.offset() - SPRING_BOTTOM * SCALE) / (SPRING_TOP * SCALE - SPRING_BOTTOM * SCALE));
            joint.set_max_linear_motor_force(joint_position.powf(3.0) * force);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(10));
        if let Some(prismatic) = link {
            let joint = prismatic.joint_mut().downcast_mut::<PrismaticJoint<f32>>().unwrap();
            joint.enable_linear_motor();
            joint.set_desired_linear_motor_velocity(SPRING_SPEED);
            let joint_position =
                1.0 + ((joint.offset() - SPRING_BOTTOM * SCALE) / (SPRING_TOP * SCALE - SPRING_BOTTOM * SCALE));
            joint.set_max_linear_motor_force(joint_position.powf(3.0) * force);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(13));
        if let Some(prismatic) = link {
            let joint = prismatic.joint_mut().downcast_mut::<PrismaticJoint<f32>>().unwrap();
            joint.enable_linear_motor();
            joint.set_desired_linear_motor_velocity(SPRING_SPEED);
            let joint_position =
                1.0 + ((joint.offset() - SPRING_BOTTOM * SCALE) / (SPRING_TOP * SCALE - SPRING_BOTTOM * SCALE));
            joint.set_max_linear_motor_force(joint_position.powf(3.0) * force);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(15));
        if let Some(prismatic) = link {
            let joint = prismatic.joint_mut().downcast_mut::<PrismaticJoint<f32>>().unwrap();
            joint.enable_linear_motor();
            joint.set_desired_linear_motor_velocity(SPRING_SPEED);
            let joint_position =
                1.0 + ((joint.offset() - SPRING_BOTTOM * SCALE) / (SPRING_TOP * SCALE - SPRING_BOTTOM * SCALE));
            joint.set_max_linear_motor_force(joint_position.powf(3.0) * force);
        }
    }

    pub fn steering(&self, bodies: &mut DefaultBodySet<f32>, angle: &mut f32) {
        // println!("{}", angle);
        bodies.multibody_mut(self.multibody_handle.unwrap()).unwrap().activate();
        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(2));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            // println!("Joint angle {} and desired angle is {}", joint.angle(), *angle);
            let how_close = (joint.angle() - *angle).abs();

            if how_close + &angle.abs() < 0.02 {
                // println!("Stopped");
                joint.disable_angular_motor();
                joint.enable_max_angle(0.0);
                joint.enable_min_angle(-0.0);
            } else {
                // println!("Started");
                joint.enable_angular_motor();
                joint.enable_max_angle(0.35);
                joint.enable_min_angle(-0.35);
                if joint.angle() < *angle {
                    joint.set_desired_angular_motor_velocity(STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                } else {
                    joint.set_desired_angular_motor_velocity(-STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                }
            }
        }

        bodies.multibody_mut(self.multibody_handle.unwrap()).unwrap().activate();
        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(5));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            let how_close = (joint.angle() - *angle).abs();
            if how_close + &angle.abs() < 0.02 {
                joint.disable_angular_motor();
                joint.enable_max_angle(0.0);
                joint.enable_min_angle(-0.0);
            } else {
                joint.enable_angular_motor();
                joint.enable_max_angle(0.35);
                joint.enable_min_angle(-0.35);
                if joint.angle() < *angle {
                    joint.set_desired_angular_motor_velocity(STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                } else {
                    joint.set_desired_angular_motor_velocity(-STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                }
            }
        }

        let opposite = *angle * -1.0;
        bodies.multibody_mut(self.multibody_handle.unwrap()).unwrap().activate();
        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(8));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            let how_close = (joint.angle() - (opposite)).abs();
            if how_close + &angle.abs() < 0.02 {
                joint.disable_angular_motor();
                joint.enable_max_angle(0.0);
                joint.enable_min_angle(-0.0);
            } else {
                joint.enable_angular_motor();
                joint.enable_max_angle(0.35);
                joint.enable_min_angle(-0.35);
                if joint.angle() < opposite {
                    joint.set_desired_angular_motor_velocity(STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                } else {
                    joint.set_desired_angular_motor_velocity(-STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                }
            }
        }

        bodies.multibody_mut(self.multibody_handle.unwrap()).unwrap().activate();
        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(11));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            let how_close = (joint.angle() - (opposite)).abs();
            if how_close + &angle.abs() < 0.02 {
                joint.disable_angular_motor();
                joint.enable_max_angle(0.0);
                joint.enable_min_angle(-0.0);
            } else {
                joint.enable_angular_motor();
                joint.enable_max_angle(0.35);
                joint.enable_min_angle(-0.35);
                if joint.angle() < opposite {
                    joint.set_desired_angular_motor_velocity(STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                } else {
                    joint.set_desired_angular_motor_velocity(-STEER_SPEED);
                    joint.set_max_angular_motor_torque(STEER_TORQUE);
                }
            }
        }
    }

    pub fn motors(&self, bodies: &mut DefaultBodySet<f32>, motor: f32) {
        bodies.multibody_mut(self.multibody_handle.unwrap()).unwrap().activate();
        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(3));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            joint.enable_angular_motor();
            joint.set_desired_angular_motor_velocity(motor);
            joint.set_max_angular_motor_torque(MOTOR_TORQUE);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(6));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            joint.enable_angular_motor();
            joint.set_desired_angular_motor_velocity(motor);
            joint.set_max_angular_motor_torque(MOTOR_TORQUE);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(9));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            joint.enable_angular_motor();
            joint.set_desired_angular_motor_velocity(motor);
            joint.set_max_angular_motor_torque(MOTOR_TORQUE);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(12));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            joint.enable_angular_motor();
            joint.set_desired_angular_motor_velocity(motor);
            joint.set_max_angular_motor_torque(MOTOR_TORQUE);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(14));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            joint.enable_angular_motor();
            joint.set_desired_angular_motor_velocity(motor);
            joint.set_max_angular_motor_torque(MOTOR_TORQUE);
        }

        let link = bodies
            .multibody_mut(self.multibody_handle.unwrap())
            .and_then(|mb| mb.link_mut(16));
        if let Some(revolute) = link {
            let joint = revolute.joint_mut().downcast_mut::<RevoluteJoint<f32>>().unwrap();
            joint.enable_angular_motor();
            joint.set_desired_angular_motor_velocity(motor);
            joint.set_max_angular_motor_torque(MOTOR_TORQUE);
        }
    }
}
