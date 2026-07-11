//! Create a human ragdoll. Port of `CreateHuman` from `shared/human.c`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    BoneId, Human, BONE_COUNT, COLOR_DODGER_BLUE, COLOR_LIGHT_YELLOW, COLOR_MEDIUM_TURQUOISE,
    COLOR_NAVAJO_WHITE, COLOR_PERU, COLOR_TAN, FILTER_JOINT_COUNT,
};
use crate::body::create_body;
use crate::geometry::Capsule;
use crate::id::{NULL_BODY_ID, NULL_JOINT_ID};
use crate::joint::{create_filter_joint, create_revolute_joint, create_spherical_joint, JointType};
use crate::math_functions::{
    normalize_quat, offset_pos, Pos, Quat, Transform, Vec2, Vec3, DEG_TO_RAD,
};
use crate::shape::create_capsule_shape;
use crate::types::{
    default_body_def, default_filter_joint_def, default_revolute_joint_def, default_shape_def,
    default_spherical_joint_def, BodyType,
};
use crate::world::World;

/// Build a `Transform` from C aggregate initializer order `{p, {q.v, q.s}}`.
#[inline]
fn xf(px: f32, py: f32, pz: f32, qx: f32, qy: f32, qz: f32, qs: f32) -> Transform {
    Transform {
        p: Vec3 {
            x: px,
            y: py,
            z: pz,
        },
        q: Quat {
            v: Vec3 {
                x: qx,
                y: qy,
                z: qz,
            },
            s: qs,
        },
    }
}

/// Spawn a capsule-bone ragdoll with spherical / revolute joint tree.
/// (CreateHuman)
pub fn create_human(
    human: &mut Human,
    world: &mut World,
    position: Pos,
    friction_torque: f32,
    hertz: f32,
    damping_ratio: f32,
    group_index: i32,
    user_data: u64,
    colorize: bool,
) {
    debug_assert!(!human.is_spawned);

    for i in 0..BONE_COUNT {
        human.bones[i].body_id = NULL_BODY_ID;
        human.bones[i].anchor_id = NULL_BODY_ID;
        human.bones[i].joint_id = NULL_JOINT_ID;
        human.bones[i].joint_friction = 1.0;
        human.bones[i].parent_index = -1;
    }

    for i in 0..FILTER_JOINT_COUNT {
        human.filter_joints[i] = NULL_JOINT_ID;
    }

    human.friction_torque = friction_torque;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.user_data = user_data;

    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = 0.2;

    let shirt_color = COLOR_MEDIUM_TURQUOISE;
    let pant_color = COLOR_DODGER_BLUE;
    let skin_colors = [
        COLOR_NAVAJO_WHITE,
        COLOR_LIGHT_YELLOW,
        COLOR_PERU,
        COLOR_TAN,
    ];
    let skin_color = skin_colors[(group_index.rem_euclid(4)) as usize];

    // pelvis
    {
        let bone = &mut human.bones[BoneId::Pelvis as usize];
        bone.parent_index = -1;
        body_def.name = "pelvis".to_string();
        bone.reference_frame = xf(0.0, 0.932087, -0.051708, 0.739169, 0.0, 0.0, 0.673520);
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, &body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.07,
                y: 0.0,
                z: -0.08,
            },
            center2: Vec3 {
                x: -0.07,
                y: 0.0,
                z: -0.08,
            },
            radius: 0.13,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { pant_color } else { 0 };
        create_capsule_shape(world, bone.body_id, &shape_def, &capsule);
    }

    // spine_01
    {
        let bone = &mut human.bones[BoneId::Spine01 as usize];
        bone.parent_index = BoneId::Pelvis as i32;
        body_def.name = "spine_01".to_string();
        bone.reference_frame = xf(0.0, 1.113505, -0.03481, 0.739973, 0.0, 0.0, 0.672637);
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, &body_def);
        body_def.type_ = BodyType::Dynamic;

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.06,
                y: -0.0,
                z: -0.052264,
            },
            center2: Vec3 {
                x: -0.06,
                y: 0.0,
                z: -0.052264,
            },
            radius: 0.12,
        };
        shape_def.filter.group_index = -group_index;
        shape_def.base_material.custom_color = if colorize { shirt_color } else { 0 };
        create_capsule_shape(world, bone.body_id, &shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(0.0, 0.0, -0.182204, -0.999999, 0.0, -0.0, 0.001194);
        bone.local_frame_b = xf(0.0, 0.0, -0.007736, -1.0, 0.0, -0.0, 0.0);
        bone.swing_limit = 25.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -15.0 * DEG_TO_RAD,
            y: 15.0 * DEG_TO_RAD,
        };
    }

    // spine_02
    {
        let bone = &mut human.bones[BoneId::Spine02 as usize];
        bone.parent_index = BoneId::Spine01 as i32;
        bone.reference_frame = xf(0.0, 1.194336, -0.027087, 0.703611, 0.0, 0.0, 0.710586);
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, &body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.08,
                y: -0.015133,
                z: -0.091801,
            },
            center2: Vec3 {
                x: -0.08,
                y: -0.015133,
                z: -0.091801,
            },
            radius: 0.10,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { shirt_color } else { 0 };
        create_capsule_shape(world, bone.body_id, &shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(0.0, -0.0, -0.088935, -0.998619, -0.0, 0.0, -0.052540);
        bone.local_frame_b = xf(-0.0, 0.0, -0.008199, -1.0, 0.0, -0.0, 0.0);
        bone.swing_limit = 25.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -15.0 * DEG_TO_RAD,
            y: 15.0 * DEG_TO_RAD,
        };
    }

    // spine_03
    {
        let bone = &mut human.bones[BoneId::Spine03 as usize];
        bone.parent_index = BoneId::Spine02 as i32;
        body_def.name = "spine_03".to_string();
        bone.reference_frame = xf(
            -0.0, 1.31043, -0.028232, 0.669856, 0.000001, -0.000001, 0.742491,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, &body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.11,
                y: -0.039753,
                z: -0.13,
            },
            center2: Vec3 {
                x: -0.11,
                y: -0.039753,
                z: -0.13,
            },
            radius: 0.145,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { shirt_color } else { 0 };
        create_capsule_shape(world, bone.body_id, &shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(
            -0.0, 0.0, -0.124298, -0.998921, 0.000001, -0.000001, -0.046434,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, -1.0, 0.0, -0.000001, 0.0);
        bone.swing_limit = 15.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -10.0 * DEG_TO_RAD,
            y: 10.0 * DEG_TO_RAD,
        };
    }

    // neck
    {
        let bone = &mut human.bones[BoneId::Neck as usize];
        bone.parent_index = BoneId::Spine03 as i32;
        body_def.name = "neck".to_string();
        bone.reference_frame = xf(0.0, 1.575582, -0.055837, 0.879922, 0.0, 0.0, 0.475118);
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, &body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: -0.000001,
                y: -0.0,
                z: -0.02,
            },
            center2: Vec3 {
                x: 0.0,
                y: -0.005,
                z: -0.08,
            },
            radius: 0.07,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { skin_color } else { 0 };
        create_capsule_shape(world, bone.body_id, &shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(
            0.000001, -0.000259, -0.266585, -0.942192, -0.000001, 0.0, 0.335074,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, -1.0, 0.0, -0.000001, 0.0);
        bone.swing_limit = 45.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -15.0 * DEG_TO_RAD,
            y: 15.0 * DEG_TO_RAD,
        };
        bone.joint_friction = 0.8;
    }

    // head
    {
        let bone = &mut human.bones[BoneId::Head as usize];
        bone.parent_index = BoneId::Neck as i32;
        body_def.name = "head".to_string();
        bone.reference_frame = xf(0.0, 1.653348, -0.003241, 0.750288, 0.0, 0.0, 0.661111);
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, &body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: -0.000001,
                y: 0.016892,
                z: -0.05869,
            },
            center2: Vec3 {
                x: 0.0,
                y: -0.003629,
                z: -0.115072,
            },
            radius: 0.0975,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { skin_color } else { 0 };
        create_capsule_shape(world, bone.body_id, &shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(0.0, 0.001321, -0.093873, -0.974301, -0.0, -0.0, -0.225251);
        bone.local_frame_b = xf(0.0, 0.001268, -0.005104, -1.0, 0.0, -0.0, 0.0);
        bone.swing_limit = 15.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -15.0 * DEG_TO_RAD,
            y: 15.0 * DEG_TO_RAD,
        };
        bone.joint_friction = 0.4;
    }

    create_legs_and_arms(
        human,
        world,
        &mut body_def,
        &mut shape_def,
        position,
        group_index,
        colorize,
        pant_color,
        shirt_color,
        skin_color,
    );
    create_bone_joints(human, world, friction_torque, hertz, damping_ratio);

    let mut filter_def = default_filter_joint_def();
    filter_def.base.body_id_a = human.bones[BoneId::ThighL as usize].body_id;
    filter_def.base.body_id_b = human.bones[BoneId::ThighR as usize].body_id;
    human.filter_joints[0] = create_filter_joint(world, &filter_def);
    human.filter_joint_count = 1;

    human.is_spawned = true;
}

#[allow(clippy::too_many_arguments)]
fn create_legs_and_arms(
    human: &mut Human,
    world: &mut World,
    body_def: &mut crate::types::BodyDef,
    shape_def: &mut crate::types::ShapeDef,
    position: Pos,
    group_index: i32,
    colorize: bool,
    pant_color: u32,
    shirt_color: u32,
    skin_color: u32,
) {
    // thigh_l
    {
        let bone = &mut human.bones[BoneId::ThighL as usize];
        bone.parent_index = BoneId::Pelvis as i32;
        body_def.name = "thigh_l".to_string();
        bone.reference_frame = xf(
            0.090416, 0.986104, -0.035090, -0.703287, -0.070715, 0.053866, 0.705327,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.023719,
                y: 0.006008,
                z: -0.039068,
            },
            center2: Vec3 {
                x: -0.064492,
                y: -0.004664,
                z: -0.424718,
            },
            radius: 0.09,
        };
        shape_def.filter.group_index = -group_index;
        shape_def.base_material.custom_color = if colorize { pant_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(
            0.05, 0.011537, -0.055325, -0.714896, -0.022305, -0.698361, -0.026790,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, -0.002064, 0.758987, 0.017046, 0.650880);
        bone.swing_limit = 10.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -60.0 * DEG_TO_RAD,
            y: 40.0 * DEG_TO_RAD,
        };
    }

    // calf_l
    {
        let bone = &mut human.bones[BoneId::CalfL as usize];
        bone.parent_index = BoneId::ThighL as i32;
        body_def.name = "calf_l".to_string();
        bone.reference_frame = xf(
            0.101198, 0.527027, -0.037374, -0.653328, -0.066860, 0.058582, 0.751838,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.001778,
                y: 0.0,
                z: 0.009841,
            },
            center2: Vec3 {
                x: -0.078577,
                y: 0.014707,
                z: -0.41816,
            },
            radius: 0.075,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { pant_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Revolute;
        bone.local_frame_a = xf(
            -0.069989, 0.000253, -0.453844, -0.000677, 0.760087, 0.105674, 0.641171,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, -0.044589, 0.765540, 0.053368, 0.639619);
        bone.twist_limit = Vec2 {
            x: -5.0 * DEG_TO_RAD,
            y: 45.0 * DEG_TO_RAD,
        };
    }

    // thigh_r
    {
        let bone = &mut human.bones[BoneId::ThighR as usize];
        bone.parent_index = BoneId::Pelvis as i32;
        body_def.name = "thigh_r".to_string();
        bone.reference_frame = xf(
            -0.090416, 0.986104, -0.03509, -0.703287, 0.070715, -0.053865, 0.705326,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: -0.023719,
                y: 0.006008,
                z: -0.039068,
            },
            center2: Vec3 {
                x: 0.064492,
                y: -0.004664,
                z: -0.424718,
            },
            radius: 0.09,
        };
        shape_def.filter.group_index = -group_index;
        shape_def.base_material.custom_color = if colorize { pant_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(
            -0.05, 0.011537, -0.055326, -0.039089, -0.714094, 0.043177, 0.697623,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, 0.758805, -0.019886, -0.651012, -0.001759);
        bone.swing_limit = 10.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -30.0 * DEG_TO_RAD,
            y: 60.0 * DEG_TO_RAD,
        };
    }

    // calf_r
    {
        let bone = &mut human.bones[BoneId::CalfR as usize];
        bone.parent_index = BoneId::ThighR as i32;
        body_def.name = "calf_r".to_string();
        bone.reference_frame = xf(
            -0.101198, 0.527027, -0.037373, -0.653327, 0.06686, -0.058582, 0.751839,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: -0.001820,
                y: 0.0,
                z: 0.010071,
            },
            center2: Vec3 {
                x: 0.077883,
                y: 0.014825,
                z: -0.418047,
            },
            radius: 0.075,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { pant_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Revolute;
        bone.local_frame_a = xf(
            0.069988, 0.000253, -0.453844, 0.760086, -0.000675, -0.641171, -0.105676,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, 0.765540, -0.044589, -0.639619, -0.053368);
        bone.twist_limit = Vec2 {
            x: -45.0 * DEG_TO_RAD,
            y: 5.0 * DEG_TO_RAD,
        };
    }

    // upper_arm_l
    {
        let bone = &mut human.bones[BoneId::UpperArmL as usize];
        bone.parent_index = BoneId::Spine03 as i32;
        body_def.name = "upper_arm_l".to_string();
        bone.reference_frame = xf(
            0.20378, 1.484275, -0.115897, 0.143082, 0.695980, -0.690130, 0.13733,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: -0.091118,
                y: 0.037775,
                z: 0.229719,
            },
            radius: 0.075,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { shirt_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(
            0.203780, -0.069369, -0.181921, -0.278486, 0.445600, -0.097014, 0.845266,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, -0.201396, -0.001586, 0.901850, 0.382234);
        bone.swing_limit = 60.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -5.0 * DEG_TO_RAD,
            y: 5.0 * DEG_TO_RAD,
        };
    }

    // lower_arm_l
    {
        let bone = &mut human.bones[BoneId::LowerArmL as usize];
        bone.parent_index = BoneId::UpperArmL as i32;
        body_def.name = "lower_arm_l".to_string();
        bone.reference_frame = xf(
            0.305614, 1.242908, -0.117599, 0.165048, 0.563437, -0.802002, 0.109959,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: -0.142406,
                y: 0.039392,
                z: 0.261092,
            },
            radius: 0.05,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { skin_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Revolute;
        bone.local_frame_a = xf(
            -0.095482, 0.039584, 0.240723, 0.512487, -0.180629, 0.839474, 0.003742,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, 0.503803, -0.029831, 0.858168, 0.094017);
        bone.twist_limit = Vec2 {
            x: -5.0 * DEG_TO_RAD,
            y: 60.0 * DEG_TO_RAD,
        };
    }

    // upper_arm_r
    {
        let bone = &mut human.bones[BoneId::UpperArmR as usize];
        bone.parent_index = BoneId::Spine03 as i32;
        body_def.name = "upper_arm_r".to_string();
        bone.reference_frame = xf(
            -0.20378, 1.484276, -0.115899, 0.143083, -0.695978, 0.690132, 0.137329,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.091118,
                y: 0.037775,
                z: 0.229718,
            },
            radius: 0.075,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { shirt_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Spherical;
        bone.local_frame_a = xf(
            -0.203779, -0.069371, -0.181922, -0.253621, -0.414842, 0.106962, 0.867261,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, -0.201397, 0.001587, -0.901850, 0.382233);
        bone.swing_limit = 60.0 * DEG_TO_RAD;
        bone.twist_limit = Vec2 {
            x: -5.0 * DEG_TO_RAD,
            y: 5.0 * DEG_TO_RAD,
        };
    }

    // lower_arm_r
    {
        let bone = &mut human.bones[BoneId::LowerArmR as usize];
        bone.parent_index = BoneId::UpperArmR as i32;
        body_def.name = "lower_arm_r".to_string();
        bone.reference_frame = xf(
            -0.305614, 1.242907, -0.117599, 0.165048, -0.563437, 0.802002, 0.109959,
        );
        body_def.rotation = bone.reference_frame.q;
        body_def.position = offset_pos(position, bone.reference_frame.p);
        bone.body_id = create_body(world, body_def);

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.142406,
                y: 0.039392,
                z: 0.261092,
            },
            radius: 0.05,
        };
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = if colorize { skin_color } else { 0 };
        create_capsule_shape(world, bone.body_id, shape_def, &capsule);

        bone.joint_type = JointType::Revolute;
        bone.local_frame_a = xf(
            0.095484, 0.039585, 0.240723, -0.180627, 0.512487, -0.003744, -0.839474,
        );
        bone.local_frame_b = xf(0.0, 0.0, 0.0, -0.029831, 0.503803, -0.094017, -0.858169);
        bone.twist_limit = Vec2 {
            x: -60.0 * DEG_TO_RAD,
            y: 5.0 * DEG_TO_RAD,
        };
    }
}

fn create_bone_joints(
    human: &mut Human,
    world: &mut World,
    friction_torque: f32,
    hertz: f32,
    damping_ratio: f32,
) {
    for i in 1..BONE_COUNT {
        let parent_index = human.bones[i].parent_index as usize;
        let body_id_a = human.bones[parent_index].body_id;
        let body_id_b = human.bones[i].body_id;

        human.bones[i].local_frame_a.q = normalize_quat(human.bones[i].local_frame_a.q);
        human.bones[i].local_frame_b.q = normalize_quat(human.bones[i].local_frame_b.q);

        let joint_type = human.bones[i].joint_type;
        let local_frame_a = human.bones[i].local_frame_a;
        let local_frame_b = human.bones[i].local_frame_b;
        let twist_limit = human.bones[i].twist_limit;
        let swing_limit = human.bones[i].swing_limit;
        let joint_friction = human.bones[i].joint_friction;

        if joint_type == JointType::Revolute {
            let mut joint_def = default_revolute_joint_def();
            joint_def.base.body_id_a = body_id_a;
            joint_def.base.body_id_b = body_id_b;
            joint_def.base.local_frame_a = local_frame_a;
            joint_def.base.local_frame_b = local_frame_b;
            joint_def.enable_limit = true;
            joint_def.lower_angle = twist_limit.x;
            joint_def.upper_angle = twist_limit.y;
            joint_def.enable_spring = hertz > 0.0;
            joint_def.hertz = hertz;
            joint_def.damping_ratio = damping_ratio;
            joint_def.enable_motor = true;
            joint_def.max_motor_torque = joint_friction * friction_torque;
            human.bones[i].joint_id = create_revolute_joint(world, &joint_def);
        } else if joint_type == JointType::Spherical {
            let mut joint_def = default_spherical_joint_def();
            joint_def.base.body_id_a = body_id_a;
            joint_def.base.body_id_b = body_id_b;
            joint_def.base.local_frame_a = local_frame_a;
            joint_def.base.local_frame_b = local_frame_b;
            joint_def.enable_cone_limit = true;
            joint_def.cone_angle = swing_limit;
            joint_def.enable_twist_limit = true;
            joint_def.lower_twist_angle = twist_limit.x;
            joint_def.upper_twist_angle = twist_limit.y;
            joint_def.enable_spring = hertz > 0.0;
            joint_def.hertz = hertz;
            joint_def.damping_ratio = damping_ratio;
            joint_def.enable_motor = true;
            joint_def.max_motor_torque = joint_friction * friction_torque;
            human.bones[i].joint_id = create_spherical_joint(world, &joint_def);
        }
    }
}
