// Typed views of the JointSim union payload (C: &base->distanceJoint and friends).
// Split from types.rs, which ports the joint.h data model, to satisfy the file-length limit.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{
    DistanceJoint, JointSim, JointUnion, MotorJoint, MoverJoint, ParallelJoint, PogoJoint,
    PrismaticJoint, RevoluteJoint, SphericalJoint, WeldJoint, WheelJoint,
};

impl JointSim {
    /// (C: &base->distanceJoint)
    pub fn distance(&self) -> &DistanceJoint {
        match &self.union_ {
            JointUnion::Distance(joint) => joint,
            _ => unreachable!("joint union is not a distance joint"),
        }
    }

    pub fn distance_mut(&mut self) -> &mut DistanceJoint {
        match &mut self.union_ {
            JointUnion::Distance(joint) => joint,
            _ => unreachable!("joint union is not a distance joint"),
        }
    }

    /// (C: &base->parallelJoint)
    pub fn parallel(&self) -> &ParallelJoint {
        match &self.union_ {
            JointUnion::Parallel(joint) => joint,
            _ => unreachable!("joint union is not a parallel joint"),
        }
    }

    pub fn parallel_mut(&mut self) -> &mut ParallelJoint {
        match &mut self.union_ {
            JointUnion::Parallel(joint) => joint,
            _ => unreachable!("joint union is not a parallel joint"),
        }
    }

    /// (C: &base->weldJoint)
    pub fn weld(&self) -> &WeldJoint {
        match &self.union_ {
            JointUnion::Weld(joint) => joint,
            _ => unreachable!("joint union is not a weld joint"),
        }
    }

    pub fn weld_mut(&mut self) -> &mut WeldJoint {
        match &mut self.union_ {
            JointUnion::Weld(joint) => joint,
            _ => unreachable!("joint union is not a weld joint"),
        }
    }

    /// (C: &base->revoluteJoint)
    pub fn revolute(&self) -> &RevoluteJoint {
        match &self.union_ {
            JointUnion::Revolute(joint) => joint,
            _ => unreachable!("joint union is not a revolute joint"),
        }
    }

    pub fn revolute_mut(&mut self) -> &mut RevoluteJoint {
        match &mut self.union_ {
            JointUnion::Revolute(joint) => joint,
            _ => unreachable!("joint union is not a revolute joint"),
        }
    }

    /// (C: &base->prismaticJoint)
    pub fn prismatic(&self) -> &PrismaticJoint {
        match &self.union_ {
            JointUnion::Prismatic(joint) => joint,
            _ => unreachable!("joint union is not a prismatic joint"),
        }
    }

    pub fn prismatic_mut(&mut self) -> &mut PrismaticJoint {
        match &mut self.union_ {
            JointUnion::Prismatic(joint) => joint,
            _ => unreachable!("joint union is not a prismatic joint"),
        }
    }

    /// (C: &base->motorJoint)
    pub fn motor(&self) -> &MotorJoint {
        match &self.union_ {
            JointUnion::Motor(joint) => joint,
            _ => unreachable!("joint union is not a motor joint"),
        }
    }

    pub fn motor_mut(&mut self) -> &mut MotorJoint {
        match &mut self.union_ {
            JointUnion::Motor(joint) => joint,
            _ => unreachable!("joint union is not a motor joint"),
        }
    }

    /// (C: &base->sphericalJoint)
    pub fn spherical(&self) -> &SphericalJoint {
        match &self.union_ {
            JointUnion::Spherical(joint) => joint,
            _ => unreachable!("joint union is not a spherical joint"),
        }
    }

    pub fn spherical_mut(&mut self) -> &mut SphericalJoint {
        match &mut self.union_ {
            JointUnion::Spherical(joint) => joint,
            _ => unreachable!("joint union is not a spherical joint"),
        }
    }

    /// (C: &base->wheelJoint)
    pub fn wheel(&self) -> &WheelJoint {
        match &self.union_ {
            JointUnion::Wheel(joint) => joint,
            _ => unreachable!("joint union is not a wheel joint"),
        }
    }

    pub fn wheel_mut(&mut self) -> &mut WheelJoint {
        match &mut self.union_ {
            JointUnion::Wheel(joint) => joint,
            _ => unreachable!("joint union is not a wheel joint"),
        }
    }

    /// (C: &base->moverJoint)
    pub fn mover(&self) -> &MoverJoint {
        match &self.union_ {
            JointUnion::Mover(joint) => joint,
            _ => unreachable!("joint union is not a mover joint"),
        }
    }

    pub fn mover_mut(&mut self) -> &mut MoverJoint {
        match &mut self.union_ {
            JointUnion::Mover(joint) => joint,
            _ => unreachable!("joint union is not a mover joint"),
        }
    }

    /// (C: &base->pogoJoint)
    pub fn pogo(&self) -> &PogoJoint {
        match &self.union_ {
            JointUnion::Pogo(joint) => joint,
            _ => unreachable!("joint union is not a pogo joint"),
        }
    }

    pub fn pogo_mut(&mut self) -> &mut PogoJoint {
        match &mut self.union_ {
            JointUnion::Pogo(joint) => joint,
            _ => unreachable!("joint union is not a pogo joint"),
        }
    }
}
