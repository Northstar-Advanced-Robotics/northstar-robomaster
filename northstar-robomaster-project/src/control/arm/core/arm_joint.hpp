#ifndef ARM_JOINT_HPP_
#define ARM_JOINT_HPP_

#include "joint_actuator_interface.hpp"
#include "joint_config.hpp"

namespace src::control::arm
{

struct ArmJointSpec
{
    const char* name;
    JointConfig config;
};

struct ArmJoint
{
    const ArmJointSpec* spec;
    JointActuatorInterface* actuator;
};

}  // namespace src::control::arm

#endif