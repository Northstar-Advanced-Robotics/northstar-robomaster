#ifndef JOINT_CONFIG_HPP_
#define JOINT_CONFIG_HPP_

namespace src::control::arm
{
struct JointConfig
{
    float minAngle, maxAngle;  /// rad, soft limits
    float maxVelocity;         ///< rad/s
    float maxTorque;           ///< N * m cap
    float kp;                  ///< N * m per rad
    float kd;                  ///< N * m per rad/s
    float angleTolerence;
};
}  // namespace src::control::arm

#endif