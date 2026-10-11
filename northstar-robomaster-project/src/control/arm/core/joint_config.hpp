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
    float angleTolerance;      ///< rad, atTarget() angle window
    float velocityTolerance;   ///< rad/s, atTarget() speed window
    float maxTrackingError;    ///< rad, how far the setpoint may lead the measured position
};
}  // namespace src::control::arm

#endif