#ifndef JOINT_ACTUATOR_INTERFACE_HPP_
#define JOINT_ACTUATOR_INTERFACE_HPP_

namespace src::control::arm
{

/// @brief Torque and speed capability of an actuator.
struct ActuatorLimits
{
    float peakTorque;        ///< @brief N * m, temporary burst torque.
    float continuousTorque;  ///< @brief N * m, continuous torque capability.
    float availableTorque;   ///< @brief N * m, torque available based on heat.
    float maxVelocity;       ///< @brief rad/s
    float thermalHeadroom;   ///< @brief 0-1 range. Hot at 1.
};

class JointActuatorInterface
{
public:
    virtual ~JointActuatorInterface() = default;

    /// @brief One time setup.
    virtual void initialize() = 0;

    /// @brief Updates the motor.
    virtual void update() = 0;

    /**
     * @brief Returns the position of the joint.
     *
     * @return The position in radians.
     */
    virtual float position() const = 0;

    /**
     * @brief Returns the velocity of the joint.
     *
     * @return The velocity in radians/sec.
     */
    virtual float velocity() const = 0;

    /**
     * @brief Returns the torque of the joint.
     *
     * @return The torque in N * m.
     */
    virtual float measuredTorque() const = 0;

    /**
     * @brief Gets if we have updated signals from the joint.
     *
     * @return True for online.
     * @return False for offline.
     */
    virtual bool online() const = 0;

    /**
     * @brief Does the motor have an absolute encoder (can tell position on power up).
     *
     * @return True for absolute.
     * @return False for not absolute.
     */
    virtual bool absolute() const = 0;

    /**
     * @brief Gets the torque and speed capability of one actuator.
     *
     * @return The joints ActuatorLimits
     */
    virtual ActuatorLimits limits() const = 0;

    /**
     * @brief Homing: declares that the joint is physically at the given angle right now.
     * Re-zeroes the position feedback; does not move the motor.
     *
     * @param rad The joint's true current angle in radians.
     */
    virtual void setCurrentPositionAs(float rad) = 0;

    /**
     * @brief Sets the current desired torque.
     *
     * @param torque in N * m
     */
    virtual void commandTorque(float torque) = 0;

    /**
     * @brief Disables the arm joint. Zero output.
     *
     */
    virtual void disable() = 0;
};
}  // namespace src::control::arm

#endif