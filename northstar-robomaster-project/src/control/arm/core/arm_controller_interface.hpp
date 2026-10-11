#ifndef ARM_CONTROLLER_INTERFACE_HPP_
#define ARM_CONTROLLER_INTERFACE_HPP_

#include <cstddef>

namespace src::control::arm
{

enum class ArmMode
{
    DISABLED,  ///< Zero torque on every joint.
    RUNNING,   ///< Moving to / holding the target.
    SAFE_HOLD  ///< Remote lost: holding the pose captured when it was lost.
};

/**
 * @brief What commands and the subsystem are allowed to ask the arm to do.
 *
 * Knows nothing about the joint count or the motors, so commands written against it work
 * with any arm. Angles are in radians, speeds in rad/s and torques in N * m, all at the joint.
 */
class ArmControllerInterface
{
public:
    virtual ~ArmControllerInterface() = default;

    /// @brief One time setup.
    virtual void initialize() = 0;

    /**
     * @brief Runs one control tick: read joints, plan the motion, command torques.
     *
     * @param dt Time since the last tick in seconds.
     */
    virtual void update(float dt) = 0;

    /**
     * @brief Sets the target angle of every joint and enables the arm.
     * Targets are clamped to each joint's soft limits.
     *
     * @param angles Target angles in radians, one per joint.
     * @param count Number of angles. Ignored unless it equals jointCount().
     */
    virtual void setJointTarget(const float* angles, std::size_t count) = 0;

    /**
     * @brief Sets the target angle of one joint and enables the arm.
     * The target is clamped to the joint's soft limits.
     *
     * @param index Which joint. Ignored if out of range.
     * @param rad Target angle in radians.
     */
    virtual void setJointTarget(std::size_t index, float rad) = 0;

    /// @brief Sets the target to the current position and enables the arm.
    virtual void holdCurrent() = 0;

    /**
     * @brief Remote lost: holds the current pose instead of going limp.
     * Safe to call every tick; the pose is only captured on the first call.
     */
    virtual void enterSafeHold() = 0;

    /// @brief Zero torque on every joint.
    virtual void disable() = 0;

    /// @brief Gets the number of joints.
    virtual std::size_t jointCount() const = 0;

    /**
     * @brief Gets the measured angle of a joint.
     *
     * @return The angle in radians. 0 if index is out of range.
     */
    virtual float jointPosition(std::size_t index) const = 0;

    /**
     * @brief Gets the measured speed of a joint.
     *
     * @return The speed in radians/sec. 0 if index is out of range.
     */
    virtual float jointVelocity(std::size_t index) const = 0;

    /**
     * @brief Gets the final target angle of a joint.
     *
     * @return The angle in radians. 0 if index is out of range.
     */
    virtual float jointTarget(std::size_t index) const = 0;

    /**
     * @brief Gets the torque commanded to a joint on the last tick.
     *
     * @return The torque in N * m. 0 if index is out of range.
     */
    virtual float commandedTorque(std::size_t index) const = 0;

    /// @brief Gets what the arm is currently doing.
    virtual ArmMode mode() const = 0;

    /**
     * @brief Gets if every joint is at its target and has stopped moving.
     *
     * @return True if within each joint's angle and velocity tolerance.
     */
    virtual bool atTarget() const = 0;

    /**
     * @brief Gets if we have updated signals from every joint.
     *
     * @return True if all joints are online.
     */
    virtual bool allOnline() const = 0;
};
}  // namespace src::control::arm

#endif
