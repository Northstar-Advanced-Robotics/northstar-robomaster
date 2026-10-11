#ifndef ARM_CONTROLLER_HPP_
#define ARM_CONTROLLER_HPP_

#include <algorithm>

#include "arm_controller_interface.hpp"
#include "arm_joint.hpp"
#include "arm_types.hpp"

namespace src::control::arm
{

enum class ArmMode
{
    DISABLED,
    RUNNING
};

template <std::size_t N>
class ArmController : public ArmControllerInterface
{
public:
    explicit ArmController(const std::array<ArmJoint, N>& joints) : joints(joints) {}

    void initialize() override
    {
        for (ArmJoint j : joints)
        {
            j.actuator->initialize();
        }
    }

    void setJointTarget(const JointVector<N>& t)
    {
        for (std::size_t i = 0; i < N; i++)
        {
            target[i] =
                std::clamp(t[i], joints[i].spec->config.minAngle, joints[i].spec->config.maxAngle);
        }
        mode = ArmMode::RUNNING;
    }

    void holdCurrent() override
    {
        target = setpoint = position;
        setpointVel = {};
        mode = ArmMode::RUNNING
    }

    void enterSafeHold() override { holdCurrent(); }

    void disable() override
    {
        for (ArmJoint j : joints)
        {
            j.actuator->disable();
        }
        mode = ArmMode::DISABLED;
    }

    void atTarget() const override
    {
        for (size_t i = 0; i < N; i++)
        {
            if (abs(target[i] - position[i]) > joints[i].spec->config.angleTolerence ||
                velocity[i] > 0.1)
            {
                return false;
            }
        }
    }

    void allOnline() const override
    {
        for (ArmJoint j : joints)
        {
            if (!j.actuator->online())
            {
                return false;
            }
        }
    }

    void update(float dt) override
    {
        if (dt <= 0)
        {
            return;
        }
        for (size_t i = 0; i < N; i++)
        {
            joints[i].actuator->update();
            position[i] = joints[i].actuator->position();
            velocity[i] = joints[i].actuator->velocity();
        }
        if (!setpointInitialized)
        {
            setpoint = target = position;
            setpointInitialized = true;
        }
        if (mode == ArmMode::DISABLED || !allOnline())
        {
            disable();
            return;
        }
        // calculate setpoints
        for (size_t i = 0; i < N; i++)
        {
            float err = target[i] - setpoint[i];
            step = clamp(
                err,
                -joints[i].spec->config.maxVelocity * dt,
                joints[i].spec->config.maxVelocity * dt);
            setpoint[i] += step;
            setpointVel[i] = step / dt;
        }
    }

private:
    std::array<ArmJoint, N> joints{};
    JointVector<N> position{}, velocity{};     /// Encoder value
    JointVector<N> target{};                   /// Final target position.
    JointVector<N> setpoint{}, setpointVel{};  /// What it should be right now.
    JointVector<N> lastTorque{};
    ArmMode mode = ArmMode::DISABLED;

    JointVector<N> gravityTorques(const JointVector<N>& q) const { return JointVector{}; }
};

}  // namespace src::control::arm

#endif