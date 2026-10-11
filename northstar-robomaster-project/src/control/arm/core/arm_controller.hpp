#ifndef ARM_CONTROLLER_HPP_
#define ARM_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

#include "arm_controller_interface.hpp"
#include "arm_joint.hpp"
#include "arm_types.hpp"

namespace src::control::arm
{

template <std::size_t N>
class ArmController : public ArmControllerInterface
{
public:
    explicit ArmController(const std::array<ArmJoint, N>& joints) : joints(joints) {}

    void initialize() override
    {
        for (const ArmJoint& j : joints)
        {
            j.actuator->initialize();
        }
    }

    void update(float dt) override
    {
        if (dt <= 0)
        {
            return;
        }
        readJoints();
        if (!setpointInitialized)
        {
            setpoint = target = position;
            setpointInitialized = true;
        }
        if (armMode == ArmMode::DISABLED || !allOnline())
        {
            setpoint = position;
            setpointVel = {};
            disable();
            return;
        }
        planSetpoints(dt);
        commandTorques();
    }

    void setJointTarget(const float* angles, std::size_t count) override
    {
        if (angles == nullptr || count != N)
        {
            return;
        }
        for (std::size_t i = 0; i < N; i++)
        {
            target[i] = clampToLimits(i, angles[i]);
        }
        armMode = ArmMode::RUNNING;
    }

    void setJointTarget(std::size_t index, float rad) override
    {
        if (index >= N)
        {
            return;
        }
        target[index] = clampToLimits(index, rad);
        armMode = ArmMode::RUNNING;
    }

    /// @brief Same as the pointer version, with the joint count checked at compile time.
    void setJointTarget(const JointVector<N>& t) { setJointTarget(t.data(), N); }

    void holdCurrent() override
    {
        target = setpoint = position;
        setpointVel = {};
        armMode = ArmMode::RUNNING;
    }

    void enterSafeHold() override
    {
        if (armMode == ArmMode::SAFE_HOLD) return;
        holdCurrent();
        armMode = ArmMode::SAFE_HOLD;
    }

    void disable() override
    {
        for (const ArmJoint& j : joints)
        {
            j.actuator->disable();
        }
        lastTorque = {};
        armMode = ArmMode::DISABLED;
    }

    std::size_t jointCount() const override { return N; }

    float jointPosition(std::size_t index) const override { return at(position, index); }

    float jointVelocity(std::size_t index) const override { return at(velocity, index); }

    float jointTarget(std::size_t index) const override { return at(target, index); }

    float commandedTorque(std::size_t index) const override { return at(lastTorque, index); }

    ArmMode mode() const override { return armMode; }

    bool atTarget() const override
    {
        for (std::size_t i = 0; i < N; i++)
        {
            const JointConfig& cfg = joints[i].spec->config;
            if (std::abs(target[i] - position[i]) > cfg.angleTolerance ||
                std::abs(velocity[i]) > cfg.velocityTolerance)
            {
                return false;
            }
        }
        return true;
    }

    bool allOnline() const override
    {
        for (const ArmJoint& j : joints)
        {
            if (!j.actuator->online())
            {
                return false;
            }
        }
        return true;
    }

private:
    std::array<ArmJoint, N> joints{};
    JointVector<N> position{}, velocity{};     /// Encoder value
    JointVector<N> target{};                   /// Final target position.
    JointVector<N> setpoint{}, setpointVel{};  /// What it should be right now.
    JointVector<N> lastTorque{};               /// Torque commanded on the last tick.
    ArmMode armMode = ArmMode::DISABLED;

    bool setpointInitialized = false;

    static float at(const JointVector<N>& v, std::size_t index)
    {
        return index < N ? v[index] : 0.0f;
    }

    float clampToLimits(std::size_t index, float rad) const
    {
        const JointConfig& cfg = joints[index].spec->config;
        return std::clamp(rad, cfg.minAngle, cfg.maxAngle);
    }

    /// Pulls the latest feedback from every joint.
    void readJoints()
    {
        for (std::size_t i = 0; i < N; i++)
        {
            JointActuatorInterface* actuator = joints[i].actuator;
            actuator->update();
            position[i] = actuator->position();
            velocity[i] = actuator->velocity();
        }
    }

    /// Moves each setpoint toward its target at no more than the joint's max velocity.
    void planSetpoints(float dt)
    {
        for (std::size_t i = 0; i < N; i++)
        {
            const JointConfig& cfg = joints[i].spec->config;
            float prev = setpoint[i];
            float step = std::clamp(target[i] - prev, -cfg.maxVelocity * dt, cfg.maxVelocity * dt);
            // Don't let the setpoint run away from a joint that can't keep up.
            setpoint[i] = std::clamp(
                prev + step,
                position[i] - cfg.maxTrackingError,
                position[i] + cfg.maxTrackingError);
            setpointVel[i] = (setpoint[i] - prev) / dt;
        }
    }

    /// PD + gravity on each joint, clamped to what the joint and its motor allow.
    void commandTorques()
    {
        JointVector<N> g = gravityTorques(position);

        for (std::size_t i = 0; i < N; i++)
        {
            const JointConfig& cfg = joints[i].spec->config;
            JointActuatorInterface* actuator = joints[i].actuator;
            float torque = g[i] + cfg.kp * (setpoint[i] - position[i]) +
                           cfg.kd * (setpointVel[i] - velocity[i]);
            float limit = std::min(cfg.maxTorque, actuator->limits().availableTorque);
            torque = std::clamp(torque, -limit, limit);
            actuator->commandTorque(torque);
            lastTorque[i] = torque;
        }
    }

    // TODO(step 4): replace with ArmModel::gravityTorques().
    JointVector<N> gravityTorques(const JointVector<N>&) const { return JointVector<N>{}; }
};

}  // namespace src::control::arm

#endif
