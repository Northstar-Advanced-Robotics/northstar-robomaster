#ifndef FAKE_JOINT_ACTUATOR_HPP_
#define FAKE_JOINT_ACTUATOR_HPP_

#include "control/arm/core/joint_actuator_interface.hpp"

namespace src::control::arm
{
/// A joint with no motor: tests set the feedback fields and read back what was commanded.
class FakeJointActuator : public JointActuatorInterface
{
public:
    float pos = 0.0f;
    float vel = 0.0f;
    bool isOnline = true;
    ActuatorLimits actuatorLimits{
        .peakTorque = 100.0f,
        .continuousTorque = 100.0f,
        .availableTorque = 100.0f,
        .maxVelocity = 100.0f,
        .thermalHeadroom = 0.0f};

    float torque = 0.0f;  ///< Last commanded torque; 0 after disable().
    bool disabled = false;
    int initializeCalls = 0;

    void initialize() override { initializeCalls++; }
    void update() override {}
    float position() const override { return pos; }
    float velocity() const override { return vel; }
    float measuredTorque() const override { return torque; }
    bool online() const override { return isOnline; }
    bool absolute() const override { return true; }
    ActuatorLimits limits() const override { return actuatorLimits; }
    void setCurrentPositionAs(float rad) override { pos = rad; }

    void commandTorque(float t) override
    {
        torque = t;
        disabled = false;
    }

    void disable() override
    {
        torque = 0.0f;
        disabled = true;
    }
};
}  // namespace src::control::arm

#endif
