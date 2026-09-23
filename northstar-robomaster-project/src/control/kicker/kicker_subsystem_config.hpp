#ifndef KICKER_SUBSYSTEM_CONFIG_HPP_
#define KICKER_SUBSYSTEM_CONFIG_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

namespace src::control::kicker
{
struct KickerSubsystemConfig
{
    /// Motor gear ratio, so we use shaft angle rather than encoder angle.
    float gearRatio;
    /// The motor ID for this motor.
    tap::motor::MotorId kickerMotorId;
    /// The CAN ID for this motor.
    tap::can::CanBus kickerCanBusId;
    /// If `true` positive rotation is clockwise when looking at the motor shaft opposite the motor.
    /// Counterclockwise if false.
    bool isKickerInverted;
    /// Velocity PID feed forward term. Scaling factor that converts desired velocity to desired
    /// motor output. When using the M308 or the M2006, motor velocity -> motor current is mostly
    /// linear since these motors take a desired current as a command. When using a motor that is
    /// controlled by sending voltage commands, this term should be 0.
    float velocityPIDFeedForwardGain;
};
}  // namespace src::control::kicker

#endif  // KICKER_SUBSYSTEM_CONFIG_HPP_