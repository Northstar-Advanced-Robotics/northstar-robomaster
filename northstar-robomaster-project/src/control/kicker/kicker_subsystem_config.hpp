#ifndef KICKER_SUBSYSTEM_CONFIG_HPP_
#define KICKER_SUBSYSTEM_CONFIG_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

namespace src::kicker
{
/**
 * @ingroup hopper_kicker
 *
 * How the kicker motor is wired and geared. Supplied per robot from
 * `robot/<target>/<target>_kicker_constants.hpp`.
 */
struct KickerSubsystemConfig
{
    /// Motor gear ratio, so we use shaft angle rather than encoder angle.
    float gearRatio;
    /// The motor ID for this motor.
    tap::motor::MotorId kickerMotorId;
    /// The CAN **bus** this motor is on (CAN_BUS1 or CAN_BUS2). The CAN ID is `kickerMotorId`.
    tap::can::CanBus kickerCanBusId;
    /// If `true` positive rotation is clockwise when looking at the motor shaft opposite the motor.
    /// Counterclockwise if false.
    bool isKickerInverted;
    /// Velocity PID feed forward term. Scaling factor that converts desired velocity to desired
    /// motor output. When using the M3508 or the M2006, motor velocity -> motor current is mostly
    /// linear since these motors take a desired current as a command. When using a motor that is
    /// controlled by sending voltage commands, this term should be 0.
    float velocityPIDFeedForwardGain;
};
}  // namespace src::kicker

#endif  // KICKER_SUBSYSTEM_CONFIG_HPP_