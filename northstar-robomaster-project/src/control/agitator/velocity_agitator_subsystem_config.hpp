#ifndef VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_
#define VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

namespace src::agitator
{
/**
 * @ingroup agitator
 *
 * How one robot's agitator motor is wired and geared, and when it should be considered jammed.
 *
 * Supplied per robot from `robot/<target>/<target>_agitator_constants.hpp`.
 *
 * Jam detection is velocity-based: while the measured velocity stays further than
 * `jammingVelocityDifference` from the setpoint for `jammingTime`, the agitator is declared
 * jammed. There is no distance- or angle-based check.
 */
struct VelocityAgitatorSubsystemConfig
{
    /// Motor gear ratio, so we use shaft angle rather than encoder angle.
    float gearRatio;
    /// The motor ID for this motor.
    tap::motor::MotorId agitatorMotorId;
    /// The motor CAN bus for this motor.
    tap::can::CanBus agitatorCanBusId;
    /// If `true` positive rotation is clockwise when looking at the motor shaft opposite the motor.
    /// Counterclockwise if false.
    bool isAgitatorInverted;
    /// How far the **measured** velocity may sit from the setpoint before the jam timer starts
    /// counting, in radians/second.
    float jammingVelocityDifference;
    /// How long the measured velocity must stay outside `jammingVelocityDifference` before the
    /// agitator is declared jammed, in milliseconds.
    uint32_t jammingTime;
    /// A flag which determines whether or not jamming detection is enabled. `true` means enabled,
    /// `false` means disabled.
    bool jamLogicEnabled;
    /// Velocity PID feed forward term. Scaling factor that converts desired velocity to desired
    /// motor output. When using the M3508 or the M2006, motor velocity -> motor current is mostly
    /// linear since these motors take a desired current as a command. When using a motor that is
    /// controlled by sending voltage commands, this term should be 0.
    float velocityPIDFeedForwardGain;
};
}  // namespace src::agitator

#endif  // VELOCITY_AGITATOR_SUBSYSTEM_CONFIG_HPP_