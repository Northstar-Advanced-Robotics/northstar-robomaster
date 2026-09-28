#pragma once

#include "tap/control/command.hpp"

#include "control/chassis/constants/chassis_constants.hpp"

namespace src
{
class Drivers;

namespace control
{
class ControlOperatorInterface;
}
}  // namespace src

namespace src::chassis
{
class ChassisSubsystem;

/**
 * @ingroup chassis
 *
 * Drives the chassis from operator input while rocking it back and forth about its center.
 *
 * Like beyblade, the point is to keep an opposing auto-aim from settling on one armor plate, but
 * the oscillation is gentler and keeps the robot roughly facing forward. The rotational velocity
 * follows a sine wave of the configured period and amplitude; translation still comes from the
 * operator and is interpreted relative to the turret.
 */
class ChassisWiggleCommand : public tap::control::Command
{
public:
    /**
     * @param[in] chassis The chassis to drive, taken as a subsystem requirement.
     * @param[in] operatorInterface The source of the operator's translation input.
     * @param[in] period Duration of one complete back-and-forth cycle, in seconds.
     * @param[in] maxWiggleSpeed Peak rotational velocity of the oscillation, in radians/second.
     */
    ChassisWiggleCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface,
        float period,
        float maxWiggleSpeed);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Chassis tank drive"; }

    /// Resets the timing state so the oscillation starts from the beginning of its cycle.
    void initialize() override;

    /// Applies the operator's translation together with this iteration's point on the oscillation.
    void execute() override;

    /**
     * Commands the chassis to a stop.
     *
     * @param[in] interrupted Ignored; the chassis is stopped either way.
     */
    void end(bool interrupted) override;

    /// @return Always `false`; this command runs until something else interrupts it.
    bool isFinished() const { return false; }

private:
    /// The chassis being driven.
    src::chassis::ChassisSubsystem *chassis;

    /// The source of operator input.
    src::control::ControlOperatorInterface *operatorInterface;

    /// Time in milliseconds of the previous iteration, used to measure `dt`.
    uint32_t prevTime;

    /// Milliseconds elapsed within the current oscillation, i.e. the phase of the sine wave.
    uint32_t accumTime;

    /// Duration of one complete oscillation, in seconds.
    float period;

    /// Peak rotational velocity of the oscillation, in radians/second.
    float maxWiggleSpeed;

    /**
     * Advances the oscillation and samples it.
     *
     * @param[in] dt Milliseconds since the previous iteration. A gap longer than 50ms means this
     *      command was not running continuously, so the phase is reset rather than jumped forward.
     * @return The rotational velocity to command this iteration, in radians/second.
     */
    float calculateWiggle(uint32_t dt);
};
}  // namespace src::chassis