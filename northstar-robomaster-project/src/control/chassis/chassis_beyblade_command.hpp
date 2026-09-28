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
 * Drives the chassis from operator input while spinning it continuously, so that incoming
 * projectiles are spread across all four armor plates instead of concentrating on one.
 *
 * Translation still comes from the operator and is interpreted relative to the turret, so the
 * robot can be driven normally while spinning. The spin rate is whatever is left over after the
 * requested translation is accounted for, and is boosted while the robot is nearly stationary,
 * since there is then no translation competing for wheel speed. Optionally the rate is varied
 * randomly, which makes the robot harder for an opposing auto-aim to track.
 */
class ChassisBeybladeCommand : public tap::control::Command
{
public:
    /**
     * @param[in] chassis The chassis to drive, taken as a subsystem requirement.
     * @param[in] operatorInterface The source of the operator's translation input.
     * @param[in] direction Which way to spin: positive for counterclockwise, negative for
     *      clockwise.
     * @param[in] isVariable `true` to jitter the spin rate randomly, `false` to spin at a
     *      constant rate.
     */
    ChassisBeybladeCommand(
        ChassisSubsystem *chassis,
        src::control::ControlOperatorInterface *operatorInterface,
        short direction,
        bool isVariable);

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Chassis beyblade drive"; }

    /// Resets the timing state, starts at full spin speed, and tells the chassis that beyblade is
    /// running so other code (notably the power limiter) can account for it.
    void initialize() override;

    /// Applies the operator's translation and the current spin rate, boosting the spin while the
    /// chassis is moving slowly enough that the extra wheel speed is available.
    void execute() override;

    /**
     * Stops the chassis and clears the beyblade status flags.
     *
     * @param[in] interrupted Ignored; the chassis is stopped either way.
     */
    void end(bool interrupted) override;

    /// @return Always `false`; this command runs until something else interrupts it.
    bool isFinished() const { return false; }

    /**
     * Computes the spin rate for this iteration. At a fixed rate this is simply `maxSpeed` in the
     * configured direction; in variable mode the scale factor is re-randomized roughly twice a
     * second and held in between.
     *
     * @param[in] maxSpeed The largest rotational speed the chassis can supply given the
     *      translation currently being requested.
     * @param[in] dt Milliseconds since the previous iteration.
     * @return The rotational velocity to command, signed by the spin direction.
     */
    float calculateBeyBladeRotationSpeed(float maxSpeed, uint32_t dt);

private:
    /// The chassis being driven.
    src::chassis::ChassisSubsystem *chassis;

    /// The source of operator input.
    src::control::ControlOperatorInterface *operatorInterface;

    /// Time in milliseconds of the previous iteration, used to measure `dt`.
    uint32_t prevTime;

    /// Milliseconds since the spin rate was last re-randomized.
    uint32_t accumTime;

    /// Scales the spin rate with distance travelled. Currently unused.
    float distScaleFactor;

    /// Spin direction: positive for counterclockwise, negative for clockwise.
    short direction;

    /// Whether the spin rate is randomly varied rather than held constant.
    bool isVariable;

    /// The current spin rate as a fraction of the maximum, held between re-randomizations.
    float calcSpeed;

    /// Chassis speed in meters/second below which the robot counts as stationary and the spin
    /// rate is boosted.
    float beyBladeFastSpinSpeedThreshold = 0.3f;
};
}  // namespace src::chassis