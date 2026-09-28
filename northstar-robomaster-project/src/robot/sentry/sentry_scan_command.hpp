#ifndef SENTRY_SCAN_COMMAND_HPP
#define SENTRY_SCAN_COMMAND_HPP

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "control/chassis/chassis_odometry.hpp"
#include "control/turret/algorithms/turret_controller_interface.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::turret::cv
{
/**
 * @ingroup robots
 *
 * Sweeps the sentry's turret back and forth looking for targets, for use when vision has nothing to
 * track.
 *
 * Pitch oscillates steadily between its limits while yaw advances in chunks, holding each new
 * heading long enough for the vision computer to actually get a frame at it. A continuous yaw sweep
 * would smear the image and defeat the point of scanning.
 *
 * Runs until interrupted, normally by `SentryCvManagerCommand` handing control back to auto-aim
 * once a target appears.
 */
class SentryScanCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] turretSubsystem The turret to sweep, taken as a subsystem requirement.
     * @param[in] yawController The controller driving yaw to its setpoint.
     * @param[in] pitchController The controller driving pitch to its setpoint.
     * @param[in] chassisOdometry Supplies the chassis heading, so the sweep covers the field
     *      rather than being relative to whichever way the chassis is pointed.
     * @param[in] MIN_PITCH_ANGLE The bottom of the pitch sweep, in radians.
     * @param[in] MAX_PITCH_ANGLE The top of the pitch sweep, in radians.
     * @param[in] PITCH_SPEED How fast pitch sweeps, in radians/second.
     * @param[in] YAW_SPEED How fast yaw moves between headings, in radians/second.
     */
    SentryScanCommand(
        tap::Drivers *drivers,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        src::chassis::ChassisOdometry *chassisOdometry,
        float MIN_PITCH_ANGLE,
        float MAX_PITCH_ANGLE,
        float PITCH_SPEED,
        float YAW_SPEED);

    /// @return `true` if the turret is online and can be scanned.
    bool isReady() override;

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Sentry scan"; }

    /// Resets the sweep timing and initializes the turret controllers.
    void initialize() override;

    /// Advances the pitch sweep and, when the current yaw heading has been held long enough, moves
    /// on to the next one.
    void execute() override;

    /// @return `true` if the turret has gone offline. Otherwise scanning continues until
    /// interrupted.
    bool isFinished() const override;

    /// Stops driving the turret motors.
    void end(bool) override;

private:
    /// The global drivers object.
    tap::Drivers *drivers;
    /// The turret being swept.
    TurretSubsystem *turretSubsystem;

    /// Time in milliseconds of the previous iteration, used to measure `dt`.
    uint32_t prevTime = 0;

    /// How long the current yaw heading has been held, in milliseconds. Holding still is what lets
    /// vision get a usable frame before the turret moves on.
    float currentYawChunkTimer = 0;
    /// The yaw heading currently being held, in radians.
    float currentYawSetpoint = 0;

    /// The controller driving yaw to its setpoint.
    algorithms::TurretYawControllerInterface *yawController;
    /// The controller driving pitch to its setpoint.
    algorithms::TurretPitchControllerInterface *pitchController;

    /// Supplies the chassis heading, so the sweep is defined in field terms.
    src::chassis::ChassisOdometry *chassisOdometry;

    /// The bottom of the pitch sweep, in radians.
    float MIN_PITCH_ANGLE;
    /// The top of the pitch sweep, in radians.
    float MAX_PITCH_ANGLE;
    /// How fast pitch sweeps, in radians/second.
    float PITCH_SPEED;
    /// How fast yaw moves between headings, in radians/second.
    float YAW_SPEED;
};
}  // namespace src::control::turret::cv

#endif  // SENTRY_TURRET_USER_CONTROL_COMMAND_HPP_
