#ifndef SENTRY_CV_MANAGER_COMMAND_HPP_
#define SENTRY_CV_MANAGER_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"
#include "tap/drivers.hpp"

#include "communication/serial/vision_comms.hpp"
#include "control/turret/cv/turret_cv_control_command.hpp"
#include "control/turret/algorithms/turret_controller_interface.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "robot/control_operator_interface.hpp"
#include "robot/sentry/sentry_scan_command.hpp"

namespace src::control::turret::cv
{
/**
 * @ingroup robots
 *
 * Runs the sentry's turret without an operator, switching between auto-aim and scanning.
 *
 * The sentry has nobody to point it, so it scans for targets whenever the vision computer has none
 * and hands control to auto-aim as soon as one appears. Being a comprised command, it owns both
 * inner commands and schedules whichever is appropriate, so the turret is only ever claimed once.
 */
class SentryCvManagerCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] visionComms The vision link, consulted for whether a target is currently tracked.
     * @param[in] sentryTurretSubsystem The turret to control, taken as a subsystem requirement.
     * @param[in] turretCVControlCommand The auto-aim command run while vision has a target.
     * @param[in] yawController The controller driving yaw while scanning.
     * @param[in] pitchController The controller driving pitch while scanning.
     * @param[in] chassisOdometry Supplies the chassis heading for the scan sweep.
     * @param[in] userYawInputScalar Scales manual yaw input, for the case where an operator does
     *      take over.
     * @param[in] userPitchInputScalar Scales manual pitch input.
     * @param[in] MIN_PITCH_ANGLE The bottom of the scan's pitch sweep, in radians.
     * @param[in] MAX_PITCH_ANGLE The top of the scan's pitch sweep, in radians.
     * @param[in] PITCH_SPEED How fast pitch sweeps while scanning, in radians/second.
     * @param[in] YAW_SPEED How fast yaw moves between scan headings, in radians/second.
     */
    SentryCvManagerCommand(
        tap::Drivers *drivers,
        src::serial::VisionComms &visionComms,
        src::control::turret::TurretSubsystem *sentryTurretSubsystem,
        src::control::turret::cv::TurretCVControlCommand &turretCVControlCommand,
        src::control::turret::algorithms::TurretYawControllerInterface *yawController,
        src::control::turret::algorithms::TurretPitchControllerInterface *pitchController,
        src::chassis::ChassisOdometry *chassisOdometry,
        float userYawInputScalar,
        float userPitchInputScalar,
        float MIN_PITCH_ANGLE,
        float MAX_PITCH_ANGLE,
        float PITCH_SPEED,
        float YAW_SPEED);

    /// @return `true` if the turret is online and can be controlled.
    bool isReady() override;

    /// Schedules whichever inner command suits the current vision state.
    void initialize() override;

    /// Switches between auto-aim and scanning as vision acquires or loses a target, scheduling the
    /// newly appropriate inner command and cancelling the other.
    void execute() override;

    /// @return `true` if the turret has gone offline. Otherwise this command runs for the whole
    /// match.
    bool isFinished() const override;

    /**
     * Cancels whichever inner command is running.
     *
     * @param[in] interrupted Ignored; the inner command is cancelled either way.
     */
    void end(bool interrupted) override;

    /// @return The name used to identify this command in logs and the scheduler.
    const char *getName() const override { return "Sentry CV"; }

private:
    /// Drives the turret from vision's aim solution while a target is tracked.
    src::control::turret::cv::TurretCVControlCommand &turretCVControlCommand;
    /// Sweeps the turret looking for targets when vision has none.
    src::control::turret::cv::SentryScanCommand turretScanCommand;
    /// The vision link, consulted for whether a target is currently tracked.
    src::serial::VisionComms &visionComms;
};  // class SentryCvManagerCommand

}  // namespace src::control::turret::cv

#endif  // SENTRY_TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
