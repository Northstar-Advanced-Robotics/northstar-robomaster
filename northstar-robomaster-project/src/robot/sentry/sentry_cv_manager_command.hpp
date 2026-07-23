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
class SentryCvManagerCommand : public tap::control::ComprisedCommand
{
public:
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

    bool isReady() override;

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

    const char *getName() const override { return "Sentry CV"; }

private:
    src::control::turret::cv::TurretCVControlCommand &turretCVControlCommand;
    src::control::turret::cv::SentryScanCommand turretScanCommand;
    src::serial::VisionComms &visionComms;
};  // class SentryCvManagerCommand

}  // namespace src::control::turret::cv

#endif  // SENTRY_TURRET_USER_WORLD_RELATIVE_COMMAND_HPP_
