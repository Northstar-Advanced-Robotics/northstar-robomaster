#ifndef SENTRY_SCAN_COMMAND_HPP
#define SENTRY_SCAN_COMMAND_HPP

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "control/chassis/chassis_odometry.hpp"
#include "control/turret/algorithms/turret_controller_interface.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::robot::sentry
{
class SentryScanCommand : public tap::control::Command
{
public:
    SentryScanCommand(
        tap::Drivers *drivers,
        src::control::turret::TurretSubsystem *turretSubsystem,
        src::control::turret::TurretYawControllerInterface *yawController,
        src::control::turret::TurretPitchControllerInterface *pitchController,
        src::control::chassis::ChassisOdometry *chassisOdometry,
        float MIN_PITCH_ANGLE,
        float MAX_PITCH_ANGLE,
        float PITCH_SPEED,
        float YAW_SPEED);

    bool isReady() override;

    const char *getName() const override { return "Sentry scan"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool) override;

private:
    tap::Drivers *drivers;
    src::control::turret::TurretSubsystem *turretSubsystem;

    uint32_t prevTime = 0;

    float currentYawChunkTimer = 0;
    float currentYawSetpoint = 0;

    src::control::turret::TurretYawControllerInterface *yawController;
    src::control::turret::TurretPitchControllerInterface *pitchController;

    src::control::chassis::ChassisOdometry *chassisOdometry;

    float MIN_PITCH_ANGLE;
    float MAX_PITCH_ANGLE;
    float PITCH_SPEED;
    float YAW_SPEED;
};
}  // namespace src::robot::sentry

#endif  // SENTRY_TURRET_USER_CONTROL_COMMAND_HPP_
