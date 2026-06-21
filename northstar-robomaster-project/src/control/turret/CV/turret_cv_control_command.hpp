#ifndef TURRET_CV_CONTROL_COMMAND_HPP_
#define TURRET_CV_CONTROL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../algorithms/turret_controller_interface.hpp"
#include "../turret_subsystem.hpp"
#include "communication/serial/vision_comms.hpp"
#include "robot/control_operator_interface.hpp"

#include "turret_cv_control_command_template.hpp"

namespace src::control::turret::cv
{
/**
 * Command that takes user input from the `ControlOperatorInterface` to control the pitch and yaw
 * axis of some turret using some passed in yaw and pitch controller upon construction.
 */
class TurretCVControlCommand : public TurretCVControlCommandTemplate
{
public:
    /**
     * @param[in] drivers Pointer to a global drivers object.
     * @param[in] turretSubsystem Pointer to the sentry turret to control.
     * @param[in] yawController Pointer to a yaw controller that will be used to control the yaw
     * axis of the turret.
     * @param[in] pitchController Pointer to a pitch controller that will be used to control the
     * pitch axis of the turret.
     * @param[in] userYawInputScalar Value to scale the user input from `ControlOperatorInterface`
     * by. Basically mouse sensitivity.
     * @param[in] userPitchInputScalar See userYawInputScalar.
     */
    TurretCVControlCommand(
        tap::Drivers *drivers,
        ControlOperatorInterface &controlOperatorInterface,
        src::serial::VisionComms &visionComms,
        TurretSubsystem *turretSubsystem,
        algorithms::TurretYawControllerInterface *yawController,
        algorithms::TurretPitchControllerInterface *pitchController,
        float userYawInputScalar,
        float userPitchInputScalar,
        uint8_t turretID = 0);

    bool isReady() override;

    const char *getName() const override { return "User turret control"; }

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

    bool isAimingWithinLaunchingTolerance([[maybe_unused]] uint8_t turretID) const
    {
        return withinAimingTolerance;
    }

    void setPitchOnlyMode(bool pitchOnly) { pitchOnlyMode = pitchOnly; }

    bool getPitchOnlyMode() const { return pitchOnlyMode; }

private:
    tap::Drivers *drivers;
    ControlOperatorInterface &controlOperatorInterface;
    src::serial::VisionComms &visionComms;
    TurretSubsystem *turretSubsystem;

    uint32_t prevTime = 0;

    algorithms::TurretYawControllerInterface *yawController;
    algorithms::TurretPitchControllerInterface *pitchController;

    const float userYawInputScalar;
    const float userPitchInputScalar;

    const uint8_t turretID;

    float AIMING_TOLERANCE_YAW = .05;
    float AIMING_TOLERANCE_PITCH = .05;

    bool withinAimingTolerance = false;

    bool pitchOnlyMode = false;
};
}  // namespace src::control::turret::cv

#endif  // TURRET_USER_CONTROL_COMMAND_HPP_
