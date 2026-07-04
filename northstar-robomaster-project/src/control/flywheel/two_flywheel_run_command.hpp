#ifndef TWO_FLYWHEEL_RUN_COMMAND
#define TWO_FLYWHEEL_RUN_COMMAND

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/command.hpp"

#include "control/flywheel/two_flywheel_subsystem.hpp"

namespace src::control::flywheel
{
class TwoFlywheelRunCommand : public tap::control::Command
{
public:
    TwoFlywheelRunCommand(
        TwoFlywheelSubsystem *flywheel,
        float launchSpeed,
        tap::communication::serial::RefSerial *refSerial);

    const char *getName() const override { return "Flywheel Run Command"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    TwoFlywheelSubsystem *flywheel;

    float launchSpeed;

    tap::communication::serial::RefSerial *refSerial;

#ifdef TARGET_HERO
    float upperLimit = 14.5f;
    float lowerLimit = 13.5f;
    float increment = 0.1f;
    float decrement = 0.1f;
    float lastShotSpeed = 14;
#else
    float upperLimit = 24.5f;
    float lowerLimit = 23.5f;
    float increment = 0.1f;
    float decrement = 0.1f;
    float lastShotSpeed = 24;
#endif
};
}  // namespace src::control::flywheel
#endif  // FLYWHEEL_RUN_COMMAND
