#ifndef ELEVATOR_SUBSYSTEM_HPP
#define ELEVATOR_SUBSYSTEM_HPP
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "control/elevator/motors/elevator_motor.hpp"

#include "elevator_config.hpp"

namespace src::control::elevator
{
class ElevatorSubsystem : public tap::control::Subsystem
{
public:
    ElevatorSubsystem(
        tap::Drivers* drivers,
        const ElevatorConfig& config,
        ElevatorMotor* elevator_motor);

    void initialize() override;

    /// Refreshes both axes' cached encoder angles. Called once per control loop iteration by the
    /// scheduler. Note it does **not** run the control loops -- the attached controllers do that,
    /// driven by whichever command is scheduled.
    void refresh() override;

    void refreshSafeDisconnect() override;

private:
    ElevatorMotor* elevator_motor;
}

}  // namespace src::control::elevator
#endif  // ELEVATOR_SUBSYSTEM_HPP