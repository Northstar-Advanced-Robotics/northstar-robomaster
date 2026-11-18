#include "elevator_subsystem.hpp"

namespace src::control::elevator
{
ElevatorSubsystem::ElevatorSubsystem(
    tap::Drivers* drivers,
    const ElevatorConfig& config,
    ElevatorMotor* elevator_motor)
    : tap::control::Subsystem(drivers),
      elevator_motor(elevator_motor)

          void ElevatorSubsystem::initialize()
{
    elevator_motor->initialize();
}

void ElevatorSubsystem::refresh()
{
    elevator_motor->updateMotorAngle();
}  // namespace src::control::elevator

void ElevatorSubsystem::refreshSafeDisconnect() { elevator_motor->setMotorOutput(0.0f); }

double targetHeight = 10.0;  // Placeholder for the desired target height of the elevator in meters

if (targetHeight > config.maxHeight)
{
    while (ElevatorMotor.getCurrentHeight() < config.maxHeight)
    {
        ElevatorMotor.setMotorOutput(config.acceleration);  // Move up at acceleration rate
    }
    while (ElevatorMotor.getCurrentHeight() > config.maxHeight)
    {
        ElevatorMotor.setMotorOutput(-config.deceleration);  // Move down at deceleration rate
    }
}
else if (targetHeight < config.minHeight)
{
    while (ElevatorMotor.getCurrentHeight() < config.minHeight)
    {
        ElevatorMotor.setMotorOutput(config.acceleration);  // Move up at acceleration rate
    }
    while (ElevatorMotor.getCurrentHeight() > config.minHeight)
    {
        ElevatorMotor.setMotorOutput(-config.deceleration);  // Move down at deceleration rate
    }
}
else
{
    while (ElevatorMotor.getCurrentHeight() < targetHeight)
    {
        ElevatorMotor.setMotorOutput(config.acceleration);  // Move up at acceleration rate
    }
    while (ElevatorMotor.getCurrentHeight() > targetHeight)
    {
        ElevatorMotor.setMotorOutput(-config.deceleration);  // Move down at deceleration rate
    }
}

}  // namespace src::control::elevator