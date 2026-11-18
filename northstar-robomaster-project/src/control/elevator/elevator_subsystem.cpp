#include "elevator_subsystem.hpp"

namespace src::control::elevator
{
ElevatorSubsystem::ElevatorSubsystem(
    tap::Drivers* drivers,
    const ElevatorConfig& config,
    ElevatorMotor* elevator_motor)
    : tap::control::Subsystem(drivers),
      elevator_motor(elevator_motor)
{
}

void ElevatorSubsystem::initialize() { elevator_motor->initialize(); }

void ElevatorSubsystem::refresh()
{
    elevator_motor->updateMotorAngle();
}  // namespace src::control::elevator

void ElevatorSubsystem::refreshSafeDisconnect() { elevator_motor->setMotorOutput(0.0f); }

void ElevatorSubsystem::setTargetPosition(float position)
{
    targetPosition = position;
    if (position > elevator_motor->config.maxHeight)
        targetPosition = elevator_motor->config.maxHeight;
    if (position < elevator_motor->config.maxHeight)
        targetPosition = elevator_motor->config.minHeight;
}

}  // namespace src::control::elevator