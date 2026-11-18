#ifndef ELEVATOR_MOTOR_HPP
#define ELEVATOR_MOTOR_HPP
#include "control/elevator/elevator_config.hpp"

namespace src::control::elevator
{

class ElevatorMotor
{
public:
    void initialize();

    /// Samples the encoder and refreshes the cached chassis-frame angle, scaling by `config.ratio`.
    /// Falls back to `config.startAngle` while the motor is offline. Call once per control loop
    /// iteration, before reading any measurement.
    void updateMotorAngle();

    void setMotorOutput(float out);

    ElevatorConfig& config;
};

}  // namespace src::control::elevator

#endif  // ELEVATOR_MOTOR_HPP