#ifndef ELEVATOR_CONFIG_HPP
#define ELEVATOR_CONFIG_HPP

namespace src::control::elevator
{

struct ElevatorConfig
{
    // double maxHeight;     // Maximum height of the elevator in meters
    // double minHeight;     // Minimum height of the elevator in meters
    double maxSpeed;      // Maximum speed of the elevator in meters per second
    double acceleration;  // Acceleration of the elevator in meters per second squared
    double deceleration;  // Deceleration of the elevator in meters per second squared

private:
    double maxHeight = 2.0;  // Enter the maximum height of the elevator in meters
    double minHeight = 0.0;  // Enter the minimum height of the elevator in meters
};

}  // namespace src::control::elevator
#endif  // ELEVATOR_CONFIG_HPP