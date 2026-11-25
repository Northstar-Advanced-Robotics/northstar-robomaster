#ifndef ELEVATOR_SUBSYSTEM_HPP
#define ELEVATOR_SUBSYSTEM_HPP
#include "tap/algorithms/smooth_pid.hpp"
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

    /// Called once per control loop iteration by the scheduler. Updates the encoder and runs the
    /// height PID toward targetPosition (see TODO(PID step 4) in the .cpp). Commands never drive
    /// the motor directly. They only call setTargetPosition().
    void refresh() override;

    void refreshSafeDisconnect() override;

    void setTargetPosition(float position);

private:
    ElevatorMotor* elevator_motor;
    float targetPosition = 0.0f;

    // TODO(PID step 3): add the loop's state.
    //   (needs #include "tap/algorithms/smooth_pid.hpp")
    //
    tap::algorithms::SmoothPid heightPid;
    //       Construct it in the constructor's initializer list: heightPid(config.pidConfig)
    float kG;                   // copy from config.kG in the constructor
    uint32_t prevTimeMs = 0;    // time of the last refresh(), used to compute dt
    float homingOutput = 0.0f;  // raw output sent while un-homed. Only the home command sets it,
    //                              // through a public setHomingOutput(float).
};

}  // namespace src::control::elevator
#endif  // ELEVATOR_SUBSYSTEM_HPP