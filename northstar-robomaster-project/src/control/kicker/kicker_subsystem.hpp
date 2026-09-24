#ifndef KICKER_SUBSYSTEM_HPP_
#define KICKER_SUBSYSTEM_HPP_

#include "tap/architecture/conditional_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/subsystem.hpp"
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "tap/control/setpoint/interfaces/integrable_setpoint_subsystem.hpp"
#include "tap/util_macros.hpp"

#include "kicker_subsystem_config.hpp"

namespace src
{
class Drivers;
}

namespace src::kicker
{
/**
 * @ingroup hopper_kicker
 *
 * The hero's kicker: a single motor that pushes a 42mm projectile from the hopper into the
 * flywheels.
 *
 * Runs a velocity PID against the motor encoder. Unlike the agitator it is not indexed to discrete
 * shots -- it is simply run at a constant velocity while firing.
 */
class KickerSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Construct a kicker with the passed in velocity PID parameters, gear ratio, and
     * kicker-specific configuration.
     *
     * @param[in] drivers Pointer to the global `tap::Drivers` object.
     * @param[in] pidParams **Velocity** PID configuration struct for the kicker motor controller.
     * @param[in] kickerSubsystemConfig Kicker configuration struct that contains
     * kicker-specific parameters.
     */
    KickerSubsystem(
        tap::Drivers* drivers,
        const tap::algorithms::SmoothPidConfig& pidParams,
        const KickerSubsystemConfig& kickerSubsystemConfig);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override { kickerMotor.setDesiredOutput(0); }

    const char* getName() const override { return "kicker"; }

    /**
     * @return `true` if the kicker motor is online (i.e.: is connected)
     */
    inline bool isOnline() { return kickerMotor.isMotorOnline(); }

    void setSetpoint(float velocity);

    inline float getCurrentVelocity() const { return kickerMotor.getEncoder()->getVelocity(); }

private:
    KickerSubsystemConfig config;

    tap::algorithms::SmoothPid velocityPid;

    /// The velocity setpoint in radians / second
    float velocitySetpoint = 0;

    /// Runes the velocity PID controller
    void runVelocityPidControl();

    uint32_t prevTime = 0;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> kickerMotor;

private:
#else
    tap::motor::DjiMotor kickerMotor;
#endif
};

}  // namespace src::kicker

#endif  // KICKER_SUBSYSTEM_HPP_