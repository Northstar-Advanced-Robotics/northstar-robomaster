#ifndef VELOCITY_AGITATOR_SUBSYSTEM_HPP_
#define VELOCITY_AGITATOR_SUBSYSTEM_HPP_

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

#include "velocity_agitator_subsystem_config.hpp"

namespace src
{
class Drivers;
}

namespace src::agitator
{
/**
 * @ingroup agitator
 *
 * Subsystem whose primary purpose is to encapsulate an agitator motor that operates using a
 * velocity controller. Also keeps track of absolute position to allow commands to rotate the
 * agitator some specific displacement.
 */
class VelocityAgitatorSubsystem : public tap::control::setpoint::IntegrableSetpointSubsystem
{
public:
    /**
     * Agitator gear ratios of different motors, for determining shaft rotation angle.
     */
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;
    static constexpr float AGITATOR_GEAR_RATIO_GM3508 = (3591.0f / 187.0f);

    /**
     * Construct an agitator with the passed in velocity PID parameters, gear ratio, and
     * agitator-specific configuration.
     *
     * @param[in] drivers Pointer to the global `tap::Drivers` object.
     * @param[in] pidParams **Velocity** PID configuration for the agitator motor controller.
     * @param[in] agitatorSubsystemConfig Motor ID, CAN bus, gearing, and jam-detection thresholds.
     *      Note unjamming parameters are **not** here -- those live in
     *      `UnjamSpokeAgitatorCommand::Config`.
     */
    VelocityAgitatorSubsystem(
        tap::Drivers* drivers,
        const tap::algorithms::SmoothPidConfig& pidParams,
        const VelocityAgitatorSubsystemConfig& agitatorSubsystemConfig);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        subsystemJamStatus = false;
        agitatorMotor.setDesiredOutput(0);
    }

    const char* getName() const override { return "velocity agitator"; }

    /// @return The velocity setpoint that some command has requested, in radians / second
    inline float getSetpoint() const override { return velocitySetpoint; }

    /**
     * Sets the velocity setpoint to the specified velocity
     *
     * @param[in] velocity The desired velocity in radians / second.
     */
    void setSetpoint(float velocity) override;

    /// @return The agitator velocity in radians / second.
    inline float getCurrentValue() const override
    {
        return agitatorMotor.getEncoder()->getVelocity();
    }

    /**
     * Meaningless function that nothing uses
     * @return 0
     */
    inline float getJamSetpointTolerance() const override { return 0; }

    /**
     * Zeroes the agitator's accumulated rotation at its current position, so
     * `getCurrentValueIntegral` reads 0 radians from here.
     *
     * @return `true` if the agitator has been successfully calibrated, `false` otherwise (for
     *      example while the motor is offline).
     */
    bool calibrateHere() override;

    /**
     * @return `true` if the jam timer has expired, signaling that the agitator is jammed. Always
     *      `false` when `config.jamLogicEnabled` is unset, regardless of the timer.
     */
    bool isJammed() override { return config.jamLogicEnabled && subsystemJamStatus; }

    /**
     * Clear the jam status of the subsystem, indicating that it has been unjammed.
     */
    inline void clearJam() override
    {
        subsystemJamStatus = false;
        jamChecker.restart();
    }

    /**
     * @return `true` if the agitator has been calibrated (`calibrateHere` has been called and the
     * agitator motor is online).
     */
    inline bool isCalibrated() override { return agitatorIsCalibrated; }

    /**
     * @return `true` if the agitator motor is online (i.e.: is connected)
     */
    inline bool isOnline() override { return agitatorMotor.isMotorOnline(); }

    /**
     * Since we don't keep track of the derivative of the velocity (since the velocity is the
     * setpoint), this function will always return 0.
     *
     * @return 0
     */
    inline float getVelocity() override { return 0; }

    /**
     * @return The calibrated agitator angle, in radians. If the agitator is uncalibrated, 0
     * radians is returned.
     */
    float getCurrentValueIntegral() const override;

private:
    VelocityAgitatorSubsystemConfig config;

    tap::algorithms::SmoothPid velocityPid;

    /// The object that runs jam detection.
    tap::control::setpoint::SetpointContinuousJamChecker jamChecker;

    /// Stores the jam state of the subsystem
    bool subsystemJamStatus = false;

    /**
     * Whether or not the agitator has been calibrated yet. You should calibrate the agitator
     * before using it.
     */
    bool agitatorIsCalibrated = false;

    /// Previous time the velocity controller was called, in milliseconds
    uint32_t prevTime = 0;

    /// The velocity setpoint in radians / second
    float velocitySetpoint = 0;

    /// Runes the velocity PID controller
    void runVelocityPidControl();

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> agitatorMotor;

private:
#else
    tap::motor::DjiMotor agitatorMotor;
#endif
};

}  // namespace src::agitator

#endif  // VELOCITY_AGITATOR_SUBSYSTEM_HPP_