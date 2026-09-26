#pragma once

#include <array>

#include "tap/algorithms/ramp.hpp"
#include "tap/drivers.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "control/turret/turret_motor.hpp"
#endif

namespace src::control::chassis
{
/**
 * @ingroup chassis
 *
 * Which motor is wired where, and how the wheel velocity loop is tuned.
 *
 * Every robot bolts its drive motors to different CAN IDs, so this is supplied per robot from
 * `robot/<target>/<target>_chassis_constants.hpp` rather than hardcoded.
 */
struct ChassisConfig
{
    /// CAN ID of the left front drive motor.
    tap::motor::MotorId leftFrontId;
    /// CAN ID of the left back drive motor.
    tap::motor::MotorId leftBackId;
    /// CAN ID of the right back drive motor.
    tap::motor::MotorId rightBackId;
    /// CAN ID of the right front drive motor.
    tap::motor::MotorId rightFrontId;
    /// The CAN bus all four drive motors are on.
    tap::can::CanBus canBus;
    /// Gains and output limit for the per-wheel velocity PID run in `refresh`.
    modm::Pid<float>::Parameter wheelVelocityPidConfig;
};

/**
 * @ingroup chassis
 *
 * The four-wheel omnidirectional drivetrain, with wheels mounted at 45 degrees on the corners.
 *
 * Resolves a translation and rotation request into four wheel speeds, ramps them so the robot does
 * not draw a current spike, scales them down to stay inside the referee system's power limit, and
 * runs a velocity PID per wheel. A positive `rotational` commands all four motors positive; the
 * right-side motors are mirrored, so this pushes the left side forward and the right side back --
 * a clockwise pivot.
 *
 * Translation is never interpreted in the chassis frame directly -- both drive methods supply a
 * heading and rotate the request into it, which is what makes turret-relative and field-relative
 * driving the same code path. See `driveBasedOnHeading`.
 */
class HolonomicChassisSubsystem : public ChassisSubsystem
{
public:
    /// Index of each drive motor within `motors`, `desiredOutput`, and `pidControllers`.
    enum class MotorId : uint8_t
    {
        LF = 0,
        LB,
        RF,
        RB,
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    /**
     * @param[in] drivers The global drivers object, used for CAN, the IMU, and the referee system.
     * @param[in] config Motor IDs, CAN bus, and wheel velocity PID gains for this robot.
     * @param[in] yawMotor The turret yaw motor, read to find where the turret points relative to
     *      the chassis.
     * @param[in] chassisOdometry Odometry to drive from and feed. Optional: when omitted, the
     *      acceleration taper and rotation budget fall back to the ramped setpoints instead of
     *      measured velocity, and no odometry is updated in `refresh`.
     */
    HolonomicChassisSubsystem(
        tap::Drivers* drivers,
        const ChassisConfig& config,
        src::control::turret::TurretMotor* yawMotor,
        ChassisOdometry* chassisOdometry = nullptr);

    /// Initializes all four drive motors. Must be called before the chassis can be driven.
    void initialize() override;

    void setVelocityTurretDrive(float forward, float sideways, float rotational) override;

    /// Implemented by passing `-getChassisYaw()` as the heading, which rotates the field-frame
    /// request back into the chassis frame.
    void setVelocityFieldDrive(float forward, float sideways, float rotational) override;

    /// Sign is inverted relative to the wheel sum, so the result is counterclockwise positive.
    float getChassisRotationSpeed() override;

    /// Takes the robot's speed from odometry, or from the ramped setpoints when no odometry is
    /// attached.
    float calculateMaxRotationSpeed() override;

    /// Derived from each motor's **commanded** output and measured speed, not from a current
    /// sensor, so it tracks the real draw only as well as the motor model does.
    float getChassisPowerDraw() override;

    /// Runs each wheel's velocity PID against the speeds computed by the last drive call, then
    /// advances the attached odometry. Called once per control loop iteration by the scheduler.
    void refresh() override;

    /// Cuts all four motors' output. Called instead of `refresh` when the remote disconnects, so
    /// the robot coasts to a stop rather than continuing on its last command.
    void refreshSafeDisconnect() override
    {
        for (auto& motor : motors)
        {
            motor.setDesiredOutput(0);
        }
    }

private:
    /**
     * The one drive path: rotates a translation request by `heading` into the chassis frame, ramps
     * it, resolves it to four wheel speeds, and scales those down to fit the power budget.
     *
     * Commands the wheels to a stop if all four motors are offline.
     *
     * @param[in] forward Desired forward velocity in the frame named by `heading`, in m/s.
     * @param[in] sideways Desired leftward velocity in the frame named by `heading`, in m/s.
     * @param[in] rotational Desired rotational velocity, in radians/second, **clockwise**
     *      positive. Not affected by `heading`.
     * @param[in] heading The angle, in radians, that the translation request is expressed relative
     *      to: 0 means it is already in the chassis frame, and larger values rotate it
     *      counterclockwise.
     */
    void driveBasedOnHeading(float forward, float sideways, float rotational, float heading);

    /**
     * Steps a ramp one iteration toward its target, choosing the acceleration limit by whether
     * that means speeding up or slowing down.
     *
     * Braking is allowed to be more aggressive than accelerating, which is why the two limits are
     * separate. Both are **magnitudes**: `Ramp::update` takes the sign from the direction of
     * travel, so passing a negative value here has no effect.
     *
     * @param[in,out] ramp The ramp to advance, in m/s for translation or radians/second for
     *      rotation.
     * @param[in] maxAcceleration Limit applied when the ramp is moving away from zero, in
     *      ramp-units per second squared.
     * @param[in] maxDeceleration Limit applied when the ramp is moving toward zero or reversing,
     *      in ramp-units per second squared.
     * @param[in] dt Time since this ramp was last advanced, in **seconds**.
     */
    static void applyAccelerationToRamp(
        tap::algorithms::Ramp& ramp,
        float maxAcceleration,
        float maxDeceleration,
        float dt);

    /**
     * Converts a desired wheel surface speed into the motor speed that produces it.
     *
     * @param[in] mps Desired speed at the wheel's contact patch, in m/s.
     * @return The corresponding **motor-shaft** speed in RPM -- before the gearbox, not wheel RPM.
     *      This matches the units the velocity PID compares against in `refresh`.
     */
    static float mpsToRpm(float mps)
    {
        return mps / (M_PI * WHEEL_DIAMETER_M) * 60.0f / CHASSIS_GEAR_RATIO;
    }

    static constexpr size_t NUM_MOTORS = static_cast<size_t>(MotorId::NUM_MOTORS);

    /// Per-wheel target speeds in motor RPM, produced by the last drive call and consumed by
    /// `refresh`. Indexed by `MotorId`.
    std::array<float, NUM_MOTORS> desiredOutput{};

    /// Per-wheel velocity controllers. Indexed by `MotorId`.
    std::array<Pid, NUM_MOTORS> pidControllers;

    /// Rate limiters for the three velocity axes, in order: forward, sideways, rotational. Ramping
    /// the request rather than stepping it is what keeps the current draw bounded.
    std::array<tap::algorithms::Ramp, 3> rampControllers;

protected:
    /// The four drive motors. Indexed by `MotorId`.
    std::array<Motor, NUM_MOTORS> motors;
};  // class HolonomicChassisSubsystem
}  // namespace src::control::chassis
