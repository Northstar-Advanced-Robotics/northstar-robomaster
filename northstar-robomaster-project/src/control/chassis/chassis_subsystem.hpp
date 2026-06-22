#pragma once

#include <array>

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
#include "control/chassis/rate_limiters/slew_rate_limiter.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"

#define FIELD

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "control/turret/turret_motor.hpp"
#endif
#include "chassis_odometry.hpp"

namespace src::chassis
{
struct ChassisConfig
{
    tap::motor::MotorId leftFrontId;
    tap::motor::MotorId leftBackId;
    tap::motor::MotorId rightBackId;
    tap::motor::MotorId rightFrontId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter wheelVelocityPidConfig;
};

/**
 * Represents a chassis subsystem
 */
class ChassisSubsystem : public tap::control::Subsystem
{
public:
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

    static constexpr float MAX_WHEELSPEED_RPM = 9000;

    ChassisSubsystem(
        tap::Drivers* drivers,
        const ChassisConfig& config,
        src::control::turret::TurretMotor* yawMotor,
        ChassisOdometry* chassisOdometry_ = nullptr);

    void initialize() override;

    /**
     * Sets the desired forward, sideways, and rotational velocity in relation to the turret.
     *
     * @param forward The desired forward velocity in MPS.
     * @param sideways The desired sideways velocity in MPS.
     * @param rotational The desired rotational velocity. Radians Per Second.
     */
    mockable void setVelocityTurretDrive(float forward, float sideways, float rotational);

    /**
     * Sets the desired forward, sideways, and rotational velocity in relation to a zero'd angle.
     *
     * @param forward The desired forward velocity in MPS.
     * @param sideways The desired sideways velocity in MPS.
     * @param rotational The desired rotational velocity. Radians Per Second.
     */
    mockable void setVelocityFieldDrive(float forward, float sideways, float rotational);

    /**
     * Sets the desired forward, sideways, and rotational velocity based on a heading.
     * The other drive methods use this one.
     *
     * @param forward The desired forward velocity in MPS.
     * @param sideways The desired sideways velocity in MPS.
     * @param rotational The desired rotational velocity. Radians Per Second.
     * @param heading The desired heading. Whatever heading is passed in will be positive forward.
     */
    void driveBasedOnHeading(float forwards, float sideways, float rotational, float heading);

    /**
     * Gets a rotation speed for the chassis based on a distance from a desired angle.
     *
     * @param angleOffset Distance away from the target angle.
     * @return A chassis speed in Radians per Second
     */
    float chassisSpeedRotationPID(float angleOffset);

    float chassisSpeedRotationAutoDrivePID(float angleOffset);

    /**
     * Calculates the max rotation speed based on the max chassis wheel speed.
     *
     * @param forward The forward commanded chassis velocity.
     * @param sideways The sideways commanded chassis velocity.
     * @return the maximum rotational speed to use based on forward and sideways velocity.
     */
    float calculateMaxRotationSpeed(float forward, float sideways);

    /**
     * Calculates the chassis rotational speed.
     *
     * @return The chassis rotational speed in Radians per Second.
     */
    float getChassisRotationSpeed();

    /**
     * Calculates the angluar distance from the chassis zero and the turret angle.
     *
     * @return The angular distance from the chassis zero to the turret angle.
     */
    float getChassisZeroTurret();

    /**
     * Gets the chassis power limit from the ref system.
     *
     * @return The max chassis power limit.
     */
    static inline float getChassisPowerLimit(tap::Drivers* drivers)
    {
        return drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    }

    float getChassisPowerDraw();

    /**
     * Gets the max chassis wheel speed based on a power limit.
     *
     * @param refSerialOnline Is the ref system online and connected.
     * @param chassisPowerLimit The chassis power limit in watts.
     * @return The max chassis wheel speed.
     */
    static inline float getMaxWheelSpeed(bool refSerialOnline, float chassisPowerLimit)
    {
        if (!refSerialOnline)
        {
            chassisPowerLimit = 75;
        }

        // only re-interpolate when needed (since this function is called a lot and the chassis
        // power limit rarely changes, this helps cut down on unnecessary array
        // searching/interpolation)
        if (lastComputedMaxWheelSpeed.first != (int)chassisPowerLimit)
        {
            lastComputedMaxWheelSpeed.first = (int)chassisPowerLimit;
            lastComputedMaxWheelSpeed.second =
                CHASSIS_POWER_TO_SPEED_INTERPOLATOR.interpolate(chassisPowerLimit);
        }

        return lastComputedMaxWheelSpeed.second;
    }

    /**
     * @param[out] ramp Ramp that should have acceleration applied to. The ramp is updated some
     * increment based on the passed in acceleration values. Ramp stores values in some units.
     * @param[in] maxAcceleration Positive acceleration value to apply to the ramp in units/time^2.
     * @param[in] maxDeceleration Negative acceleration value to apply to the ramp, in units/time^2.
     * @param[in] dt Change in time since this function was last called, in units of some time.
     */
    static inline void applyAccelerationToRamp(
        tap::algorithms::Ramp& ramp,
        float maxAcceleration,
        float maxDeceleration,
        float dt)
    {
        if (getSign(ramp.getTarget()) == getSign(ramp.getValue()) &&
            abs(ramp.getTarget()) > abs(ramp.getValue()))
        {
            // we are trying to speed up
            ramp.update(maxAcceleration * dt);
        }
        else
        {
            // we are trying to slow down
            ramp.update(maxDeceleration * dt);
        }
    }

    ChassisOdometry* getChassisOdometry() { return chassisOdometry; }

    void refresh() override;

    void refreshSafeDisconnect() override
    {
        for (size_t i = 0; i < motors.size(); i++)
        {
            motors[i].setDesiredOutput(0);
        }
    }

    virtual const char* getName() const override { return "Chassis"; }

    inline float getChassisYaw()
    {
        return modm::Angle::normalize(drivers->bmi088.getYaw() - getTurretYaw());
    }

    inline float getDifferenceToTargetAngle(float targetAngle)
    {
        return modm::Angle::normalize(targetAngle - getChassisYaw());
    }

    void setIsSprinting(bool sprinting) { isSprinting = sprinting; }

    float getWheelRpm()
    {
        float wheelSum = abs(motors.at(0).getEncoder()->getVelocity()) +
                         abs(motors.at(1).getEncoder()->getVelocity()) +
                         abs(motors.at(2).getEncoder()->getVelocity()) +
                         abs(motors.at(3).getEncoder()->getVelocity());
        return wheelSum / 4.0f * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO;
    }

    bool isBeybladingOnly{false};
    bool isPeeking{false};
    bool isPeekingLeft{false};
    int linearVelocity{0};

private:
    /**
     * Gets the wheel RPM from a desired mps
     *
     * @param mps Desired wheel mps
     * @returns Calcualted R wheel RPM
     */
    inline float mpsToRpm(float mps)
    {
        return mps / (M_PI * src::chassis::WHEEL_DIAMETER_M) * 60.0f / CHASSIS_GEAR_RATIO;
    }

    static modm::Pair<int, float> lastComputedMaxWheelSpeed;

    src::chassis::ChassisOdometry* chassisOdometry;

    src::control::turret::TurretMotor* yawMotor;

    bool isSprinting{false};

    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> desiredOutput;

    std::array<Pid, static_cast<uint8_t>(MotorId::NUM_MOTORS)> pidControllers;

    std::array<tap::algorithms::Ramp, static_cast<uint8_t>(3)> rampControllers;

    /**
     * @return The turret yaw angle in radians.
     */
    inline float getTurretYaw();

protected:
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> motors;
};  // class ChassisSubsystem
}  // namespace src::chassis
