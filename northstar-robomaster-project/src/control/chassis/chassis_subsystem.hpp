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
    /// Gains for the per-wheel velocity PID run in `refresh`.
    modm::Pid<float>::Parameter wheelVelocityPidConfig;
};

/**
 * @ingroup chassis
 *
 * The four-wheel omnidirectional drivetrain.
 *
 * Callers ask for a translation and a rotation; this class resolves that into four wheel speeds,
 * ramps them so the robot does not draw a current spike, scales them down to stay inside the
 * referee system's power limit, and runs a velocity PID per wheel.
 *
 * **Coordinate frame.** This class defines the robot's canonical frame, and it is right-handed:
 *
 * - **+X is forward**
 * - **+Y is left**
 * - **+heading is counterclockwise** seen from above
 *
 * `ChassisOdometry` reports in this same frame, so its positions and velocities can be passed to
 * the drive methods as-is, with no axis swap.
 *
 * @warning **The `rotational` argument of the drive methods is clockwise positive**, the opposite
 *      of the heading convention above. A positive value commands all four motors positive; the
 *      right-side motors are mirrored, so this pushes the left side forward and the right side
 *      back -- a clockwise pivot. Headings, `getChassisYaw`, `getDifferenceToTargetAngle`, and
 *      the *measured* `getChassisRotationSpeed` are all counterclockwise positive, so a
 *      counterclockwise quantity must be negated before it is passed in as `rotational`.
 * `ChassisOrientDriveCommand` gets this right by feeding in `getChassisZeroTurret()`, which is
 * already the clockwise error.
 *
 * Translation is never interpreted in this frame directly -- every drive method takes a heading and
 * rotates the request into it, which is what makes turret-relative and field-relative driving the
 * same code path. See `driveBasedOnHeading`.
 */
class ChassisSubsystem : public tap::control::Subsystem
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

    /// @warning Currently unreferenced. The wheel speed ceiling actually enforced is
    /// `MAX_CHASSIS_WHEEL_SPEED` in this robot's chassis constants.
    static constexpr float MAX_WHEELSPEED_RPM = 9000;

    /**
     * @param[in] drivers The global drivers object, used for CAN, the IMU, and the referee system.
     * @param[in] config Motor IDs, CAN bus, and wheel velocity PID gains for this robot.
     * @param[in] yawMotor The turret yaw motor, read to find where the turret points relative to
     *      the chassis.
     * @param[in] chassisOdometry_ Odometry to drive from and feed. Optional: when omitted, the
     *      acceleration taper and rotation budget fall back to the ramped setpoints instead of
     *      measured velocity, and no odometry is updated in `refresh`.
     */
    ChassisSubsystem(
        tap::Drivers* drivers,
        const ChassisConfig& config,
        src::control::turret::TurretMotor* yawMotor,
        ChassisOdometry* chassisOdometry_ = nullptr);

    /// Initializes all four drive motors. Must be called before the chassis can be driven.
    void initialize() override;

    /**
     * Drives with translation interpreted relative to where the turret points.
     *
     * Pushing "forward" moves the robot the way the operator is looking down the turret, whatever
     * direction the chassis happens to face.
     *
     * @param[in] forward Desired velocity along the turret's forward axis, in m/s.
     * @param[in] sideways Desired velocity to the turret's left, in m/s.
     * @param[in] rotational Desired rotational velocity, in radians/second, **clockwise**
     *      positive. See the class-level warning.
     */
    mockable void setVelocityTurretDrive(float forward, float sideways, float rotational);

    /**
     * Drives with translation interpreted relative to the field.
     *
     * "Forward" is a fixed direction down the field regardless of how the chassis or turret are
     * oriented, which keeps driving intuitive when the operator loses track of the robot's facing.
     * The field direction is the chassis IMU's yaw origin, fixed at calibration. Implemented by
     * passing `-getChassisYaw()` as the heading, which rotates the field-frame request back into
     * the chassis frame.
     *
     * Takes the same frame `ChassisOdometry` reports in, so odometry-derived velocities can be
     * passed straight through.
     *
     * @param[in] forward Desired velocity along the field's forward axis, in m/s.
     * @param[in] sideways Desired velocity to the field's left, in m/s.
     * @param[in] rotational Desired rotational velocity, in radians/second, **clockwise**
     *      positive. See the class-level warning.
     */
    mockable void setVelocityFieldDrive(float forward, float sideways, float rotational);

    /**
     * The one drive path: rotates a translation request by `heading` into the chassis frame, ramps
     * it, resolves it to four wheel speeds, and scales those down to fit the power budget.
     *
     * Both `setVelocityTurretDrive` and `setVelocityFieldDrive` are thin wrappers that differ only
     * in the heading they supply. Commands the wheels to a stop if all four motors are offline.
     *
     * @param[in] forwards Desired forward velocity in the frame named by `heading`, in m/s.
     * @param[in] sideways Desired leftward velocity in the frame named by `heading`, in m/s.
     * @param[in] rotational Desired rotational velocity, in radians/second, **clockwise**
     *      positive. Not affected by `heading`.
     * @param[in] heading The angle, in radians, that the translation request is expressed relative
     *      to: 0 means it is already in the chassis frame, and larger values rotate it
     *      counterclockwise.
     */
    void driveBasedOnHeading(float forwards, float sideways, float rotational, float heading);

    /**
     * Rotation controller used to hold the chassis at a target angle under operator control.
     *
     * Proportional on the angle error, derivative on the IMU's measured yaw rate. Errors under 3
     * degrees are treated as zero, so the chassis settles instead of hunting.
     *
     * @param[in] angleOffset How far the chassis is from the target angle, in radians. The output
     *      follows this sign, so pass the **clockwise** error (as `getChassisZeroTurret` returns)
     *      to get a value usable as `rotational`.
     * @return The rotational velocity to command, in radians/second, clamped to
     *      `CHASSIS_ROTATION_MAX_VEL`. Zero inside the deadzone.
     */
    float chassisSpeedRotationPID(float angleOffset);

    /**
     * Rotation controller used while following an auto-drive path.
     *
     * Same shape as `chassisSpeedRotationPID` but tuned harder and with no deadzone, and it takes
     * its derivative from the wheel-derived rotation speed rather than the IMU.
     *
     * @warning The derivative term is `-getChassisRotationSpeed()`, which is counterclockwise
     *      positive, so this controller is only consistent when `angleOffset` is the
     *      **counterclockwise** error -- and its output is then counterclockwise positive, the
     *      opposite of what `rotational` expects. Negate the whole output (not just the input,
     *      which would flip P but not D) before driving with it. `ChassisAutoDrive` currently does
     *      not; see its `calculateRotationToFacePoint`.
     *
     * @param[in] angleOffset How far the chassis is from the target angle, in radians,
     *      counterclockwise positive.
     * @return The rotational velocity to command, in radians/second, counterclockwise positive,
     *      clamped to `CHASSIS_ROTATION_MAX_VEL`.
     */
    float chassisSpeedRotationAutoDrivePID(float angleOffset);

    /**
     * How much rotational speed is left over once translation has been paid for.
     *
     * Translation and rotation compete for the same finite wheel speed, so beyblade uses this to
     * spin as fast as the remaining budget allows.
     *
     * @param[in] forward Ignored. The implementation takes the robot's speed from odometry (or
     *      from the ramped setpoints when no odometry is attached) rather than from this argument,
     *      so passing a different value here changes nothing.
     * @param[in] sideways Ignored, as above.
     * @return The largest rotational speed the chassis can still produce, in radians/second.
     *      A magnitude: never negative, and valid in either direction.
     */
    float calculateMaxRotationSpeed(float forward, float sideways);

    /**
     * @return The chassis' **measured** rotational speed, in radians/second, derived from the four
     *      wheel encoders rather than from the commanded setpoint or the IMU. Sign is inverted
     *      relative to the wheel sum, so the result is counterclockwise positive.
     */
    float getChassisRotationSpeed();

    /**
     * @return The angle from the chassis' forward axis to the turret's, in radians, wrapped to
     *      (-pi, pi], measured **clockwise** (it is the negated turret yaw). Zero when the turret
     *      points straight ahead. Because it is clockwise, it can be fed through
     *      `chassisSpeedRotationPID` and passed as `rotational` directly, which is how
     *      `ChassisOrientDriveCommand` squares the chassis up with the turret.
     */
    float getChassisZeroTurret();

    /**
     * @param[in] drivers The global drivers object.
     * @return This robot's chassis power budget in watts, as most recently reported by the referee
     *      system. Stale or zero if the referee system is offline; callers should pair this with
     *      `refSerial.getRefSerialReceivingData()`, as `getMaxWheelSpeed` does.
     */
    static inline float getChassisPowerLimit(tap::Drivers* drivers)
    {
        return drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    }

    /**
     * @return An estimate of the power the drivetrain is currently drawing, in watts, summed over
     *      the four motors. Derived from each motor's **commanded** output and measured speed, not
     *      from a current sensor, so it tracks the real draw only as well as the motor model does.
     */
    float getChassisPowerDraw();

    /**
     * Converts a power budget into the wheel speed that fits inside it, by interpolating the
     * measured `CHASSIS_POWER_TO_MAX_SPEED_LUT`.
     *
     * @param[in] refSerialOnline Whether the referee system is currently sending data. When
     *      `false`, `chassisPowerLimit` is **ignored** and a conservative 75 W is assumed, so an
     *      unplugged referee system slows the robot rather than letting it draw freely.
     * @param[in] chassisPowerLimit The chassis power budget, in watts.
     * @return The per-wheel speed ceiling, in **motor-shaft** RPM. The result is cached and only
     *      recomputed when the power limit changes, since this is called several times per
     *      iteration and the limit rarely moves.
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
     * Steps a ramp one iteration toward its target, choosing the acceleration limit by whether
     * that means speeding up or slowing down.
     *
     * Braking is allowed to be more aggressive than accelerating, which is why the two limits are
     * separate. Both are **magnitudes**: `Ramp::update` takes the sign from the direction of
     * travel, so passing a negative value here has no effect.
     *
     * @param[in,out] ramp The ramp to advance. Its units are whatever the caller ramps -- this
     *      class uses m/s for translation and radians/second for rotation.
     * @param[in] maxAcceleration Limit applied when the ramp is moving away from zero, in
     *      ramp-units per second squared.
     * @param[in] maxDeceleration Limit applied when the ramp is moving toward zero or reversing,
     *      in ramp-units per second squared. A magnitude, despite the name.
     * @param[in] dt Time since this ramp was last advanced, in **seconds**.
     */
    static inline void applyAccelerationToRamp(
        tap::algorithms::Ramp& ramp,
        float maxAcceleration,
        float maxDeceleration,
        float dt)
    {
        if (tap::algorithms::getSign(ramp.getTarget()) ==
                tap::algorithms::getSign(ramp.getValue()) &&
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

    /// @return The odometry this chassis feeds, or `nullptr` if none was attached.
    ChassisOdometry* getChassisOdometry() { return chassisOdometry; }

    /// Runs each wheel's velocity PID against the speeds computed by the last drive call, then
    /// advances the attached odometry. Called once per control loop iteration by the scheduler.
    void refresh() override;

    /// Cuts all four motors' output. Called instead of `refresh` when the remote disconnects, so
    /// the robot coasts to a stop rather than continuing on its last command.
    void refreshSafeDisconnect() override
    {
        for (size_t i = 0; i < motors.size(); i++)
        {
            motors[i].setDesiredOutput(0);
        }
    }

    /// @return The name used to identify this subsystem in logs and the scheduler.
    virtual const char* getName() const override { return "Chassis"; }

    /**
     * @return Where the chassis points, in radians, wrapped to (-pi, pi]. The IMU sits on the
     *      turret, so the turret's yaw relative to the chassis is subtracted out.
     *
     * @note This is the raw IMU heading. `ChassisOdometry::calculateRobotHeading` additionally
     *      applies the vision localization offset, so the two disagree after an AprilTag fix.
     */
    inline float getChassisYaw()
    {
        return modm::Angle::normalize(drivers->bmi088.getYaw() - getTurretYaw());
    }

    /**
     * @param[in] targetAngle The heading to compare against, in radians.
     * @return How far the chassis must rotate to reach `targetAngle`, in radians, wrapped to
     *      (-pi, pi]. Positive means counterclockwise.
     */
    inline float getDifferenceToTargetAngle(float targetAngle)
    {
        return modm::Angle::normalize(targetAngle - getChassisYaw());
    }

    /**
     * Records whether the robot is sprinting.
     *
     * @param[in] sprinting `true` while sprinting.
     * @warning The flag this sets is currently never read, so calling this has no effect on how
     *      the chassis drives.
     */
    void setIsSprinting(bool sprinting) { isSprinting = sprinting; }

    /**
     * @return The average wheel speed, ignoring direction, in **motor-shaft** RPM -- not wheel RPM,
     *      despite the name. Averaging the absolute values means a stationary robot that is
     *      beyblading still reports a large value.
     */
    float getWheelRpm()
    {
        float wheelSum = abs(motors.at(0).getEncoder()->getVelocity()) +
                         abs(motors.at(1).getEncoder()->getVelocity()) +
                         abs(motors.at(2).getEncoder()->getVelocity()) +
                         abs(motors.at(3).getEncoder()->getVelocity());
        return wheelSum / 4.0f * 60.0f / M_TWOPI / CHASSIS_GEAR_RATIO;
    }

    /// Set by `ChassisBeybladeCommand` while it is scheduled. Read by the HUD to decide whether to
    /// nag the operator to start spinning. An *input* to this class, not something it computes.
    bool beyBladeCommandRunning{false};
    /// Set by beyblade and the sentry state machine while the robot is spinning but barely
    /// translating. Raises the power budget in `driveBasedOnHeading`, since wheel speed not spent
    /// on translation is available for rotation. Also an input.
    bool isBeybladingOnly{false};
    /// Computed by `driveBasedOnHeading`: `true` while the robot has meaningful sideways velocity.
    /// An *output*, read by the HUD's peeking lines.
    bool isPeeking{false};
    /// Computed alongside `isPeeking`: `true` when that sideways motion is to the chassis' left.
    bool isPeekingLeft{false};
    /// @warning Currently unreferenced -- never read and never written.
    int linearVelocity{0};

private:
    /**
     * Converts a desired wheel surface speed into the motor speed that produces it.
     *
     * @param[in] mps Desired speed at the wheel's contact patch, in m/s.
     * @return The corresponding **motor-shaft** speed in RPM -- before the gearbox, not wheel RPM.
     *      Dividing by `CHASSIS_GEAR_RATIO` (187/3591) is what scales wheel revolutions up to
     *      motor revolutions. This matches the units the velocity PID compares against in
     *      `refresh`.
     */
    inline float mpsToRpm(float mps)
    {
        return mps / (M_PI * src::control::chassis::WHEEL_DIAMETER_M) * 60.0f / CHASSIS_GEAR_RATIO;
    }

    /// Caches the last power-limit-to-wheel-speed interpolation, keyed by the power limit that
    /// produced it, so `getMaxWheelSpeed` can skip the lookup on the common unchanged case.
    static modm::Pair<int, float> lastComputedMaxWheelSpeed;

    /// Odometry driven from the wheel encoders in `refresh`, and read back for the acceleration
    /// taper and rotation budget. `nullptr` if none was attached.
    src::control::chassis::ChassisOdometry* chassisOdometry;

    /// The turret yaw motor, used to relate the chassis frame to the turret's.
    src::control::turret::TurretMotor* yawMotor;

    /// @warning Written by `setIsSprinting` but never read.
    bool isSprinting{false};

    /// Per-wheel target speeds in motor RPM, produced by the last drive call and consumed by
    /// `refresh`. Indexed by `MotorId`.
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> desiredOutput;

    /// Per-wheel velocity controllers. Indexed by `MotorId`.
    std::array<Pid, static_cast<uint8_t>(MotorId::NUM_MOTORS)> pidControllers;

    /// Rate limiters for the three velocity axes, in order: forward, sideways, rotational. Ramping
    /// the request rather than stepping it is what keeps the current draw bounded.
    std::array<tap::algorithms::Ramp, static_cast<uint8_t>(3)> rampControllers;

    /**
     * @return The turret's yaw relative to the chassis, in radians, wrapped to (-pi, pi]. Zero when
     *      the turret points along the chassis' forward axis.
     */
    float getTurretYaw();

protected:
    /// The four drive motors. Indexed by `MotorId`.
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> motors;
};  // class ChassisSubsystem
}  // namespace src::control::chassis
