#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/math/geometry/angle.hpp"

#include "chassis_odometry.hpp"

namespace src::control::turret
{
class TurretMotor;
}

namespace src::control::chassis
{
/**
 * @ingroup chassis
 *
 * The drivetrain-agnostic interface every chassis implements.
 *
 * Callers ask for a translation and a rotation; how that becomes wheel commands is up to the
 * concrete drivetrain (see `HolonomicChassisSubsystem`). Commands, the HUD, and the state machine
 * hold a `ChassisSubsystem*` and should only need what is declared here.
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
 *      of the heading convention above. Headings, `getChassisYaw`, `getDifferenceToTargetAngle`,
 *      and the *measured* `getChassisRotationSpeed` are all counterclockwise positive, so a
 *      counterclockwise quantity must be negated before it is passed in as `rotational`.
 * `ChassisOrientDriveCommand` gets this right by feeding in `getChassisZeroTurret()`, which is
 * already the clockwise error.
 */
class ChassisSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param[in] drivers The global drivers object, used for CAN, the IMU, and the referee system.
     * @param[in] yawMotor The turret yaw motor, read to find where the turret points relative to
     *      the chassis.
     * @param[in] chassisOdometry Odometry to drive from and feed. Optional; may be `nullptr`.
     */
    ChassisSubsystem(
        tap::Drivers* drivers,
        src::control::turret::TurretMotor* yawMotor,
        ChassisOdometry* chassisOdometry = nullptr)
        : Subsystem(drivers),
          chassisOdometry(chassisOdometry),
          yawMotor(yawMotor)
    {
    }

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
    virtual void setVelocityTurretDrive(float forward, float sideways, float rotational) = 0;

    /**
     * Drives with translation interpreted relative to the field.
     *
     * "Forward" is a fixed direction down the field regardless of how the chassis or turret are
     * oriented. The field direction is the chassis IMU's yaw origin, fixed at calibration.
     *
     * Takes the same frame `ChassisOdometry` reports in, so odometry-derived velocities can be
     * passed straight through.
     *
     * @param[in] forward Desired velocity along the field's forward axis, in m/s.
     * @param[in] sideways Desired velocity to the field's left, in m/s.
     * @param[in] rotational Desired rotational velocity, in radians/second, **clockwise**
     *      positive. See the class-level warning.
     */
    virtual void setVelocityFieldDrive(float forward, float sideways, float rotational) = 0;

    /**
     * @return The chassis' **measured** rotational speed, in radians/second, derived from the wheel
     *      encoders rather than from the commanded setpoint or the IMU. Counterclockwise positive.
     */
    virtual float getChassisRotationSpeed() = 0;

    /**
     * How much rotational speed is left over once translation has been paid for.
     *
     * Translation and rotation compete for the same finite wheel speed, so beyblade uses this to
     * spin as fast as the remaining budget allows.
     *
     * @return The largest rotational speed the chassis can still produce, in radians/second.
     *      A magnitude: never negative, and valid in either direction.
     */
    virtual float calculateMaxRotationSpeed() = 0;

    /**
     * @return An estimate of the power the drivetrain is currently drawing, in watts.
     */
    virtual float getChassisPowerDraw() = 0;

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
     * @return The angle from the chassis' forward axis to the turret's, in radians, wrapped to
     *      (-pi, pi], measured **clockwise** (it is the negated turret yaw). Zero when the turret
     *      points straight ahead. Because it is clockwise, it can be fed through
     *      `chassisSpeedRotationPID` and passed as `rotational` directly, which is how
     *      `ChassisOrientDriveCommand` squares the chassis up with the turret.
     */
    float getChassisZeroTurret() { return modm::Angle::normalize(-getTurretYaw()); }

    /**
     * @return Where the chassis points, in radians, wrapped to (-pi, pi]. The IMU sits on the
     *      turret, so the turret's yaw relative to the chassis is subtracted out.
     *
     * @note This is the raw IMU heading. `ChassisOdometry::calculateRobotHeading` additionally
     *      applies the vision localization offset, so the two disagree after an AprilTag fix.
     */
    float getChassisYaw()
    {
        return modm::Angle::normalize(drivers->bmi088.getYaw() - getTurretYaw());
    }

    /**
     * @param[in] targetAngle The heading to compare against, in radians.
     * @return How far the chassis must rotate to reach `targetAngle`, in radians, wrapped to
     *      (-pi, pi]. Positive means counterclockwise.
     */
    float getDifferenceToTargetAngle(float targetAngle)
    {
        return modm::Angle::normalize(targetAngle - getChassisYaw());
    }

    /// @return The odometry this chassis feeds, or `nullptr` if none was attached.
    ChassisOdometry* getChassisOdometry() { return chassisOdometry; }

    /// @return The name used to identify this subsystem in logs and the scheduler.
    const char* getName() const override { return "Chassis"; }

    /// Set by `ChassisBeybladeCommand` while it is scheduled. Read by the HUD to decide whether to
    /// nag the operator to start spinning.
    void setBeybladeCommandRunning(bool running) { beybladeCommandRunning = running; }
    bool isBeybladeCommandRunning() const { return beybladeCommandRunning; }

    /// Set by beyblade and the sentry state machine while the robot is spinning but barely
    /// translating. Lets the drivetrain raise its power budget, since wheel speed not spent on
    /// translation is available for rotation.
    void setBeybladingOnly(bool beybladingOnly) { this->beybladingOnly = beybladingOnly; }
    bool isBeybladingOnly() const { return beybladingOnly; }

    /// @return `true` while the robot has meaningful sideways velocity, as computed by the last
    ///     drive call. Read by the HUD's peeking lines.
    bool isPeeking() const { return peeking; }
    /// @return `true` when the sideways motion reported by `isPeeking` is to the chassis' left.
    bool isPeekingLeft() const { return peekingLeft; }

protected:
    /// Called by the drivetrain from its drive path to report sideways motion to the HUD.
    void setPeeking(bool peeking, bool left)
    {
        this->peeking = peeking;
        peekingLeft = peeking && left;
    }

    /**
     * @return The turret's yaw relative to the chassis, in radians, wrapped to (-pi, pi]. Zero when
     *      the turret points along the chassis' forward axis.
     */
    float getTurretYaw();

    /// Odometry fed by the drivetrain. `nullptr` if none was attached.
    ChassisOdometry* chassisOdometry;

    /// The turret yaw motor, used to relate the chassis frame to the turret's.
    src::control::turret::TurretMotor* yawMotor;

private:
    bool beybladeCommandRunning{false};
    bool beybladingOnly{false};
    bool peeking{false};
    bool peekingLeft{false};
};  // class ChassisSubsystem
}  // namespace src::control::chassis
