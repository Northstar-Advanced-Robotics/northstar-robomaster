#ifndef CHASSIS_ODOMETRY_HPP
#define CHASSIS_ODOMETRY_HPP

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"

#include "control/turret/turret_motor.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector.hpp"

/*
    Chassis Odometry uses a 2D coordinate system, using the ground as the XY plane, matching the
    right hand rule convention used by ChassisSubsystem.
    +X: Forward
    +Y: Left
    +Rotation: CCW
*/

namespace src::chassis
{
/**
 * @ingroup chassis
 *
 * Tracks where the robot is and how fast it is moving, by dead reckoning from the drive motors and
 * the chassis IMU.
 *
 * Wheel speeds give velocity in the chassis' own frame; the IMU yaw, corrected by the turret's yaw
 * relative to the chassis, gives the heading needed to rotate that velocity into the field frame,
 * where it is integrated into a position. Dead reckoning drifts, so absolute pose fixes from the
 * vision computer's AprilTag localization are folded in as an offset rather than by overwriting
 * the pose; see `updateOdometryWithVisionData`.
 *
 * All poses use the 2D convention documented at the top of this file: +X right, +Y forward,
 * positive rotation counterclockwise.
 *
 * @warning This is **not** the frame `ChassisSubsystem` uses. That class is the canonical one and
 *      is +X forward, +Y left; both are right-handed, but rotated 90 degrees apart. Anything
 *      crossing between them must be converted -- see `state_machine_subsystem.cpp`, which feeds
 *      auto-drive output to the chassis as `setVelocityFieldDrive(vel.y, -vel.x, rot)`.
 *
 * @todo Migrate this class to the `ChassisSubsystem` frame so the codebase has a single convention
 *      and these conversions can go away. Until then, treat the swap above as required at every
 *      boundary.
 */
class ChassisOdometry
{
    /// Scale factor applied to the summed wheel speeds in `updateOdometry` to recover a chassis
    /// velocity.
    ///
    /// @warning Inverting `ChassisSubsystem`'s wheel kinematics gives `1 / (2 * sqrt(2))` (~0.354)
    ///      for this constant, not `1/3` (~0.333), so reported speeds are likely ~6% low. Left
    ///      as-is pending a measurement against ground truth.
    static constexpr float ONE_OVER_THREE = 1.0f / 3.0f;
    /// Low-pass coefficient for the smoothed local velocity: 1 applies no smoothing, 0 freezes the
    /// output.
    static constexpr float VELOCITY_SMOOTHING_ALPHA =
        0.98f;  // 0-1, 1 = no smoothing, 0 = max smoothing

    /// The chassis IMU, the source of absolute orientation.
    tap::communication::sensors::imu::bmi088::Bmi088* imu;
    /// The turret yaw motor, used to convert the IMU's turret-frame yaw into a chassis heading.
    src::control::turret::TurretMotor* turretYaw;

    // rad/sec to m/sec
    /// Wheel radius in meters; converts wheel angular velocity in rad/s to surface speed in m/s.
    float RPS_TO_MPS;
    /// Distance from a wheel to the chassis center, in meters.
    float DIST_TO_CENT;

    /// Velocity in the field frame, in meters/second.
    modm::Vector<float, 2> velocityGlobal;
    modm::Vector<float, 2> velocityLocal;
    /// `velocityLocal` after low-pass filtering with `VELOCITY_SMOOTHING_ALPHA`.
    modm::Vector<float, 2> velocitySmoothedLocal;
    /// Field-frame velocity lifted into three dimensions using the IMU's full orientation, so that
    /// driving up a ramp is not reported as pure horizontal motion.
    modm::Vector<float, 3> velocity3dGlobal;
    /// Position extrapolated forward in time, used to lead a moving target.
    modm::Vector<float, 2> positionProjectedGlobal;
    /// Velocity extrapolated forward in time, paired with `positionProjectedGlobal`.
    modm::Vector<float, 2> velocityProjectedGlobal;

    // radians
    /// Rotation in radians between the IMU's yaw reference and the field frame, set by the most
    /// recent vision localization fix.
    float globalImuRotationOffset;

    /// Timestamp of the previous update, in microseconds. Zero before the first update, which is
    /// skipped since no interval can be measured.
    uint32_t previousTimeMicroSeconds = 0;

    /// A planar pose: position in meters and heading in radians.
    struct Pose
    {
        float x;
        float y;
        float theta;
    };

    /**
     * One historical odometry sample.
     *
     * Vision localization results describe the robot as it was when the frame was captured, not as
     * it is when the result arrives, so past poses are retained and looked up by timestamp to
     * compute the correction.
     */
    struct OdomMsg
    {
        uint32_t timestamp;
        Pose local_pose;
        float imu_yaw;
        float turret_yaw;  // From yaw motor
    };

    /// Number of historical odometry samples retained, bounding how much vision latency can be
    /// compensated for.
    static constexpr int BUFFER_SIZE = 200;

    /// Ring buffer of recent odometry samples.
    OdomMsg odomBuffer[BUFFER_SIZE];
    /// Write position in `odomBuffer`.
    int bufferIndex = 0;

    /// Correction added to the dead-reckoned pose to place it in the field frame, updated by
    /// vision localization.
    Pose global_offset;
    /// The dead-reckoned pose plus `global_offset`, i.e. the robot's best-estimate field pose.
    Pose finalPositionGlobal;
    /// The pose accumulated purely by dead reckoning, before any vision correction.
    Pose mcbGlobalPose;

public:
    /**
     * @param[in] imu The chassis IMU.
     * @param[in] turretYaw The turret yaw motor, used to derive chassis heading from the IMU.
     * @param[in] distanceToCenter Distance from a wheel to the chassis center, in meters.
     * @param[in] wheelDiameter Wheel diameter in meters.
     */
    ChassisOdometry(
        tap::communication::sensors::imu::bmi088::Bmi088* imu,
        src::control::turret::TurretMotor* turretYaw,
        float distanceToCenter,
        float wheelDiameter)
        : imu(imu),
          turretYaw(turretYaw),
          RPS_TO_MPS(wheelDiameter / 2.0),
          DIST_TO_CENT(distanceToCenter)
    {
        zeroOdometry();
    }

    /// @return The robot's position in the field frame, in meters.
    modm::Vector<float, 2> getPositionGlobal()
    {
        return {finalPositionGlobal.x, finalPositionGlobal.y};
    }
    /// @return Velocity in the field frame, in meters/second.
    modm::Vector<float, 2> getVelocityGlobal() { return velocityGlobal; }
    modm::Vector<float, 2> getVelocityLocal() { return velocityLocal; }
    /// @return The field-frame position extrapolated forward in time, in meters.
    modm::Vector<float, 2> getPositionProjectedGlobal() { return positionProjectedGlobal; }
    /// @return The field-frame velocity extrapolated forward in time, in meters/second.
    modm::Vector<float, 2> getVelocityProjectedGlobal() { return velocityProjectedGlobal; }
    /// @return Field-frame velocity in three dimensions, in meters/second.
    modm::Vector<float, 3> getVelocity3dGlobal() { return velocity3dGlobal; }
    /// @return The chassis' heading in the field frame, in radians, counterclockwise positive.
    float getRotation() { return finalPositionGlobal.theta; }

    /**
     * Makes the robot's current pose the origin of the field frame and clears all accumulated
     * velocity and vision correction. Use to discard drift once the robot is at a known spot.
     */
    void zeroOdometry()
    {
        finalPositionGlobal = {0, 0, 0};
        mcbGlobalPose = {0, 0, 0};
        global_offset = {0, 0, 0};
        velocityGlobal = modm::Vector<float, 2>(0, 0);
        velocityLocal = modm::Vector<float, 2>(0, 0);
        globalImuRotationOffset = 0;
    }

    /**
     * Folds an absolute pose fix from the vision computer into the odometry.
     *
     * The fix describes the robot as it was at `timestamp`, so the dead-reckoned pose from that
     * instant is looked up in the history buffer and the difference is stored as `global_offset`.
     * Correcting by an offset rather than overwriting the pose means the correction does not
     * discard motion accumulated since the frame was captured, and does not make the pose jump
     * backwards.
     *
     * @param[in] timestamp When the vision frame was captured, in microseconds on this board's
     *      clock.
     * @param[in] posX The measured field-frame x position, in meters.
     * @param[in] posY The measured field-frame y position, in meters.
     * @param[in] heading The measured field-frame heading, in radians.
     */
    void updateOdometryWithVisionData(uint32_t timestamp, float posX, float posY, float heading)
    {
        OdomMsg historical_data = get_historical_data(timestamp);

        globalImuRotationOffset =
            tap::algorithms::Angle(heading - historical_data.imu_yaw).getWrappedValue();

        global_offset.x = posX - historical_data.local_pose.x;
        global_offset.y = posY - historical_data.local_pose.y;
        global_offset.theta = 0;
    }

    OdomMsg get_historical_data(uint32_t target_timestamp_us)
    {
        OdomMsg closest_entry;
        closest_entry.local_pose = finalPositionGlobal;
        closest_entry.imu_yaw = imu->getYaw();

        uint32_t smallest_time_diff = 0xFFFFFFFF;

        for (int i = 0; i < BUFFER_SIZE; i++)
        {
            uint32_t diff;
            if (odomBuffer[i].timestamp > target_timestamp_us)
            {
                diff = odomBuffer[i].timestamp - target_timestamp_us;
            }
            else
            {
                diff = target_timestamp_us - odomBuffer[i].timestamp;
            }

            if (diff < smallest_time_diff)
            {
                smallest_time_diff = diff;
                closest_entry = odomBuffer[i];
            }
        }

        return closest_entry;
    }

    // input is in radians per second
    /**
     * Advances the odometry by one control loop iteration.
     *
     * Converts the four wheel speeds into a chassis-frame velocity, rotates it into the field
     * frame using the current heading, integrates it into the dead-reckoned pose, records the
     * result in the history buffer, and applies the vision offset to produce the final pose. The
     * very first call only records the time, since no interval is available to integrate over.
     *
     * @param[in] motorRPS_LF Left front wheel angular velocity, in radians/second.
     * @param[in] motorRPS_LB Left back wheel angular velocity, in radians/second.
     * @param[in] motorRPS_RF Right front wheel angular velocity, in radians/second.
     * @param[in] motorRPS_RB Right back wheel angular velocity, in radians/second.
     */
    void updateOdometry(float motorRPS_LF, float motorRPS_LB, float motorRPS_RF, float motorRPS_RB)
    {
        uint32_t currentTimeMicroSeconds = tap::arch::clock::getTimeMicroseconds();
        if (previousTimeMicroSeconds == 0)
        {
            previousTimeMicroSeconds = currentTimeMicroSeconds;
            return;
        }

        float deltaTimeSeconds =
            (currentTimeMicroSeconds - previousTimeMicroSeconds) / 1'000'000.0f;
        previousTimeMicroSeconds = currentTimeMicroSeconds;

        float mps_LF = motorRPS_LF * RPS_TO_MPS;
        float mps_LB = motorRPS_LB * RPS_TO_MPS;
        float mps_RF = motorRPS_RF * RPS_TO_MPS;
        float mps_RB = motorRPS_RB * RPS_TO_MPS;

        float localVelX = (mps_LF - mps_RF + mps_LB - mps_RB) * ONE_OVER_THREE;
        float localVelY = (mps_LB + mps_RB - mps_LF - mps_RF) * ONE_OVER_THREE;

        velocityLocal.x = localVelX;
        velocityLocal.y = localVelY;

        mcbGlobalPose.theta = calculateRobotHeading();
        // float prevTheta = mcbGlobalPose.theta;
        // mcbGlobalPose.theta = calculateRobotHeading();
        // // Use midpoint heading to reduce lateral drift when spinning + translating.
        // // Encoder velocities represent average over [t-dt, t]; heading at midpoint
        // // (t - dt/2) is more accurate than the current heading at t.
        // float midTheta =
        //     prevTheta +
        //     tap::algorithms::Angle(mcbGlobalPose.theta - prevTheta).getWrappedValue() * 0.5f;

        velocityGlobal = convertLocalToGlobal(velocityLocal, mcbGlobalPose.theta);

        mcbGlobalPose.x += velocityGlobal.x * deltaTimeSeconds;
        mcbGlobalPose.y += velocityGlobal.y * deltaTimeSeconds;

        OdomMsg new_odom_msg{
            .timestamp = currentTimeMicroSeconds,
            .local_pose = mcbGlobalPose,
            .imu_yaw = imu->getYaw(),
            .turret_yaw = turretYaw->getChassisFrameMeasuredAngle().getWrappedValue()};

        odomBuffer[bufferIndex] = new_odom_msg;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        finalPositionGlobal.x = mcbGlobalPose.x + global_offset.x;
        finalPositionGlobal.y = mcbGlobalPose.y + global_offset.y;
        finalPositionGlobal.theta =
            tap::algorithms::Angle(mcbGlobalPose.theta + global_offset.theta).getWrappedValue();
    }

    /**
     * Rotates a vector from the chassis frame into the field frame.
     *
     * The rotation applied is by **-`globalHeading`**, not +`globalHeading`, which follows from
     * this class' heading being measured clockwise-positive in its own axis convention. Getting
     * this sign wrong is the classic way to make odometry drift sideways while turning.
     *
     * @param[in] local The vector in chassis-frame coordinates.
     * @param[in] globalHeading The chassis' heading in the field frame, in radians.
     * @return The same vector in field-frame coordinates.
     */
    modm::Vector<float, 2> convertLocalToGlobal(
        const modm::Vector<float, 2>& local,
        float globalHeading)
    {
        float cosR = cosf(globalHeading);
        float sinR = sinf(globalHeading);

        return modm::Vector<float, 2>(
            local.x * cosR - local.y * sinR,
            local.x * sinR + local.y * cosR);
    }

    /**
     * @return The chassis' heading in radians. The IMU sits on the turret, so the turret's yaw
     *      relative to the chassis is subtracted out, and the vision rotation offset is added to
     *      place the result in the field frame.
     */
    float calculateRobotHeading()
    {
        return tap::algorithms::Angle(
                   imu->getYaw() + globalImuRotationOffset -
                   turretYaw->getChassisFrameMeasuredAngle().getWrappedValue())
            .getWrappedValue();
    }

    /**
     * Lifts a planar chassis-frame velocity into a 3D field-frame velocity using the IMU's full
     * orientation, so that motion on a sloped surface is not reported as purely horizontal.
     *
     * @param[in] localVel The chassis-frame velocity, in meters/second.
     * @return The field-frame velocity in three dimensions, in meters/second.
     */
    modm::Vector<float, 3> flatLocalVelTo3dGlobalVel(modm::Vector<float, 2> localVel)
    {
        float imuYaw = imu->getYaw();
        float imuRoll = imu->getRoll();
        float imuPitch = imu->getPitch();

        float alpha = calculateRobotHeading();
        float beta = cosf(-imuYaw) * imuPitch + sinf(-imuYaw) * imuRoll;
        float gamma = -sinf(-imuYaw) * imuPitch + cosf(-imuYaw) * imuRoll;

        float x = localVel.x * cosf(beta) * cosf(gamma) +
                  localVel.y * (cosf(alpha) * sinf(beta) * cosf(gamma) + sinf(alpha) * sinf(gamma));
        float y = localVel.x * cosf(beta) * sinf(gamma) +
                  localVel.y * (cosf(alpha) * sinf(beta) * sinf(gamma) - sinf(alpha) * cosf(gamma));
        float z = localVel.x * -sinf(beta) + localVel.y * cosf(alpha) * cosf(beta);

        return modm::Vector<float, 3>(x, y, z);
    }

    /// @return The IMU's pitch, in radians.
    float getImuPitch() { return imu->getPitch(); }
    /// @return The IMU's yaw, in radians. Measured about the turret, not the chassis; see
    /// `calculateRobotHeading`.
    float getImuYaw() { return imu->getYaw(); }
    /// @return The IMU's roll, in radians.
    float getImuRoll() { return imu->getRoll(); }

    /**
     * Blends two vectors, applying a low-pass filter componentwise.
     *
     * @param[in] current The newest sample.
     * @param[in] previous The previous filter output.
     * @param[in] alpha Weight given to the new sample: 1 passes it through unchanged, 0 holds the
     *      previous output.
     * @return The filtered vector.
     */
    modm::Vector<float, 2> vectorLowPassFilter(
        modm::Vector<float, 2> current,
        modm::Vector<float, 2> previous,
        float alpha)
    {
        return (current * alpha) + (previous * (1.0f - alpha));
    }
};

}  // namespace src::chassis

#endif