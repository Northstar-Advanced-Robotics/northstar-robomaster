#ifndef CHASSIS_ODOMETRY_HPP
#define CHASSIS_ODOMETRY_HPP

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"

#include "control/turret/turret_motor.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector.hpp"

/*
    Chassis Odometry uses a 2D coordinate system, using the ground as the XY plane
    +X: Right
    +Y: Forward
    +Rotation: CCW
*/

namespace src::chassis
{
class ChassisOdometry
{
    static constexpr float ONE_OVER_THREE = 1.0f / 3.0f;
    static constexpr float VELOCITY_SMOOTHING_ALPHA =
        0.98f;  // 0-1, 1 = no smoothing, 0 = max smoothing

    tap::communication::sensors::imu::bmi088::Bmi088* imu;
    src::control::turret::TurretMotor* turretYaw;

    // rad/sec to m/sec
    float RPS_TO_MPS;
    float DIST_TO_CENT;

    modm::Vector<float, 2> velocityGlobal;
    modm::Vector<float, 2> velocityGlobalVision;
    modm::Vector<float, 2> velocityLocal;
    modm::Vector<float, 2> velocitySmoothedLocal;
    modm::Vector<float, 3> velocity3dGlobal;
    modm::Vector<float, 2> positionProjectedGlobal;
    modm::Vector<float, 2> velocityProjectedGlobal;

    // radians
    float globalImuRotationOffset;

    uint32_t previousTimeMicroSeconds = 0;

    struct Pose
    {
        float x;
        float y;
        float theta;
    };

    struct OdomMsg
    {
        uint32_t timestamp;
        Pose local_pose;
        float imu_yaw;
        float turret_yaw;  // From yaw motor
    };

    static constexpr int BUFFER_SIZE = 200;

    OdomMsg odomBuffer[BUFFER_SIZE];
    int bufferIndex = 0;

    Pose global_offset;
    Pose finalPositionGlobal;
    Pose mcbGlobalPose;

public:
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

    modm::Vector<float, 2> getPositionGlobal()
    {
        return {finalPositionGlobal.x, finalPositionGlobal.y};
    }
    modm::Vector<float, 2> getVelocityGlobal() { return velocityGlobal; }
    modm::Vector<float, 2> getVelocityGlobalVision() { return velocityGlobalVision; }
    modm::Vector<float, 2> getVelocityLocal() { return velocityLocal; }
    modm::Vector<float, 2> getPositionProjectedGlobal() { return positionProjectedGlobal; }
    modm::Vector<float, 2> getVelocityProjectedGlobal() { return velocityProjectedGlobal; }
    modm::Vector<float, 3> getVelocity3dGlobal() { return velocity3dGlobal; }
    float getRotation() { return finalPositionGlobal.theta; }

    void zeroOdometry()
    {
        finalPositionGlobal = {0, 0, 0};
        mcbGlobalPose = {0, 0, 0};
        global_offset = {0, 0, 0};
        velocityGlobal = modm::Vector<float, 2>(0, 0);
        velocityGlobalVision = modm::Vector<float, 2>(0, 0);
        velocityLocal = modm::Vector<float, 2>(0, 0);
        globalImuRotationOffset = 0;
    }

    void updateOdometryWithVisionData(uint32_t timestamp, float posX, float posY, float heading)
    {
        OdomMsg historical_data = get_historical_data(timestamp);

        globalImuRotationOffset =
            tap::algorithms::Angle(-heading - historical_data.imu_yaw + M_PI_2).getWrappedValue();

        float vision_chassis_heading =
            tap::algorithms::Angle(-heading + M_PI_2 - historical_data.turret_yaw)
                .getWrappedValue();

        global_offset.x = posX - historical_data.local_pose.x;
        global_offset.y = posY - historical_data.local_pose.y;
        global_offset.theta =
            tap::algorithms::Angle(vision_chassis_heading - historical_data.local_pose.theta)
                .getWrappedValue();
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

        float localVelX = (mps_LF + mps_RF - mps_LB - mps_RB) * ONE_OVER_THREE;
        float localVelY = (mps_LF - mps_RF + mps_LB - mps_RB) * ONE_OVER_THREE;

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

        velocityGlobalVision = velocityGlobal.rotate(globalImuRotationOffset);

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

    modm::Vector<float, 2> convertLocalToGlobal(
        const modm::Vector<float, 2>& local,
        float globalHeading)
    {
        float cosR = cosf(globalHeading);
        float sinR = sinf(globalHeading);

        return modm::Vector<float, 2>(
            local.x * cosR + local.y * sinR,
            -local.x * sinR + local.y * cosR);
    }

    float calculateRobotHeading()
    {
        return tap::algorithms::Angle(
                   imu->getYaw() + globalImuRotationOffset -
                   turretYaw->getChassisFrameMeasuredAngle().getWrappedValue())
            .getWrappedValue();
    }

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

    float getImuPitch() { return imu->getPitch(); }
    float getImuYaw() { return imu->getYaw(); }
    float getImuRoll() { return imu->getRoll(); }

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