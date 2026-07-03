#ifndef CHASSIS_AUTO_DRIVE_HPP
#define CHASSIS_AUTO_DRIVE_HPP

#include <deque>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "control/algorithms/CubicBezier.hpp"

#include "chassis_odometry.hpp"
#include "chassis_subsystem.hpp"

namespace src::chassis
{
class ChassisAutoDrive
{
    static constexpr float MAXIMUM_MPS = 1.5f;
    static constexpr float MINIMUM_MPS = 0.65f;

    static constexpr float T_INCREASE_MULT = 0.01f;
    static constexpr float T_INCREASE = T_INCREASE_MULT * MAXIMUM_MPS;

    static constexpr float T_CHECK_MULT = 0.025f;
    static constexpr float T_CHECK = T_CHECK_MULT * MAXIMUM_MPS;

    static constexpr float T_LOOKAHEAD_MULT = 0.08f;
    static constexpr float T_LOOKAHEAD = T_LOOKAHEAD_MULT * MAXIMUM_MPS;

    static constexpr float SLOWDOWN_DISTANCE = 0.15f;
    static constexpr float DEGEN_CURVE_LENGTH = 0.1f;

    src::chassis::ChassisSubsystem* chassis;
    src::chassis::ChassisOdometry* chassisOdometry;

    CubicBezier* currentCurve;
    float currentT = 0;

    modm::Vector<float, 2> desiredGlobalVelocity;
    float desiredRotation;  // radians per second

public:
    ChassisAutoDrive(ChassisSubsystem* chassis, ChassisOdometry* chassisOdometry);

    modm::Vector<float, 2> getDesiredGlobalVelocity() { return desiredGlobalVelocity; }
    float getDesiredRotation() { return desiredRotation; }

    void resetPath();
    void setCurve(CubicBezier* newCurve);
    void updateAutoDrive();

    float getOdometryRotation() { return chassisOdometry->getRotation(); }

    bool hasValidPath() { return currentCurve != NULL && currentT < 1; }

    modm::Vector<float, 2> getDirectionToCurve(float t)
    {
        return currentCurve->evaluate(t) - chassisOdometry->getPositionGlobal();
    }

    float getLookahead(float lookaheadVal)
    {
        float lookahead = currentT + lookaheadVal;
        if (lookahead > 1)
        {
            lookahead = 1;
        }

        return lookahead;
    }

    modm::Vector<float, 2> getDirectionToLookaheadPoint(float t, float lookaheadVal)
    {
        if (currentCurve->getLength() <= DEGEN_CURVE_LENGTH)
        {
            return currentCurve->getEnd() - currentCurve->getStart();
        }

        float lookahead = getLookahead(lookaheadVal);
        if (lookahead < 1)
        {
            return currentCurve->evaluate(lookahead) - chassisOdometry->getPositionGlobal();
        }
        else
        {
            return currentCurve->getEnd() - currentCurve->evaluate(0.975f);
        }
    }

    modm::Vector<float, 2> getLookaheadDeriv(float t, float lookaheadVal)
    {
        float lookahead = getLookahead(lookaheadVal);

        return currentCurve->evaluateDerivative(lookahead);
    }

private:
    modm::Vector<float, 2> clampMagnitude(modm::Vector<float, 2> orig, float min, float max)
    {
        float length = orig.getLength();

        if (length > max)
        {
            return (orig / length) * max;
        }
        else if (length < min)
        {
            return (orig / length) * min;
        }
        else
        {
            return orig;
        }
    }

    float approximateDistanceToEndOfCurve()
    {
        float length = currentCurve->getLength();
        return length - (length * currentT);
    }

    float distanceToEndPoint()
    {
        return (chassisOdometry->getPositionGlobal() - currentCurve->getEnd()).getLength();
    }

    float approximateTClosestToPoint(modm::Vector<float, 2> pos)
    {
        float t = 0.0f;
        float d = FLT_MAX;

        while (t < 1)
        {
            float currentDistToTarget = (pos - currentCurve->evaluate(t)).getLength();
            if (currentDistToTarget > d)
            {
                return t;
            }

            d = currentDistToTarget;
            t += 0.002f;
        }

        return 1;
    }

    void calculateRotationToFacePoint(modm::Vector<float, 2> localPoint)
    {
        float desiredWorldAngle = -atan2(localPoint.y, localPoint.x);
        float differenceInDesiredFacingRadians =
            chassis->getDifferenceToTargetAngle((desiredWorldAngle + M_PI_2));

        float rotationFromPID = chassis->chassisSpeedRotationAutoDrivePID(
            tap::algorithms::WrappedFloat(differenceInDesiredFacingRadians, -M_PI_4, M_PI_4)
                .getWrappedValue());

        desiredRotation = rotationFromPID;
    }
};

}  // namespace src::chassis

#endif