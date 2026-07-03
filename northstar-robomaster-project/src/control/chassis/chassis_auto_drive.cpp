#include "chassis_auto_drive.hpp"

#include "tap/algorithms/wrapped_float.hpp"

namespace src::chassis
{
ChassisAutoDrive::ChassisAutoDrive(
    ChassisSubsystem* chassis,
    src::chassis::ChassisOdometry* chassisOdometry)
    : chassis(chassis),
      chassisOdometry(chassisOdometry),
      currentCurve(NULL)
{
}

void ChassisAutoDrive::resetPath()
{
    currentCurve = NULL;
    currentT = 0;
}

void ChassisAutoDrive::setCurve(CubicBezier* newPoint)
{
    currentCurve = newPoint;
    currentT = approximateTClosestToPoint(chassisOdometry->getPositionGlobal());
}

void ChassisAutoDrive::updateAutoDrive()
{
    if (!tryUpdatePath())
    {
        desiredGlobalVelocity = modm::Vector<float, 2>(0, 0);
        desiredRotation = 0;
        return;
    }

    modm::Vector<float, 2> dirToTarget = getDirectionToCurve(currentT);
    float distanceToTarget = dirToTarget.getLength();
    float distanceStraightToEndPoint = distanceToEndPoint();

    if (distanceToTarget < T_CHECK && distanceStraightToEndPoint > T_CHECK && currentT < 1)
    {
        currentT += T_INCREASE;
        return;
    }

    if (currentT > 1)
    {
        currentT = 1;
    }

    float distanceToEnd = approximateDistanceToEndOfCurve();
    float slowdownMult = 1;

    if (distanceToEnd < SLOWDOWN_DISTANCE)
    {
        slowdownMult = distanceToEnd / SLOWDOWN_DISTANCE;
        slowdownMult = tap::algorithms::limitVal(slowdownMult, 0.2f, 1.0f);
    }

    modm::Vector<float, 2> lookaheadDerivative = getLookaheadDeriv(currentT, T_LOOKAHEAD);
    modm::Vector<float, 2> lookaheadDirection = getDirectionToLookaheadPoint(currentT, T_LOOKAHEAD);
    modm::Vector<float, 2> globalVelocity = chassisOdometry->getVelocityGlobal();

    float globalSpeed = globalVelocity.getLength();
    if (globalSpeed < 0.01f)
    {
        globalSpeed = 0.01f;
    }

    float dot =
        lookaheadDirection.dot(globalVelocity) / (lookaheadDirection.getLength() * globalSpeed);
    dot *= 0.5f;
    dot = tap::algorithms::limitVal(dot, 0.5f, 1.0f);

    desiredGlobalVelocity = clampMagnitude(
        ((dirToTarget / distanceToTarget) *
         (lookaheadDerivative.getLength() / currentCurve->getLength())) *
            (slowdownMult * dot),
        MINIMUM_MPS,
        MAXIMUM_MPS);

    calculateRotationToFacePoint(lookaheadDirection);
}

};  // namespace src::chassis