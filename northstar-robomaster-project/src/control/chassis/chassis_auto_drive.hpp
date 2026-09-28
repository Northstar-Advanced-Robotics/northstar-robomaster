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
/**
 * @ingroup chassis
 *
 * Follows a cubic Bezier path sent by the vision computer, producing the velocity and rotation the
 * chassis should be driven at.
 *
 * The follower is pure pursuit: a target point advances along the curve as the robot catches up to
 * it, and the robot is steered toward that point rather than toward the end of the path. A second
 * point further ahead sets the heading the chassis is rotated to face, so the robot turns into
 * corners before it reaches them. Speed is scaled down near the end of the path and whenever the
 * robot's current velocity is poorly aligned with where the path is going, so it does not overshoot
 * a turn.
 *
 * This class only computes setpoints; something else must read them and drive the chassis.
 */
class ChassisAutoDrive
{
    /// Fastest the follower will ask the chassis to travel, in meters/second.
    static constexpr float MAXIMUM_MPS = 1.5f;
    /// Slowest the follower will ask the chassis to travel while a path is active, in
    /// meters/second. A floor keeps the robot from stalling against friction near the end of a
    /// path.
    static constexpr float MINIMUM_MPS = 0.65f;

    /// Curve parameter advance per iteration, per meter/second of maximum speed.
    static constexpr float T_INCREASE_MULT = 0.01f;
    /// How far along the curve the target point advances each time the robot reaches it.
    static constexpr float T_INCREASE = T_INCREASE_MULT * MAXIMUM_MPS;

    /// Target-reached radius, per meter/second of maximum speed.
    static constexpr float T_CHECK_MULT = 0.025f;
    /// How close the robot must get to the target point, in meters, before the target advances.
    static constexpr float T_CHECK = T_CHECK_MULT * MAXIMUM_MPS;

    /// Lookahead distance along the curve, per meter/second of maximum speed.
    static constexpr float T_LOOKAHEAD_MULT = 0.08f;
    /// How far ahead of the target point to look when choosing the heading to face. Larger values
    /// turn into corners earlier but cut them more.
    static constexpr float T_LOOKAHEAD = T_LOOKAHEAD_MULT * MAXIMUM_MPS;

    /// Distance from the end of the path, in meters, at which the robot starts decelerating.
    static constexpr float SLOWDOWN_DISTANCE = 0.15f;
    /// Path length in meters below which the curve is too short to derive a meaningful heading
    /// from, and the straight line from start to end is used instead.
    static constexpr float DEGEN_CURVE_LENGTH = 0.1f;

    /// The chassis, used for its rotation PID controller.
    src::chassis::ChassisSubsystem* chassis;
    /// The odometry supplying the robot's position and velocity.
    src::chassis::ChassisOdometry* chassisOdometry;

    /// The path being followed, or `NULL` when there is none.
    CubicBezier* currentCurve;
    /// Position of the target point along `currentCurve`, from 0 at the start to 1 at the end.
    float currentT = 0;

    /// The field-frame velocity the chassis should be driven at, in meters/second.
    modm::Vector<float, 2> desiredGlobalVelocity;
    /// The rotational velocity the chassis should be driven at, in radians/second.
    float desiredRotation;  // radians per second

public:
    /**
     * @param[in] chassis The chassis whose rotation controller is used to face the path.
     * @param[in] chassisOdometry The odometry supplying position and velocity.
     */
    ChassisAutoDrive(ChassisSubsystem* chassis, ChassisOdometry* chassisOdometry);

    /// @return The field-frame velocity computed by the last `updateAutoDrive`, in meters/second.
    modm::Vector<float, 2> getDesiredGlobalVelocity() { return desiredGlobalVelocity; }
    /// @return The rotational velocity computed by the last `updateAutoDrive`, in radians/second.
    float getDesiredRotation() { return desiredRotation; }

    /// Discards the current path, after which the follower commands zero velocity.
    void resetPath();
    /**
     * Begins following a new path, starting from whichever point on it the robot is already
     * nearest so that a path handed over mid-drive does not send the robot back to its start.
     *
     * @param[in] newCurve The path to follow. Not owned; must outlive the follower's use of it.
     */
    void setCurve(CubicBezier* newCurve);
    /// Recomputes the desired velocity and rotation for this iteration, advancing the target point
    /// if the robot has reached it. Call once per control loop iteration.
    void updateAutoDrive();

    /// @return The chassis' heading in the field frame, in radians.
    float getOdometryRotation() { return chassisOdometry->getRotation(); }

    /// @return `true` if a path is set and the robot has not yet reached its end.
    bool hasValidPath() { return currentCurve != NULL && currentT < 1; }

    /**
     * @param[in] t Position along the curve, from 0 at the start to 1 at the end.
     * @return The field-frame vector from the robot to that point on the curve, in meters.
     */
    modm::Vector<float, 2> getDirectionToCurve(float t)
    {
        return currentCurve->evaluate(t) - chassisOdometry->getPositionGlobal();
    }

    /**
     * @param[in] lookaheadVal How far past the target point to look, in curve parameter units.
     * @return The lookahead position along the curve, clamped to the end of the path.
     */
    float getLookahead(float lookaheadVal)
    {
        float lookahead = currentT + lookaheadVal;
        if (lookahead > 1)
        {
            lookahead = 1;
        }

        return lookahead;
    }

    /**
     * Computes the direction the chassis should face: toward a point some distance further along
     * the path than the target. Once the lookahead runs past the end of the path the tangent at
     * the end is used instead, so the robot holds the path's final heading rather than spinning.
     *
     * @param[in] t Unused; the current target position is read from `currentT`.
     * @param[in] lookaheadVal How far past the target point to look, in curve parameter units.
     * @return The field-frame vector to face along, in meters.
     */
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

    /**
     * @param[in] t Unused; the current target position is read from `currentT`.
     * @param[in] lookaheadVal How far past the target point to look, in curve parameter units.
     * @return The curve's derivative at the lookahead point, whose magnitude indicates how
     *      quickly the path is moving there and is used to scale the commanded speed.
     */
    modm::Vector<float, 2> getLookaheadDeriv(float t, float lookaheadVal)
    {
        float lookahead = getLookahead(lookaheadVal);

        return currentCurve->evaluateDerivative(lookahead);
    }

private:
    /**
     * Constrains a vector's length to a range without changing its direction.
     *
     * @param[in] orig The vector to clamp.
     * @param[in] min The smallest allowed length.
     * @param[in] max The largest allowed length.
     * @return The clamped vector.
     */
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

    /**
     * @return Roughly how far the target point still has to travel, in meters, assuming the curve
     *      parameter advances uniformly with arc length.
     */
    float approximateDistanceToEndOfCurve()
    {
        float length = currentCurve->getLength();
        return length - (length * currentT);
    }

    /// @return The straight-line distance from the robot to the end of the path, in meters.
    float distanceToEndPoint()
    {
        return (chassisOdometry->getPositionGlobal() - currentCurve->getEnd()).getLength();
    }

    /**
     * Finds where along the curve a point lies, by walking the curve in small steps and stopping
     * once the distance starts growing again. Only finds the first local minimum, which is what is
     * wanted here: on a path that doubles back, the robot should resume at the nearest point it
     * has not yet passed.
     *
     * @param[in] pos The field-frame point to locate, in meters.
     * @return The curve parameter of the closest point found, from 0 to 1.
     */
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

    /**
     * Runs the chassis' auto drive rotation PID to turn the robot toward a point, storing the
     * result in `desiredRotation`.
     *
     * @warning Likely sign-inverted, and untested since the coordinate frame refactor.
     *      `getDifferenceToTargetAngle` yields a **counterclockwise** error and
     *      `chassisSpeedRotationAutoDrivePID` returns a counterclockwise-positive output, but
     *      `desiredRotation` is eventually passed to the chassis as `rotational`, which is
     *      **clockwise** positive. As written the robot should turn away from the lookahead point.
     *      The fix is to negate the PID's whole output here (negating only the input would flip P
     *      but not D). Verify with a path-follow on the robot before relying on this.
     *
     * @param[in] localPoint The direction to face, as a **field-frame** vector from the robot to
     *      the point (despite the name, not chassis-local).
     */
    void calculateRotationToFacePoint(modm::Vector<float, 2> localPoint)
    {
        float desiredWorldAngle = atan2(localPoint.y, localPoint.x);
        float differenceInDesiredFacingRadians =
            chassis->getDifferenceToTargetAngle(desiredWorldAngle);

        float rotationFromPID = chassis->chassisSpeedRotationAutoDrivePID(
            tap::algorithms::WrappedFloat(differenceInDesiredFacingRadians, -M_PI_4, M_PI_4)
                .getWrappedValue());

        desiredRotation = rotationFromPID;
    }
};

}  // namespace src::chassis

#endif