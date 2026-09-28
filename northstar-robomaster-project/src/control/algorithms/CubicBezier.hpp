#ifndef CUBIC_BEZIER_HPP
#define CUBIC_BEZIER_HPP

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_subsystem.hpp"

/**
 * @ingroup util
 *
 * A planar cubic Bezier curve, used to describe the paths the chassis follows under auto drive.
 *
 * The curve is defined by a start and end point plus two control points, and is parameterized by
 * `t` in [0, 1]. Because the parameterization is not arc length, `t` does not advance uniformly
 * along the curve; `getLength` gives an approximate arc length for converting between the two.
 */
class CubicBezier
{
public:
    /**
     * The four control points that define the curve, in meters, along with its cached approximate
     * arc length. Packed so it can be received directly from a serial message.
     */
    struct CurveData
    {
        modm::Vector<float, 2> start;
        modm::Vector<float, 2> end;
        modm::Vector<float, 2> startControl;
        modm::Vector<float, 2> endControl;
        float length;
    } modm_packed;

    /**
     * Constructs a curve from data received over the wire.
     *
     * @param[in] curveData The control points and length. The length is trusted as given rather
     *      than recomputed.
     */
    CubicBezier(CurveData curveData) : curveData(curveData) {}

    /**
     * Constructs a curve and estimates its arc length.
     *
     * @param[in] start The point the curve begins at.
     * @param[in] end The point the curve ends at.
     * @param[in] startControl The control point pulling the curve away from `start`.
     * @param[in] endControl The control point pulling the curve into `end`.
     */
    CubicBezier(
        modm::Vector<float, 2> start,
        modm::Vector<float, 2> end,
        modm::Vector<float, 2> startControl,
        modm::Vector<float, 2> endControl)
    {
        curveData.start = start;
        curveData.end = end;
        curveData.startControl = startControl;
        curveData.endControl = endControl;
        curveData.length = estimateLength();
    }

    /**
     * Constructs a curve with a caller-supplied arc length, skipping the estimation.
     *
     * @param[in] start The point the curve begins at.
     * @param[in] end The point the curve ends at.
     * @param[in] startControl The control point pulling the curve away from `start`.
     * @param[in] endControl The control point pulling the curve into `end`.
     * @param[in] length The curve's arc length in meters.
     */
    CubicBezier(
        modm::Vector<float, 2> start,
        modm::Vector<float, 2> end,
        modm::Vector<float, 2> startControl,
        modm::Vector<float, 2> endControl,
        float length)
    {
        curveData.start = start;
        curveData.end = end;
        curveData.startControl = startControl;
        curveData.endControl = endControl;
        curveData.length = length;
    }

    /// @return The point the curve begins at.
    modm::Vector<float, 2> getStart() { return curveData.start; }
    /// @return The point the curve ends at.
    modm::Vector<float, 2> getEnd() { return curveData.end; }
    /// @return The control point associated with the start of the curve.
    modm::Vector<float, 2> getStartControl() { return curveData.startControl; }
    /// @return The control point associated with the end of the curve.
    modm::Vector<float, 2> getEndControl() { return curveData.endControl; }
    /// @return The curve's approximate arc length, in meters.
    float getLength() { return curveData.length; }

    /**
     * @param[in] t Position along the curve, from 0 at the start to 1 at the end.
     * @return The point on the curve at `t`.
     */
    modm::Vector<float, 2> evaluate(float t)
    {
        float oneMinusT = 1 - t;
        return (oneMinusT * oneMinusT * oneMinusT) * curveData.start +
               (3.0f * (oneMinusT * oneMinusT) * t) * curveData.startControl +
               (3.0f * oneMinusT * (t * t)) * curveData.endControl + (t * t * t) * curveData.end;
    }

    /**
     * @param[in] t Position along the curve, from 0 at the start to 1 at the end.
     * @return The first derivative at `t`, i.e. a vector tangent to the curve whose magnitude is
     *      the rate of change of position with respect to `t`.
     */
    modm::Vector<float, 2> evaluateDerivative(float t)
    {
        float oneMinusT = 1 - t;
        return (3.0f * (oneMinusT * oneMinusT)) * (curveData.startControl - curveData.start) +
               (6.0f * oneMinusT * t) * (curveData.endControl - curveData.startControl) +
               (3.0f * (t * t)) * (curveData.end - curveData.endControl);
    }

    /**
     * @param[in] t Position along the curve, from 0 at the start to 1 at the end.
     * @return The second derivative at `t`, used together with the first to compute curvature.
     */
    modm::Vector<float, 2> evaluateSecondDerivative(float t)
    {
        return (6 * (1 - t)) *
                   (curveData.endControl - 2.0f * curveData.startControl + curveData.start) +
               (6 * t) * (curveData.end - 2 * curveData.endControl + curveData.startControl);
    }

    /**
     * Approximates the curve's arc length by sampling it at 16 evenly spaced values of `t` and
     * summing the distances between consecutive samples. Always an underestimate, since the
     * polyline cuts the corners of the curve.
     *
     * @return The estimated arc length, in meters.
     */
    float estimateLength()
    {
        float length = 0.0f;
        modm::Vector<float, 2> prevPoint = curveData.start;

        for (int i = 0; i < 16; i++)
        {
            modm::Vector<float, 2> point = evaluate((i + 1) / 16.0f);
            length += prevPoint.getDistanceTo(point);
            prevPoint = point;
        }

        return length;
    }

    /**
     * Computes how fast the chassis must rotate to stay tangent to the curve while travelling
     * along it at a given speed, i.e. the curvature at `t` scaled by that speed.
     *
     * @param[in] t Position along the curve, from 0 at the start to 1 at the end.
     * @param[in] linearVelocity The speed along the curve, in meters/second.
     * @return The required rotational velocity in radians/second, positive counterclockwise. Zero
     *      where the curve degenerates to a point and curvature is undefined.
     */
    float getRotationalVelocity(float t, float linearVelocity)
    {
        modm::Vector<float, 2> d = evaluateDerivative(t);
        modm::Vector<float, 2> dd = evaluateSecondDerivative(t);

        float numerator = (d[0] * dd[1]) - (d[1] * dd[0]);

        float tangentMagnitudeSquared = (d[0] * d[0]) + (d[1] * d[1]);
        float denominator = powf(tangentMagnitudeSquared, 1.5f);

        if (denominator < 1e-5f)
        {
            return 0.0f;
        }

        float curvature = numerator / denominator;
        return linearVelocity * curvature;
    }

private:
    /// The control points and cached arc length defining this curve.
    CurveData curveData;
};

#endif