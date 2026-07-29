#ifndef CUBIC_BEZIER_HPP
#define CUBIC_BEZIER_HPP

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_subsystem.hpp"

class CubicBezier
{
public:
    // Used as a wire format (memcpy'd from received messages in vision_comms.cpp).
    // The members are all 4-byte-aligned floats, so the natural layout has no
    // padding; the static_assert below guarantees this stays true.
    struct CurveData
    {
        modm::Vector<float, 2> start;
        modm::Vector<float, 2> end;
        modm::Vector<float, 2> startControl;
        modm::Vector<float, 2> endControl;
        float length;
    };
    static_assert(sizeof(CurveData) == 9 * sizeof(float), "CurveData must be packed");

    CubicBezier(CurveData curveData) : curveData(curveData) {}

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

    modm::Vector<float, 2> getStart() { return curveData.start; }
    modm::Vector<float, 2> getEnd() { return curveData.end; }
    modm::Vector<float, 2> getStartControl() { return curveData.startControl; }
    modm::Vector<float, 2> getEndControl() { return curveData.endControl; }
    float getLength() { return curveData.length; }

    modm::Vector<float, 2> evaluate(float t)
    {
        float oneMinusT = 1 - t;
        return (oneMinusT * oneMinusT * oneMinusT) * curveData.start +
               (3.0f * (oneMinusT * oneMinusT) * t) * curveData.startControl +
               (3.0f * oneMinusT * (t * t)) * curveData.endControl + (t * t * t) * curveData.end;
    }

    modm::Vector<float, 2> evaluateDerivative(float t)
    {
        float oneMinusT = 1 - t;
        return (3.0f * (oneMinusT * oneMinusT)) * (curveData.startControl - curveData.start) +
               (6.0f * oneMinusT * t) * (curveData.endControl - curveData.startControl) +
               (3.0f * (t * t)) * (curveData.end - curveData.endControl);
    }

    modm::Vector<float, 2> evaluateSecondDerivative(float t)
    {
        return (6 * (1 - t)) *
                   (curveData.endControl - 2.0f * curveData.startControl + curveData.start) +
               (6 * t) * (curveData.end - 2 * curveData.endControl + curveData.startControl);
    }

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
    CurveData curveData;
};

#endif