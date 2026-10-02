#pragma once

#include <algorithm>
#include <cmath>

#include "tap/drivers.hpp"

#include "control/chassis/constants/chassis_constants.hpp"

namespace src::control::chassis
{
/**
 * @ingroup chassis
 *
 * @param[in] drivers The global drivers object.
 * @return This robot's chassis power budget in watts, as most recently reported by the referee
 *      system. Stale or zero if the referee system is offline; callers should pair this with
 *      `refSerial.getRefSerialReceivingData()`, as `getChassisPowerTarget` does.
 */
inline float getChassisPowerLimit(tap::Drivers* drivers)
{
    return drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
}

/**
 * @ingroup chassis
 *
 * The chassis power the power loop should hold the drivetrain to right now.
 *
 * The referee limit, read live, cut back once the referee's power buffer falls more than
 * `POWER_BUFFER_DEADBAND_J` below full, so the buffer is never spent down toward the 5 second
 * chassis power-off. Never above the limit. See the buffer constants in `chassis_constants.hpp`.
 *
 * @param[in] drivers The global drivers object.
 * @return The power target, in watts. `POWER_FALLBACK_LIMIT_W` while the referee system is
 *      offline. Never negative.
 */
inline float getChassisPowerTarget(tap::Drivers* drivers)
{
    if (!drivers->refSerial.getRefSerialReceivingData())
    {
        return POWER_FALLBACK_LIMIT_W;
    }

    float limit = getChassisPowerLimit(drivers);
    float buffer = drivers->refSerial.getRobotData().chassis.powerBuffer;

    // SUPERCAP: if (capacitorBank && capacitorBank->isEnabled()) {
    //     // The cap board charges from whatever the chassis leaves under the limit.
    //     capacitorBank->setPowerLimit(limit);
    //     capacitorBank->setEnergyBuffer(buffer);
    //     // Sprint power comes from the cap, so the referee buffer is still untouched.
    //     // Needs a sprint flag from the operator interface (Shift held, or stick past walk speed).
    //     if (sprinting && capacitorBank->canSprint(SPRINT_THRESHOLD_PERCENT))
    //     {
    //         return limit + SUPERCAP_SPRINT_BONUS_W;
    //     }
    //     // Walking: fall through to the buffer-protecting target below; the cap recharges.
    // }

    float bufferDeficit =
        std::max((POWER_BUFFER_FULL_J - POWER_BUFFER_DEADBAND_J) - buffer, 0.0f);
    return std::max(limit - POWER_BUFFER_GAIN_W_PER_J * bufferDeficit, 0.0f);
}

/**
 * @ingroup chassis
 *
 * The chassis power model from `chassis_constants.hpp`, accumulated over the drive motors and
 * kept in a form that can be evaluated with every motor current scaled by a common factor `k`:
 *
 *     P(k) = a*k^2 + b*k + c
 *
 * `a` is the copper loss, `b` the mechanical power, and `c` the speed-dependent and static draw
 * (neither of which depends on current).
 */
struct PowerModel
{
    float a = 0.0f;
    float b = 0.0f;
    float c = POWER_MODEL_STATIC_W;

    /**
     * @param[in] currentA The motor's current, in amps.
     * @param[in] velocity The wheel shaft speed, in radians/second.
     */
    void addMotor(float currentA, float velocity)
    {
        a += POWER_MODEL_COPPER_LOSS_W_PER_A2 * currentA * currentA;
        b += M3508_TORQUE_CONSTANT_NM_PER_A * currentA * velocity;
        c += POWER_MODEL_SPEED_LOSS_W_PER_RAD2 * velocity * velocity;
    }

    /// @return The predicted draw with every current scaled by `k`, in watts. Can be negative
    ///     while braking.
    float at(float k) const { return a * k * k + b * k + c; }
};

/**
 * @ingroup chassis
 *
 * Finds how far to scale every motor current so the predicted draw meets a power target.
 *
 * @param[in] model The draw at the currents about to be commanded.
 * @param[in] targetW The power to stay under, in watts.
 * @return The largest scale in [0, 1] with `model.at(scale) <= targetW`: 1 when the currents
 *      already fit, 0 when even zero current would exceed the target.
 */
inline float solvePowerScale(const PowerModel& model, float targetW)
{
    if (model.at(1.0f) <= targetW)
    {
        return 1.0f;
    }
    if (model.c >= targetW)
    {
        return 0.0f;
    }

    // P(0) < target < P(1), so exactly one root of P(k) = target lies in (0, 1).
    float d = model.c - targetW;
    float k;
    if (model.a > 1e-6f)
    {
        // a > 0 and d < 0, so the discriminant is positive and this is the positive root.
        k = (-model.b + sqrtf(model.b * model.b - 4.0f * model.a * d)) / (2.0f * model.a);
    }
    else
    {
        // Linear: P(1) > target > P(0) means b > 0.
        k = -d / model.b;
    }
    return std::clamp(k, 0.0f, 1.0f);
}
}  // namespace src::control::chassis
