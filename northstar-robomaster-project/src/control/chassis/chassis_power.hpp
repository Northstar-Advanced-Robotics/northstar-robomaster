#pragma once

#include "tap/drivers.hpp"

#include "control/chassis/constants/chassis_constants.hpp"
#include "modm/container/pair.hpp"

namespace src::control::chassis
{
/**
 * @ingroup chassis
 *
 * @param[in] drivers The global drivers object.
 * @return This robot's chassis power budget in watts, as most recently reported by the referee
 *      system. Stale or zero if the referee system is offline; callers should pair this with
 *      `refSerial.getRefSerialReceivingData()`, as `getMaxWheelSpeed` does.
 */
inline float getChassisPowerLimit(tap::Drivers* drivers)
{
    return drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
}

/// Caches the last power-limit-to-wheel-speed interpolation, keyed by the power limit that
/// produced it, so `getMaxWheelSpeed` can skip the lookup on the common unchanged case.
inline modm::Pair<int, float> lastComputedMaxWheelSpeed = CHASSIS_POWER_TO_MAX_SPEED_LUT[0];

/**
 * @ingroup chassis
 *
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
inline float getMaxWheelSpeed(bool refSerialOnline, float chassisPowerLimit)
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
}  // namespace src::control::chassis
