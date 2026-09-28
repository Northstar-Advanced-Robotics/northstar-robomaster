/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HEAT_LIMIT_GOVERNOR_HPP_
#define HEAT_LIMIT_GOVERNOR_HPP_

#include <cassert>

#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "ref_system_constants.hpp"

namespace src::control::governor
{
/**
 * @ingroup governors
 *
 * Blocks firing when the barrel's referee-reported heat is close enough to its limit that one more
 * projectile would exceed it.
 *
 * Overheating costs health, so this predicts the next shot's cost rather than reacting after the
 * fact.
 *
 * @note Permits firing -- i.e. does no limiting at all -- when the referee system is not sending
 *      data, or when the configured mechanism ID is neither `TURRET_17MM*` nor `TURRET_42MM`.
 */
class HeatLimitGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] drivers The global drivers object, used to read referee system heat.
     * @param[in] firingSystemMechanismID Which barrel's heat to track.
     * @param[in] heatLimitBuffer Extra margin, in referee-system heat units, required on top of the
     *      next projectile's cost before firing is allowed. For scale, a 17mm shot costs 10 and a
     *      42mm shot costs 100; see `ref_system_constants.hpp`.
     */
    HeatLimitGovernor(
        tap::Drivers &drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
        const uint16_t heatLimitBuffer)
        : drivers(drivers),
          firingSystemMechanismID(firingSystemMechanismID),
          heatLimitBuffer(heatLimitBuffer)
    {
    }

    bool isReady() final { return enoughHeatToLaunchProjectile(); }

    bool isFinished() final { return !enoughHeatToLaunchProjectile(); }

private:
    tap::Drivers &drivers;

    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID;

    const uint16_t heatLimitBuffer;

    bool enoughHeatToLaunchProjectile() const
    {
        if (!drivers.refSerial.getRefSerialReceivingData())
        {
            return true;
        }

        const auto &robotData = drivers.refSerial.getRobotData();

        uint16_t heat = 0, heatLimit = robotData.turret.heatLimit, nextCost = 0;

        switch (firingSystemMechanismID)
        {
            case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM:
                heat = robotData.turret.heat17;
                nextCost = src::constants::HEAT_COST_17MM;
                break;
            case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM:
                heat = robotData.turret.heat42;
                nextCost = src::constants::HEAT_COST_42MM;
                break;
            default:
                // don't perform heat limiting
                heat = 0;
                nextCost = 0;
                heatLimit = heatLimitBuffer;
        }

        const bool heatBelowLimit = heat + nextCost + heatLimitBuffer <= heatLimit;

        return !tap::communication::serial::RefSerial::heatAndLimitValid(heat, heatLimit) ||
               heatBelowLimit;
    }
};
}  // namespace src::control::governor

#endif  //  HEAT_LIMIT_GOVERNOR_HPP_
