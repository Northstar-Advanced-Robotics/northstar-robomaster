/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FLYWHEEL_ON_GOVERNOR_HPP_
#define FLYWHEEL_ON_GOVERNOR_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

#include "control/agitator/velocity_agitator_subsystem.hpp"
#include "control/flywheel/flywheel_interface.hpp"

namespace src::control::governor
{
/**
 * @ingroup governors
 *
 * Gates commands on the flywheels actually running at the speed they were asked for.
 *
 * Used to stop the agitator feeding projectiles into flywheels that are not ready, which would
 * launch them at the wrong speed or jam them.
 *
 * The check is a **band**, not a floor: the measured average must be at least
 * `MINIMUM_SPEED_THRESHOLD_FRACTION` and at most `MAXIMUM_SPEED_THRESHOLD_FRACTION` of the desired
 * speed, so flywheels overspinning by more than 5% also block firing. Requesting a speed of
 * approximately zero blocks unconditionally.
 */
class FlywheelOnGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] flywheel Reference to the friction wheel subsystem being used in the
     * governor's behavior.
     */
    FlywheelOnGovernor(src::control::flywheel::FlywheelInterface &flywheel) : flywheel(flywheel) {}

    bool isReady() final
    {
        return (
            !tap::algorithms::compareFloatClose(flywheel.getDesiredFlywheelSpeed(), .0f, 1) &&
            abs(flywheel.getCurrentFlywheelAverageMotorRPM()) >=
                flywheel.getDesiredFlywheelSpeed() * MINIMUM_SPEED_THRESHOLD_FRACTION &&
            abs(flywheel.getCurrentFlywheelAverageMotorRPM()) <=
                flywheel.getDesiredFlywheelSpeed() * MAXIMUM_SPEED_THRESHOLD_FRACTION);
    }

    bool isFinished() final { return !isReady(); }

private:
    src::control::flywheel::FlywheelInterface &flywheel;

    static constexpr float MINIMUM_SPEED_THRESHOLD_FRACTION = 0.9;
    static constexpr float MAXIMUM_SPEED_THRESHOLD_FRACTION = 1.05;
};
}  // namespace src::control::governor

#endif  // FLYWHEEL_ON_GOVERNOR_HPP_
