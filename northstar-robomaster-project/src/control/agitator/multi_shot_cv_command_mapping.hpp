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

#ifndef MULTI_SHOT_CV_COMMAND_MAPPING_HPP_
#define MULTI_SHOT_CV_COMMAND_MAPPING_HPP_

#include <optional>

#include "tap/control/hold_repeat_command_mapping.hpp"

#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/governor/cv_on_target_governor.hpp"

#include "manual_fire_rate_reselection_manager.hpp"

namespace src
{
class Drivers;
}

namespace src::control::agitator
{
/**
 * Class that stores and allows the user to set some LaunchMode. Possible launch modes include
 * single, 10 Hz, or 20 Hz full auto mode.
 *
 * This object is a HoldRepeatCommandMapping. An instance of this object should be added to the
 * global CommandMapper to use it. This object contains a launch Command that it will schedule. How
 * often/when the launch Command will be schedule is based on the launch mode. This object controls
 * the fire rate and firing frequency of the launch Command based on the launch mode.
 *
 * If vision is running, the fire rate should not be limited and the launcher should be in full auto
 * mode, so this object checks the launchMode of a CvOnTargetGovernor when setting the fire rate.
 */
class MultiShotCvCommandMapping : public tap::control::HoldRepeatCommandMapping
{
public:
    /**
     * State of the shooting mechanism, how many times the associated command mapping should
     * reschedule the command when the mapping is met.
     */
    enum LaunchMode : uint8_t
    {
        SINGLE = 0,
#ifndef TARGET_HERO
        NO_HEATING,
        LIMITED_10HZ,
        LIMITED_20HZ,
        FULL_AUTO,
#endif
        BURST,
        NUM_SHOOTER_STATES,
    };

    /**
     * @param[in] drivers Reference to global drivers object.
     * @param[in] launchCommand Command that when scheduled launches a single projectile.
     * @param[in] rms The remote mapping that controls when the launch command should be scheduled.
     * @param[in] fireRateReselectionManager An optional argument, the fire rate reselection manager
     * that controls the fire rate of the launch command. If provided, the manager's fire rate is
     * updated based on the current LaunchMode.
     * @param[in] cvOnTargetGovernor The governor whose state will be used to override the current
     * LaunchMode. This allows us to override user-defined launch mode when CV is controlling the
     * launching frequency.
     */
    MultiShotCvCommandMapping(
        tap::Drivers &drivers,
        tap::control::Command &launchCommand,
        const tap::control::GenericRemoteMapState &rms,
        std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager,
        governor::CvOnTargetGovernor &cvOnTargetGovernor,
        std::optional<ConstantVelocityAgitatorCommand *> command = std::nullopt);

    void setShooterState(LaunchMode mode)
    {
        if (mode < NUM_SHOOTER_STATES)
        {
            this->launchMode = mode;
        }
    }

    LaunchMode getLaunchMode() const { return launchMode; }

    void executeCommandMapping(const tap::control::GenericRemoteMapState &currState);

private:
    std::optional<ManualFireRateReselectionManager *> fireRateReselectionManager;
    governor::CvOnTargetGovernor &cvOnTargetGovernor;
#ifdef TARGET_SENTRY
    LaunchMode launchMode = LIMITED_10HZ;
#else
    LaunchMode launchMode = SINGLE;
#endif
    std::optional<ConstantVelocityAgitatorCommand *> command;

    int getCurrentBarrelCoolingRate() const
    {
        int coolingRate = drivers->refSerial.getRobotData().turret.coolingRate;
#if defined(TARGET_HERO)
        return coolingRate / 100.0f;
#else
        return coolingRate / 10.0f;
#endif
    }

    int getMaxBurst(int targetBurstSize = 10)
    {
#if defined(TARGET_HERO)
        // Allow for overheating but not to the point of disable. allow for 2 shot burst. We can
        // likely just do return 2 i think.
        int heat = drivers->refSerial.getRobotData().turret.heat42;
        int heatLimit = drivers->refSerial.getRobotData().turret.heatLimit;
        if (heat < heatLimit - 100)
        {
            return 2;
        }
        else if (heat < heatLimit)
        {
            return 1;
        }
        else
        {
            return 0;
        }
#else
        /* With the 17 shots, get the max heat allowed and our current heat and calculate how many
           shots we can take. Ideally we do not oveheat. If we cannot reach the passed in target
           burst size return how many shots we can take without overheating. If we can reach the
           target burst size, return the target burst size.
        */
        int heat = drivers->refSerial.getRobotData().turret.heat17;
        int heatLimit = drivers->refSerial.getRobotData().turret.heatLimit;
        return std::min(targetBurstSize, (heatLimit - heat) / 10);
        /* With the 17 shots, get the max heat allowed and our current heat and calculate how many
           shots we can take. Ideally we do not oveheat. If we cannot reach the passed in target
           burst size return how many shots we can take without overheating. If we can reach the
           target burst size, return the target burst size.
        */
#endif
    }
};

}  // namespace src::control::agitator

#endif  // MULTI_SHOT_CV_COMMAND_MAPPING_HPP_
