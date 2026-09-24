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

#ifndef CV_ON_TARGET_GOVERNOR_HPP_
#define CV_ON_TARGET_GOVERNOR_HPP_

#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "communication/serial/vision_comms.hpp"
#include "control/turret/cv/turret_cv_control_command_template.hpp"

namespace src::control::governor
{
/**
 * @ingroup governors
 *
 * Gates firing on auto-aim actually being on target.
 *
 * Consults the running `TurretCVControlCommandTemplate` for whether the turret is aimed closely
 * enough that a projectile launched now would hit, so the agitator does not waste ammunition while
 * vision is still slewing onto a plate.
 *
 * Two behaviours worth knowing:
 * - On a non-sentry robot the aim check is only applied while gating is enabled; with it off,
 *   `isReady` returns `true` regardless of aim, so the operator can always fire manually.
 * - On the sentry the gating toggle is bypassed and aim is **always** required, even with vision
 *   offline -- which means an offline vision computer stops the sentry firing entirely.
 */
class CvOnTargetGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] visionComms The vision link, consulted for whether vision is online.
     * @param[in] turretCVCommand The CV control command asked whether the turret is on target.
     * @param[in] turretID Which turret to ask about.
     * @param[in] sentry `true` on the sentry, which always requires aim; see the class note.
     *
     * @warning `turretID` is marked `[[maybe_unused]]` and is **not stored**. The member of the
     *      same name is left uninitialized and then read by `isReady`, so the turret actually
     *      queried is indeterminate.
     */
    CvOnTargetGovernor(
        tap::Drivers *drivers,
        src::serial::VisionComms &visionComms,
        src::control::turret::cv::TurretCVControlCommandTemplate &turretCVCommand,
        [[maybe_unused]] uint8_t turretID = 0,
        bool sentry = false)
        : drivers(drivers),
          visionComms(visionComms),
          turretCVCommand(turretCVCommand),
          sentry(sentry)
    {
    }

    mockable void setGovernorEnabled(bool enabled) { this->enabled = enabled; }

    mockable bool isGoverEnabled() const { return this->enabled; }

    /**
     * @return true if gating is being performed. If gating is being performed, projectiles will be
     * launched if the CV system decides they should be. The criteria are: 1. CV is onine and
     * connected. 2. the robot is executing the CV command. 3. CV Gating mode is enabled.
     * Otherwise, the system will not
     * be gated and projectiles may be launched independently of CV logic.
     */
    mockable bool isGovernorGating() const
    {
        bool isCvOnline = visionComms.isCvOnline();

        bool isCvRunning = drivers->commandScheduler.isCommandScheduled(&turretCVCommand);

        return isCvOnline && enabled && isCvRunning;
    }

    bool isReady() final_mockable
    {
        if (!sentry)
        {
            if (!isGovernorGating())
            {
                return true;
            }

            return turretCVCommand.isAimingWithinLaunchingTolerance(turretID);
        }
        else
        {
            return turretCVCommand.isAimingWithinLaunchingTolerance(turretID);
        }
    }

    bool isFinished() final_mockable
    {
        // Once started, CV will not stop the target command; it is allowed to run to completion.
        // This enables firing a whole round, or burst, without interruption.
        return false;
    }

private:
    tap::Drivers *drivers;
    src::serial::VisionComms &visionComms;
    src::control::turret::cv::TurretCVControlCommandTemplate &turretCVCommand;
    uint8_t turretID;
    bool enabled = true;
    bool sentry;
};
}  // namespace src::control::governor

#endif  // CV_ON_TARGET_GOVERNOR_HPP_