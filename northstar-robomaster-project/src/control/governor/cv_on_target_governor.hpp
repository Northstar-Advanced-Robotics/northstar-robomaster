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
 * A governor that allows a Command to run when a TurretCVCommand has acquired and is aiming at a
 * target.
 */
class CvOnTargetGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    CvOnTargetGovernor(
        tap::Drivers *drivers,
        src::serial::VisionComms &visionComms,
        src::control::turret::cv::TurretCVControlCommandTemplate &turretCVCommand,
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

            return turretCVCommand.isAimingWithinLaunchingTolerance();
        }
        else
        {
            return turretCVCommand.isAimingWithinLaunchingTolerance();
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
    bool enabled = true;
    bool sentry;
};
}  // namespace src::control::governor

#endif  // CV_ON_TARGET_GOVERNOR_HPP_