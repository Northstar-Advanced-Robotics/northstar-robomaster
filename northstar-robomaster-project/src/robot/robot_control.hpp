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

#ifndef ROBOT_CONTROL_HPP_
#define ROBOT_CONTROL_HPP_

#include "control/imu/imu_calibrate_command.hpp"
#include "robot/standard/standard_drivers.hpp"
#include "robot/turret/turret_drivers.hpp"

#ifdef TARGET_STANDARD
namespace src::standard
#elif defined(TARGET_SENTRY)
namespace src::sentry
#elif defined(TARGET_HERO)
namespace src::hero
#elif defined(TARGET_TURRET)
namespace src::gyro
#elif defined(TARGET_TEST_BED)
namespace src::testbed
#endif
{
void initSubsystemCommands(Drivers *drivers);
// #ifndef TARGET_TEST_BED
src::control::imu::ImuCalibrateCommandBase *getImuCalibrateCommand();
// #endif
}  // namespace tbh whatever you want it to be

#endif  // ROBOT_CONTROL_HPP_
