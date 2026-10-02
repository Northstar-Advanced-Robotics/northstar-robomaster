/*
 * Copyright (c) 2020-2021 NorthStart
 *
 * This file is part of NorthStarControls.
 *
 * NorthStarControls is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NorthStarControls is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NorthStarControls.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers_singleton.hpp"

namespace src::robot_loop
{
// The robot's own Drivers type (not the generic src::Drivers), selected by the build target.
#ifdef TARGET_STANDARD
using src::robot::standard::Drivers;
#elif TARGET_SENTRY
using src::robot::sentry::Drivers;
#elif TARGET_HERO
using src::robot::hero::Drivers;
#elif TARGET_TURRET
using src::robot::turret::Drivers;
#elif TARGET_TEST_BED
using src::robot::testbed::Drivers;
#endif

/// Everything main() did before the loop: early IO init, optional remote-off safety wait,
/// remaining IO init, then initSubsystemCommands(). Does NOT call Board::initialize() (main.cpp keeps that).
/// @param waitForRemoteOff  true on the robot (main.cpp); false in the simulator.
void robotInit(Drivers *drivers, bool waitForRemoteOff);

/// Runs every pass of the main loop, as fast as possible (was: updateIo + visionComms.sendMessage).
void robotLoopFast(Drivers *drivers);

/// Runs once per tap::Drivers::DT ms (was: the body of `if (sendMotorTimeout.execute())`):
/// IMU periodic update, encoder update, commandScheduler.run, motor TX / turret CAN sends.
void robotLoopTick(Drivers *drivers);
}  // namespace src::robot_loop
