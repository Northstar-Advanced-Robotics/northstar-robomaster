/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_CONSTANTS_HPP_
#define TURRET_CONSTANTS_HPP_

#include <stdint.h>

#include "tap/util_macros.hpp"

/**
 * Selects the turret constants for the robot currently being built.
 *
 * Motor IDs, encoder offsets, pitch and yaw travel limits, and controller gains live in each
 * target's `*_turret_constants.hpp`; `standard` is used as the fallback when no known target is
 * defined.
 */
#ifdef TARGET_STANDARD
#include "robot/standard/standard_turret_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_turret_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_turret_constants.hpp"
#elif TURRET
#include "robot/standard/standard_turret_constants.hpp"
#elif TARGET_TEST_BED
#include "robot/testbed/testbed_turret_constants.hpp"

#else
#include "robot/standard/standard_turret_constants.hpp"

#endif

#endif  // TURRET_CONSTANTS_HPP_
