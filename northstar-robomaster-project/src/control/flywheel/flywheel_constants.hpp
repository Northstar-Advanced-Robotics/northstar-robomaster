#ifndef FLYWHEEL_CONSTANTS_HPP_
#define FLYWHEEL_CONSTANTS_HPP_

#include "tap/motor/dji_motor.hpp"

#include "modm/math/interpolation/linear.hpp"

/**
 * Selects the flywheel constants for the robot currently being built.
 *
 * Each robot fires a different projectile at a different speed, so wheel geometry, launch speed
 * limits, and the launch-speed-to-RPM mapping live in each target's `*_flywheel_constants.hpp`.
 */
#ifdef TARGET_STANDARD
#include "robot/standard/standard_flywheel_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_flywheel_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_flywheel_constants.hpp"
#elif TURRET
#include "robot/standard/standard_flywheel_constants.hpp"
#else
#include "robot/hero/hero_flywheel_constants.hpp"
#endif

namespace src::flywheel
{
}  // namespace src::flywheel

#endif  // FLYWHEEL_CONSTANTS_HPP_
