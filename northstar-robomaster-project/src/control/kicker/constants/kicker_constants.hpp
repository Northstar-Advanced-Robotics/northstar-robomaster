#ifndef KICKER_CONSTANTS_HPP_
#define KICKER_CONSTANTS_HPP_

/**
 * Selects the kicker constants for the robot currently being built.
 *
 * Motor IDs, gear ratios, and velocity setpoints live in each target's `*_kicker_constants.hpp`;
 * `standard` is used as the fallback when no known target is defined.
 */
#ifdef TARGET_STANDARD
#include "robot/standard/standard_kicker_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_kicker_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_kicker_constants.hpp"
#elif TURRET
#include "robot/standard/standard_kicker_constants.hpp"
#elif TARGET_TEST_BED
#include "robot/hero/hero_kicker_constants.hpp"
#else
#include "robot/standard/standard_kicker_constants.hpp"
#endif

#endif  // KICKER_CONSTANTS_HPP_
