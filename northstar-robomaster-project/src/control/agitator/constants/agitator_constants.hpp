#ifndef AGITATOR_CONSTANTS_HPP_
#define AGITATOR_CONSTANTS_HPP_

#include "robot/testbed/test_def.hpp"

/**
 * Selects the agitator constants for the robot currently being built.
 *
 * Motor IDs, gear ratios, shot spacing, and unjamming thresholds live in each target's
 * `*_agitator_constants.hpp`. The test bed can be built with either its own agitator or the hero's,
 * selected by `USING_HERO_AGITATOR`; `standard` is used as the fallback when no known target is
 * defined.
 */
#ifdef TARGET_STANDARD
#include "robot/standard/standard_agitator_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_agitator_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_agitator_constants.hpp"
#elif TURRET
#include "robot/standard/standard_agitator_constants.hpp"
#elif TARGET_TEST_BED
#ifdef USING_HERO_AGITATOR
#include "robot/hero/hero_agitator_constants.hpp"
#else
#include "robot/testbed/testbed_agitator_constants.hpp"
#endif
#else
#include "robot/standard/standard_agitator_constants.hpp"
#endif

#endif  // AGITATOR_CONSTANTS_HPP_
