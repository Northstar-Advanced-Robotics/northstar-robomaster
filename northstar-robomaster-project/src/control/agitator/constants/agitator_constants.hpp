#ifndef AGITATOR_CONSTANTS_HPP_
#define AGITATOR_CONSTANTS_HPP_

#include "robot/testbed/test_def.hpp"

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
