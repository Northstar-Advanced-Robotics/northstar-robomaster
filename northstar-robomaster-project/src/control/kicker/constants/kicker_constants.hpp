#ifndef KICKER_CONSTANTS_HPP_
#define KICKER_CONSTANTS_HPP_

#ifdef TARGET_STANDARD
#include "robot/standard/standard_kicker_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_kicker_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_kicker_constants.hpp"
#elif TURRET
#include "robot/standard/standard_kicker_constants.hpp"
#elif TARGET_TEST_BED
// #include "robot/testbed/testbed_kicker_constants.hpp"
#else
#include "robot/standard/standard_kicker_constants.hpp"
#endif

#endif  // KICKER_CONSTANTS_HPP_
