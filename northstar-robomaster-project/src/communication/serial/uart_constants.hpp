#ifndef UART_CONSTANTS_HPP_
#define UART_CONSTANTS_HPP_

/**
 * Selects the UART constants (baud rates and port assignments) for the robot currently being
 * built.
 *
 * Every robot wires its serial peripherals to different UART ports, so each target provides its
 * own `*_uart_constants.hpp`. Including this header pulls in the right one; `standard` is used as
 * the fallback when no known target is defined.
 */
#ifdef TARGET_STANDARD
#include "robot/standard/standard_uart_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_uart_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_uart_constants.hpp"
#elif TURRET
#include "robot/standard/standard_uart_constants.hpp"
#else
#include "robot/standard/standard_uart_constants.hpp"
#endif

namespace src::serial
{
}  // namespace src::serial

#endif  // UART_CONSTANTS_HPP
