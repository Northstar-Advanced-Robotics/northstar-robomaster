#ifndef UART_CONSTANTS_HPP_
#define UART_CONSTANTS_HPP_

#ifdef TARGET_STANDARD
#include "robot/standard/standard_uart_constants.hpp"
#elif defined(TARGET_SENTRY)
#include "robot/sentry/sentry_uart_constants.hpp"
#elif defined(TARGET_HERO)
#include "robot/hero/hero_uart_constants.hpp"
#elif defined(TARGET_TURRET)
#include "robot/standard/standard_uart_constants.hpp"
#else
#include "robot/standard/standard_uart_constants.hpp"
#endif

namespace src::serial
{
}  // namespace src::serial

#endif  // UART_CONSTANTS_HPP
