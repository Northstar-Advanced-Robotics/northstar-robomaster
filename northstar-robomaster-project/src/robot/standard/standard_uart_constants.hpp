#ifndef STANDARD_UART_CONSTANTS_HPP
#define STANDARD_UART_CONSTANTS_HPP

#include <cstdint>

#include "tap/architecture/periodic_timer.hpp"

#ifndef UART_CONSTANTS_HPP_
#error "Do not include this file directly! Use uart_constants.hpp instead."
#endif

namespace src::serial
{
static constexpr uint32_t TIME_BEFORE_UART_START =
    1000;  // initial delay of 1 second to allow time for things to start up
static tap::arch::MilliTimeout messageOffsetInitializationTimeout{TIME_BEFORE_UART_START};

// The message frequencies should be (primeScaleFactor) * (a prime number) so that they do not align
// within (primeScaleFactor) milis of each other
static constexpr uint16_t primeScaleFactor = 10;

/** Time in ms between sending the odometry message. */
static constexpr uint32_t TIME_BTWN_SENDING_ODOMETRY_MSG = 2 * primeScaleFactor;
static constexpr uint32_t TIME_BEFORE_SENDING_ODOMETRY_MSG =
    TIME_BTWN_SENDING_ODOMETRY_MSG + TIME_BEFORE_UART_START;
static tap::arch::PeriodicMilliTimer sendOdometryMsgTimeout{4};

/** Time in ms between sending the Robot Health message. */
static constexpr uint32_t TIME_BTWN_SENDING_HEALTH_MSG = 3 * primeScaleFactor;
static constexpr uint32_t TIME_BEFORE_SENDING_HEALTH_MSG =
    TIME_BTWN_SENDING_HEALTH_MSG + TIME_BEFORE_UART_START;
static tap::arch::PeriodicMilliTimer sendHealthMsgTimeout{TIME_BTWN_SENDING_HEALTH_MSG};

/** Time in ms between sending the Robot Health message. */
static constexpr uint32_t TIME_BTWN_SENDING_REF_TURRET_DATA_MSG = 5 * primeScaleFactor;
static constexpr uint32_t TIME_BEFORE_SENDING_REF_TURRET_DATA_MSG =
    TIME_BTWN_SENDING_REF_TURRET_DATA_MSG + TIME_BEFORE_UART_START;
static tap::arch::PeriodicMilliTimer sendRefTurretDataMsgTimeout{
    TIME_BTWN_SENDING_REF_TURRET_DATA_MSG};

}  // namespace src::serial
#endif