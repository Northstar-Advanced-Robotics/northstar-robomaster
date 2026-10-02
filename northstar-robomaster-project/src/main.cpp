// #define FLYSKY

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

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers_singleton.hpp"

/* robot loop (init + loop body, shared with the simulator) -----------------*/
#include "robot_loop.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(tap::Drivers::DT);
// tap::arch::PeriodicMilliTimer revTxPublisherTimeout(20);
// tap::arch::PeriodicMilliTimer revHeartBeatTimeout(100);

#ifdef TARGET_STANDARD
using namespace src::robot::standard;
#elif TARGET_SENTRY
using namespace src::robot::sentry;
#elif TARGET_HERO
using namespace src::robot::hero;
#elif TARGET_TURRET
using namespace src::robot::turret;
#elif TARGET_TEST_BED
using namespace src::robot::testbed;
#endif

// using namespace std::chrono_literals;

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    Drivers *drivers = DoNotUse_getDrivers();

    Board::initialize();
    src::robot_loop::robotInit(drivers, true);

    while (1)
    {
        src::robot_loop::robotLoopFast(drivers);

        if (sendMotorTimeout.execute())
        {
            src::robot_loop::robotLoopTick(drivers);
        }
        // #if defined(TARGET_STANDARD) || defined(TARGET_SENTRY)
        //         if (revTxPublisherTimeout.execute())
        //         {
        //             PROFILE(drivers->profiler, drivers->revMotorTxHandler.encodeAndSendCanData,
        //             ());
        //         }
        //         if (revHeartBeatTimeout.execute())
        //         {
        //             PROFILE(drivers->profiler, drivers->revMotorTxHandler.heartBeat, ());
        //         }
        // #endif
        modm::delay_us(10);
    }
    return 0;
}
