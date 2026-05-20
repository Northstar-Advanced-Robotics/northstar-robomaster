/*
 * Copyright (c) 2020-2021 NorthStart
 *
 * This file is part of NorthStarFleet2025.
 *
 * NorthStarFleet2025 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NorthStarFleet2025 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NorthStarFleet2025.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "communication/serial/fly_sky.hpp"

#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "robot/robot_control.hpp"

/* robot includes ---------------------------------------------------------*/

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(tap::Drivers::DT);
// tap::arch::PeriodicMilliTimer revTxPublisherTimeout(20);
// tap::arch::PeriodicMilliTimer revHeartBeatTimeout(100);

#ifdef TARGET_STANDARD
using namespace src::standard;
#elif TARGET_SENTRY
using namespace src::sentry;
#elif TARGET_HERO
using namespace src::hero;
#elif TURRET
#include "communication/can/chassis/chassis_mcb_can_comm.hpp"
using namespace src::gyro;
ChassisMcbCanComm chassisMcbCanComm(DoNotUse_getDrivers());
#elif TARGET_TEST_BED
using namespace src::testbed;
#endif

// using namespace std::chrono_literals;

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.

uint16_t deltaTime = 0;
uint16_t lastTime = 0;

static void updateIo(Drivers *drivers);
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
    initializeIo(drivers);
    initSubsystemCommands(drivers);
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        //         // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            uint16_t currentTTime = tap::arch::clock::getTimeMicroseconds();
            deltaTime = currentTTime - lastTime;
            lastTime = currentTTime;

            PROFILE(drivers->profiler, drivers->bmi088.periodicIMUUpdate, ());

            PROFILE(drivers->profiler, drivers->encoder.update, ());

            // PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
#ifdef TURRET
            PROFILE(drivers->profiler, chassisMcbCanComm.sendIMUData, ());
            PROFILE(drivers->profiler, chassisMcbCanComm.sendSynchronizationRequest, ());
#else
            // PROFILE(drivers->profiler, drivers->turretMCBCanCommBus2.sendData, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
#endif
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
        PROFILE(drivers->profiler, drivers->visionComms.sendMessage, ());

        // #endif
        modm::delay_us(10);
    }
    return 0;
}
static void initializeIo(Drivers *drivers)
{
    // things we need to check controller
    drivers->remote.initialize();
    drivers->analog.init();
    drivers->digital.init();
    drivers->leds.init();

    // if controller is on when the robot turns on, wait for it to be off.
    // This is to prevent the shredding of wires
    modm::delay_ms(3000);
    drivers->leds.set(tap::gpio::Leds::Red, true);
    int i = 0;
    while (i < 5000)
    {
        drivers->remote.read();
        if (drivers->remote.isConnected())
            i = 0;
        else
            i++;
        modm::delay_us(10);
    }

    drivers->leds.set(tap::gpio::Leds::Blue, true);

    drivers->pwm.init();
    drivers->can.initialize();
    drivers->errorController.init();

    drivers->encoder.initialize();
    drivers->visionComms.initializeUartDelays();

    drivers->refSerial.initialize();

    drivers->bmi088.initialize(500, 0.001f, 0.000f);
    drivers->bmi088.setTargetTemperature(35.0f);
    drivers->bmi088.setCalibrationSamples(1000);

#ifndef FLY_SKY
    drivers->visionComms.initializeCV();
#endif
}
float debugYaw = 0.0f;
float debugPitch = 0.0f;
float debugRoll = 0.0f;
float debugYawV = 0.0f;
float debugPitchV = 0.0f;
float debugRollV = 0.0f;
bool conneccc = false;
float debugLastAimDataYaw = 0.0f;
float debugLastAimDataPitch = 0.0f;
float dddddgfregr = 0;
bool uartOnline = false;

bool cal = false;
bool calibrated = false;
static void updateIo(Drivers *drivers)
{
    // #ifndef TARGET_TEST_BED
    if (!calibrated && drivers->remote.isConnected())
    {
        drivers->commandScheduler.addCommand(getImuCalibrateCommand());
        calibrated = true;
    }
// #endif
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->bmi088.read();

#ifndef TURRET
    drivers->refSerial.updateSerial();
#ifndef FLY_SKY
    drivers->visionComms.updateSerial();
#endif

    drivers->remote.read();

    if (cal)
    {
        cal = false;
        drivers->bmi088.requestCalibration();
    }
    debugYawV = drivers->bmi088.getGz();
    debugYaw = modm::toDegree(drivers->bmi088.getYaw());
    debugPitchV = drivers->bmi088.getGy();
    debugPitch = modm::toDegree(drivers->bmi088.getPitch());
    debugRollV = drivers->bmi088.getGx();
    debugRoll = modm::toDegree(drivers->bmi088.getRoll());
    conneccc = drivers->remote.isConnected();
    dddddgfregr = drivers->encoder.getPosition().getUnwrappedValue();
    uartOnline = drivers->refSerial.getRefSerialReceivingData();

#endif
}