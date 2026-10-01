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
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "robot/robot_control.hpp"

/* robot includes ---------------------------------------------------------*/
#include "tap/communication/gpio/pwm.hpp"

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
#include "communication/can/chassis/chassis_mcb_can_comm.hpp"
using namespace src::robot::turret;
src::communication::can::ChassisMcbCanComm chassisMcbCanComm(DoNotUse_getDrivers());
#elif TARGET_TEST_BED
using namespace src::robot::testbed;
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

    while (1)
    {
        //         // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            uint16_t currentTTime = tap::arch::clock::getTimeMicroseconds();
            deltaTime = currentTTime - lastTime;
            lastTime = currentTTime;

            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());

#ifndef TARGET_TURRET
            PROFILE(drivers->profiler, drivers->encoder.update, ());
#endif

            // PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
#ifdef TARGET_TURRET
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
#ifndef TARGET_TURRET
        PROFILE(drivers->profiler, drivers->visionComms.sendMessage, ());
#endif

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
    drivers->pwm.init();

    // if controller is on when the robot turns on, wait for it to be off.
    // This is to prevent the shredding of wires
    modm::delay_ms(3000);
    drivers->leds.set(tap::gpio::Leds::Red, true);
    int i = 0;
    while (i < 5000)
    {
        drivers->remote.read();
        if (drivers->remote.isConnected())
        {
            i = 0;
            drivers->pwm.write(0.5f, tap::gpio::Pwm::Buzzer);
            drivers->pwm.setTimerFrequency(tap::gpio::Pwm::TIMER12, 1500);  // buzzer timer on Type A
        }
        else
        {
            i++;
            drivers->pwm.write(0.0f, tap::gpio::Pwm::Buzzer);
        }

        modm::delay_us(10);
    }

    drivers->leds.set(tap::gpio::Leds::Green, true);  // Type A has no blue LED

    drivers->can.initialize();
    drivers->errorController.init();

#ifndef TARGET_TURRET
    drivers->encoder.initialize();
    drivers->visionComms.initializeUartDelays();
#endif

    drivers->refSerial.initialize();

#ifdef TARGET_HERO
    drivers->mpu6500.initialize(500, 0.1f, 0.000f);
    drivers->mpu6500.setTargetTemperature(35.0f);
    drivers->mpu6500.setCalibrationSamples(2000);
#else
    drivers->mpu6500.initialize(500, 0.05f, 0.000f);
    drivers->mpu6500.setTargetTemperature(35.0f);
    drivers->mpu6500.setCalibrationSamples(2000);
#endif

#ifndef TARGET_TURRET
    drivers->visionComms.initializeCV();
#endif
}
float debugXAccel = 0.0f;
float debugYAccel = 0.0f;
float debugZAccel = 0.0f;
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
RefSerialData::Rx::RobotData robotData;
uint16_t heat17;
uint32_t rfidStat;
static void updateIo(Drivers *drivers)
{
    // #ifndef TARGET_TEST_BED
    if (!calibrated && drivers->remote.isConnected())
    {
        drivers->commandScheduler.addCommand(getImuCalibrateCommand());
        calibrated = true;
    }
// #endif
    drivers->canRxHandler.pollCanData();
    drivers->mpu6500.read();

#ifndef TARGET_TURRET
    drivers->refSerial.updateSerial();
#ifndef FLYSKY
    drivers->visionComms.updateSerial();
#endif

    drivers->remote.read();

    if (cal)
    {
        cal = false;
        drivers->mpu6500.requestCalibration();
    }
    debugXAccel = drivers->mpu6500.getAx();
    debugYAccel = drivers->mpu6500.getAy();
    debugZAccel = drivers->mpu6500.getAz();
    debugYawV = drivers->mpu6500.getGz();
    debugYaw = modm::toDegree(drivers->mpu6500.getYaw());
    debugPitchV = drivers->mpu6500.getGy();
    debugPitch = modm::toDegree(drivers->mpu6500.getPitch());
    debugRollV = drivers->mpu6500.getGx();
    debugRoll = modm::toDegree(drivers->mpu6500.getRoll());
    conneccc = drivers->remote.isConnected();
    dddddgfregr = drivers->encoder.getPosition().getUnwrappedValue();
    uartOnline = drivers->refSerial.getRefSerialReceivingData();
    robotData = drivers->refSerial.getRobotData();
    heat17 = drivers->refSerial.getRobotData().turret.heat17;
    rfidStat = drivers->refSerial.getRobotData().rfidStatus.value;
#endif
}