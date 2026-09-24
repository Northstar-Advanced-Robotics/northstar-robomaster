/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef IMU_CALIBRATE_COMMAND_HPP_
#define IMU_CALIBRATE_COMMAND_HPP_

#include <vector>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "communication/can/turret/turret_mcb_can_comm.hpp"
#include "control/buzzer/play_song_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/turret_subsystem.hpp"

#include "imu_calibrate_template.hpp"

using namespace tap::algorithms;

namespace src::control::imu
{
/**
 * @ingroup util
 *
 * Calibrates the onboard BMI088 IMU, holding the robot still while it happens.
 *
 * A gyroscope can only measure its own bias while stationary, so the command takes the turret and
 * chassis away from the operator, parks them, waits for motion to settle, and only then asks the
 * IMU to recalibrate. Requires the robot to have a turret and a chassis subsystem.
 *
 * When scheduled it performs the following actions:
 * 1. Wait until the turret and the IMU are online.
 * 2. Command the pitch and yaw gimbals to their configured `startAngle`.
 * 3. Command the chassis to stay still.
 * 4. Pause until the chassis and turret are no longer moving.
 * 5. Signal the onboard IMU to recalibrate.
 * 6. Wait for calibration to complete, play a tone, and end.
 *
 * @note An earlier revision also calibrated a turret-mounted IMU over `TurretMCBCanComm`. That
 *      path is commented out throughout this class, so no turret MCB is required or contacted.
 */
class ImuCalibrateCommand : public ImuCalibrateCommandBase
{
public:
    /**
     * Specifies the current calibration state that command is in.
     */
    enum class CalibrationState
    {
        /** While in this state, the command waits for the turret to be online and the IMUs to be
           online. */
        WAITING_FOR_SYSTEMS_ONLINE,
        /** The command holds the turret at its configured `startAngle` and waits for it to settle
           before requesting calibration. */
        LOCKING_TURRET,
        /** The command waits for the BMI088 to report that calibration has finished. */
        CALIBRATING_IMU,
        /** @warning Never entered. Nothing assigns this state; the completion tone is scheduled
           from `WAITING_CALIBRATION_COMPLETE` instead. */
        BUZZING,
        /** The command waits a short settling time after calibration completes, during which the
           completion tone is scheduled, then finishes. */
        WAITING_CALIBRATION_COMPLETE,
    };

    /**
     * Threshold around 0 where turret pitch and yaw velocity is considered to be 0, in radians/s
     */
    const float velocityZeroThreshold;
    /**
     * Threshold around 0 where turret pitch and yaw position from the center considered to be 0,
     * in radians
     */
    const float positionZeroThreshold;

    struct TurretIMUCalibrationConfig
    {
        /// (Unused: the turret-MCB IMU member this described is commented out below.)
        // src::can::TurretMCBCanComm *turretMCBCanComm;
        /// A `TurretSubsystem` that this command will control (will lock the turret).
        turret::TurretSubsystem *turret;
        /// A chassis relative yaw controller used to lock the turret.
        turret::algorithms::TurretYawControllerInterface *yawController;
        /// A chassis relative pitch controller used to lock the turret.
        turret::algorithms::TurretPitchControllerInterface *pitchController;
        /**
         * `true` if the turret IMU is mounted on the pitch axis of the
         * turret. In this case the pitch controller doesn't have to reach the horizontal setpoint
         * before calibration is performed.
         */
        bool turretImuOnPitch;
    };

    /**
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] turretsAndControllers A list of TurretIMUCalibrationConfig structs containing
     * turret and turret IMU information necessary for calibrating the IMU
     * @param[in] chassis A `ChassisSubsystem` that this command will control (will set the desired
     * movement to 0).
     * @param[in] song Tone scheduled once calibration completes, so the operator knows the robot
     * is usable again. Optional.
     * @param[in] velocityZeroThreshold Threshold around 0 where turret pitch and yaw velocity is
     * considered to be 0, in radians/s.
     * @param[in] positionZeroThreshold Threshold around 0 where turret pitch and yaw position from
     * the center considered to be 0, in radians.
     */
    ImuCalibrateCommand(
        tap::Drivers *drivers,
        const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
        chassis::ChassisSubsystem *chassis,
        src::control::buzzer::PlaySongCommand *song = nullptr,
        float velocityZeroThreshold = ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
        float positionZeroThreshold = ImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD);

    const char *getName() const override { return "Calibrate IMU"; }

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    /**
     * @return The current calibration state of the command.
     */
    CalibrationState getCalibrationState() const { return calibrationState; }

protected:
    /**
     * Wait a minimum of this time to allow the turret to settle at a locked position (in ms).
     */
    static constexpr uint32_t WAIT_TIME_TURRET_RESPONSE_MS = 2000;
    /**
     * @warning Unreferenced. The code hardcodes a 200 ms settle instead. Was: wait this long
     * after the onboard IMU finishes calibrating to ensure the turret MCB's IMU is
     * calibrated.
     */
    static constexpr uint32_t TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS = 2000;
    /**
     * Wait timeout (after state `WAITING_FOR_SYSTEMS_ONLINE` is complete) for the command to wait
     * until it gives up. Should never (and has never) happen but is a safety precaution to avoid
     * getting stuck in calibration forever.
     */
    static constexpr uint32_t MAX_CALIBRATION_WAITTIME_MS = 20000;

    static bool COMMAND_IS_RUNNING;

    tap::Drivers *drivers;
    std::vector<TurretIMUCalibrationConfig> turretsAndControllers;
    chassis::ChassisSubsystem *chassis;
    src::control::buzzer::PlaySongCommand *song;

    CalibrationState calibrationState;

    uint32_t prevTime = 0;

    /**
     * Timeout that we set after initially starting the turret PID controller to allow any residual
     * movement from starting the new PID controller to be resolved.
     *
     * Also the delay that we set after the onboard BMI088 is calibrated to ensure that turret IMU has
     * enough time to successfully calibrate.
     */
    tap::arch::MilliTimeout calibrationTimer;

    tap::arch::MilliTimeout buzzerTimer;

    /**
     * Timeout used to determine if we should give up on calibration.
     */
    tap::arch::MilliTimeout calibrationLongTimeout;

    inline bool turretReachedCenterAndNotMoving(turret::TurretSubsystem *turret, bool ignorePitch)
        const
    {
        return compareFloatClose(
                   0.0f,
                   turret->yawMotor.getChassisFrameVelocity(),
                   velocityZeroThreshold) &&
               (turret->yawMotor.getChassisFrameMeasuredAngle().minDifference(0) <
                positionZeroThreshold) &&
               (ignorePitch || (compareFloatClose(
                                    0.0f,
                                    turret->pitchMotor.getChassisFrameVelocity(),
                                    velocityZeroThreshold) &&
                                (turret->pitchMotor.getChassisFrameMeasuredAngle().minDifference(
                                     0) < positionZeroThreshold)));
    }

public:
    static bool GetIsComandRunning() { return COMMAND_IS_RUNNING; }
};
}  // namespace src::control::imu

#endif  // IMU_CALIBRATE_COMMAND_HPP_
