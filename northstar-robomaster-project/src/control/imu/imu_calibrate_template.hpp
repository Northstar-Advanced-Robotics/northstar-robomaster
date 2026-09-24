#ifndef IMU_CALIBRATE_COMMAND_BASE_HPP_
#define IMU_CALIBRATE_COMMAND_BASE_HPP_

#include <vector>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

namespace src::control::imu
{
/**
 * @ingroup util
 *
 * Interface for the IMU calibration command, so `robot_control.hpp` can hand one back without every
 * robot's control file depending on the concrete type.
 *
 * @note The state enum and timing constants declared here are shadowed by identically named members
 *      in `ImuCalibrateCommand`; the copies in this base are inert.
 */
class ImuCalibrateCommandBase : public tap::control::Command
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
           before requesting calibration of the onboard BMI088. */
        LOCKING_TURRET,
        /** While in this state, the command waits until calibration of the IMUs are complete. */
        CALIBRATING_IMU,
        /** @warning Never entered; see `ImuCalibrateCommand`. */
        BUZZING,
        /** The command waits a short settling time after calibration completes, then finishes. */
        WAITING_CALIBRATION_COMPLETE,
    };

    static constexpr float DEFAULT_VELOCITY_ZERO_THRESHOLD = modm::toRadian(1e-2);
    static constexpr float DEFAULT_POSITION_ZERO_THRESHOLD = modm::toRadian(3.0f);

    const char* getName() const override { return "Calibrate IMU"; }

    virtual bool isReady() override = 0;
    virtual void initialize() override = 0;
    virtual void execute() override = 0;
    virtual void end(bool interrupted) override = 0;
    virtual bool isFinished() const override = 0;

protected:
    static constexpr uint32_t WAIT_TIME_TURRET_RESPONSE_MS = 2000;
    static constexpr uint32_t TURRET_IMU_EXTRA_WAIT_CALIBRATE_MS = 2000;
    static constexpr uint32_t MAX_CALIBRATION_WAITTIME_MS = 20000;

    tap::Drivers* drivers;

    CalibrationState calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;

    uint32_t prevTime = 0;
    tap::arch::MilliTimeout calibrationTimer;
    tap::arch::MilliTimeout buzzerTimer;
    tap::arch::MilliTimeout calibrationLongTimeout;
};

}  // namespace src::control::imu

#endif
