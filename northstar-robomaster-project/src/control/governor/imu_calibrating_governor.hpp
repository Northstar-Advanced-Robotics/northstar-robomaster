#ifndef IMU_CALIBRATING_HPP
#define IMU_CALIBRATING_HPP

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "control/imu/imu_calibrate_command.hpp"

namespace src::control::governor
{
/**
 * @ingroup governors
 *
 * Blocks commands while the IMU is being calibrated.
 *
 * Calibration requires the robot to sit still, so anything that would move it must not be allowed
 * to start, and anything already running must be stopped. Gate a command on this governor to get
 * both behaviors.
 */
class ImuCalibratingGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] drivers The global drivers object.
     */
    ImuCalibratingGovernor(tap::Drivers *drivers) : drivers(drivers) {}

    /// @return `true` when no IMU calibration is in progress, so a gated command may start.
    bool isReady() final { return !src::control::imu::ImuCalibrateCommand::GetIsComandRunning(); }

    /// @return `true` once a calibration starts, ending any gated command that was already
    /// running.
    bool isFinished() final { return !isReady(); }

private:
    /// The global drivers object.
    tap::Drivers *drivers;
};
}  // namespace src::control::governor

#endif  // IMU_CALIBRATING_HPP
