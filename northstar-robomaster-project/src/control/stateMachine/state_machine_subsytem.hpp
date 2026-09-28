#ifndef STATE_MACHINE_SUBSYTEM_HPP
#define STATE_MACHINE_SUBSYTEM_HPP

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "control/chassis/chassis_auto_drive.hpp"
#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"

namespace src::stateMachine
{
/**
 * @ingroup util
 *
 * Drives the chassis autonomously during a match, on robots that have no operator.
 *
 * Runs only while the referee system reports a match in progress, or while a specific remote switch
 * combination overrides that for testing. When the vision computer has supplied a path the chassis
 * follows it; with no path it holds position. In either case the robot beyblades, spinning faster
 * when it is not translating, since the wheel speed not spent on translation is available for
 * rotation.
 */
class StateMachineSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param[in] drivers The global drivers object, used to read the remote override.
     * @param[in] chassisSubsystem The chassis to drive.
     * @param[in] chassisAutoDrive The path follower supplying velocity and rotation setpoints.
     * @param[in] beybladeCommand Supplies the beyblade spin rate. May be `nullptr` to drive
     *      without spinning.
     * @param[in] matchRunningGovernor Reports whether a match is in progress.
     */
    StateMachineSubsystem(
        tap::Drivers* drivers,
        src::chassis::ChassisSubsystem* chassisSubsystem,
        src::chassis::ChassisAutoDrive* chassisAutoDrive,
        src::chassis::ChassisBeybladeCommand* beybladeCommand,
        src::control::governor::MatchRunningGovernor* matchRunningGovernor);

    /// Does nothing; this subsystem holds no hardware of its own.
    void initialize() override;

    /// Advances the path follower and drives the chassis for this iteration, or leaves it stopped
    /// if the match is not running. Called once per control loop iteration by the scheduler.
    void refresh() override;

    /// Does nothing; the chassis subsystem stops itself when the remote disconnects.
    void refreshSafeDisconnect() override {}

    /// @return The name used to identify this subsystem in logs and the scheduler.
    const char* getName() const override { return "StateMachine"; }

private:
    /// The chassis being driven.
    src::chassis::ChassisSubsystem* chassisSubsystem;
    /// The path follower supplying velocity and rotation setpoints.
    src::chassis::ChassisAutoDrive* chassisAutoDrive;
    /// Supplies the beyblade spin rate. `nullptr` if this robot drives without spinning.
    src::chassis::ChassisBeybladeCommand* beybladeCommand;
    /// Reports whether the referee system says a match is in progress.
    src::control::governor::MatchRunningGovernor* matchRunningGovernor;

    /// The global drivers object, used to read the remote override.
    tap::Drivers* drivers;
    /// Time in milliseconds of the previous refresh, used to advance the beyblade spin rate.
    uint32_t prevTime = 0;
};

}  // namespace src::stateMachine

#endif