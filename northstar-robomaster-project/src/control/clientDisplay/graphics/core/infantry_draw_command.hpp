#pragma once

#include "tap/control/command.hpp"

#include "control/clientDisplay/graphics/graphics_objects/indicators/agitator_jammed_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/all_robot_health_numbers.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/chassis_orientation_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/chassis_power_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/countdown.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/cv_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/dot_crosshair.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/firemode_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/flywheel_ready_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/hit_ring.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/hopper_lid_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/imu_cal_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/imu_recalibration_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/lane_assist_lines.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/linear_velocity_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/peeking_lines.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/predicted_remaining_shots_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/remote_connected_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/reticle.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/supercap_charge_indicator.hpp"

#include "drivers.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Assembles and drives the operator HUD for the standard and other infantry robots.
 *
 * Owns every indicator on the screen as a member and registers them with its `GraphicsContainer`
 * base, so the `UISubsystem` can walk them as one tree. Each iteration it gives every indicator a
 * chance to update from the robot's current state; the subsystem then works out which of them
 * actually changed and sends only those.
 *
 * Indicators are members rather than pointers so their lifetime matches the command's, which is
 * what makes it safe to register them with the container in the constructor.
 */
class InfantryDrawCommand : public tap::control::Command, GraphicsContainer
{
public:
    /**
     * Registers every indicator with the graphics container, in the order they are drawn.
     *
     * @param[in] drivers The global drivers object, the source of most displayed state.
     * @param[in] ui The UI subsystem that draws this command's graphics, taken as a subsystem
     *      requirement.
     * @param[in] turret Supplies the turret's orientation, which several indicators are drawn
     *      relative to.
     * @param[in] agitator Supplies the agitator's jam state.
     * @param[in] chassis Supplies the chassis' orientation, speed, and power draw.
     * @param[in] flywheelGovernor Reports whether the flywheels are spun up.
     * @param[in] multiShotCvCommandMapping Reports the selected fire mode.
     * @param[in] imuCalibrateCommand Reports the IMU calibration's progress.
     * @param[in] visionComms Reports whether vision is online and tracking a target.
     * @param[in] cvOnTargetGovernor Reports whether the turret is aimed closely enough to fire.
     */
    InfantryDrawCommand(
        tap::Drivers* drivers,
        UISubsystem* ui,
        src::control::turret::TurretSubsystem* turret,
        // src::control::flywheel::TwoFlywheelSubsystem* flywheel,
        src::agitator::VelocityAgitatorSubsystem* agitator,
        src::chassis::ChassisSubsystem* chassis,
        control::governor::FlywheelOnGovernor* flywheelGovernor,
        control::agitator::MultiShotCvCommandMapping* multiShotCvCommandMapping,
        imu::ImuCalibrateCommand* imuCalibrateCommand,
        src::serial::VisionComms* visionComms,
        src::control::governor::CvOnTargetGovernor* cvOnTargetGovernor)
        : drivers(drivers),
          ui(ui),
          turret(turret),
          //   flywheel(flywheel),
          agitator(agitator),
          chassis(chassis),
          flywheelGovernor(flywheelGovernor),
          multiShotCvCommandMapping(multiShotCvCommandMapping),
          imuCalibrateCommand(imuCalibrateCommand),
          visionComms(visionComms),
          cvOnTargetGovernor(cvOnTargetGovernor)
    {
        addSubsystemRequirement(ui);

        addGraphicsObject(&lane);
        // addGraphicsObject(&supercap);
        addGraphicsObject(&orient);
        // addGraphicsObject(&peek);
        // addGraphicsObject(&reticle);
        addGraphicsObject(&ring);
        // addGraphicsObject(&remain);
        addGraphicsObject(&numbers);
        addGraphicsObject(&countdown);
        // addGraphicsObject(&velo);
        // addGraphicsObject(&recal);
        addGraphicsObject(&chassisPower);
        addGraphicsObject(&firemode);
        // addGraphicsObject(&flywheelReady);
        addGraphicsObject(&imuCalIndicator);
        addGraphicsObject(&dotCrosshair);
        addGraphicsObject(&cvIndicator);
        addGraphicsObject(&agitatorJammed);
        addGraphicsObject(&remoteConnectedIndicator);
    };

    /// Hands this command's graphics tree to the UI subsystem, which then starts drawing it.
    void initialize() override { ui->setTopLevelContainer(this); };

    /// Lets every indicator refresh itself from the robot's current state. The UI subsystem works
    /// out which graphics actually changed and sends only those.
    void execute() override
    {
        lane.update();
        // supercap.update();
        orient.update();
        // peek.update();
        // reticle.update();
        ring.update();
        // remain.update();
        numbers.update();
        countdown.update();
        // velo.update();
        // recal.update();
        // logo doesn't need updating
        chassisPower.update();
        firemode.update();
        // flywheelReady.update();
        imuCalIndicator.update();
        cvIndicator.update();
        agitatorJammed.update();
        remoteConnectedIndicator.update();
    };

    // ui subsystem won't do anything until its top level container is set, so we are ok to add
    // objects to the command in the constructor
    /// Does nothing. The UI subsystem keeps drawing the tree this command handed it, which is why
    /// the indicators are members and outlive any single scheduling of the command.
    void end(bool) override{/*ui->setTopLevelContainer(nullptr);*/};

    /// @return Always `false`; the HUD is drawn for as long as the robot is running.
    bool isFinished() const override { return false; };  // never done drawing ui

    /// @return The name used to identify this command in logs and the scheduler.
    const char* getName() const override { return "infantry ui draw command"; }

private:
    tap::Drivers* drivers;
    UISubsystem* ui;
    src::control::turret::TurretSubsystem* turret;
    // src::control::flywheel::TwoFlywheelSubsystem* flywheel;
    src::agitator::VelocityAgitatorSubsystem* agitator;
    src::chassis::ChassisSubsystem* chassis;
    control::governor::FlywheelOnGovernor* flywheelGovernor;
    control::agitator::MultiShotCvCommandMapping* multiShotCvCommandMapping;
    imu::ImuCalibrateCommand* imuCalibrateCommand;
    src::serial::VisionComms* visionComms;
    src::control::governor::CvOnTargetGovernor* cvOnTargetGovernor;

    // add top level graphics objects here and in the constructor
    LaneAssistLines lane{turret};
    // SupercapChargeIndicator supercap{chassis};
    ChassisOrientationIndicator orient{true, drivers, turret, chassis};
    PeekingLines peek{chassis, turret};
    // Reticle reticle{drivers, turret /*agitator*/};
    HitRing ring{drivers, turret};
    // PredictedRemainingShotsIndicator remain{drivers, agitator};
    AllRobotHealthNumbers numbers{drivers};
    Countdown countdown{drivers};
    // LinearVelocityIndicator velo{chassis};
    // ImuRecalibrationIndicator recal{drivers};
    ChassisPowerIndicator chassisPower{drivers, chassis};
    // LinearVelocityIndicator velo{chassis};
    FiremodeIndicator firemode{drivers, multiShotCvCommandMapping, flywheelGovernor};
    // FlywheelReadyIndicator flywheelReady{drivers, flywheelGovernor};
    ImuCalIndicator imuCalIndicator{drivers, imuCalibrateCommand};
    DotCrosshair dotCrosshair{drivers};
    CVIndicator cvIndicator{drivers, visionComms, cvOnTargetGovernor};
    AgitatorJammedIndicator agitatorJammed{drivers, agitator};
    RemoteConnectedIndicator remoteConnectedIndicator{drivers};
};
}  // namespace src::control::client_display::graphics