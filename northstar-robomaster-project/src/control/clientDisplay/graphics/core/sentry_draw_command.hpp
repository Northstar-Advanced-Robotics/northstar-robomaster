#pragma once

#include "tap/control/command.hpp"

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
#include "control/clientDisplay/graphics/graphics_objects/indicators/reticle.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/supercap_charge_indicator.hpp"

#include "drivers.hpp"

namespace src::control::client_display::graphics
{
class SentryDrawCommand : public tap::control::Command, GraphicsContainer
{
public:
    SentryDrawCommand(
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
        addGraphicsObject(&velo);
        // addGraphicsObject(&recal);
        addGraphicsObject(&chassisPower);
        addGraphicsObject(&firemode);
        addGraphicsObject(&flywheelReady);
        addGraphicsObject(&imuCalIndicator);
        addGraphicsObject(&dotCrosshair);
        addGraphicsObject(&cvIndicator);
    };

    void initialize() override { ui->setTopLevelContainer(this); };

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
        flywheelReady.update();
        cvIndicator.update();
    };

    // ui subsystem won't do anything until its top level container is set, so we are ok to add
    // objects to the command in the constructor
    void end(bool) override{/*ui->setTopLevelContainer(nullptr);*/};

    bool isFinished() const override { return false; };  // never done drawing ui

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
    LinearVelocityIndicator velo{chassis};
    FiremodeIndicator firemode{drivers, multiShotCvCommandMapping};
    FlywheelReadyIndicator flywheelReady{drivers, flywheelGovernor};
    ImuCalIndicator imuCalIndicator{drivers, imuCalibrateCommand};
    DotCrosshair dotCrosshair{drivers};
    CVIndicator cvIndicator{drivers, visionComms, cvOnTargetGovernor};
};
}  // namespace src::control::client_display::graphics