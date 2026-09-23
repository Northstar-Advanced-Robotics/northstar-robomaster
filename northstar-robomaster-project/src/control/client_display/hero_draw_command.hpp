#pragma once

#include "tap/control/command.hpp"

#include "control/client_display/indicators/agitator_jammed_indicator.hpp"
#include "control/client_display/indicators/all_robot_health_numbers.hpp"
#include "control/client_display/indicators/chassis_orientation_indicator.hpp"
#include "control/client_display/indicators/chassis_power_indicator.hpp"
#include "control/client_display/indicators/countdown.hpp"
#include "control/client_display/indicators/cv_indicator.hpp"
#include "control/client_display/indicators/firemode_indicator.hpp"
#include "control/client_display/indicators/hit_ring.hpp"
#include "control/client_display/indicators/imu_cal_indicator.hpp"
#include "control/client_display/indicators/lane_assist_lines.hpp"
#include "control/client_display/indicators/linear_velocity_indicator.hpp"
#include "control/client_display/indicators/remote_connected_indicator.hpp"
#include "control/client_display/indicators/reticle.hpp"

#include "drivers.hpp"

namespace src::control::client_display
{
class HeroDrawCommand : public tap::control::Command, GraphicsContainer
{
public:
    HeroDrawCommand(
        tap::Drivers* drivers,
        UISubsystem* ui,
        src::control::turret::TurretSubsystem* turret,
        // src::control::flywheel::TwoFlywheelSubsystem* flywheel,
        src::control::agitator::VelocityAgitatorSubsystem* agitator,
        src::control::chassis::ChassisSubsystem* chassis,
        control::governor::FlywheelOnGovernor* flywheelGovernor,
        control::agitator::MultiShotCvCommandMapping* multiShotCvCommandMapping,
        imu::ImuCalibrateCommand* imuCalibrateCommand,
        src::communication::serial::VisionComms* visionComms,
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
        addGraphicsObject(&reticle);
        addGraphicsObject(&ring);
        // addGraphicsObject(&remain);
        addGraphicsObject(&numbers);
        addGraphicsObject(&countdown);
        // addGraphicsObject(&velo);
        // addGraphicsObject(&recal);
        addGraphicsObject(&firemode);
        addGraphicsObject(&chassisPower);
        addGraphicsObject(&imuCalIndicator);
        // addGraphicsObject(&velo);
        addGraphicsObject(&cvIndicator);
        addGraphicsObject(&agitatorJammed);
        addGraphicsObject(&remoteConnectedIndicator);
    };

    void initialize() override
    {
        ui->setTopLevelContainer(this);
        for (int i = 0; i < reticle.NUM_THINGS; i++)
        {
            reticle.update();
        }
    };

    void execute() override
    {
        lane.update();
        // supercap.update();
        orient.update();
        ring.update();
        // remain.update();
        numbers.update();
        countdown.update();
        // velo.update();
        // recal.update();
        chassisPower.update();
        firemode.update();

        // logo doesn't need updating
        // velo.update();
        imuCalIndicator.update();
        cvIndicator.update();
        agitatorJammed.update();
        remoteConnectedIndicator.update();
    };

    // ui subsystem won't do anything until its top level container is set, so we are ok to add
    // objects to the command in the constructor
    void end(bool) override{/*ui->setTopLevelContainer(nullptr);*/};

    bool isFinished() const override { return false; };  // never done drawing ui

    const char* getName() const override { return "hero ui draw command"; }

private:
    tap::Drivers* drivers;
    UISubsystem* ui;
    src::control::turret::TurretSubsystem* turret;
    // src::control::flywheel::TwoFlywheelSubsystem* flywheel;
    src::control::agitator::VelocityAgitatorSubsystem* agitator;
    src::control::chassis::ChassisSubsystem* chassis;

    control::governor::FlywheelOnGovernor* flywheelGovernor;
    control::agitator::MultiShotCvCommandMapping* multiShotCvCommandMapping;
    imu::ImuCalibrateCommand* imuCalibrateCommand;
    src::communication::serial::VisionComms* visionComms;
    src::control::governor::CvOnTargetGovernor* cvOnTargetGovernor;

    // add top level graphics objects here and in the constructor
    LaneAssistLines lane{turret};
    // SupercapChargeIndicator supercap{chassis};
    ChassisOrientationIndicator orient{true, drivers, turret, chassis};
    HitRing ring{drivers, turret};
    Reticle reticle{drivers, turret};
    // PredictedRemainingShotsIndicator remain{drivers, agitator};
    AllRobotHealthNumbers numbers{drivers};
    Countdown countdown{drivers};
    // LinearVelocityIndicator velo{chassis};
    // ImuRecalibrationIndicator recal{drivers};
    ChassisPowerIndicator chassisPower{drivers, chassis};
    // LinearVelocityIndicator velo{chassis};
    ImuCalIndicator imuCalIndicator{drivers, imuCalibrateCommand};
    FiremodeIndicator firemode{drivers, multiShotCvCommandMapping, flywheelGovernor};
    CVIndicator cvIndicator{drivers, visionComms, cvOnTargetGovernor};
    AgitatorJammedIndicator agitatorJammed{drivers, agitator};
    RemoteConnectedIndicator remoteConnectedIndicator{drivers};
};
}  // namespace src::control::client_display