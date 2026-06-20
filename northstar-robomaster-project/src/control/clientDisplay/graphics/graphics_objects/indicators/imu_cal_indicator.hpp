#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
#include "control/imu/imu_calibrate_command.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class ImuCalIndicator : public GraphicsContainer
{
public:
    ImuCalIndicator(tap::Drivers* drivers, imu::ImuCalibrateCommand* imuCalibrateCommand)
        : drivers(drivers),
          imuCalibrateCommand(imuCalibrateCommand)
    {
        addGraphicsObject(&stage);
    }

    void update()
    {
        // if(drivers->remote.keyPressed(Remote::Key::R))
        //     drivers->recal.requestRecalibration();

        if (!imuCalibrateCommand->GetIsComandRunning())
        {
            stage.setString("Calibration: Ctrl+Z to calibrate");
        }
        else
        {
            switch (imuCalibrateCommand->getCalibrationState())
            {
                case control::imu::ImuCalibrateCommand::CalibrationState::
                    WAITING_FOR_SYSTEMS_ONLINE:
                    stage.setString("Calibration: Wating for systems");
                    break;
                case control::imu::ImuCalibrateCommand::CalibrationState::LOCKING_TURRET:
                    stage.setString("Calibration: Locking turret");
                    break;
                case control::imu::ImuCalibrateCommand::CalibrationState::CALIBRATING_IMU:
                    stage.setString("Calibration: Calibrating IMU");
                    break;
                case control::imu::ImuCalibrateCommand::CalibrationState::BUZZING:
                    stage.setString("Calibration: Buzzing");
                    break;
                case control::imu::ImuCalibrateCommand::CalibrationState::
                    WAITING_CALIBRATION_COMPLETE:
                    stage.setString("Calibration: Waiting for completion");
                    break;
                default:
                    stage.setString("Calibration: None");
                    break;
            }
        }

        stage.calculateNumbers();
        // stage.x = X_POSITION - stage.width / 2;
    }

private:
    tap::Drivers* drivers;

    imu::ImuCalibrateCommand* imuCalibrateCommand;

    static constexpr uint16_t X_POSITION = 20;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 800;   // pixels, all numbers at the same y level on
                                                  // screen
    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    StringGraphic stage{UISubsystem::Color::GREEN, "Calibration: ", X_POSITION, Y_POSITION, 20, 3};
};

}  // namespace src::control::client_display::graphics