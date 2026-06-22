#pragma once

// #include "subsystems/chassis/chassisSubsystem.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
class LinearVelocityIndicator : public GraphicsContainer
{
public:
    LinearVelocityIndicator(src::chassis::ChassisSubsystem* chassis)
        : chassis(chassis),
          odometry(chassis->getChassisOdometry())
    {
        addGraphicsObject(&number);
        number.x = CENTER_X;
        number.y = POSITION_Y;
        number.height = HEIGHT;
    }

    void update()
    {
        modm::Vector<float, 2> localVelocity = odometry->getVelocityLocal();
        number.integer = localVelocity.getLength();
        number.calculateNumbers();
        number.x = CENTER_X - number.width / 2;

        if (chassis->isBeybladingOnly)
        {
            number.show();
            number.color = UISubsystem::Color::GREEN;
        }
        else
        {
            number.hide();
            number.color = UISubsystem::Color::ORANGE;
        }
    }

private:
    src::chassis::ChassisSubsystem* chassis;
    src::chassis::ChassisOdometry* odometry;

    static constexpr uint16_t CENTER_X =
        1393;  // pixels, specific to line up with fire rate and projectile allowance
    static constexpr uint16_t POSITION_Y =
        620;  // pixels, to be above fire rate and projectile allowance
    static constexpr uint16_t HEIGHT = 20;    // pixels
    static constexpr uint16_t THICKNESS = 2;  // pixels

    IntegerGraphic number;
};

}  // namespace src::control::client_display::graphics