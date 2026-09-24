#pragma once

#include "control/chassis/chassis_subsystem.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Shows how many projectiles are estimated to be left in the hopper.
 *
 * The referee system reports projectiles bought but not what is physically loaded, so this counts
 * down from the hopper's capacity as the referee system reports shots being launched.
 */
class ShotsRemainingIndicator : public GraphicsContainer
{
public:
    ShotsRemainingIndicator(tap::Drivers* drivers) : drivers(drivers), chassis(chassis)
    {
        addGraphicsObject(&shotsRemaining);
        shotsRemaining.x = X_POSITION;
        shotsRemaining.y = Y_POSITION;
        shotsRemaining.height = LINE_HEIGHT;
        shotsRemaining.integer = MAX_HOPPER_CAPACITY;
    }

    /// Decrements the count when the referee system reports a newly launched projectile, and
    /// recenters the number.
    void update()
    {
        if (drivers->refSerial.getRefSerialReceivingData())
        {
            tap::communication::serial::RefSerial::Rx::RobotData robotData =
                drivers->refSerial.getRobotData();
            shotsRemaining.calculateNumbers();
            shotsRemaining.x = X_POSITION - shotsRemaining.width / 2;
            auto currentTime = tap::arch::clock::getTimeMilliseconds();
            if (currentTime - robotData.turret.lastReceivedLaunchingInfoTimestamp >
                (drivers->DT / 1000.0f))
            {
                shotsRemaining.integer--;
            }
        }
    }

private:
    tap::Drivers* drivers;

    src::chassis::ChassisSubsystem* chassis;

    static constexpr uint16_t X_POSITION =
        600;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 300;  // pixels, all numbers at the same y level on
                                                 // screen
    static constexpr uint16_t LINE_HEIGHT = 50;  // pixels, this is a large number

#if defined(HERO_TARGET)
    uint16_t MAX_HOPPER_CAPACITY = 3;
#else
    uint16_t MAX_HOPPER_CAPACITY = 2000000000;
#endif
    IntegerGraphic shotsRemaining{};
};

}  // namespace src::control::client_display::graphics