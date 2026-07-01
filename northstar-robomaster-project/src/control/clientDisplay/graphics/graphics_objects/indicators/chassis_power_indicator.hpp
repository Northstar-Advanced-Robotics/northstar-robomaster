#pragma once

#include "control/chassis/chassis_subsystem.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class ChassisPowerIndicator : public GraphicsContainer
{
public:
    ChassisPowerIndicator(tap::Drivers* drivers, src::chassis::ChassisSubsystem* chassis)
        : drivers(drivers),
          chassis(chassis)
    {
        addGraphicsObject(&powerDraw);
        powerDraw.x = X_POSITION;
        powerDraw.y = Y_POSITION;
        powerDraw.height = LINE_HEIGHT;
        powerDraw.thickness = TEXT_THICKNESS;
        // addGraphicsObject(&chargeBar);
    }

    void update()
    {
        float rawPower = chassis->getChassisPowerDraw();  // chassis->getWheelRpm();

        powerBuffer[bufferIndex] = rawPower;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        float chassisPower = 0.0f;
        for (uint8_t i = 0; i < BUFFER_SIZE; i++)
        {
            chassisPower += powerBuffer[i];
        }
        chassisPower /= BUFFER_SIZE;

        powerDraw.integer = static_cast<int32_t>(chassisPower);
        powerDraw.calculateNumbers();
        powerDraw.x = X_POSITION - powerDraw.width / 2;

        if (chassisPower > chassis->ChassisSubsystem::getChassisPowerLimit(drivers))
        {
            powerDraw.color = UISubsystem::Color::RED_AND_BLUE;
            energyInBuffer -=
                (rawPower - chassis->ChassisSubsystem::getChassisPowerLimit(drivers)) *
                drivers->DT / 1000.0f;
        }
        else
        {
            powerDraw.color = UISubsystem::Color::WHITE;
            energyInBuffer += RECHARGE_PER_CYCLE;
        }

        // if (energyInBuffer < 0) energyInBuffer = 0;
        // if (energyInBuffer > 60) energyInBuffer = 60;
        // float chargeBarLength = (energyInBuffer / 60.0f) * LINE_WIDTH;
        // chargeBar.x2 = X_POSITION - LINE_WIDTH / 2 + chargeBarLength;
    }

private:
    tap::Drivers* drivers;

    src::chassis::ChassisSubsystem* chassis;

    static constexpr uint16_t X_POSITION = UISubsystem::HALF_SCREEN_WIDTH;
    static constexpr uint16_t Y_POSITION = 100;
    static constexpr uint16_t LINE_HEIGHT = 100;
    static constexpr uint16_t LINE_WIDTH = 600;
    static constexpr uint16_t TEXT_HEIGHT = 60;

    static constexpr uint16_t TEXT_THICKNESS = 3;

    float RECHARGE_PER_CYCLE = 60 * drivers->DT / 1000.0f;

    float energyInBuffer = 60.0f;

    static constexpr uint8_t BUFFER_SIZE = 250;
    float powerBuffer[BUFFER_SIZE] = {0.0f};
    uint8_t bufferIndex = 0;

    IntegerGraphic powerDraw{};
    // Line chargeBar{
    //     UISubsystem::Color::RED_AND_BLUE,
    //     X_POSITION - LINE_WIDTH / 2,
    //     Y_POSITION,
    //     X_POSITION + LINE_WIDTH / 2,
    //     Y_POSITION,
    //     16};
};

}  // namespace src::control::client_display::graphics