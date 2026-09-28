#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
/**
 * @ingroup client_display
 *
 * Shows the health of all six robots on the field, as a row of numbers.
 *
 * Laid out to line up with the robot icons the client already draws, so a number reads as belonging
 * to the icon above it. A number flashes when that robot takes damage, which is how the operator
 * notices a teammate under fire without watching the whole row.
 */
class AllRobotHealthNumbers : public GraphicsContainer
{
public:
    AllRobotHealthNumbers(tap::Drivers* drivers) : drivers(drivers)
    {
        for (int i = 0; i < 6; i++)
        {
            numbers[i].y = Y_POSITION;
            numbers[i].height = LINE_HEIGHT;
            numbers[i].thickness = 3;
            this->addGraphicsObject(numbers + i);  // pointer math

            expirationTimeouts[i].stop();  // timers might be initialized started, we need them
                                           // to be stopped until we get hit
            canTakeDamage[i] = false;
        }
    }

    /// Reads every robot's health from the referee system, flashes any that just took damage, and
    /// hides the row during the countdown when the client's own icons disappear.
    void update()
    {
        if (drivers->refSerial.getRefSerialReceivingData())
        {
            // set numbers
            RefSerialData::Rx::RobotHpData data = drivers->refSerial.getRobotData().allRobotHp;
            numbers[0].integer = data.red.sentry7;
            numbers[1].integer = data.red.standard3;
            numbers[2].integer = data.red.hero1;

            numbers[3].integer = data.blue.hero1;
            numbers[4].integer = data.blue.standard3;
            numbers[5].integer = data.blue.sentry7;

            // if in countdown, remove numbers because their robot icons disappear
            if (drivers->refSerial.getGameData().gameStage ==
                RefSerialData::Rx::GameStage::COUNTDOWN)
            {
                uint16_t x_i = INITIAL_X_OFFSET;
                uint16_t x_j = UISubsystem::SCREEN_WIDTH - INITIAL_X_OFFSET;
                for (int i = 0; i < 3; i++)
                {
                    int j = 5 - i;  // if i is 0, j is 5; 1 and 4; 2 and 3

                    bool iIsHidden = numbers[i].integer == 0;
                    bool jIsHidden = numbers[j].integer == 0;

                    numbers[i].setHidden(iIsHidden);
                    numbers[j].setHidden(jIsHidden);

                    centerXs[i] = x_i;
                    centerXs[j] = x_j;
                    if (!iIsHidden)
                    {
                        x_i += SUBSEQUENT_X_OFFSET;
                        canTakeDamage[i] = true;
                    }
                    if (!jIsHidden)
                    {
                        x_j -= SUBSEQUENT_X_OFFSET;
                        canTakeDamage[j] = true;
                    }
                }
            }

            // reset colors
            for (int i = 0; i < 3; i++)
            {
                numbers[i].color = UISubsystem::Color::PINK;
                numbers[i + 3].color = UISubsystem::Color::CYAN;
            }

            for (int i = 0; i < 6; i++)
            {
                // highlight own robot
                if (drivers->refSerial.getRobotData().robotId == ids[i])
                    numbers[i].color = UISubsystem::Color::RED_AND_BLUE;

                // highlight robots taking damage by turning the number white and doubling the
                // size for one frame if(numbers[i].integerChanged()){
                //     numbers[i].color = UISubsystem::Color::WHITE;
                //     numbers[i].y = Y_POSITION-LINE_HEIGHT;
                //     numbers[i].height = 2*LINE_HEIGHT;
                // } else {
                //     numbers[i].y = Y_POSITION;
                //     numbers[i].height = LINE_HEIGHT;
                // }

                // calulate width and center the numbers
                numbers[i].calculateNumbers();
                numbers[i].x = centerXs[i] - numbers[i].width / 2;

                // make dead robots black, invinible robots green
                if (numbers[i].integer == 0 && canTakeDamage[i])
                {  // if dead and don't know about it yet
                    canTakeDamage[i] = false;
                }

                // done being invincible
                if (expirationTimeouts[i].isExpired())
                {
                    expirationTimeouts[i].stop();
                    canTakeDamage[i] = true;
                }

                if (!canTakeDamage[i])
                {
                    if (numbers[i].integer == 0)
                    {
                        numbers[i].color = UISubsystem::Color::BLACK;
                    }
                    else
                    {
                        if (expirationTimeouts[i].isStopped())
                        {
                            // start of invicibility
                            expirationTimeouts[i].restart(INVINICIBLE_TIME);
                        }
                        else
                        {
                            // still invincible
                            numbers[i].color = UISubsystem::Color::GREEN;
                        }
                    }
                }
            }
        }
    }

private:
    tap::Drivers* drivers;

    static constexpr uint16_t Y_POSITION = 850;  // pixels, all numbers at the same y level on
                                                 // screen
    static constexpr uint16_t LINE_HEIGHT = 20;  // pixels, all numbers at the same size

    static constexpr uint16_t INITIAL_X_OFFSET =
        180;  // pixels, distance from left or right edge to center of number
    static constexpr uint16_t SUBSEQUENT_X_OFFSET =
        120;  // pixels, distance between centers of numbers

    static constexpr uint32_t INVINICIBLE_TIME =
        10000;  // then next it is invinicible for 10 seconds

    // red sentry, red standard3, red hero, blue hero, blue standard3, blue sentry
    tap::arch::MilliTimeout expirationTimeouts[6];  // for knowing how long ago a robot died and
                                                    // when it will be able to be shot again
    bool canTakeDamage[6];                          // starts at 0
    RefSerialData::RobotId ids[6] = {
        RefSerialData::RobotId::RED_SENTINEL,
        RefSerialData::RobotId::RED_SOLDIER_1,
        RefSerialData::RobotId::RED_HERO,
        RefSerialData::RobotId::BLUE_HERO,
        RefSerialData::RobotId::BLUE_SOLDIER_1,
        RefSerialData::RobotId::BLUE_SENTINEL};
    IntegerGraphic numbers[6];

    uint16_t centerXs[6];
};
}  // namespace src::control::client_display::graphics