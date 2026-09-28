#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
// #include "subsystems/turret/turretSubsystem.hpp"
#include "control/turret/turret_subsystem.hpp"

namespace src::control::client_display::graphics
{
// looks like
//    __
//   /
//
//
// if someone hit you in that direction
/**
 * @ingroup client_display
 *
 * Draws an arc at the center of the screen pointing in the direction a hit came from.
 *
 * The referee system reports which armor plate was struck, which is relative to the chassis, so the
 * turret's yaw is subtracted to place the arc where the operator is actually looking. Several arcs
 * at increasing radii record the last few hits, each fading out on its own timer, so a burst from
 * one direction is distinguishable from fire from several.
 */
class HitRing : public GraphicsContainer
{
public:
    HitRing(tap::Drivers* drivers, src::control::turret::TurretSubsystem* turret)
        : drivers(drivers),
          turret(turret)
    {
        // initialize rings
        for (int i = 0; i < NUM_HISTORY; i++)
        {
            rings[i].width = STARTING_SIZE + SIZE_INCREMENT * i;
            rings[i].height = rings[i].width;
            rings[i].cx = UISubsystem::HALF_SCREEN_WIDTH;
            rings[i].cy = UISubsystem::HALF_SCREEN_HEIGHT;
            expirationTimeouts[i].stop();  // timers might be initialized started, we need them
                                           // to be stopped until we get hit
            this->addGraphicsObject(rings + i);  // pointer math
            rings[i].hide();
            rings[i].thickness = THICKNESS;
        }
    }

    /// Draws an arc for any newly reported hit and clears the ones whose timers have expired.
    void update()
    {
        float encoder =
            turret->yawMotor.getChassisFrameMeasuredAngle().getWrappedValue() * 180 / PI;
        float imu = drivers->bmi088.getYaw();
        // if the turret compared to the drivetrain (from the encoder) is facing forward,
        // heading would be 360, if facing right, heading would be 90

        // update/clear old hits
        bool allSlotsUnused = true;
        for (int i = 0; i < NUM_HISTORY; i++)
        {
            if (expirationTimeouts[i].isStopped())
                continue;  // if timer not running, ignore this ring and go to the next

            // downgrade white to purple
            if (expirationTimeouts[i].timeRemaining() - EXPIRATION_TIME > RECENT_TIME)
            {
                rings[i].color = UISubsystem::Color::PURPLISH_RED;
            }

            // expire and reclaim slot
            if (expirationTimeouts[i].isExpired())
            {
                rings[i].hide();
                expirationTimeouts[i]
                    .stop();  // need to stop so we know to not check this one this next time
            }
            else
            {
                // don't get to reclaim slot, update rotation
                allSlotsUnused = false;

                updateRing(i, imu);
            }
        }

        // if no rings on screen, make it so the next time we get hit it starts in the center
        if (allSlotsUnused) nextIndex = 0;

        // check for a new hit
        if (drivers->refSerial.getRefSerialReceivingData())
        {
            RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            if (previousHp > robotData.currentHp &&
                (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE ||
                 robotData.damageType == RefSerialData::Rx::DamageType::COLLISION))
            {
                // took some sort of damage and we think we took panel damage

                // damagedArmorId==0 is forward, add 0*90 degrees
                // 1 is left, add 1*90 degrees
                // 2 is back, add 2*90 degrees
                // 3 is right, add 3*90 degrees
                // 4 is top, don't care because we don't have panels on top (yet?)
                hitOrientations[nextIndex] =
                    -encoder + imu + 90 * ((uint16_t)robotData.damagedArmorId);
                rings[nextIndex].show();
                expirationTimeouts[nextIndex].restart(RECENT_TIME + EXPIRATION_TIME);
                rings[nextIndex].color = UISubsystem::Color::WHITE;
                updateRing(nextIndex, imu);

                // get the next index
                nextIndex++;
                if (nextIndex == NUM_HISTORY)
                    nextIndex = 0;  // cycle back around and overwrite if we get hit really often
            }

            previousHp = robotData.currentHp;
        }
    }

    float getAngleToTurnForSentry()
    {
        if (expirationTimeouts[0].isStopped()) return PLACEHOLDER_ANGLE;

        expirationTimeouts[0].stop();
        float inDegrees = rings[0].startAngle + ARC_LEN / 2;
        if (inDegrees > 180) inDegrees -= 360;
        return inDegrees * PI / 180;
    }

private:
    tap::Drivers* drivers;
    src::control::turret::TurretSubsystem* turret;

    uint16_t previousHp;

    static constexpr uint16_t THICKNESS = 10;       // pixels
    static constexpr uint16_t ARC_LEN = 10;         // degrees
    static constexpr uint16_t STARTING_SIZE = 180;  // pixels
    static constexpr uint16_t SIZE_INCREMENT =
        15;  // pixels. If 0, the history of lines will all be overlapping

    static constexpr float PLACEHOLDER_ANGLE =
        NAN;  // a special value for telling jetson that you weren't hit

#if defined(SENTRY)
    static constexpr int NUM_HISTORY =
        1;  // keep track of 1 to send to jetson, when we send it skip the timer and expire it
#else
    static constexpr int NUM_HISTORY = 3;  // how many shots to keep track of
#endif
    static constexpr uint32_t RECENT_TIME = 500;       // once hit, it shows for 0.5 seconds white
    static constexpr uint32_t EXPIRATION_TIME = 5000;  // then next it shows for 5 seconds purple

    Arc rings[NUM_HISTORY];
    int nextIndex = 0;
    tap::arch::MilliTimeout expirationTimeouts[NUM_HISTORY];  // for knowing how old a hit is,
                                                              // stopped if not hit recently

    // need to know which orientation (compared to turret) we got hit.
    // This is a combination of which panel got hit and what angle the drivetrain is at
    // (compared to turret)
    float hitOrientations[NUM_HISTORY];

    // once we know what direction we hit in, we no longer care about the drivetrain spinning
    // (encoder) we just need to know if the head moved in space (imu)
    void updateRing(int i, float imu)
    {
        rings[i].startAngle =
            static_cast<uint16_t>(3 * 360 + imu - hitOrientations[i] - ARC_LEN / 2);
        UISubsystem::fixAngle(&rings[i].startAngle);
        rings[i].endAngle = rings[i].startAngle + ARC_LEN;
    }
};
}  // namespace src::control::client_display::graphics