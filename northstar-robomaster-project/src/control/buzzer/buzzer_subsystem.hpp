#ifndef BUZZER_SUBSYSTEM_HPP
#define BUZZER_SUBSYSTEM_HPP

#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
namespace src::control::buzzer
{
/**
 * @ingroup util
 *
 * The buzzer, driven as a PWM tone generator.
 *
 * Holds whatever note it was last given until told otherwise, which is why `PlaySongCommand` has to
 * sequence notes itself. Used for audible status cues -- startup, and IMU calibration finishing.
 */
class BuzzerSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @brief Constructs the BuzzerSubsystem.
     *
     * @param drivers A pointer to the global drivers object.
     */
    explicit BuzzerSubsystem(tap::Drivers* drivers);

    /**
     * @brief Plays a single note immediately.
     * * The note will play indefinitely until stop() or another note is played.
     *
     * @param noteIndex The index of the note to play (0-63).
     */
    void playNote(uint8_t noteIndex);

    /**
     * @brief Stops any sound currently playing on the buzzer.
     */
    void stop();
};

// Calculate the number of notes in the array for bounds checking
static constexpr size_t NUM_NOTES = 64;

}  // namespace src::control::buzzer

#endif  // BUZZER_SUBSYSTEM_HPP