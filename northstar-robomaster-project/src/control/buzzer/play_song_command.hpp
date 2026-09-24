#ifndef PLAY_SONG_COMMAND_HPP
#define PLAY_SONG_COMMAND_HPP

#include "tap/control/command.hpp"

#include "buzzer_subsystem.hpp"
#include "song_types.hpp"

namespace src::control::buzzer
{
/**
 * @ingroup util
 *
 * Plays a `Song` on the buzzer, one note at a time.
 *
 * The buzzer holds whatever note it was last given, so this command tracks how much of the current
 * note's duration is left and advances to the next note when it runs out. The command finishes
 * once the last note has played, and silences the buzzer whether it finished or was interrupted.
 */
class PlaySongCommand : public tap::control::Command
{
public:
    /**
     * @param[in] buzzer The buzzer to play on. Taken as a subsystem requirement, so only one song
     *      can be playing at a time.
     * @param[in] song The notes to play. Copied, so the caller need not keep it alive.
     */
    PlaySongCommand(BuzzerSubsystem* buzzer, const Song& song);

    /// Rewinds to the first note and starts playing it. An empty song does nothing.
    void initialize() override;

    /// Counts down the current note and starts the next one once its duration has elapsed.
    void execute() override;

    /**
     * Silences the buzzer.
     *
     * @param[in] interrupted Ignored; the buzzer is stopped either way.
     */
    void end(bool interrupted) override;

    /// @return `true` once every note has been played, or immediately if the song was empty.
    bool isFinished() const override;

    const char* getName() const override { return "PlaySongCommand"; }

private:
    /// The buzzer being played.
    BuzzerSubsystem* buzzer;
    /// The notes to play, copied at construction.
    const Song songToPlay;

    /// Index into `songToPlay` of the note currently sounding.
    size_t currentNoteIndex;
    /// Milliseconds left before the current note ends.
    uint32_t noteTimeRemaining_ms;
};
}  // namespace src::control::buzzer

#endif  // PLAY_SONG_COMMAND_HPP