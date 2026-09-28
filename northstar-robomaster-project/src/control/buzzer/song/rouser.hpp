#ifndef ROUSER_SONG_HPP
#define ROUSER_SONG_HPP

#include "control/buzzer/song_types.hpp"

/// "Bow Down to Washington" (the Rouser), played on the buzzer at startup or on command.
/// Durations are given directly in milliseconds rather than derived from a tempo.
const Song rouser_song = {

    // --- MAIN THEME [A] ---
    // B B B B B G B D (q q q q e q e h)
    {NOTE_B4, 400},
    {NOTE_B4, 400},
    {NOTE_B4, 400},
    {NOTE_B4, 400},
    {NOTE_B4, 200},
    {NOTE_G4, 400},
    {NOTE_B4, 200},
    {NOTE_D5, 800},

    // A A A A A D C B A D C B (q q q q e q e e. s e e)
    {NOTE_A4, 400},
    {NOTE_A4, 400},
    {NOTE_A4, 400},
    {NOTE_A4, 400},
    {NOTE_A4, 200},
    {NOTE_D5, 400},
    {NOTE_C5, 200},
    {NOTE_B4, 300},
    {NOTE_A4, 100},
    {NOTE_D5, 200},
    {NOTE_C5, 200},

    // B B B B B G B D D (q q q q e q e q. e)
    {NOTE_B4, 400},
    {NOTE_B4, 400},
    {NOTE_B4, 400},
    {NOTE_B4, 400},
    {NOTE_B4, 200},
    {NOTE_G4, 400},
    {NOTE_B4, 200},
    {NOTE_D5, 600},
    {NOTE_D5, 200},

    // --- THE BIG FINISH ---
    // E E D D C C B E D C B D B G A D G (e e e e e e q e er e er e er e er e q e q q h)
    {NOTE_E5, 200},
    {NOTE_E5, 200},
    {NOTE_D5, 200},
    {NOTE_D5, 200},
    {NOTE_C5, 200},
    {NOTE_C5, 200},
    {NOTE_B4, 400},
    {NOTE_E5, 200},
    {REST, 200},  // E er
    {NOTE_D5, 200},
    {REST, 200},  // D er
    {NOTE_C5, 200},
    {REST, 200},  // C er
    {NOTE_B4, 200},
    {REST, 200},  // B er
    {NOTE_D5, 200},
    {NOTE_B4, 200},
    {NOTE_G4, 400},
    {NOTE_A4, 400},
    {NOTE_D5, 400},
    {NOTE_G4, 800},

    {0, 0}  // Sentinel to stop playback
};

#endif  // MEGALOVANIA_SONG_HPP