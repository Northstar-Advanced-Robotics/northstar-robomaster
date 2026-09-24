#include "control/buzzer/song_types.hpp"

// Define the tempo for THIS song
/// Tempo of `twinkleTwinkle`, in beats per minute.
constexpr uint32_t TWINKLE_BPM = 120;

/// One quarter note at `TWINKLE_BPM`, in milliseconds.
uint32_t QN = quarterNote(TWINKLE_BPM);
/// One half note at `TWINKLE_BPM`, in milliseconds.
uint32_t HN = halfNote(TWINKLE_BPM);

/// "Twinkle, Twinkle, Little Star", useful as a short recognizable test tune when bringing up
/// the buzzer.
const Song twinkleTwinkle = {
    // Twin-kle, twin-kle, lit-tle star
    {NOTE_C4, QN},
    {NOTE_C4, QN},
    {NOTE_G4, QN},
    {NOTE_G4, QN},
    {NOTE_A4, QN},
    {NOTE_A4, QN},
    {NOTE_G4, HN},

    // How I won-der what you are
    {NOTE_F4, QN},
    {NOTE_F4, QN},
    {NOTE_E4, QN},
    {NOTE_E4, QN},
    {NOTE_D4, QN},
    {NOTE_D4, QN},
    {NOTE_C4, HN},
};