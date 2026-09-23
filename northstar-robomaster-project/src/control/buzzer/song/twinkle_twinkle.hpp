#include "control/buzzer/song_types.hpp"

namespace src::control::buzzer
{
// Define the tempo for THIS song
constexpr uint32_t TWINKLE_BPM = 120;

uint32_t QN = quarterNote(TWINKLE_BPM);
uint32_t HN = halfNote(TWINKLE_BPM);

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

}  // namespace src::control::buzzer
