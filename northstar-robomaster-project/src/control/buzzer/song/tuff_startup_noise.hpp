#ifndef TSN_SONG_HPP
#define TSN_SONG_HPP

#include "control/buzzer/song_types.hpp"

namespace src::control::buzzer
{
const Song tsnSong = {
    {REST, 150},
    {NOTE_G4, 80},
    {NOTE_C5, 80},
    {NOTE_E5, 80},
    {NOTE_G5, 400},
};

constexpr uint32_t TWR_BPM = 190;
constexpr uint32_t TWR_QUARTER = quarterNote(TWR_BPM);
constexpr uint32_t TWR_EIGHTH = TWR_QUARTER / 2;
constexpr uint32_t TWR_SIXTEENTH = TWR_EIGHTH / 2;
constexpr uint32_t TWR_WHOLE = TWR_QUARTER * 4;

const Song theWorldRevolving{
    {REST, TWR_QUARTER},
    {NOTE_C6, TWR_WHOLE},
    {REST, TWR_EIGHTH},
    {NOTE_C6, TWR_EIGHTH},
    {NOTE_B5, TWR_EIGHTH},
    {NOTE_C6, TWR_EIGHTH},
    {NOTE_G6, TWR_QUARTER},
    {NOTE_B5, TWR_EIGHTH},
    {NOTE_C6, TWR_WHOLE + TWR_QUARTER},
    {NOTE_C6, TWR_EIGHTH},
    {NOTE_B5, TWR_EIGHTH},
    {NOTE_A5, TWR_EIGHTH},
    {NOTE_G5, TWR_QUARTER},
    {NOTE_B5, TWR_QUARTER},
    {NOTE_C6, TWR_EIGHTH},
    {NOTE_B5, TWR_EIGHTH},
    {NOTE_C6, TWR_EIGHTH},
    {NOTE_G6, TWR_QUARTER + TWR_EIGHTH},
    {NOTE_G6, TWR_EIGHTH},
    {NOTE_A6, TWR_EIGHTH},
    {NOTE_G6, TWR_QUARTER},
    {NOTE_F6, TWR_EIGHTH},
    {NOTE_E6, TWR_QUARTER + TWR_EIGHTH},
    {NOTE_C6, TWR_QUARTER},
    {NOTE_D6, TWR_QUARTER + TWR_EIGHTH},
    {NOTE_A5, TWR_QUARTER + TWR_EIGHTH},
    {NOTE_D6, TWR_QUARTER},
    {NOTE_E6, TWR_QUARTER + TWR_EIGHTH},
    {NOTE_F6, TWR_SIXTEENTH},
    {NOTE_E6, TWR_SIXTEENTH},
    {NOTE_D6, TWR_QUARTER + TWR_QUARTER},
};

}  // namespace src::control::buzzer

#endif