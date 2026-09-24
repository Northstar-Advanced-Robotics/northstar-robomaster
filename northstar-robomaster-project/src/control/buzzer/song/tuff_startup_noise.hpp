#ifndef TSN_SONG_HPP
#define TSN_SONG_HPP

#include "control/buzzer/song_types.hpp"

/// A short rising arpeggio played once at startup, as an audible sign that the board has booted.
const Song tsnSong = {
    {REST, 150},
    {NOTE_G4, 80},
    {NOTE_C5, 80},
    {NOTE_E5, 80},
    {NOTE_G5, 400},
};

/// Tempo of `theWorldRevolving`, in beats per minute.
constexpr uint32_t TWR_BPM = 190;
/// One quarter note at `TWR_BPM`, in milliseconds.
constexpr uint32_t TWR_QUARTER = quarterNote(TWR_BPM);
/// One eighth note at `TWR_BPM`, in milliseconds.
constexpr uint32_t TWR_EIGHTH = TWR_QUARTER / 2;
/// One sixteenth note at `TWR_BPM`, in milliseconds.
constexpr uint32_t TWR_SIXTEENTH = TWR_EIGHTH / 2;
/// One whole note at `TWR_BPM`, in milliseconds.
constexpr uint32_t TWR_WHOLE = TWR_QUARTER * 4;

/// A longer tune, played on command rather than at startup.
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

#endif