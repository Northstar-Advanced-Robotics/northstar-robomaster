#ifndef SONG_TYPES_HPP
#define SONG_TYPES_HPP

#include <cstdint>
#include <vector>

// --- Core Data Structures ---

/**
 * @ingroup util
 * @brief Represents a single musical note with a duration.
 */
struct songNote
{
    uint8_t noteIndex;     // Index from the NOTE_FREQUENCIES table (0-63)
    uint32_t duration_ms;  // Duration to play the note in milliseconds
};

/**
 * @brief A Song is defined as a sequence (vector) of Notes.
 */
using Song = std::vector<songNote>;

// --- Note Index Constants ---
// Based on the NOTE_FREQUENCIES array where A4 = index 34.

// --- Note Index Constants (C2 to D7) ---

constexpr uint8_t REST = 0;

// -- Octave 2 --
constexpr uint8_t NOTE_C2 = 1;
constexpr uint8_t NOTE_CS2 = 2;  // C Sharp
constexpr uint8_t NOTE_D2 = 3;
constexpr uint8_t NOTE_DS2 = 4;  // D Sharp
constexpr uint8_t NOTE_E2 = 5;
constexpr uint8_t NOTE_F2 = 6;
constexpr uint8_t NOTE_FS2 = 7;  // F Sharp
constexpr uint8_t NOTE_G2 = 8;
constexpr uint8_t NOTE_GS2 = 9;  // G Sharp
constexpr uint8_t NOTE_A2 = 10;
constexpr uint8_t NOTE_AS2 = 11;  // A Sharp
constexpr uint8_t NOTE_B2 = 12;

// -- Octave 3 --
constexpr uint8_t NOTE_C3 = 13;
constexpr uint8_t NOTE_CS3 = 14;
constexpr uint8_t NOTE_D3 = 15;
constexpr uint8_t NOTE_DS3 = 16;
constexpr uint8_t NOTE_E3 = 17;
constexpr uint8_t NOTE_F3 = 18;
constexpr uint8_t NOTE_FS3 = 19;
constexpr uint8_t NOTE_G3 = 20;
constexpr uint8_t NOTE_GS3 = 21;
constexpr uint8_t NOTE_A3 = 22;
constexpr uint8_t NOTE_AS3 = 23;
constexpr uint8_t NOTE_B3 = 24;

// -- Octave 4 --
constexpr uint8_t NOTE_C4 = 25;
constexpr uint8_t NOTE_CS4 = 26;
constexpr uint8_t NOTE_D4 = 27;
constexpr uint8_t NOTE_DS4 = 28;
constexpr uint8_t NOTE_E4 = 29;
constexpr uint8_t NOTE_F4 = 30;
constexpr uint8_t NOTE_FS4 = 31;
constexpr uint8_t NOTE_G4 = 32;
constexpr uint8_t NOTE_GS4 = 33;
constexpr uint8_t NOTE_A4 = 34;  // 440 Hz Reference
constexpr uint8_t NOTE_AS4 = 35;
constexpr uint8_t NOTE_B4 = 36;

// -- Octave 5 --
constexpr uint8_t NOTE_C5 = 37;
constexpr uint8_t NOTE_CS5 = 38;
constexpr uint8_t NOTE_D5 = 39;
constexpr uint8_t NOTE_DS5 = 40;
constexpr uint8_t NOTE_E5 = 41;
constexpr uint8_t NOTE_F5 = 42;
constexpr uint8_t NOTE_FS5 = 43;
constexpr uint8_t NOTE_G5 = 44;
constexpr uint8_t NOTE_GS5 = 45;
constexpr uint8_t NOTE_A5 = 46;
constexpr uint8_t NOTE_AS5 = 47;
constexpr uint8_t NOTE_B5 = 48;

// -- Octave 6 --
constexpr uint8_t NOTE_C6 = 49;
constexpr uint8_t NOTE_CS6 = 50;
constexpr uint8_t NOTE_D6 = 51;
constexpr uint8_t NOTE_DS6 = 52;
constexpr uint8_t NOTE_E6 = 53;
constexpr uint8_t NOTE_F6 = 54;
constexpr uint8_t NOTE_FS6 = 55;
constexpr uint8_t NOTE_G6 = 56;
constexpr uint8_t NOTE_GS6 = 57;
constexpr uint8_t NOTE_A6 = 58;
constexpr uint8_t NOTE_AS6 = 59;
constexpr uint8_t NOTE_B6 = 60;

// -- Octave 7 --
constexpr uint8_t NOTE_C7 = 61;
constexpr uint8_t NOTE_CS7 = 62;
constexpr uint8_t NOTE_D7 = 63;

// Note value 34 is assigned to A4 = 440Hz and note value 0 means silence, making the
// range 1 = C2 -> 63 = D7
// generated in python with `440 * 2**((np.arange(64)-34)/12)`
static constexpr float NOTE_FREQUENCIES[] = {
    0.0,           65.40639133,   69.29565774,   73.41619198,   77.78174593,   82.40688923,
    87.30705786,   92.49860568,   97.998859,     103.82617439,  110.,          116.54094038,
    123.47082531,  130.81278265,  138.59131549,  146.83238396,  155.56349186,  164.81377846,
    174.61411572,  184.99721136,  195.99771799,  207.65234879,  220.,          233.08188076,
    246.94165063,  261.6255653,   277.18263098,  293.66476792,  311.12698372,  329.62755691,
    349.22823143,  369.99442271,  391.99543598,  415.30469758,  440.,          466.16376152,
    493.88330126,  523.2511306,   554.36526195,  587.32953583,  622.25396744,  659.25511383,
    698.45646287,  739.98884542,  783.99087196,  830.60939516,  880.,          932.32752304,
    987.76660251,  1046.5022612,  1108.73052391, 1174.65907167, 1244.50793489, 1318.51022765,
    1396.91292573, 1479.97769085, 1567.98174393, 1661.21879032, 1760.,         1864.65504607,
    1975.53320502, 2093.0045224,  2217.46104781, 2349.31814334};

// --- Note Duration Helpers ---
// These functions calculate note durations in milliseconds based on a given tempo.

constexpr uint32_t quarterNote(uint32_t bpm) { return 60000 / bpm; }

constexpr uint32_t eighthNote(uint32_t bpm) { return quarterNote(bpm) / 2; }

constexpr uint32_t halfNote(uint32_t bpm) { return quarterNote(bpm) * 2; }

constexpr uint32_t wholeNote(uint32_t bpm) { return quarterNote(bpm) * 4; }

constexpr uint32_t dottedQuarterNote(uint32_t bpm) { return quarterNote(bpm) + eighthNote(bpm); }

#endif  // SONG_TYPES_HPP