#ifndef TESTBED_FLYWHEEL_CONSTANTS_HPP_
#define TESTBED_FLYWHEEL_CONSTANTS_HPP_
#include <modm/container/pair.hpp>

#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace src::control::flywheel
{
static constexpr float FRICTION_WHEEL_RAMP_SPEED = 3.0f;

static constexpr tap::motor::MotorId LEFT_MOTOR_ID_DJI = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId RIGHT_MOTOR_ID_DJI = tap::motor::MOTOR2;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;

// dji constants
static constexpr float FLYWHEEL_PID_KP_DJI = 30.0f;
static constexpr float FLYWHEEL_PID_KI_DJI = 0.3f;
static constexpr float FLYWHEEL_PID_KD_DJI = 0.0f;
static constexpr float FLYWHEEL_PID_MAX_ERROR_SUM_DJI = 4'000.0f;
static constexpr float FLYWHEEL_PID_MAX_OUTPUT_DJI = 25'000.0f;

static constexpr float MAX_DESIRED_LAUNCH_SPEED_RPM = 8000;

// TODO make these correct
enum Spin : u_int8_t
{
    SPIN_90 = 0,
    SPIN_100 = 1,
    SPIN_110 = 2,

    SPIN_COUNT
};

static std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT>
    SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
        {{{{0.0f, 0.0f}, {15.0f, 4714.0f}, {18.0f, 5621.0f}, {24.5f, 7700.0f}}},    // SPIN_90
         {{{0.0f, 0.0f}, {15.0f, 4714.0f}, {18.0f, 5621.0f}, {24.5f, 7700.0f}}},    // SPIN_100
         {{{0.0f, 0.0f}, {15.0f, 4714.0f}, {18.0f, 5621.0f}, {24.5f, 7700.0f}}}}};  // SPIN_110

static constexpr modm::Pair<float, float> MPS_TO_RPM[] = {
    {0.0f, 0.0f},
    {15.0f, 4714.0f},
    {18.0f, 5621.0f},
    {24.5f, 7700.0f}};
// SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
//     {{{{0.0f, 0.0f},
//        {15.0f, 4'500.0f},
//        {18.0f, 5'700.0f},
//        {30.0f, 6'400.0f},
//        {32.0f, 7'000.0f}}},  // SPIN_90
//      {{{0.0f, 0.0f},
//        {15.0f, 4'500.0f},
//        {18.0f, 5'700.0f},
//        {30.0f, 6'400.0f},
//        {32.0f, 7'000.0f}}},  // SPIN_100
//      {{
//          {0.0f, 0.0f},
//          {15.0f, 4'500.0f},
//          {18.0f, 5'700.0f},
//          {30.0f, 6'400.0f},
//          {32.0f, 7'000.0f}  // SPIN_110
//      }}}};

inline std::optional<Spin> toSpinPreset(int value)
{
    switch (value)
    {
        case 90:
            return SPIN_90;
        case 100:
            return SPIN_100;
        case 110:
            return SPIN_110;
        default:
            return SPIN_100;  // invalid input
    }
}

// static std::unordered_map<u_int16_t, std::vector<modm::Pair<float, float>>>
//     SPIN_TO_INTERPOLATABLE_MPS_TO_RPM = {
//         {90, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},
//         {100, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}},
//         {110, {{0.0f, 0.0f}, {15.0f, .45f}, {18.0f, .57f}, {30.0f, .64f}, {32.0f, .7f}}}};
}  // namespace src::control::flywheel

#endif