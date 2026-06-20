#ifndef USING__HERO_AGITATOR_HPP_
#define USING__HERO_AGITATOR_HPP_

#include "control/governor/fire_rate_limit_governor.hpp"
#include "control/governor/fired_recently_governor.hpp"
#include "control/governor/plate_hit_governor.hpp"
#include "robot/testbed/test_def.hpp"

using namespace tap::control::setpoint;
using namespace tap::control;
using namespace src::agitator;
using namespace src::control::agitator;
using namespace tap::communication::serial;
using namespace src::control::governor;
using namespace tap::control::governor;
using namespace src::control::flywheel;

#ifdef USING_HERO_AGITATOR

// flywheel
DJITwoFlywheelSubsystem flywheel(drivers(), LEFT_MOTOR_ID, RIGHT_MOTOR_ID, CAN_BUS, true);

TwoFlywheelRunCommand flywheelRunCommand(&flywheel, 12);

// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

src::kicker::KickerSubsystem kicker(
    drivers(),
    src::control::kicker::constants::KICKER_PID_CONFIG,
    src::control::kicker::constants::KICKER_CONFIG);

ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

src::control::kicker::ConstantVelocityKickerCommand rotateKicker(&kicker, 40.0f);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// ConcurrentCommand<2> rotateAndUnjamAgitatorWithKicker(
//     {&rotateAndUnjamAgitator, &rotateKicker},
//     "Rotate and Unjam Agitator with Kicker");

// agitator governors
FlywheelOnGovernor flywheelOnGovernor(flywheel);

RefSystemProjectileLaunchedGovernor refSystemProjectileLaunchedGovernor(
    drivers()->refSerial,
    tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);

ManualFireRateReselectionManager manualFireRateReselectionManager;

SetFireRateCommand setFireRateCommand1RPS(&dummySubsystem, manualFireRateReselectionManager, 1);
SetFireRateCommand setFireRateCommand5SPR(&dummySubsystem, manualFireRateReselectionManager, 5);

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<3> rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched(
    {&agitator},
    rotateAndUnjamAgitator,
    {&refSystemProjectileLaunchedGovernor, &fireRateLimitGovernor, &flywheelOnGovernor});

ConcurrentRaceCommand<2> rotateAndUnjamAgitatorWithKicker(
    {&rotateAndUnjamAgitatorWhenFrictionWheelsOnUntilProjectileLaunched, &rotateKicker},
    "Rotate and Unjam Agitator with Kicker");

Trigger leftSwitchDownRotateAndUnjamAgitatorWithKicker =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN)
        .onTrue(&rotateAndUnjamAgitatorWithKicker);

Trigger leftSwitchUpRunFlywheel =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)
        .toggleOnTrue(&flywheelRunCommand);

// agitator mappings
// RemoteMapState qPressed({tap::communication::serial::Remote::Key::Q});
// auto qPressed1RPS = std::make_unique<ToggleCommandMapping>(
//     drivers(),
//     std::vector<Command *>{&setFireRateCommand1RPS},
//     &qPressed);

// RemoteMapState ePressed({tap::communication::serial::Remote::Key::E});
// auto ePressed5RPS = std::make_unique<ToggleCommandMapping>(
//     drivers(),
//     std::vector<Command *>{&setFireRateCommand5SPR},
//     &ePressed);

#endif

#endif  // USING_AGITATOR_HPP_