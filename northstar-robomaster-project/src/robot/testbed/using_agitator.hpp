#ifndef USING_AGITATOR_HPP_
#define USING_AGITATOR_HPP_

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

#ifdef USING_AGITATOR

// agitator subsystem
VelocityAgitatorSubsystem agitator(
    drivers(),
    constants::AGITATOR_PID_CONFIG,
    constants::AGITATOR_CONFIG);

// agitator commands
ConstantVelocityAgitatorCommand rotateAgitator(agitator, constants::AGITATOR_ROTATE_CONFIG);

UnjamSpokeAgitatorCommand unjamAgitator(agitator, constants::AGITATOR_UNJAM_CONFIG);

MoveUnjamIntegralComprisedCommand rotateAndUnjamAgitator(
    *drivers(),
    agitator,
    rotateAgitator,
    unjamAgitator);

// agitator governors
ManualFireRateReselectionManager manualFireRateReselectionManager;

SetFireRateCommand setFireRateCommand1RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    1,
    &rotateAgitator);

SetFireRateCommand setFireRateCommand10RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    10,
    &rotateAgitator);
SetFireRateCommand setFireRateCommand20RPS(
    &dummySubsystem,
    manualFireRateReselectionManager,
    20,
    &rotateAgitator);
SetFireRateCommand setFireRateCommandFullAuto(
    &dummySubsystem,
    manualFireRateReselectionManager,
    40,
    &rotateAgitator);

Trigger leftSwitchDown1RPS =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)
        .onTrue(&setFireRateCommand1RPS);

Trigger rightSwitchUp10RPS =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)
        .onTrue(&setFireRateCommand10RPS);

Trigger rightSwitchMid20RPS =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID)
        .onTrue(&setFireRateCommand20RPS);

Trigger rightSwitchDownFullAuto =
    TriggerHelpers::switchState(drivers(), Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN)
        .onTrue(&setFireRateCommandFullAuto);

FireRateLimitGovernor fireRateLimitGovernor(manualFireRateReselectionManager);

GovernorLimitedCommand<1> rotateAndUnjamAgitatorLimited(
    {&agitator},
    rotateAndUnjamAgitator,
    {&fireRateLimitGovernor});

Trigger leftSwitchDownPressedShoot =
    TriggerHelpers::switchState(drivers(), Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN)
        .whileTrue(&rotateAndUnjamAgitatorLimited);

#endif

#endif  // USING_AGITATOR_HPP_