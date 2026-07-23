#ifdef TARGET_TEST_BED

#ifndef TEST_DEF_HPP_
#define TEST_DEF_HPP_

#include <memory>

#include "tap/control/concurrent_command.hpp"
#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/control/trigger.hpp"
#include "tap/control/trigger_helpers.hpp"

#include "control/agitator/multi_shot_cv_command_mapping.hpp"

// #define USING_CHASSIS
// #define USING_TURRET
#define USING_AGITATOR
// #define USING_HERO_AGITATOR
// #define USING_FLYWHEEL
// #define USING_REV
// #define USING_HUD

#include "control/dummy_subsystem.hpp"

#include "drivers_singleton.hpp"

src::testbed::driversFunc drivers = src::testbed::DoNotUse_getDrivers;
DummySubsystem dummySubsystem(drivers());

#ifdef USING_CHASSIS

#include "control/chassis/chassis_beyblade_command.hpp"
#include "control/chassis/chassis_drive_command.hpp"
#include "control/chassis/chassis_orient_drive_command.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_wiggle_command.hpp"
#include "control/chassis/constants/chassis_constants.hpp"
#include "control/turret/constants/turret_constants.hpp"

#endif

#ifdef USING_TURRET

#include "control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "control/turret/constants/turret_constants.hpp"
#include "control/turret/user/turret_user_world_relative_command.hpp"
// turret
#include "control/turret/cv/turret_cv_control_command.hpp"
#include "control/turret/algorithms/world_frame_turret_can_imu_turret_controller.hpp"
#include "control/turret/turret_subsystem.hpp"
#include "control/turret/user/turret_quick_turn_command.hpp"
#include "control/turret/user/turret_user_control_command.hpp"
#include "robot/standard/standard_turret_subsystem.hpp"

// testbed turret
#include "control/turret/test/turret_test_command.hpp"

#endif

#if defined(USING_TURRET) && defined(USING_REV)

#include "control/turret/rev_turret_subsystem.hpp"
#include "control/turret/user/neo_turret_user_control_command.hpp"

#endif

#ifdef USING_AGITATOR

#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"

#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/manual_fire_rate_reselection_manager.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

#endif

#ifdef USING_HERO_AGITATOR
#include "tap/control/setpoint/commands/move_unjam_integral_comprised_command.hpp"

#include "control/governor/flywheel_on_governor.hpp"
#include "control/governor/ref_system_projectile_launched_governor.hpp"

// flywheel
#include "control/flywheel/dji_two_flywheel_subsystem.hpp"
#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/two_flywheel_run_command.hpp"

// agitator
#include "control/agitator/constant_velocity_agitator_command.hpp"
#include "control/agitator/constants/agitator_constants.hpp"
#include "control/agitator/set_fire_rate_command.hpp"
#include "control/agitator/unjam_spoke_agitator_command.hpp"
#include "control/agitator/velocity_agitator_subsystem.hpp"

// kicker
#include "control/kicker/constant_velocity_kicker_command.hpp"
#include "control/kicker/constants/kicker_constants.hpp"
#include "control/kicker/kicker_subsystem.hpp"
#include "control/kicker/kicker_subsystem_config.hpp"

#endif

#ifdef USING_FLYWHEEL

#include "control/flywheel/flywheel_constants.hpp"
#include "control/flywheel/flywheel_run_command.hpp"
#include "control/flywheel/flywheel_subsystem.hpp"

#endif

#ifdef USING_HUD

#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "control/clientDisplay/client_display_command.hpp"
#include "control/clientDisplay/client_display_subsystem.hpp"
#include "control/clientDisplay/indicators/ammo_indicator.hpp"
#include "control/clientDisplay/indicators/chassis_power_indicator.hpp"
#include "control/clientDisplay/indicators/circle_crosshair.hpp"
#include "control/clientDisplay/indicators/cv_aiming_indicator.hpp"
#include "control/clientDisplay/indicators/flywheel_indicator.hpp"
#include "control/clientDisplay/indicators/hud_indicator.hpp"
#include "control/clientDisplay/indicators/shooting_mode_indicator.hpp"
#include "control/clientDisplay/indicators/text_hud_indicators.hpp"
#include "control/clientDisplay/indicators/vision_indicator.hpp"

#endif

#endif

#endif