/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SENTRY_TURRET_CONSTANTS_HPP_
#define SENTRY_TURRET_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/turret/turret_motor_config.hpp"
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly: use turret_constants.hpp instead.
#ifndef TURRET_CONSTANTS_HPP_
#error "Do not include this file directly! Use turret_controller_constants.hpp instead."
#endif

using tap::motor::DjiMotor;

namespace DjiMotorConstants
{
// Output is in mV
static constexpr uint16_t MAX_OUTPUT_GM6020 = 25000;
}  // namespace DjiMotorConstants

namespace src::control::turret
{
static constexpr uint8_t NUM_TURRETS = 1;

static constexpr float USER_YAW_INPUT_SCALAR = 0.02f;
static constexpr float USER_PITCH_INPUT_SCALAR = 0.02f;

static constexpr tap::can::CanBus CAN_BUS_YAW = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus CAN_BUS_PITCH = tap::can::CanBus::CAN_BUS2;
static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR5;  // 1

static constexpr tap::motor::MotorId YAW_MOTOR_ID_1 =
    tap::motor::MotorId::MOTOR7;  // May have to change these
static constexpr tap::motor::MotorId YAW_MOTOR_ID_2 = tap::motor::MotorId::MOTOR6;

static constexpr TurretMotorConfig YAW_MOTOR_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,  // Normal bot = 7519 + 341,  // Test bot = 450
    .minAngle = 0,
    .maxAngle = M_PI / 4,
    .limitMotorAngles = false,
};

static constexpr TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .startAngle = modm::toRadian(0),  // 7.45
    .startEncoderValue = 4750,
    .minAngle = modm::toRadian(-10),
    .maxAngle = modm::toRadian(45),
    .limitMotorAngles = true,
};

static constexpr TurretMotorConfig YAW_MOTOR_REV_CONFIG = {
    .startAngle = 0,
    .startEncoderValue = 0,  // enc res 8191
    .minAngle = 0,
    .maxAngle = 0,
    .limitMotorAngles = false,
};

static constexpr float TURRET_CG_X = 10.0f;                     // 30.17;
static constexpr float TURRET_CG_Z = 15.0f;                     // 34.02;
static constexpr float GRAVITY_COMPENSATION_SCALAR = -3300.0f;  // 12000 // 7'000;

namespace world_rel_turret_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_CONFIG = {
    // .kp = 60.0f,
    // .ki = 0.0f,
    // .kd = 5.0f,
    .kp = 12.0f,  // Test bot = kp 25.0f * 2 / 3, kd 2.4 * 2 / 3 // Main bot = kp 30, kd 3
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 22.0f,
    // .ki = 0.0f,
    // .kd = 0.3f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 0.3f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_VEL_PID_CONFIG = {
    // .kp = 2000.0f,
    // .ki = 20.0f,
    // .kd = 500.0f,
    .kp = 2300.0f,  // Test bot = kp 9000.0f * 2 / 3, kd 900.0f * 2 / 3 // Main Bot kp = 10000 kd =
                    // 1000
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 1'000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 20'000.0f,
    // .ki = 100.0f,
    // .kd = 0.0f,
    // .maxICumulative = 2'000.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.5f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_CONFIG = {
    .kp = 20.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.5f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,  // 0.0f
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 22.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .maxICumulative = 0.5f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_POS_PID_AUTO_AIM_CONFIG = {
    .kp = 45.0f,
    .ki = 0.6f,
    .kd = 1.0f,
    .maxICumulative = 0.5f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,  // 0.0f
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_VEL_PID_CONFIG = {
    .kp = 6'000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,  // 0.0f
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.5f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
    // .kp = 10'000.0f,
    // .ki = 0.0f,
    // .kd = 100.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 1.0f,
    // .tRDerivativeKalman = 0.0f,
    // .tQProportionalKalman = 1.0f,
    // .tRProportionalKalman = 0.5f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

}  // namespace world_rel_turret_imu

namespace world_rel_chassis_imu
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 140'000.0f,
    .ki = 100.0f,
    .kd = 10'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,

    // .kp = 100'000.0f,
    // .ki = 0.0f,
    // .kd = 2000.0f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.1f,
    // .tRDerivativeKalman = 70.0f,
    // .tQProportionalKalman = 0.1f,
    // .tRProportionalKalman = 0.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,

    // .kp = 3'000.1f,
    // .ki = 0.0f,
    // .kd = 500.2f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.0001f,
    // .tRDerivativeKalman = 100000.0f,
    // .tQProportionalKalman = 0.00001f,
    // .tRProportionalKalman =10000.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    // .kp = 100'183.1f,
    // .ki = 0.0f,
    // .kd = 3'448.5f,
    // .maxICumulative = 0.0f,
    // .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,
    // .tQDerivativeKalman = 0.1f,
    // .tRDerivativeKalman = 10.0f,
    // .tQProportionalKalman = 0.1f,
    // .tRProportionalKalman = 2.0f,
    // .errDeadzone = 0.0f,
    // .errorDerivativeFloor = 0.0f,

    .kp = 100'000.0f,
    .ki = 0.0f,
    .kd = 3000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,  // 0.0f
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
}  // namespace world_rel_chassis_imu

namespace chassis_rel
{
static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 8000.0f,
    .ki = 0.0f,
    .kd = 900.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig YAW_IMU_CAL_PID_CONFIG = {
    .kp = 8000.0f,
    .ki = 0.0f,
    .kd = 900.0f,
    .maxICumulative = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 70.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 60'000.0f,  // TODO for imu cal command
    .ki = 0.0f,
    .kd = 5000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,  // 0.0f
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_IMU_CAL_PID_CONFIG = {
    .kp = 60'000.0f,  // TODO for imu cal command
    .ki = 0.0f,
    .kd = 5000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotorConstants::MAX_OUTPUT_GM6020,  // 0.0f
    .tQDerivativeKalman = 0.1f,
    .tRDerivativeKalman = 10.0f,
    .tQProportionalKalman = 0.1f,
    .tRProportionalKalman = 2.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

}  // namespace chassis_rel

}  // namespace src::control::turret
#endif