#ifndef IMU_CAN_CONSTANTS_HPP_
#define IMU_CAN_CONSTANTS_HPP_

#include "modm/math/geometry/angle.hpp"

namespace src::communication::can
{
/**
 * Scale of the raw angular velocity field in the IMU messages between the chassis and turret
 * boards: rad/s per count, i.e. +/-2000 deg/s mapped onto int16.
 *
 * This is part of the CAN protocol, not a property of the local IMU. The two ends can sit on
 * different boards (MPU6500 vs BMI088), so both must use this constant rather than their own
 * IMU's. It equals `Bmi088::GYRO_RAD_PER_S_PER_GYRO_COUNT`, which the protocol originally used.
 */
static constexpr float CAN_GYRO_RAD_PER_S_PER_COUNT = modm::toRadian(2000.0f) / 32767.0f;
}  // namespace src::communication::can

#endif  // IMU_CAN_CONSTANTS_HPP_
