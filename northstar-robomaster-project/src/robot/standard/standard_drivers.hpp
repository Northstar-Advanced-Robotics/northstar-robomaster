#ifndef STANDARD_DRIVERS_HPP_
#define STANDARD_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"

#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "communication/sensors/encoder/pwm_encoder.hpp"
#include "communication/serial/vision_comms.hpp"
#include "robot/control_operator_interface.hpp"

#endif

namespace src::standard
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
          controlOperatorInterface(this),
          visionComms(this),
          encoder(true, 1.0f)
    {
    }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
#else
public:
    control::ControlOperatorInterface controlOperatorInterface;
    serial::VisionComms visionComms;
    tap::encoder::PwmEncoder encoder;
#endif
};  // class src::StandardDrivers
}  // namespace src::standard

#endif  // STANDARD_DRIVERS_HPP_
