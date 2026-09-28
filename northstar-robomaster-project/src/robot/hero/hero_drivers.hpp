#ifndef HERO_DRIVERS_HPP_
#define HERO_DRIVERS_HPP_

#include "tap/drivers.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/imu_terminal_serial_handler_mock.hpp"
#else
#include "tap/communication/sensors/imu/imu_terminal_serial_handler.hpp"

#include "communication/sensors/encoder/pwm_encoder.hpp"
#include "communication/serial/vision_comms.hpp"
#include "robot/control_operator_interface.hpp"

#endif

namespace src::hero
{
/**
 * @ingroup robots
 *
 * The drivers object for the hero, extending taproot's with the peripherals and interfaces this
 * robot actually has.
 *
 * There is exactly one instance, owned by `DriversSingleton`; the constructor is private so no
 * other copy can be made, which is what keeps two pieces of code from independently claiming the
 * same peripheral. Unit tests are the exception, and are allowed to construct their own.
 */
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    /// Constructs this robot's drivers. Private outside of unit tests; use `DriversSingleton` to
    /// obtain the single instance.
    Drivers()
        : tap::Drivers(),
          controlOperatorInterface(this),
          visionComms(this),
          encoder(false, 1.0f)
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
};  // class src::HeroDrivers
}  // namespace src::hero

#endif  // HERO_DRIVERS_HPP_
