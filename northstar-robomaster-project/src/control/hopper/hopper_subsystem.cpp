#include "hopper_subsystem.hpp"

#include "tap/drivers.hpp"

namespace src::control::hopper
{
HopperSubsystem::HopperSubsystem(
    tap::Drivers* drivers,
    float minPwm,
    float maxPwm,
    tap::gpio::Pwm::Pin pwmPin)
    : tap::control::Subsystem(drivers),
      pwmPin(pwmPin),
      minPwm(minPwm),
      maxPwm(maxPwm),
      pwm(minPwm)
{
}

void HopperSubsystem::initialize()
{
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, 50);  // Timer 8 for C7 Pin
}

void HopperSubsystem::open()
{
    pwm = maxPwm;
    isOpen = true;
}

void HopperSubsystem::setPWM(float dutyCycle) { drivers->pwm.write(dutyCycle, pwmPin); }

void HopperSubsystem::close()
{
    pwm = minPwm;
    isOpen = false;
}

void HopperSubsystem::refresh() { setPWM(pwm); }

}  // namespace src::control::hopper
