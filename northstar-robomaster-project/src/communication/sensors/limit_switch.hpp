#ifndef LIMIT_SWITCH_HPP
#define LIMIT_SWITCH_HPP

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"

namespace src::communication::sensors
{
class LimitSwitch : public tap::communication::sensors::limit_switch::LimitSwitchInterface
{
private:
    const tap::gpio::Digital* digital;
    const tap::gpio::Digital::InputPin pin;
    bool inverted;

public:
    LimitSwitch(
        tap::gpio::Digital* digital,
        tap::gpio::Digital::InputPin pin,
        bool inverted = false)
        : digital(digital),
          pin(pin),
          inverted(inverted)
    {
    }

    bool getLimitSwitchDepressed() const override
    {
        if (inverted)
        {
            return !digital->read(pin);
        }
        else
        {
            return digital->read(pin);
        }
    }
};
}  // namespace src::communication::sensors

#endif  // LIMIT_SWITCH_HPP