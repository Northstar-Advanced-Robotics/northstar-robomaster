#ifndef LIMIT_SWITCH_HPP
#define LIMIT_SWITCH_HPP

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"

namespace src::communication::sensors::limit_switch
{
/**
 * @ingroup communication
 *
 * A limit switch read from a single digital input pin.
 *
 * Wraps a `tap::gpio::Digital` pin behind taproot's `LimitSwitchInterface` so that switches wired
 * active-high and active-low can be used interchangeably by the code that consumes them.
 */
class LimitSwitch : public tap::communication::sensors::limit_switch::LimitSwitchInterface
{
private:
    const tap::gpio::Digital* digital;
    const tap::gpio::Digital::InputPin pin;
    bool inverted;

public:
    /**
     * @param[in] digital The digital IO driver used to read the pin.
     * @param[in] pin The input pin the switch is wired to.
     * @param[in] inverted `true` if the switch pulls the pin low when depressed (active low),
     *      `false` if it pulls the pin high (active high).
     */
    LimitSwitch(
        tap::gpio::Digital* digital,
        tap::gpio::Digital::InputPin pin,
        bool inverted = false)
        : digital(digital),
          pin(pin),
          inverted(inverted)
    {
    }

    /**
     * @return `true` if the switch is currently depressed, accounting for the configured
     *      inversion.
     */
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
}  // namespace src::communication::sensors::limit_switch

#endif  // LIMIT_SWITCH_HPP