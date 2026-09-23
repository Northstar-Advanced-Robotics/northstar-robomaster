#ifndef TAPROOT_PWM_ENCODER_HPP_
#define TAPROOT_PWM_ENCODER_HPP_

// #include <modm/platform/gpio/gpio.hpp>

#include "tap/board/board.hpp"
#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"

#include "modm/platform/timer/timer_12.hpp"

namespace src::communication::sensors
{
class PwmEncoder : public tap::encoder::WrappedEncoder
{
public:
    static constexpr uint16_t ENC_RESOLUTION = 4096;

    /**
     * @param isInverted Whether the encoder counts backwards
     * @param gearRatio The gear ratio between encoder and output
     */
    PwmEncoder(bool isInverted, float gearRatio = 1.0f);

    void initialize() override;

    /**
     * Calculates the position based on the PWM duty cycle captured by the timer.
     */
    void update();

    bool isOnline() const override;
};

}  // namespace src::communication::sensors

#endif  // TAPROOT_PWM_ENCODER_HPP_