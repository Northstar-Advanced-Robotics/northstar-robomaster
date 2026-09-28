#ifndef TAPROOT_PWM_ENCODER_HPP_
#define TAPROOT_PWM_ENCODER_HPP_

// #include <modm/platform/gpio/gpio.hpp>

#include "tap/board/board.hpp"
#include "tap/communication/sensors/encoder/wrapped_encoder.hpp"

#include "modm/platform/timer/timer_12.hpp"


namespace tap::encoder
{
/**
 * @ingroup communication
 *
 * An absolute position encoder that reports its angle as a PWM duty cycle, read via Timer12's
 * input capture.
 *
 * Unlike the DJI motors' built-in encoders, this one is a separate sensor: channel 1 captures the
 * period on the rising edge and channel 2 the pulse width on the falling edge, and the ratio of the
 * two is the shaft angle.
 */
class PwmEncoder : public WrappedEncoder
{
public:
    /// Counts per revolution the duty cycle is mapped onto.
    static constexpr uint16_t ENC_RESOLUTION = 4096;

    /**
     * @param[in] isInverted `true` if the encoder's counts increase opposite to the direction the
     *      output shaft is considered to turn positively.
     * @param[in] gearRatio Output revolutions per encoder revolution. Forwarded unchanged to
     *      `WrappedEncoder`.
     */
    PwmEncoder(bool isInverted, float gearRatio = 1.0f);

    /// Configures Timer12 for input capture. Must be called before `update`.
    void initialize() override;

    /**
     * Samples the captured period and pulse width and converts them into an encoder position.
     *
     * @note Silently returns without updating when the period reads zero, or when it falls outside
     *      1365 +/- 50 microseconds -- so a disconnected or noisy signal leaves the last good
     *      position latched rather than reporting an error.
     */
    void update();

    /**
     * @return `true` if Timer12 has ever captured a period.
     *
     * @warning This is not a liveness check. The capture register keeps its last value after the
     *      signal stops, so once a single edge has been seen this reports online forever.
     */
    bool isOnline() const override;
};

}  // namespace tap::encoder

#endif  // TAPROOT_PWM_ENCODER_HPP_