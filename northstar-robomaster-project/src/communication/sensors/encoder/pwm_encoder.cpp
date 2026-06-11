#include "pwm_encoder.hpp"

// We need device.hpp to ensure definitions are visible
#include <modm/platform/device.hpp>

namespace tap::encoder
{
PwmEncoder::PwmEncoder(bool isInverted, float gearRatio)
    : WrappedEncoder(isInverted, ENC_RESOLUTION, gearRatio)
{
}

void PwmEncoder::initialize()
{
    // 1. Configure Pin PB14 -> Connect to Timer 12 Channel 1
    // (This remains the same)
    modm::platform::GpioB14::Ch1<modm::platform::Peripheral::Tim12>::connect();

    // 2. Enable Timer 12
    modm::platform::Timer12::enable();

    // 3. Configure Timer for PWM Input
    // Set Prescaler to 84 so we get 1MHz counting frequency (1us resolution)
    modm::platform::Timer12::setPrescaler(84);

    // --- Channel 1 Setup (Period / Rising Edge) ---
    // Use InputOwn (was Direct)
    modm::platform::Timer12::configureInputChannel(
        1,
        modm::platform::Timer12::InputCaptureMapping::InputOwn,
        modm::platform::Timer12::InputCapturePrescaler::Div1,
        modm::platform::Timer12::InputCapturePolarity::Rising,
        15  // Filter
    );

    // --- Channel 2 Setup (Duty Cycle / Falling Edge) ---
    // Use InputOther (was Indirect) so it looks at Ch1's pin
    modm::platform::Timer12::configureInputChannel(
        2,
        modm::platform::Timer12::InputCaptureMapping::InputOther,
        modm::platform::Timer12::InputCapturePrescaler::Div1,
        modm::platform::Timer12::InputCapturePolarity::Falling,
        15  // Filter
    );

    // 4. Reset counter on Rising Edge (Slave Mode)
    // We set the timer to reset itself (SlaveMode::Reset) whenever it sees
    // the trigger signal from "TimerInput1Filtered" (TI1FP1).
    modm::platform::Timer12::setMode(
        modm::platform::Timer12::Mode::UpCounter,
        modm::platform::Timer12::SlaveMode::Reset,
        modm::platform::Timer12::SlaveModeTrigger::TimerInput1Filtered);

    // 5. Start
    modm::platform::Timer12::start();
}

void PwmEncoder::update()
{
    // Channel 2 holds Pulse Width (falling edge capture)
    uint32_t pulseWidth = modm::platform::Timer12::getCompareValue(2);
    // Channel 1 holds Period (rising edge capture/reset)
    uint32_t period = modm::platform::Timer12::getCompareValue(1);

    if (period == 0) return;

    const uint32_t EXPECTED_PERIOD = 1365;
    const uint32_t PERIOD_TOLERANCE = 50;

    if (period < (EXPECTED_PERIOD - PERIOD_TOLERANCE) ||
        period > (EXPECTED_PERIOD + PERIOD_TOLERANCE))
    {
        std::cout << "fuck" << std::endl;
        return;
    }

    if (pulseWidth > period)
    {
        pulseWidth = period;
    }

    // Calculate position: (PulseWidth / Period) * 1024
    uint16_t encoderActual = (pulseWidth * ENC_RESOLUTION) / period;
    updateEncoderValue(encoderActual);
}

bool PwmEncoder::isOnline() const
{
    // Check if Channel 1 (Period) is receiving data
    return modm::platform::Timer12::getCompareValue(1) > 0;
}

}  // namespace tap::encoder