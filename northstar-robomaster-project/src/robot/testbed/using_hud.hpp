/**
 * Declares the client display: the subsystem and the indicators drawn on it.
 *
 * `testbed_control.cpp` includes this file unconditionally; the `#ifdef USING_HUD` below is what
 * actually selects it, and that switch is set in `test_def.hpp`.
 *
 * @warning `USING_HUD` is currently commented out, and the `ClientDisplayCommand` and indicator
 *      list inside this file are commented out as well. It builds nothing today, and the indicator
 *      classes it names have been superseded by `control/clientDisplay/graphics/`.
 *
 * Everything is declared at file scope, which is why exactly one translation unit may include it.
 */
#ifdef USING_HUD

#ifndef USING_HUD_HPP_
#define USING_HUD_HPP_


using namespace src::control::client_display;
using namespace tap::communication::serial;

ClientDisplaySubsystem clientDisplay(drivers());
tap::communication::serial::RefSerialTransmitter refSerialTransmitter(drivers());

AmmoIndicator ammoIndicator(refSerialTransmitter, drivers()->refSerial);

VisionIndicator visionIndicator(refSerialTransmitter, drivers()->refSerial, drivers()->visionComms);

CircleCrosshair circleCrosshair(refSerialTransmitter);

// FlywheelIndicator flyWheelIndicator(refSerialTransmitter, drivers()->refSerial,
// flywheelOnGovernor);

// ShootingModeIndicator shootingModeIndicator(
//     refSerialTransmitter,
//     drivers()->refSerial,
//     leftMousePressedShoot);

// CvAimingIndicator cvAimingIndicator(refSerialTransmitter, drivers()->refSerial,
// cvOnTargetGovernor);

// TextHudIndicators textHudIndicators(
//     *drivers(),
//     agitator,
//     // imuCalibrateCommand,
//     {&chassisWiggleCommand, &chassisBeyBladeCommand},
//     refSerialTransmitter);


// std::vector<HudIndicator *> hudIndicators = {
//     &ammoIndicator,
//     &circleCrosshair,
//     // &textHudIndicators,
//     // &visionIndicator,
//     // &flyWheelIndicator,
//     //&shootingModeIndicator,
//     /*&cvAimingIndicator*/};

// ClientDisplayCommand clientDisplayCommand(*drivers(), clientDisplay, hudIndicators);

#endif

#endif
