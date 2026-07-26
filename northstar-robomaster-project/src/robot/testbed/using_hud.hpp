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
