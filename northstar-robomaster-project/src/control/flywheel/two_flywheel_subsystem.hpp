#ifndef TWO_FLYWHEEL_SUBSYSTEM_HPP_
#define TWO_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/control/subsystem.hpp"

#include "control/flywheel/flywheel_constants.hpp"

#include "flywheel_interface.hpp"

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * Base class for a two-wheel flywheel: one wheel above the projectile and one below, which launch
 * it by pinching it between them.
 *
 * Holds the launch-speed-to-RPM interpolation shared by every two-wheel implementation. The
 * relationship is not linear, since the projectile slips more the faster the wheels turn, so it is
 * interpolated over speeds measured on the real hardware rather than computed from wheel geometry.
 * Subclasses supply the motors and the control loop that drives them.
 */
class TwoFlywheelSubsystem : public tap::control::Subsystem, public FlywheelInterface
{
public:
    /**
     * @param[in] drivers The global drivers object.
     */
    explicit TwoFlywheelSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem::Subsystem(drivers),
          launchSpeedLinearInterpolator(MPS_TO_RPM, MODM_ARRAY_SIZE(MPS_TO_RPM))
    {
    }

protected:
    /// Maps launch speed in meters/second to motor RPM, interpolating between the measured points
    /// in this robot's `MPS_TO_RPM` table.
    modm::interpolation::Linear<modm::Pair<float, float>> launchSpeedLinearInterpolator;
};

}  // namespace src::control::flywheel

#endif  // THREE_FLYWHEEL_SUBSYSTEM_HPP_