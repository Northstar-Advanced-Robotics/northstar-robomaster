#ifndef THREE_FLYWHEEL_SUBSYSTEM_HPP_
#define THREE_FLYWHEEL_SUBSYSTEM_HPP_

#include <modm/container/pair.hpp>

#include "tap/control/subsystem.hpp"

#include "control/flywheel/flywheel_constants.hpp"

#include "flywheel_interface.hpp"

namespace src::control::flywheel
{
/**
 * @ingroup flywheel
 *
 * Base class for a three-wheel flywheel, used where the projectile needs spin imparted as well as
 * speed.
 *
 * Running the wheels at deliberately unequal speeds puts backspin or topspin on the projectile,
 * which flattens or steepens its trajectory. The requested spin selects which of several measured
 * launch-speed-to-RPM tables is used, since each spin setting has its own relationship between
 * commanded RPM and the speed the projectile actually leaves at. Subclasses supply the motors and
 * the control loop that drives them.
 */
class ThreeFlywheelSubsystem : public tap::control::Subsystem, public FlywheelInterface
{
public:
    /**
     * @param[in] drivers The global drivers object.
     * @param[in] spinToRPMMap One launch-speed-to-RPM table per spin setting, indexed by `Spin`.
     */
    explicit ThreeFlywheelSubsystem(
        tap::Drivers *drivers,
        std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT> spinToRPMMap)
        : tap::control::Subsystem::Subsystem(drivers),
          spinToRPMMap(spinToRPMMap)
    {
    }

    /**
     * Selects how much spin to impart, which changes both the wheel speed ratio and the table used
     * to convert launch speed to RPM.
     *
     * @param[in] spin The spin setting as a percentage; see this robot's `Spin` enum for the
     *      supported values.
     */
    virtual void setDesiredSpin(u_int16_t spin) = 0;

    /// @return The currently selected spin setting, as a percentage.
    virtual float getDesiredSpin() const = 0;

protected:
    /// The selected spin setting, as an index into `spinToRPMMap`.
    Spin desiredSpin = SPIN_100;
    /// The selected spin setting as a percentage, i.e. what the operator asked for.
    u_int16_t desiredSpinValue = 100;  // percent of spin

    /// Measured launch-speed-to-RPM points, one table per spin setting.
    std::array<std::array<modm::Pair<float, float>, 4>, SPIN_COUNT> spinToRPMMap;
};

}  // namespace src::control::flywheel

#endif  // THREE_FLYWHEEL_SUBSYSTEM_HPP_