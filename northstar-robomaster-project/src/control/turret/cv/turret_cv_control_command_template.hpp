#ifndef TURRET_CV_CONTROL_COMMAND_TEMPLATE_HPP_
#define TURRET_CV_CONTROL_COMMAND_TEMPLATE_HPP_

#include "tap/control/command.hpp"

namespace src::control::turret::cv
{
/**
 * @ingroup turret
 *
 * The interface a CV turret control command presents to the governors that decide whether firing is
 * allowed.
 *
 * Firing is gated on the turret actually pointing at the target, but how close counts as close
 * enough depends on how the turret is being controlled. Going through this interface lets
 * `CvOnTargetGovernor` gate the agitator without knowing which CV command is running.
 */
class TurretCVControlCommandTemplate : public tap::control::Command
{
public:
    /**
     * @param[in] turretId Index of the turret to query.
     * @return `true` if the turret is aimed closely enough at its target that a projectile
     *      launched now would hit. The tolerance narrows with distance, since a fixed angular
     *      error spans more of the plate up close than far away.
     */
    virtual bool isAimingWithinLaunchingTolerance(uint8_t turretId) const = 0;
};
}  // namespace src::control::turret::cv

#endif  // TURRET_USER_CONTROL_COMMAND_HPP_
