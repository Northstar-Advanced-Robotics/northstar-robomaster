#ifndef TURRET_CV_CONTROL_COMMAND_TEMPLATE_HPP_
#define TURRET_CV_CONTROL_COMMAND_TEMPLATE_HPP_

#include "tap/control/command.hpp"

namespace src::control::turret
{
class TurretCVControlCommandTemplate : public tap::control::Command
{
public:
    virtual bool isAimingWithinLaunchingTolerance(uint8_t turretId) const = 0;
};
}  // namespace src::control::turret

#endif  // TURRET_USER_CONTROL_COMMAND_HPP_
