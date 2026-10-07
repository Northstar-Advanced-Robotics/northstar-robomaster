#ifndef SIX_DOF_ARM_SUBSYSTEM_HPP_
#define SIX_DOF_ARM_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace tap
{
class Drivers;
}

namespace src::control::arm
{
class SixDofArmSubsystem : public tap::control::Subsystem
{
public:
    explicit SixDofArmSubsystem(tap::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override {}

    const char* getName() const override { return "six dof arm subsystem"; }
};
}  // namespace src::control::arm

#endif  // SIX_DOF_ARM_SUBSYSTEM_HPP_
