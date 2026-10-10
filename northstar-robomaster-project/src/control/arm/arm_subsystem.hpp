#ifndef ARM_SUBSYSTEM_HPP_
#define ARM_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace tap
{
class Drivers;
}

namespace src::control::arm
{
class ArmSubsystem : public tap::control::Subsystem
{
public:
    explicit ArmSubsystem(tap::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override {}

    const char* getName() const override { return "arm subsystem"; }
};
}  // namespace src::control::arm

#endif  // ARM_SUBSYSTEM_HPP_
