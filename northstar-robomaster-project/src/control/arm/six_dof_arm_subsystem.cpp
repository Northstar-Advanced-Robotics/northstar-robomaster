#include "six_dof_arm_subsystem.hpp"

#include "tap/drivers.hpp"

namespace src::control::arm
{
SixDofArmSubsystem::SixDofArmSubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers) {}

void SixDofArmSubsystem::initialize() {}

void SixDofArmSubsystem::refresh() {}
}  // namespace src::control::arm
