#include "arm_subsystem.hpp"

#include "tap/drivers.hpp"

namespace src::control::arm
{
ArmSubsystem::ArmSubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers) {}

void ArmSubsystem::initialize() {}

void ArmSubsystem::refresh() {}
}  // namespace src::control::arm
