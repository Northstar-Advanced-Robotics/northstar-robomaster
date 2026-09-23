#ifndef DUMMY_SUBSYSTEM_HPP_
#define DUMMY_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace src::control
{
class DummySubsystem : public tap::control::Subsystem
{
public:
    DummySubsystem(tap::Drivers *drivers) : tap::control::Subsystem(drivers) {}
};

}  // namespace src::control

#endif  // DUMMY_SUBSYSTEM_HPP_
