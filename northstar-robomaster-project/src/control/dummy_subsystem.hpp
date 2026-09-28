#ifndef DUMMY_SUBSYSTEM_HPP_
#define DUMMY_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

/**
 * @ingroup util
 *
 * A subsystem that does nothing.
 *
 * Commands must declare at least one subsystem requirement to be scheduled. This stands in for a
 * real one where a command has no hardware to reserve, for example a command that only reads
 * sensors or toggles a piece of software state.
 */
class DummySubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param[in] drivers The global drivers object.
     */
    DummySubsystem(tap::Drivers *drivers) : tap::control::Subsystem(drivers) {}
};

#endif  // DUMMY_SUBSYSTEM_HPP_
