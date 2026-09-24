/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SAFE_DISCONNECT_HPP_
#define SAFE_DISCONNECT_HPP_

#include "tap/control/command_scheduler.hpp"
#include "tap/drivers.hpp"

namespace src::control
{
/**
 * @ingroup util
 *
 * Defines "safely disconnected" as the remote being disconnected.
 *
 * When this reports true the scheduler ends every running command and refuses new ones, so a robot
 * that loses its operator coasts to a stop instead of continuing on its last instruction.
 */
class RemoteSafeDisconnectFunction : public tap::control::SafeDisconnectFunction
{
public:
    /**
     * @param[in] drivers The global drivers object, used to poll the remote's connection state.
     */
    RemoteSafeDisconnectFunction(tap::Drivers *drivers);

    /// @return `true` while the remote is disconnected.
    virtual bool operator()();

private:
    tap::Drivers *drivers;
};
}  // namespace src::control

#endif  // SAFE_DISCONNECT_HPP_
