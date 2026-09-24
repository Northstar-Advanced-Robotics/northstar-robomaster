/*
 * Copyright (c) 2020-2021 NorthStart
 *
 * This file is part of NorthStarFleet2025.
 *
 * NorthStarFleet2025 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NorthStarFleet2025 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NorthStarFleet2025.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "tap/drivers.hpp"

namespace src
{
/**
 * @ingroup util
 *
 * The project's hardware access point, extending taproot's `tap::Drivers` with anything specific to
 * this codebase.
 *
 * There is exactly one instance, owned by `DriversSingleton`; the constructor is private so no
 * other copy can be made, which is what keeps two pieces of code from independently claiming the
 * same peripheral. Unit tests are the exception, and are allowed to construct their own.
 */
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    /// Constructs the drivers. Private outside of unit tests; use `DriversSingleton` to obtain the
    /// single instance.
    Drivers() : tap::Drivers() {}

public:
};  // class Drivers

}  // namespace src

#endif  // DRIVERS_HPP_
