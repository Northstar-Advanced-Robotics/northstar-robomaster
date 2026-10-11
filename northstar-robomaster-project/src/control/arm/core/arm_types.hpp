#ifndef ARM_TYPES_HPP_
#define ARM_TYPES_HPP_

#include <array>
#include <cstddef>

namespace src::control::arm
{
template <std::size_t N>
using JointVector = std::array<float, N>;
}

#endif