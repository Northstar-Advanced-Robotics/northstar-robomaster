#ifndef CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
#define CONTROL_OPERATOR_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "robot/control_operator_interface.hpp"

namespace src::mock
{
/**
 * Mock of the ControlOperatorInterface, used by the robot Drivers classes when
 * building for the unit test environment.
 */
class ControlOperatorInterfaceMock : public control::ControlOperatorInterface
{
public:
    ControlOperatorInterfaceMock(tap::Drivers *drivers)
        : control::ControlOperatorInterface(drivers)
    {
    }

    MOCK_METHOD(float, getTurretYawInput, (), ());
    MOCK_METHOD(float, getTurretPitchInput, (), ());
};

}  // namespace src::mock

#endif  // CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
