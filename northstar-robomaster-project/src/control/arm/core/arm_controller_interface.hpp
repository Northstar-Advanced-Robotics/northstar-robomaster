#ifndef ARM_CONTROLLER_INTERFACE_HPP_
#define ARM_CONTROLLER_INTERFACE_HPP_

namespace src::control::arm
{
class ArmControllerInterface
{
public:
    virtual ~ArmControllerInterface() = default;
    virtual void initialize() = 0;
    virtual void update(float dt) = 0;
    virtual void holdCurrent() = 0;    // target = where we are now
    virtual void enterSafeHold() = 0;  // remote lost: hold + gravity, never limp
    virtual void disable() = 0;        // zero torque everywhere
    virtual bool atTarget() const = 0;
    virtual bool allOnline() const = 0;
};
}  // namespace src::control::arm

#endif