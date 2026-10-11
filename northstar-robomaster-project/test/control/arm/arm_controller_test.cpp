#include "control/arm/core/arm_controller.hpp"

#include <gtest/gtest.h>

#include "fake_joint_actuator.hpp"

using namespace src::control::arm;

namespace
{
constexpr float DT = 0.002f;

constexpr JointConfig CONFIG{
    .minAngle = -1.0f,
    .maxAngle = 1.0f,
    .maxVelocity = 1.0f,
    .maxTorque = 8.0f,
    .kp = 40.0f,
    .kd = 3.0f,
    .angleTolerance = 0.01f,
    .velocityTolerance = 0.1f,
    .maxTrackingError = 0.15f};

class ArmControllerTest : public ::testing::Test
{
protected:
    ArmJointSpec spec{"joint", CONFIG};
    FakeJointActuator a, b;
    ArmController<2> controller{{ArmJoint{&spec, &a}, ArmJoint{&spec, &b}}};
    ArmControllerInterface& arm = controller;

    void tick(int n = 1)
    {
        for (int i = 0; i < n; i++)
        {
            arm.update(DT);
        }
    }
};
}  // namespace

TEST_F(ArmControllerTest, initialize_initializes_every_actuator)
{
    arm.initialize();
    EXPECT_EQ(1, a.initializeCalls);
    EXPECT_EQ(1, b.initializeCalls);
}

TEST_F(ArmControllerTest, starts_disabled_and_commands_no_torque)
{
    a.pos = 0.5f;
    tick();
    EXPECT_EQ(ArmMode::DISABLED, arm.mode());
    EXPECT_TRUE(a.disabled);
    EXPECT_TRUE(b.disabled);
}

TEST_F(ArmControllerTest, state_getters_report_feedback_and_guard_the_index)
{
    a.pos = 0.3f;
    b.vel = -0.2f;
    tick();
    EXPECT_EQ(2u, arm.jointCount());
    EXPECT_FLOAT_EQ(0.3f, arm.jointPosition(0));
    EXPECT_FLOAT_EQ(-0.2f, arm.jointVelocity(1));
    EXPECT_FLOAT_EQ(0.0f, arm.jointPosition(2));
    EXPECT_FLOAT_EQ(0.0f, arm.jointVelocity(2));
    EXPECT_FLOAT_EQ(0.0f, arm.jointTarget(2));
    EXPECT_FLOAT_EQ(0.0f, arm.commandedTorque(2));
}

TEST_F(ArmControllerTest, target_is_clamped_to_soft_limits)
{
    const float pose[] = {5.0f, -5.0f};
    arm.setJointTarget(pose, 2);
    EXPECT_FLOAT_EQ(CONFIG.maxAngle, arm.jointTarget(0));
    EXPECT_FLOAT_EQ(CONFIG.minAngle, arm.jointTarget(1));
    EXPECT_EQ(ArmMode::RUNNING, arm.mode());
}

TEST_F(ArmControllerTest, wrong_sized_pose_is_ignored)
{
    const float pose[] = {0.5f, 0.5f, 0.5f};
    arm.setJointTarget(pose, 3);
    arm.setJointTarget(nullptr, 2);
    EXPECT_FLOAT_EQ(0.0f, arm.jointTarget(0));
    EXPECT_EQ(ArmMode::DISABLED, arm.mode());
}

TEST_F(ArmControllerTest, single_joint_target_leaves_other_joints_alone)
{
    a.pos = 0.2f;
    b.pos = -0.4f;
    tick();
    arm.setJointTarget(1, 0.6f);
    arm.setJointTarget(2, 0.9f);  // out of range: ignored
    EXPECT_FLOAT_EQ(0.2f, arm.jointTarget(0));
    EXPECT_FLOAT_EQ(0.6f, arm.jointTarget(1));
    EXPECT_EQ(ArmMode::RUNNING, arm.mode());
}

TEST_F(ArmControllerTest, typed_target_matches_pointer_target)
{
    controller.setJointTarget({0.25f, 5.0f});
    EXPECT_FLOAT_EQ(0.25f, arm.jointTarget(0));
    EXPECT_FLOAT_EQ(CONFIG.maxAngle, arm.jointTarget(1));
}

TEST_F(ArmControllerTest, setpoint_ramps_at_max_velocity)
{
    tick();
    arm.setJointTarget(0, 1.0f);
    tick();
    // One tick of ramp: the setpoint leads by maxVelocity * dt and moves at maxVelocity.
    float expected = CONFIG.kp * CONFIG.maxVelocity * DT + CONFIG.kd * CONFIG.maxVelocity;
    EXPECT_NEAR(expected, a.torque, 1e-4f);
    EXPECT_FLOAT_EQ(a.torque, arm.commandedTorque(0));
    EXPECT_FLOAT_EQ(0.0f, b.torque);
}

TEST_F(ArmControllerTest, blocked_joint_torque_stops_growing)
{
    tick();
    arm.setJointTarget(0, 1.0f);
    tick(2000);  // joint never moves
    EXPECT_NEAR(CONFIG.kp * CONFIG.maxTrackingError, a.torque, 1e-3f);
}

TEST_F(ArmControllerTest, torque_is_clamped_to_joint_and_actuator_limits)
{
    JointConfig stiff = CONFIG;
    stiff.kp = 1000.0f;
    ArmJointSpec stiffSpec{"stiff", stiff};
    ArmController<1> one{{ArmJoint{&stiffSpec, &a}}};
    one.update(DT);
    one.setJointTarget(0, 1.0f);
    for (int i = 0; i < 2000; i++) one.update(DT);
    EXPECT_FLOAT_EQ(stiff.maxTorque, a.torque);

    a.actuatorLimits.availableTorque = 2.0f;
    one.update(DT);
    EXPECT_FLOAT_EQ(2.0f, a.torque);
}

TEST_F(ArmControllerTest, at_target_needs_position_and_low_speed)
{
    a.pos = 0.5f;
    tick();
    arm.setJointTarget(0, 0.5f);
    tick();
    EXPECT_TRUE(arm.atTarget());

    a.vel = -0.5f;
    tick();
    EXPECT_FALSE(arm.atTarget());

    a.vel = 0.0f;
    arm.setJointTarget(0, 0.8f);
    tick();
    EXPECT_FALSE(arm.atTarget());
}

TEST_F(ArmControllerTest, offline_joint_disables_the_arm)
{
    tick();
    arm.setJointTarget(0, 1.0f);
    tick();
    ASSERT_NE(0.0f, a.torque);

    b.isOnline = false;
    tick();
    EXPECT_FALSE(arm.allOnline());
    EXPECT_EQ(ArmMode::DISABLED, arm.mode());
    EXPECT_TRUE(a.disabled);
    EXPECT_TRUE(b.disabled);
    EXPECT_FLOAT_EQ(0.0f, arm.commandedTorque(0));
}

TEST_F(ArmControllerTest, re_enabling_starts_from_the_current_position)
{
    tick();
    arm.disable();
    a.pos = 0.7f;  // moved by hand while limp
    tick();
    arm.holdCurrent();
    tick();
    EXPECT_NEAR(0.0f, a.torque, 1e-4f);
    EXPECT_FLOAT_EQ(0.7f, arm.jointTarget(0));
}

TEST_F(ArmControllerTest, safe_hold_keeps_the_first_pose)
{
    a.pos = 0.3f;
    tick();
    arm.enterSafeHold();
    EXPECT_EQ(ArmMode::SAFE_HOLD, arm.mode());

    a.pos = 0.25f;  // sagging
    tick();
    arm.enterSafeHold();
    tick();
    EXPECT_FLOAT_EQ(0.3f, arm.jointTarget(0));
    EXPECT_GT(a.torque, 0.0f);  // pushing back up toward the captured pose
}

TEST_F(ArmControllerTest, non_positive_dt_does_nothing)
{
    a.pos = 0.4f;
    arm.update(0.0f);
    arm.update(-1.0f);
    EXPECT_FLOAT_EQ(0.0f, arm.jointPosition(0));
}
