#include "robot_kinematics_kdl.hpp"

#include <gtest/gtest.h>

namespace {

TEST(JointLimitsTest, ValidatesPositionsWithinLimits) {
    RobotKinematicsKDL::JointLimits limits(2);
    limits.min_positions[0] = -1.0;
    limits.max_positions[0] = 1.0;
    limits.min_positions[1] = -2.0;
    limits.max_positions[1] = 2.0;

    Eigen::VectorXd positions(2);
    positions << 0.5, -1.5;

    EXPECT_TRUE(limits.validatePositions(positions));
}

TEST(JointLimitsTest, RejectsPositionsOutsideLimits) {
    RobotKinematicsKDL::JointLimits limits(2);
    limits.min_positions[0] = -1.0;
    limits.max_positions[0] = 1.0;
    limits.min_positions[1] = -2.0;
    limits.max_positions[1] = 2.0;

    Eigen::VectorXd positions(2);
    positions << 1.5, 0.0;

    EXPECT_FALSE(limits.validatePositions(positions));
}

TEST(JointLimitsTest, RejectsMismatchedDimensions) {
    RobotKinematicsKDL::JointLimits limits(2);
    Eigen::VectorXd positions(3);
    positions << 0.0, 0.0, 0.0;

    EXPECT_FALSE(limits.validatePositions(positions));
}

}  // namespace
