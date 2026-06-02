#include "control_utils/ugv_identification.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

using namespace control_utils::ugv_identification;

constexpr double kPi = 3.14159265358979323846;

TEST(UgvIdentificationModelTest, FirstOrderExactStepConvergesAndRejectsInvalidDt) {
    double state = 0.0;
    for (int i = 0; i < 200; ++i) {
        state = firstOrderExactStep(state, 2.0, 1.5, 0.3, 0.02);
    }
    EXPECT_NEAR(state, 3.0, 1e-5);

    EXPECT_DOUBLE_EQ(firstOrderExactStep(1.0, 3.0, 2.0, 0.2, -0.1), 1.0);
    EXPECT_DOUBLE_EQ(firstOrderExactStep(1.0, 3.0, 2.0, 0.0, 0.1), 1.0);
}

TEST(UgvIdentificationModelTest, ZeroOrderCommandInterpolationUsesPreviousSample) {
    const std::vector<CommandSample> commands = {
        {0.0, 0.1, 0.0},
        {1.0, 0.2, 0.3},
        {2.0, 0.4, -0.1},
    };

    EXPECT_DOUBLE_EQ(interpolateCommandZeroOrder(commands, -1.0).linear_velocity_cmd, 0.1);
    EXPECT_DOUBLE_EQ(interpolateCommandZeroOrder(commands, 0.5).linear_velocity_cmd, 0.1);
    EXPECT_DOUBLE_EQ(interpolateCommandZeroOrder(commands, 1.5).yaw_rate_cmd, 0.3);
    EXPECT_DOUBLE_EQ(interpolateCommandZeroOrder(commands, 5.0).linear_velocity_cmd, 0.4);
}

TEST(UgvIdentificationModelTest, UnicycleArcIntegrationMatchesCircularMotion) {
    BodyState state;
    state.linear_velocity = 1.0;
    state.yaw_rate = 1.0;

    ActuatorParameters actuator;
    actuator.k_v = 1.0;
    actuator.tau_v = 0.001;
    actuator.k_w = 1.0;
    actuator.tau_w = 0.001;

    const BodyState next = stepActuatorAndUnicycle(state, 1.0, 1.0, actuator, 0.5);
    EXPECT_NEAR(next.x, std::sin(0.5), 1e-4);
    EXPECT_NEAR(next.y, 1.0 - std::cos(0.5), 1e-4);
    EXPECT_NEAR(next.yaw, 0.5, 1e-6);
}

TEST(UgvIdentificationModelTest, PoseResidualHandlesPositionAndYawWrap) {
    PoseSample measured;
    measured.position = Eigen::Vector3d(1.0, 2.0, 0.1);
    measured.orientation = quaternionFromYaw(kPi - 0.02);

    PoseSample predicted;
    predicted.position = Eigen::Vector3d(1.1, 1.8, 0.3);
    predicted.orientation = quaternionFromYaw(-kPi + 0.03);

    const Eigen::Vector4d residual = poseResidual(measured, predicted, 2.0, 3.0);
    EXPECT_NEAR(residual.x(), 0.2, 1e-12);
    EXPECT_NEAR(residual.y(), -0.4, 1e-12);
    EXPECT_NEAR(residual.z(), 0.4, 1e-12);
    EXPECT_NEAR(residual.w(), 3.0 * 0.05, 1e-12);
}

TEST(UgvIdentificationModelTest, MarkerPoseAppliesBodyToMarkerLeverArm) {
    BodyState body;
    body.x = 1.0;
    body.y = 2.0;
    body.yaw = 0.5 * kPi;

    ExtrinsicEstimate extrinsic;
    extrinsic.xyz = Eigen::Vector3d(0.2, -0.1, 0.3);
    extrinsic.rpy.z() = 0.1;

    const PoseSample marker = markerPoseFromBodyState(body, extrinsic);
    EXPECT_NEAR(marker.position.x(), 1.1, 1e-12);
    EXPECT_NEAR(marker.position.y(), 2.2, 1e-12);
    EXPECT_NEAR(marker.position.z(), 0.3, 1e-12);
    EXPECT_NEAR(yawFromQuaternion(marker.orientation), 0.5 * kPi + 0.1, 1e-12);
}

TEST(UgvIdentificationValidationTest, RejectsEmptyNonFiniteAndNonMonotonicData) {
    std::string error;
    EXPECT_FALSE(validateCommandSamples({}, &error));
    EXPECT_FALSE(error.empty());

    std::vector<CommandSample> commands = {
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
    };
    EXPECT_FALSE(validateCommandSamples(commands, &error));

    commands[1].time_s = 1.0;
    commands[1].linear_velocity_cmd = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(validateCommandSamples(commands, &error));

    PoseSample pose0;
    pose0.time_s = 0.0;
    PoseSample pose1;
    pose1.time_s = 0.0;
    EXPECT_FALSE(validatePoseSamples({pose0, pose1}, &error));

    pose1.time_s = 0.1;
    pose1.orientation = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    EXPECT_FALSE(validatePoseSamples({pose0, pose1}, &error));
}

TEST(UgvIdentificationMetricsTest, ComputesTrajectoryErrors) {
    PoseSample measured0;
    measured0.time_s = 0.0;
    measured0.orientation = quaternionFromYaw(0.0);
    PoseSample measured1;
    measured1.time_s = 1.0;
    measured1.orientation = quaternionFromYaw(0.0);
    measured1.position.x() = 1.0;

    PoseSample predicted0 = measured0;
    PoseSample predicted1 = measured1;
    predicted1.position.x() = 1.2;
    predicted1.orientation = quaternionFromYaw(0.1);

    const IdentificationMetrics metrics =
        computeMetrics({measured0, measured1}, {predicted0, predicted1});
    EXPECT_GT(metrics.pose_position_rmse, 0.0);
    EXPECT_NEAR(metrics.final_position_error, 0.2, 1e-12);
    EXPECT_NEAR(metrics.final_yaw_error, 0.1, 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
