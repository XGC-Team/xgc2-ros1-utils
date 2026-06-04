#include "control_utils/ugv_identification.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

using namespace control_utils::ugv_identification;

std::vector<CommandSample> makeExcitationCommands(double duration_s, double dt_s) {
    std::vector<CommandSample> commands;
    for (int i = 0; i <= static_cast<int>(duration_s / dt_s); ++i) {
        const double t = static_cast<double>(i) * dt_s;
        CommandSample sample;
        sample.time_s = t;
        if (t < 2.0) {
            sample.linear_velocity_cmd = 0.4;
            sample.yaw_rate_cmd = 0.0;
        } else if (t < 4.0) {
            sample.linear_velocity_cmd = 0.0;
            sample.yaw_rate_cmd = 0.7;
        } else if (t < 7.0) {
            sample.linear_velocity_cmd = 0.45;
            sample.yaw_rate_cmd = 0.45;
        } else if (t < 10.0) {
            sample.linear_velocity_cmd = 0.35;
            sample.yaw_rate_cmd = -0.5;
        } else {
            sample.linear_velocity_cmd = 0.2 * std::sin(2.0 * t);
            sample.yaw_rate_cmd = 0.4 * std::sin(1.3 * t);
        }
        commands.push_back(sample);
    }
    return commands;
}

std::vector<double> makeSampleTimes(double duration_s, double dt_s) {
    std::vector<double> times;
    for (int i = 0; i <= static_cast<int>(duration_s / dt_s); ++i) {
        times.push_back(static_cast<double>(i) * dt_s);
    }
    return times;
}

}  // namespace

TEST(UgvIdentificationIntegrationTest, RecoversSyntheticGazeboLikeTrajectory) {
    const std::vector<CommandSample> commands = makeExcitationCommands(12.0, 0.1);
    const std::vector<double> sample_times = makeSampleTimes(12.0, 0.1);

    ActuatorParameters truth_actuator;
    truth_actuator.k_v = 0.92;
    truth_actuator.tau_v = 0.28;
    truth_actuator.k_w = 1.08;
    truth_actuator.tau_w = 0.18;

    ExtrinsicEstimate truth_extrinsic;
    truth_extrinsic.xyz = Eigen::Vector3d(0.16, -0.07, 0.22);
    truth_extrinsic.rpy.z() = 0.0;

    BodyState initial_state;
    initial_state.x = 0.2;
    initial_state.y = -0.1;
    initial_state.yaw = 0.3;

    const double truth_delay = 0.03;
    const std::vector<PoseSample> measured = simulateMarkerTrajectory(
        commands, sample_times, truth_actuator, truth_extrinsic, initial_state, truth_delay);

    IdentificationOptions options;
    options.initial_actuator.k_v = 1.0;
    options.initial_actuator.tau_v = 0.35;
    options.initial_actuator.k_w = 1.0;
    options.initial_actuator.tau_w = 0.25;
    options.body_to_marker_prior.xyz = Eigen::Vector3d(0.05, 0.02, 0.15);
    options.body_to_marker_prior.rpy.z() = 0.0;
    options.extrinsic_translation_prior_weight = 0.01;
    options.extrinsic_yaw_prior_weight = 1.0;
    options.initial_delay_s = 0.0;
    options.delay_bound_s = 0.08;
    options.delay_prior_weight = 0.1;
    options.pose_position_weight = 10.0;
    options.pose_yaw_weight = 3.0;
    options.robust_loss_scale = 0.5;
    options.max_iterations = 120;

    const IdentificationResult result = identifyActuatorAndExtrinsic(commands, measured, options);

    ASSERT_TRUE(result.success) << result.message;
    EXPECT_NEAR(result.actuator.k_v, truth_actuator.k_v, 0.08);
    EXPECT_NEAR(result.actuator.tau_v, truth_actuator.tau_v, 0.12);
    EXPECT_NEAR(result.actuator.k_w, truth_actuator.k_w, 0.08);
    EXPECT_NEAR(result.actuator.tau_w, truth_actuator.tau_w, 0.12);
    EXPECT_NEAR(result.body_to_marker.xyz.x(), truth_extrinsic.xyz.x(), 0.05);
    EXPECT_NEAR(result.body_to_marker.xyz.y(), truth_extrinsic.xyz.y(), 0.05);
    EXPECT_NEAR(result.body_to_marker.xyz.z(), truth_extrinsic.xyz.z(), 0.03);
    EXPECT_NEAR(result.delay_s, truth_delay, 0.04);
    EXPECT_LT(result.calibration.pose_position_rmse, 0.03);
    EXPECT_LT(result.calibration.yaw_rmse, 0.03);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
