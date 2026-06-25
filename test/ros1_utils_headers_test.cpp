#include <cmath>
#include <deque>

#include <gtest/gtest.h>

#include "ros1_utils/namespace_utils.h"
#include "ros1_utils/param_utils.h"
#include "ros1_utils/topic_stats.h"

TEST(NamespaceUtilsTest, ExtractsRobotNameFromNamespace) {
    EXPECT_EQ(ros1_utils::nameFromNamespacePrefix("/swarm/uav12/controller", "/uav"), "uav12");
    EXPECT_EQ(ros1_utils::nameFromNamespacePrefix("/uav7", "/uav"), "uav7");
    EXPECT_EQ(ros1_utils::nameFromNamespacePrefix("/robot7", "/uav"), "");
    EXPECT_EQ(ros1_utils::nameFromNamespacePrefix("/uav/controller", "/uav"), "");
}

TEST(PositionQualityDetectorTest, FlagsRepeatedAndJumpedPositionSamples) {
    ros1_utils::PositionQualityConfig config;
    config.window_size = 3;
    config.duplicate_threshold = 0.001;
    config.jump_threshold = 0.5;

    ros1_utils::PositionQualityDetector detector(config);
    detector.process(1.0, 2.0, 3.0);
    const auto repeated = detector.process(1.0, 2.0, 3.0);
    EXPECT_FALSE(repeated.frame_is_valid);

    const auto jumped = detector.process(2.0, 2.0, 3.0);
    EXPECT_TRUE(jumped.position_jump_detected);
    EXPECT_NEAR(jumped.last_jump_magnitude, 1.0, 1e-9);
}

TEST(TopicStatsManagerTest, ComputesDtJitter) {
    const std::deque<double> dt_window{0.1, 0.2, 0.3};
    const double jitter = ros1_utils::TopicStatsManager::calculateJitter(dt_window);
    EXPECT_NEAR(jitter, std::sqrt(2.0 / 300.0), 1e-12);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
