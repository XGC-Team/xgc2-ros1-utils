#include <cmath>
#include <deque>

#include <gtest/gtest.h>

#include "ros1_utils/namespace_utils.h"
#include "ros1_utils/param_utils.h"
#include "ros1_utils/time_utils.h"
#include "ros1_utils/topic_stats.h"

namespace {

struct TestSample {
    double stamp_sec{0.0};
    double period_sec{0.0};
    bool received{false};
};

}  // namespace

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

TEST(TimeUtilsTest, UsesNowWhenMessageStampIsZero) {
    const ros::Time non_zero_stamp(12, 345);
    EXPECT_EQ(ros1_utils::messageStampOrNow(non_zero_stamp), non_zero_stamp);

    const ros::Time before = ros::Time::now();
    const ros::Time fallback = ros1_utils::messageStampOrNow(ros::Time());
    const ros::Time after = ros::Time::now();

    EXPECT_LE(before, fallback);
    EXPECT_LE(fallback, after);
}

TEST(TimeUtilsTest, ConvertsPositiveSecondsAndFallsBackForInvalidSeconds) {
    const ros::Time stamp = ros1_utils::timeSecOrNow(12.25);
    EXPECT_DOUBLE_EQ(stamp.toSec(), 12.25);

    const ros::Time before = ros::Time::now();
    const ros::Time fallback = ros1_utils::timeSecOrNow(0.0);
    const ros::Time after = ros::Time::now();

    EXPECT_LE(before, fallback);
    EXPECT_LE(fallback, after);
}

TEST(TimeUtilsTest, UpdatesSamplePeriodFromPreviousStamp) {
    TestSample sample;
    ros1_utils::updateSamplePeriod(sample, 1.0);
    EXPECT_DOUBLE_EQ(sample.period_sec, 0.0);

    sample.received = true;
    sample.stamp_sec = 1.0;
    ros1_utils::updateSamplePeriod(sample, 1.25);
    EXPECT_DOUBLE_EQ(sample.period_sec, 0.25);
}

int main(int argc, char** argv) {
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
