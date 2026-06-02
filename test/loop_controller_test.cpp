#include <ros1_utils/loop_controller.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <numeric>
#include <vector>

namespace {

TEST(LoopControllerTest, WallClockRuntimeDoesNotUseSimTime) {
    ros::Time::setNow(ros::Time(42.0));

    ros1_utils::LoopControllerOptions options;
    options.frequency_hz = 200.0;
    options.runtime_clock = "wall";
    ros1_utils::LoopController loop(options);

    std::vector<ros::Time> samples;
    loop.run([&](const ros::Time& now) {
        samples.push_back(now);
        if (samples.size() >= 3) {
            loop.requestStop();
        }
    });

    ASSERT_EQ(samples.size(), 3u);
    EXPECT_FALSE(samples.front().isZero());
    EXPECT_NEAR(samples.front().toSec(), ros::WallTime::now().toSec(), 1.0);
    EXPECT_GT(std::abs(samples.front().toSec() - 42.0), 1.0);
    EXPECT_LE(samples[0], samples[1]);
    EXPECT_LE(samples[1], samples[2]);
}

TEST(LoopControllerTest, RosRuntimeUsesRosTime) {
    ros::Time::setNow(ros::Time(123.456));

    ros1_utils::LoopControllerOptions options;
    options.frequency_hz = 200.0;
    options.runtime_clock = "ros";
    ros1_utils::LoopController loop(options);

    std::vector<ros::Time> samples;
    loop.run([&](const ros::Time& now) {
        samples.push_back(now);
        if (samples.size() >= 3) {
            loop.requestStop();
        }
    });

    ASSERT_EQ(samples.size(), 3u);
    for (const auto& sample : samples) {
        EXPECT_DOUBLE_EQ(sample.toSec(), 123.456);
    }
}

TEST(LoopControllerTest, WallLoopPeriodPrecisionIsBounded) {
    ros1_utils::LoopControllerOptions options;
    options.frequency_hz = 100.0;
    options.runtime_clock = "wall";
    ros1_utils::LoopController loop(options);

    constexpr int kTicks = 40;
    std::vector<std::chrono::steady_clock::time_point> tick_times;
    tick_times.reserve(kTicks);
    loop.run([&](const ros::Time&) {
        tick_times.push_back(std::chrono::steady_clock::now());
        if (tick_times.size() >= kTicks) {
            loop.requestStop();
        }
    });

    ASSERT_EQ(tick_times.size(), static_cast<size_t>(kTicks));

    std::vector<double> periods;
    periods.reserve(kTicks - 1);
    for (size_t i = 1; i < tick_times.size(); ++i) {
        periods.push_back(std::chrono::duration<double>(tick_times[i] - tick_times[i - 1]).count());
    }

    const double mean_period =
        std::accumulate(periods.begin(), periods.end(), 0.0) / periods.size();
    double max_abs_error = 0.0;
    for (const double period : periods) {
        max_abs_error = std::max(max_abs_error, std::abs(period - 0.01));
    }

    EXPECT_NEAR(mean_period, 0.01, 0.003);
    EXPECT_LE(max_abs_error, 0.03);
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "loop_controller_test");
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
