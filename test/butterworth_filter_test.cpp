#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "control_utils/butterworth_filter.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

using control_utils::SecondOrderButterworthLowPass;

double estimateRmsRatio(double signal_hz,
                        double cutoff_hz,
                        double sample_frequency_hz,
                        double warmup_seconds = 3.0,
                        double measure_seconds = 5.0) {
    const double dt = 1.0 / sample_frequency_hz;
    SecondOrderButterworthLowPass filter(cutoff_hz, 0.0);

    const int warmup_samples = static_cast<int>(warmup_seconds * sample_frequency_hz);
    const int measure_samples = static_cast<int>(measure_seconds * sample_frequency_hz);

    double input_energy = 0.0;
    double output_energy = 0.0;
    for (int n = 0; n < warmup_samples + measure_samples; ++n) {
        const double t = static_cast<double>(n) * dt;
        const double x = std::sin(2.0 * kPi * signal_hz * t);
        const double y = filter.filter(x, dt);
        if (n >= warmup_samples) {
            input_energy += x * x;
            output_energy += y * y;
        }
    }

    return std::sqrt(output_energy / input_energy);
}

TEST(SecondOrderButterworthLowPassTest,
     DefaultConstructedFilterActsAsPassThroughWhenCutoffIsZero) {
    SecondOrderButterworthLowPass filter;
    const double dt = 0.01;

    EXPECT_NEAR(filter.filter(1.23, dt), 1.23, 1.0e-12);
    EXPECT_NEAR(filter.value(), 1.23, 1.0e-12);
    EXPECT_NEAR(filter.filter(2.0, dt), 2.0, 1.0e-12);
    EXPECT_NEAR(filter.value(), 2.0, 1.0e-12);
}

TEST(SecondOrderButterworthLowPassTest,
     ResetWithInitialValueDoesNotCreateTransientForConstantInput) {
    const double cutoff_hz = 5.0;
    const double dt = 0.01;
    const double initial_value = 3.0;

    SecondOrderButterworthLowPass filter(cutoff_hz, initial_value);
    for (int i = 0; i < 100; ++i) {
        EXPECT_NEAR(filter.filter(initial_value, dt), initial_value, 1.0e-12);
    }
    EXPECT_NEAR(filter.value(), initial_value, 1.0e-12);
}

TEST(SecondOrderButterworthLowPassTest, ConstantInputConvergesToSameValue) {
    SecondOrderButterworthLowPass filter(3.0, 0.0);
    double y = 0.0;
    for (int i = 0; i < 2000; ++i) {
        y = filter.filter(1.0, 0.01);
        ASSERT_TRUE(std::isfinite(y));
    }

    EXPECT_NEAR(y, 1.0, 1.0e-6);
    EXPECT_NEAR(filter.value(), 1.0, 1.0e-6);
}

TEST(SecondOrderButterworthLowPassTest, StepResponseIsFiniteAndConverges) {
    SecondOrderButterworthLowPass filter(5.0, 0.0);
    double y = 0.0;

    for (int i = 0; i < 100; ++i) {
        y = filter.filter(0.0, 0.01);
        ASSERT_TRUE(std::isfinite(y));
    }

    for (int i = 0; i < 1000; ++i) {
        y = filter.filter(1.0, 0.01);
        ASSERT_TRUE(std::isfinite(y));
        EXPECT_GT(y, -0.2);
        EXPECT_LT(y, 1.3);
    }

    EXPECT_NEAR(y, 1.0, 1.0e-5);
}

TEST(SecondOrderButterworthLowPassTest, CutoffFrequencyIsApproximatelyMinus3Db) {
    const double cutoff_hz = 5.0;
    const double ratio = estimateRmsRatio(cutoff_hz, cutoff_hz, 200.0);
    EXPECT_NEAR(ratio, 1.0 / std::sqrt(2.0), 0.03);
}

TEST(SecondOrderButterworthLowPassTest, LowFrequencyPassesAndHighFrequencyIsAttenuated) {
    const double cutoff_hz = 5.0;
    const double sample_frequency_hz = 200.0;

    const double low_ratio = estimateRmsRatio(1.0, cutoff_hz, sample_frequency_hz);
    const double high_ratio = estimateRmsRatio(20.0, cutoff_hz, sample_frequency_hz);

    EXPECT_GT(low_ratio, 0.95);
    EXPECT_LT(high_ratio, 0.12);
}

TEST(SecondOrderButterworthLowPassTest,
     NonFiniteInputReturnsPreviousOutputAndDoesNotPoisonState) {
    SecondOrderButterworthLowPass filter(5.0, 0.0);
    for (int i = 0; i < 100; ++i) {
        filter.filter(1.0, 0.01);
    }

    const double previous = filter.value();
    ASSERT_TRUE(std::isfinite(previous));

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    EXPECT_NEAR(filter.filter(nan, 0.01), previous, 1.0e-12);
    EXPECT_NEAR(filter.value(), previous, 1.0e-12);
    EXPECT_NEAR(filter.filter(inf, 0.01), previous, 1.0e-12);
    EXPECT_NEAR(filter.value(), previous, 1.0e-12);

    const double after = filter.filter(1.0, 0.01);
    EXPECT_TRUE(std::isfinite(after));
}

TEST(SecondOrderButterworthLowPassTest,
     InvalidDtResetsStateAndReturnsRawInputCurrentBehavior) {
    SecondOrderButterworthLowPass filter(5.0, 0.0);
    for (int i = 0; i < 100; ++i) {
        filter.filter(1.0, 0.01);
    }

    const double y = filter.filter(42.0, 0.0);
    EXPECT_NEAR(y, 42.0, 1.0e-12);
    EXPECT_NEAR(filter.value(), 42.0, 1.0e-12);
    EXPECT_NEAR(filter.filter(42.0, 0.01), 42.0, 1.0e-12);
}

TEST(SecondOrderButterworthLowPassTest, CutoffAboveLimitIsClampedCurrentBehavior) {
    const double sample_frequency_hz = 100.0;
    const double dt = 1.0 / sample_frequency_hz;
    const double nyquist_hz = 0.5 * sample_frequency_hz;
    const double clamped_cutoff_hz = 0.45 * nyquist_hz;

    SecondOrderButterworthLowPass clamped_filter(1000.0, 0.0);
    SecondOrderButterworthLowPass reference_filter(clamped_cutoff_hz, 0.0);

    for (int i = 0; i < 1000; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double x = std::sin(2.0 * kPi * 3.0 * t) +
                         0.5 * std::sin(2.0 * kPi * 20.0 * t);
        EXPECT_NEAR(clamped_filter.filter(x, dt),
                    reference_filter.filter(x, dt),
                    1.0e-12);
    }
}

TEST(SecondOrderButterworthLowPassTest, ResetStateChangesInternalStateButKeepsCutoff) {
    const double cutoff_hz = 5.0;
    const double dt = 0.01;

    SecondOrderButterworthLowPass filter(cutoff_hz, 0.0);
    for (int i = 0; i < 200; ++i) {
        filter.filter(1.0, dt);
    }

    filter.resetState(10.0);
    EXPECT_NEAR(filter.value(), 10.0, 1.0e-12);
    EXPECT_NEAR(filter.cutoffFrequencyHz(), cutoff_hz, 1.0e-12);

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(filter.filter(10.0, dt), 10.0, 1.0e-12);
    }
}

TEST(SecondOrderButterworthLowPassTest,
     ExtremeButValidParametersDoNotProduceNanOrInf) {
    const double cutoff_hz = 0.1;
    const double sample_frequency_hz = 10000.0;
    const double dt = 1.0 / sample_frequency_hz;

    SecondOrderButterworthLowPass filter(cutoff_hz, 0.0);
    for (int i = 0; i < 20000; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double x = 1.0 + 0.1 * std::sin(2.0 * kPi * 1.0 * t);
        const double y = filter.filter(x, dt);
        ASSERT_TRUE(std::isfinite(y));
        ASSERT_TRUE(std::isfinite(filter.value()));
    }
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
