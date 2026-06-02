#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "control_utils/butterworth_filter.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

using control_utils::SecondOrderButterworthLowPass;

double estimateAmplitudeAtFrequency(const std::vector<double>& samples,
                                    double target_hz,
                                    double sample_frequency_hz) {
    const double dt = 1.0 / sample_frequency_hz;
    double sin_projection = 0.0;
    double cos_projection = 0.0;

    for (std::size_t n = 0; n < samples.size(); ++n) {
        const double t = static_cast<double>(n) * dt;
        const double phase = 2.0 * kPi * target_hz * t;
        sin_projection += samples[n] * std::sin(phase);
        cos_projection += samples[n] * std::cos(phase);
    }

    const double scale = 2.0 / static_cast<double>(samples.size());
    sin_projection *= scale;
    cos_projection *= scale;
    return std::sqrt(sin_projection * sin_projection +
                     cos_projection * cos_projection);
}

TEST(SecondOrderButterworthLowPassIntegrationTest,
     SyntheticSensorSignalSuppressesHighFrequencyNoise) {
    const double cutoff_hz = 5.0;
    const double sample_frequency_hz = 200.0;
    const double dt = 1.0 / sample_frequency_hz;

    SecondOrderButterworthLowPass filter(cutoff_hz, 0.0);
    const int warmup_samples = static_cast<int>(3.0 * sample_frequency_hz);
    const int measure_samples = static_cast<int>(5.0 * sample_frequency_hz);

    std::vector<double> raw_samples;
    std::vector<double> filtered_samples;
    raw_samples.reserve(measure_samples);
    filtered_samples.reserve(measure_samples);

    for (int n = 0; n < warmup_samples + measure_samples; ++n) {
        const double t = static_cast<double>(n) * dt;
        const double clean = std::sin(2.0 * kPi * 1.0 * t);
        const double noise = 0.5 * std::sin(2.0 * kPi * 30.0 * t);
        const double raw = clean + noise;
        const double filtered = filter.filter(raw, dt);

        if (n >= warmup_samples) {
            raw_samples.push_back(raw);
            filtered_samples.push_back(filtered);
        }
    }

    const double raw_low_amp =
        estimateAmplitudeAtFrequency(raw_samples, 1.0, sample_frequency_hz);
    const double raw_high_amp =
        estimateAmplitudeAtFrequency(raw_samples, 30.0, sample_frequency_hz);
    const double filtered_low_amp =
        estimateAmplitudeAtFrequency(filtered_samples, 1.0, sample_frequency_hz);
    const double filtered_high_amp =
        estimateAmplitudeAtFrequency(filtered_samples, 30.0, sample_frequency_hz);

    EXPECT_GT(filtered_low_amp, 0.90 * raw_low_amp);
    EXPECT_LT(filtered_high_amp, 0.20 * raw_high_amp);
}

TEST(SecondOrderButterworthLowPassIntegrationTest,
     ReplayRepresentativeSensorLogProducesSafeAndSmoothOutput) {
    const double cutoff_hz = 5.0;
    const double dt = 0.01;
    SecondOrderButterworthLowPass filter(cutoff_hz, 0.0);

    const std::vector<double> raw_log = {
        0.01, 0.04, 0.03, 0.09, 0.15, 0.11, 0.18, 0.21,
        0.25, 0.22, 0.29, 0.35, 0.32, 0.40, 0.44, 0.41,
        0.49, 0.55, 0.50, 0.58, 0.61, 0.57, 0.64, 0.68,
    };
    ASSERT_FALSE(raw_log.empty());

    double previous_output = filter.filter(raw_log.front(), dt);
    for (std::size_t i = 1; i < raw_log.size(); ++i) {
        const double y = filter.filter(raw_log[i], dt);
        ASSERT_TRUE(std::isfinite(y));
        EXPECT_GT(y, -1000.0);
        EXPECT_LT(y, 1000.0);

        const double derivative = std::abs((y - previous_output) / dt);
        EXPECT_LT(derivative, 10000.0);
        previous_output = y;
    }
}

TEST(SecondOrderButterworthLowPassIntegrationTest,
     HandlesModerateSampleTimeJitterWithoutNanOrExplosion) {
    const double cutoff_hz = 5.0;
    const double nominal_dt = 0.01;
    SecondOrderButterworthLowPass filter(cutoff_hz, 0.0);

    double t = 0.0;
    double max_abs_output = 0.0;
    for (int i = 0; i < 5000; ++i) {
        const double dt =
            nominal_dt * (1.0 + 0.2 * std::sin(2.0 * kPi * 0.5 * t));
        t += dt;
        const double input = std::sin(2.0 * kPi * 1.0 * t) +
                             0.2 * std::sin(2.0 * kPi * 30.0 * t);
        const double y = filter.filter(input, dt);
        ASSERT_TRUE(std::isfinite(y));
        max_abs_output = std::max(max_abs_output, std::abs(y));
    }

    EXPECT_LT(max_abs_output, 2.0);
}

TEST(SecondOrderButterworthLowPassIntegrationTest,
     ClosedLoopToyPlantRemainsStableWithFilter) {
    const double dt = 0.001;
    SecondOrderButterworthLowPass measurement_filter(20.0, 0.0);

    double plant_position = 0.0;
    double plant_velocity = 0.0;
    const double target_position = 1.0;
    const double kp = 30.0;
    const double kd = 3.0;
    double previous_filtered_position = 0.0;

    for (int i = 0; i < 10000; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double measurement_noise = 0.01 * std::sin(2.0 * kPi * 200.0 * t);
        const double measured_position = plant_position + measurement_noise;
        const double filtered_position = measurement_filter.filter(measured_position, dt);
        const double filtered_velocity =
            (filtered_position - previous_filtered_position) / dt;
        previous_filtered_position = filtered_position;

        const double error = target_position - filtered_position;
        const double control = kp * error - kd * filtered_velocity;
        const double damping = 2.0;
        const double acceleration = control - damping * plant_velocity;

        plant_velocity += acceleration * dt;
        plant_position += plant_velocity * dt;

        ASSERT_TRUE(std::isfinite(plant_position));
        ASSERT_TRUE(std::isfinite(plant_velocity));
        ASSERT_TRUE(std::isfinite(filtered_position));
        EXPECT_LT(std::abs(plant_position), 5.0);
        EXPECT_LT(std::abs(plant_velocity), 50.0);
    }

    EXPECT_NEAR(plant_position, target_position, 0.1);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
