#pragma once

#include <cmath>

#include <ros/ros.h>

namespace ros1_utils {

inline ros::Time messageStampOrNow(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now() : stamp;
}

inline ros::Time timeSecOrNow(double stamp_sec) {
    return std::isfinite(stamp_sec) && stamp_sec > 0.0 ? ros::Time(stamp_sec) : ros::Time::now();
}

inline double samplePeriodSec(bool sample_received, double previous_stamp_sec, double stamp_sec) {
    return sample_received && std::isfinite(previous_stamp_sec) && std::isfinite(stamp_sec)
               ? stamp_sec - previous_stamp_sec
               : 0.0;
}

template <typename Sample> void updateSamplePeriod(Sample& sample, double stamp_sec) {
    sample.period_sec = samplePeriodSec(sample.received, sample.stamp_sec, stamp_sec);
}

}  // namespace ros1_utils
