#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace ros1_utils {

struct TopicStats {
    double frequency_hz{-1.0};
    double dt_max{0.0};
    double time_since_last_msg{0.0};
    double jitter{0.0};
    ros::Time last_message_time{0, 0};
    bool is_active{false};
    bool is_new{false};
};

struct PositionQualityStats {
    bool x_is_valid{true};
    bool y_is_valid{true};
    bool z_is_valid{true};
    bool frame_is_valid{true};

    bool position_jump_detected{false};
    double last_jump_magnitude{0.0};

    double effective_frequency_hz{-1.0};

    double x_mean_deviation{0.0};
    double y_mean_deviation{0.0};
    double z_mean_deviation{0.0};
};

struct PositionQualityConfig {
    int window_size{10};
    double duplicate_threshold{0.001};
    double jump_threshold{0.5};
};

class PositionQualityDetector {
public:
    explicit PositionQualityDetector(const PositionQualityConfig& config = PositionQualityConfig())
        : config_(config) {}

    PositionQualityStats process(double x, double y, double z) {
        PositionQualityStats stats;

        x_window_.push_back(x);
        y_window_.push_back(y);
        z_window_.push_back(z);

        if (static_cast<int>(x_window_.size()) > config_.window_size) {
            x_window_.pop_front();
            y_window_.pop_front();
            z_window_.pop_front();
        }

        if (x_window_.size() >= 2) {
            stats.x_mean_deviation = calculateMeanDeviation(x_window_, x);
            stats.y_mean_deviation = calculateMeanDeviation(y_window_, y);
            stats.z_mean_deviation = calculateMeanDeviation(z_window_, z);

            stats.x_is_valid = stats.x_mean_deviation >= config_.duplicate_threshold;
            stats.y_is_valid = stats.y_mean_deviation >= config_.duplicate_threshold;
            stats.z_is_valid = stats.z_mean_deviation >= config_.duplicate_threshold;
            stats.frame_is_valid = stats.x_is_valid && stats.y_is_valid && stats.z_is_valid;
        }

        if (has_prev_) {
            const double dx = x - prev_x_;
            const double dy = y - prev_y_;
            const double dz = z - prev_z_;
            const double jump_magnitude = std::sqrt(dx * dx + dy * dy + dz * dz);

            stats.position_jump_detected = jump_magnitude > config_.jump_threshold;
            stats.last_jump_magnitude = jump_magnitude;
        }

        prev_x_ = x;
        prev_y_ = y;
        prev_z_ = z;
        has_prev_ = true;

        return stats;
    }

    void setConfig(const PositionQualityConfig& config) { config_ = config; }
    const PositionQualityConfig& getConfig() const { return config_; }

private:
    static double calculateMeanDeviation(const std::deque<double>& window, double current) {
        if (window.empty()) {
            return 0.0;
        }

        double sum = 0.0;
        for (double value : window) {
            sum += std::abs(value - current);
        }
        return sum / window.size();
    }

    PositionQualityConfig config_;
    std::deque<double> x_window_;
    std::deque<double> y_window_;
    std::deque<double> z_window_;
    double prev_x_{0.0};
    double prev_y_{0.0};
    double prev_z_{0.0};
    bool has_prev_{false};
};

class TopicStatsManager {
private:
    struct TopicRegistration {
        std::string name;
        ros::Subscriber subscriber;
        std::deque<ros::Time> time_window;
        std::deque<double> dt_window;
        TopicStats* stats_output{nullptr};
        ros::Time last_time;

        PositionQualityStats* quality_output{nullptr};
        int valid_frame_count{0};

        static constexpr int WINDOW_SIZE = 10;
    };

public:
    explicit TopicStatsManager(ros::NodeHandle& nh) : nh_(nh) {
        topics_.reserve(20);
        ROS_INFO("[TopicStatsManager] Initialized (timers not started yet)");
    }

    void start() {
        if (!registration_enabled_) {
            ROS_WARN("[TopicStatsManager] Already started, ignoring duplicate call");
            return;
        }

        stats_timer_ =
            nh_.createTimer(ros::Duration(0.1), &TopicStatsManager::updateAllStats, this);
        heartbeat_timer_ =
            nh_.createTimer(ros::Duration(1.0), &TopicStatsManager::checkAllHeartbeats, this);

        registration_enabled_ = false;

        ROS_INFO("[TopicStatsManager] Started with %zu topics (10Hz stats update, 1Hz heartbeat)",
                 topics_.size());
    }

    void resetNewFlags() {
        for (auto& reg : topics_) {
            reg.stats_output->is_new = false;
        }
    }

    void updateAllStats(const ros::TimerEvent&) {
        for (auto& reg : topics_) {
            if (reg.time_window.size() < 2) {
                continue;
            }

            const double duration = (reg.time_window.back() - reg.time_window.front()).toSec();
            if (duration > 0.0) {
                reg.stats_output->frequency_hz =
                    static_cast<double>(reg.time_window.size() - 1) / duration;
            }

            if (!reg.dt_window.empty()) {
                double dt_max_val = 0.0;
                for (double dt : reg.dt_window) {
                    dt_max_val = std::max(dt_max_val, dt);
                }
                reg.stats_output->dt_max = dt_max_val;
            }

            reg.stats_output->jitter = calculateJitter(reg.dt_window);

            if (reg.quality_output && duration > 0.0) {
                reg.quality_output->effective_frequency_hz =
                    static_cast<double>(reg.valid_frame_count) / duration;
                reg.valid_frame_count = 0;
            }
        }
    }

    void checkAllHeartbeats(const ros::TimerEvent&) {
        const auto now = ros::Time::now();

        for (auto& reg : topics_) {
            if (reg.time_window.empty()) {
                continue;
            }

            const double time_since_last = (now - reg.time_window.back()).toSec();
            reg.stats_output->time_since_last_msg = time_since_last;

            const bool was_active = reg.stats_output->is_active;
            reg.stats_output->is_active = (time_since_last <= 2.5);

            if (was_active && !reg.stats_output->is_active) {
                ROS_WARN("[TopicStatsManager] Topic %s timeout detected (%.2fs since last message)",
                         reg.name.c_str(), time_since_last);
            } else if (!was_active && reg.stats_output->is_active) {
                ROS_INFO("[TopicStatsManager] Topic %s resumed (receiving messages again)",
                         reg.name.c_str());
            }
        }
    }

    static double calculateJitter(const std::deque<double>& dt_window) {
        if (dt_window.size() < 2) {
            return 0.0;
        }

        double mean = 0.0;
        for (double dt : dt_window) {
            mean += dt;
        }
        mean /= dt_window.size();

        double variance = 0.0;
        for (double dt : dt_window) {
            const double diff = dt - mean;
            variance += diff * diff;
        }
        variance /= dt_window.size();

        return std::sqrt(variance);
    }

    template <typename MessageType, typename ClassType>
    void register_topic(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                        void (ClassType::*callback)(const typename MessageType::ConstPtr&),
                        ClassType* obj, TopicStats* stats_output) {
        registerTopicImpl<MessageType, ClassType>(nh, topic, queue_size, callback, obj,
                                                  stats_output, nullptr);
    }

    template <typename MessageType, typename ClassType>
    void register_topic(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                        void (ClassType::*callback)(const typename MessageType::ConstPtr&),
                        ClassType* obj, TopicStats* stats_output,
                        PositionQualityStats* quality_output) {
        registerTopicImpl<MessageType, ClassType>(nh, topic, queue_size, callback, obj,
                                                  stats_output, quality_output);
    }

private:
    template <typename MessageType, typename ClassType>
    void registerTopicImpl(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                           void (ClassType::*callback)(const typename MessageType::ConstPtr&),
                           ClassType* obj, TopicStats* stats_output,
                           PositionQualityStats* quality_output) {
        if (!registration_enabled_) {
            ROS_ERROR("[TopicStatsManager] Cannot register topic '%s' - manager already started",
                      topic.c_str());
            return;
        }

        TopicRegistration reg;
        reg.name = topic;
        reg.stats_output = stats_output;
        reg.quality_output = quality_output;
        reg.last_time = ros::Time(0);

        const size_t topic_index = topics_.size();

        reg.subscriber = nh.subscribe<MessageType>(
            topic, queue_size,
            [this, callback, obj, topic_index](const typename MessageType::ConstPtr& msg) {
                TopicRegistration& reg = topics_[topic_index];

                const auto current_time = ros::Time::now();
                const double dt =
                    reg.last_time.isZero() ? 0.0 : (current_time - reg.last_time).toSec();

                reg.time_window.push_back(current_time);
                reg.dt_window.push_back(dt);

                if (reg.time_window.size() > TopicRegistration::WINDOW_SIZE) {
                    reg.time_window.pop_front();
                    reg.dt_window.pop_front();
                }

                reg.last_time = current_time;
                reg.stats_output->last_message_time = current_time;
                reg.stats_output->is_active = true;
                reg.stats_output->is_new = true;

                (obj->*callback)(msg);

                if (reg.quality_output && reg.quality_output->frame_is_valid) {
                    reg.valid_frame_count++;
                }
            });

        topics_.push_back(std::move(reg));

        if (quality_output) {
            ROS_DEBUG("[TopicStatsManager] Registered topic: %s (with quality detection)",
                      topic.c_str());
        } else {
            ROS_DEBUG("[TopicStatsManager] Registered topic: %s", topic.c_str());
        }
    }

    std::vector<TopicRegistration> topics_;
    ros::Timer stats_timer_;
    ros::Timer heartbeat_timer_;
    ros::NodeHandle& nh_;
    bool registration_enabled_{true};
};

}  // namespace ros1_utils
