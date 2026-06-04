#include "control_utils/ugv_identification.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#ifdef CONTROL_UTILS_WITH_CERES
#include <ceres/ceres.h>
#include <ceres/dynamic_numeric_diff_cost_function.h>
#endif

namespace control_utils {
namespace ugv_identification {
namespace {

constexpr double kMinTau = 1.0e-4;
constexpr double kEps = 1.0e-9;

bool finite(double value) {
    return std::isfinite(value);
}

bool finiteVector(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

Eigen::Matrix3d rotationFromRpy(const Eigen::Vector3d& rpy) {
    const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

std::vector<double> poseTimes(const std::vector<PoseSample>& poses) {
    std::vector<double> times;
    times.reserve(poses.size());
    for (const auto& pose : poses) {
        times.push_back(pose.time_s);
    }
    return times;
}

BodyState initialStateFromFirstPose(const PoseSample& pose,
                                    const ExtrinsicEstimate& body_to_marker_prior) {
    BodyState state;
    const double marker_yaw = yawFromQuaternion(pose.orientation);
    state.yaw = normalizeAngle(marker_yaw - body_to_marker_prior.rpy.z());
    const Eigen::Matrix3d world_body_rotation =
        Eigen::AngleAxisd(state.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d body_origin =
        pose.position - world_body_rotation * body_to_marker_prior.xyz;
    state.x = body_origin.x();
    state.y = body_origin.y();
    state.linear_velocity = 0.0;
    state.yaw_rate = 0.0;
    return state;
}

IdentificationMetrics metricsOrEmpty(const std::vector<PoseSample>& measured,
                                     const std::vector<PoseSample>& predicted) {
    if (measured.empty() || measured.size() != predicted.size()) {
        return {};
    }
    return computeMetrics(measured, predicted);
}

#ifdef CONTROL_UTILS_WITH_CERES
struct TrajectoryResidualFunctor {
    const std::vector<CommandSample>& commands;
    const std::vector<PoseSample>& poses;
    const IdentificationOptions& options;

    bool operator()(double const* const* parameters, double* residuals) const {
        const double* actuator_log = parameters[0];
        const double* extrinsic = parameters[1];
        const double* delay = parameters[2];
        const double* initial = parameters[3];

        ActuatorParameters actuator;
        actuator.k_v = std::exp(actuator_log[0]);
        actuator.tau_v = std::max(kMinTau, std::exp(actuator_log[1]));
        actuator.k_w = std::exp(actuator_log[2]);
        actuator.tau_w = std::max(kMinTau, std::exp(actuator_log[3]));

        ExtrinsicEstimate body_to_marker = options.body_to_marker_prior;
        body_to_marker.xyz = Eigen::Vector3d(extrinsic[0], extrinsic[1], extrinsic[2]);
        body_to_marker.rpy.z() = extrinsic[3];

        BodyState state;
        state.x = initial[0];
        state.y = initial[1];
        state.yaw = initial[2];
        state.linear_velocity = initial[3];
        state.yaw_rate = initial[4];

        size_t offset = 0;
        for (size_t i = 0; i < poses.size(); ++i) {
            if (i > 0) {
                const double dt = poses[i].time_s - poses[i - 1].time_s;
                const CommandSample command =
                    interpolateCommandZeroOrder(commands, poses[i - 1].time_s - delay[0]);
                state = stepActuatorAndUnicycle(state, command.linear_velocity_cmd,
                                                command.yaw_rate_cmd, actuator, dt);
            }
            const PoseSample predicted = markerPoseFromBodyState(state, body_to_marker);
            const Eigen::Vector4d residual = poseResidual(
                poses[i], predicted, options.pose_position_weight, options.pose_yaw_weight);
            residuals[offset++] = residual.x();
            residuals[offset++] = residual.y();
            residuals[offset++] = residual.z();
            residuals[offset++] = residual.w();
        }

        const Eigen::Vector3d translation_prior =
            body_to_marker.xyz - options.body_to_marker_prior.xyz;
        residuals[offset++] = options.extrinsic_translation_prior_weight * translation_prior.x();
        residuals[offset++] = options.extrinsic_translation_prior_weight * translation_prior.y();
        residuals[offset++] = options.extrinsic_translation_prior_weight * translation_prior.z();
        residuals[offset++] =
            options.extrinsic_yaw_prior_weight *
            normalizeAngle(body_to_marker.rpy.z() - options.body_to_marker_prior.rpy.z());
        residuals[offset++] = options.delay_prior_weight * (delay[0] - options.initial_delay_s);
        return true;
    }
};
#endif

}  // namespace

double normalizeAngle(double angle) {
    if (!std::isfinite(angle)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double wrapped = std::remainder(angle, 2.0 * M_PI);
    if (std::abs(std::abs(wrapped) - M_PI) < 1.0e-12) {
        return std::copysign(M_PI, angle);
    }
    return wrapped;
}

double yawFromQuaternion(const Eigen::Quaterniond& q) {
    if (!finite(q.x()) || !finite(q.y()) || !finite(q.z()) || !finite(q.w()) || q.norm() <= kEps) {
        return 0.0;
    }
    const Eigen::Quaterniond normalized = q.normalized();
    const double qw = normalized.w();
    const double qx = normalized.x();
    const double qy = normalized.y();
    const double qz = normalized.z();
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Quaterniond quaternionFromRpy(const Eigen::Vector3d& rpy) {
    return Eigen::Quaterniond(rotationFromRpy(rpy)).normalized();
}

Eigen::Quaterniond quaternionFromYaw(double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).normalized();
}

bool validateCommandSamples(const std::vector<CommandSample>& commands, std::string* error) {
    if (commands.empty()) {
        if (error) {
            *error = "command samples are empty";
        }
        return false;
    }
    for (size_t i = 0; i < commands.size(); ++i) {
        const auto& sample = commands[i];
        if (!finite(sample.time_s) || !finite(sample.linear_velocity_cmd) ||
            !finite(sample.yaw_rate_cmd)) {
            if (error) {
                *error = "command sample contains non-finite value";
            }
            return false;
        }
        if (i > 0 && sample.time_s <= commands[i - 1].time_s) {
            if (error) {
                *error = "command timestamps must be strictly increasing";
            }
            return false;
        }
    }
    return true;
}

bool validatePoseSamples(const std::vector<PoseSample>& poses, std::string* error) {
    if (poses.size() < 2) {
        if (error) {
            *error = "at least two pose samples are required";
        }
        return false;
    }
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& sample = poses[i];
        if (!finite(sample.time_s) || !finiteVector(sample.position) ||
            !finite(sample.orientation.x()) || !finite(sample.orientation.y()) ||
            !finite(sample.orientation.z()) || !finite(sample.orientation.w()) ||
            sample.orientation.norm() <= kEps) {
            if (error) {
                *error = "pose sample contains invalid value";
            }
            return false;
        }
        if (i > 0 && sample.time_s <= poses[i - 1].time_s) {
            if (error) {
                *error = "pose timestamps must be strictly increasing";
            }
            return false;
        }
    }
    return true;
}

CommandSample interpolateCommandZeroOrder(const std::vector<CommandSample>& commands,
                                          double query_time_s) {
    if (commands.empty()) {
        return {};
    }
    if (query_time_s <= commands.front().time_s) {
        return commands.front();
    }
    const auto upper =
        std::upper_bound(commands.begin(), commands.end(), query_time_s,
                         [](double t, const CommandSample& sample) { return t < sample.time_s; });
    if (upper == commands.begin()) {
        return commands.front();
    }
    return *(upper - 1);
}

double firstOrderExactStep(double state, double command, double gain, double tau_s, double dt_s) {
    if (!finite(state) || !finite(command) || !finite(gain) || !finite(tau_s) || !finite(dt_s) ||
        tau_s <= 0.0 || dt_s < 0.0) {
        return state;
    }
    const double alpha = std::exp(-dt_s / std::max(kMinTau, tau_s));
    return alpha * state + (1.0 - alpha) * gain * command;
}

BodyState stepActuatorAndUnicycle(const BodyState& state, double linear_velocity_cmd,
                                  double yaw_rate_cmd, const ActuatorParameters& actuator,
                                  double dt_s) {
    BodyState next = state;
    if (!finite(dt_s) || dt_s <= 0.0) {
        return next;
    }
    next.linear_velocity = firstOrderExactStep(state.linear_velocity, linear_velocity_cmd,
                                               actuator.k_v, actuator.tau_v, dt_s);
    next.yaw_rate =
        firstOrderExactStep(state.yaw_rate, yaw_rate_cmd, actuator.k_w, actuator.tau_w, dt_s);

    const double dtheta = next.yaw_rate * dt_s;
    if (std::abs(next.yaw_rate) > 1.0e-8) {
        next.x += next.linear_velocity / next.yaw_rate *
                  (std::sin(state.yaw + dtheta) - std::sin(state.yaw));
        next.y += -next.linear_velocity / next.yaw_rate *
                  (std::cos(state.yaw + dtheta) - std::cos(state.yaw));
    } else {
        next.x += next.linear_velocity * std::cos(state.yaw) * dt_s;
        next.y += next.linear_velocity * std::sin(state.yaw) * dt_s;
    }
    next.yaw = normalizeAngle(state.yaw + dtheta);
    return next;
}

PoseSample markerPoseFromBodyState(const BodyState& body, const ExtrinsicEstimate& body_to_marker) {
    PoseSample pose;
    pose.time_s = 0.0;
    const Eigen::Matrix3d world_body_rotation =
        Eigen::AngleAxisd(body.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.position = Eigen::Vector3d(body.x, body.y, 0.0) + world_body_rotation * body_to_marker.xyz;
    const Eigen::Quaterniond world_body = quaternionFromYaw(body.yaw);
    pose.orientation = (world_body * quaternionFromRpy(body_to_marker.rpy)).normalized();
    return pose;
}

Eigen::Vector4d poseResidual(const PoseSample& measured_marker_pose,
                             const PoseSample& predicted_marker_pose, double position_weight,
                             double yaw_weight) {
    Eigen::Vector4d residual;
    residual.head<3>() =
        position_weight * (predicted_marker_pose.position - measured_marker_pose.position);
    const double yaw_error = normalizeAngle(yawFromQuaternion(predicted_marker_pose.orientation) -
                                            yawFromQuaternion(measured_marker_pose.orientation));
    residual.w() = yaw_weight * yaw_error;
    return residual;
}

std::vector<PoseSample> simulateMarkerTrajectory(const std::vector<CommandSample>& commands,
                                                 const std::vector<double>& sample_times_s,
                                                 const ActuatorParameters& actuator,
                                                 const ExtrinsicEstimate& body_to_marker,
                                                 const BodyState& initial_state, double delay_s) {
    std::vector<PoseSample> poses;
    poses.reserve(sample_times_s.size());
    if (sample_times_s.empty()) {
        return poses;
    }

    BodyState state = initial_state;
    for (size_t i = 0; i < sample_times_s.size(); ++i) {
        if (i > 0) {
            const double dt = sample_times_s[i] - sample_times_s[i - 1];
            const CommandSample command =
                interpolateCommandZeroOrder(commands, sample_times_s[i - 1] - delay_s);
            state = stepActuatorAndUnicycle(state, command.linear_velocity_cmd,
                                            command.yaw_rate_cmd, actuator, dt);
        }
        PoseSample pose = markerPoseFromBodyState(state, body_to_marker);
        pose.time_s = sample_times_s[i];
        poses.push_back(pose);
    }
    return poses;
}

IdentificationMetrics computeMetrics(const std::vector<PoseSample>& measured,
                                     const std::vector<PoseSample>& predicted) {
    IdentificationMetrics metrics;
    if (measured.empty() || measured.size() != predicted.size()) {
        return metrics;
    }

    double position_error_sum = 0.0;
    double yaw_error_sum = 0.0;
    for (size_t i = 0; i < measured.size(); ++i) {
        const Eigen::Vector3d position_error = predicted[i].position - measured[i].position;
        const double yaw_error = normalizeAngle(yawFromQuaternion(predicted[i].orientation) -
                                                yawFromQuaternion(measured[i].orientation));
        position_error_sum += position_error.squaredNorm();
        yaw_error_sum += yaw_error * yaw_error;
    }
    metrics.pose_position_rmse =
        std::sqrt(position_error_sum / static_cast<double>(measured.size()));
    metrics.yaw_rmse = std::sqrt(yaw_error_sum / static_cast<double>(measured.size()));
    metrics.final_position_error = (predicted.back().position - measured.back().position).norm();
    metrics.final_yaw_error =
        std::abs(normalizeAngle(yawFromQuaternion(predicted.back().orientation) -
                                yawFromQuaternion(measured.back().orientation)));
    return metrics;
}

IdentificationResult identifyActuatorAndExtrinsic(const std::vector<CommandSample>& commands,
                                                  const std::vector<PoseSample>& poses,
                                                  const IdentificationOptions& options) {
    IdentificationResult result;
    result.actuator = options.initial_actuator;
    result.body_to_marker = options.body_to_marker_prior;
    result.delay_s = options.initial_delay_s;

    std::string error;
    if (!validateCommandSamples(commands, &error) || !validatePoseSamples(poses, &error)) {
        result.message = error;
        return result;
    }
    result.initial_state = initialStateFromFirstPose(poses.front(), options.body_to_marker_prior);

#ifndef CONTROL_UTILS_WITH_CERES
    result.message = "Ceres is not available; install libceres-dev and rebuild control_utils";
    const std::vector<PoseSample> predicted =
        simulateMarkerTrajectory(commands, poseTimes(poses), result.actuator, result.body_to_marker,
                                 result.initial_state, result.delay_s);
    result.calibration = metricsOrEmpty(poses, predicted);
    return result;
#else
    double actuator_log[4] = {
        std::log(std::max(kMinTau, options.initial_actuator.k_v)),
        std::log(std::max(kMinTau, options.initial_actuator.tau_v)),
        std::log(std::max(kMinTau, options.initial_actuator.k_w)),
        std::log(std::max(kMinTau, options.initial_actuator.tau_w)),
    };
    double extrinsic[4] = {
        options.body_to_marker_prior.xyz.x(),
        options.body_to_marker_prior.xyz.y(),
        options.body_to_marker_prior.xyz.z(),
        options.body_to_marker_prior.rpy.z(),
    };
    double delay[1] = {options.initial_delay_s};
    double initial[5] = {
        result.initial_state.x,        result.initial_state.y,
        result.initial_state.yaw,      result.initial_state.linear_velocity,
        result.initial_state.yaw_rate,
    };

    ceres::Problem problem;
    auto* cost_function =
        new ceres::DynamicNumericDiffCostFunction<TrajectoryResidualFunctor, ceres::CENTRAL>(
            new TrajectoryResidualFunctor{commands, poses, options});
    cost_function->AddParameterBlock(4);
    cost_function->AddParameterBlock(4);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(5);
    cost_function->SetNumResiduals(static_cast<int>(poses.size() * 4 + 5));

    ceres::LossFunction* loss = nullptr;
    if (options.robust_loss_scale > 0.0) {
        loss = new ceres::HuberLoss(options.robust_loss_scale);
    }
    problem.AddResidualBlock(cost_function, loss, actuator_log, extrinsic, delay, initial);

    for (int i = 0; i < 4; ++i) {
        problem.SetParameterLowerBound(actuator_log, i, std::log(kMinTau));
    }
    problem.SetParameterLowerBound(delay, 0, options.initial_delay_s - options.delay_bound_s);
    problem.SetParameterUpperBound(delay, 0, options.initial_delay_s + options.delay_bound_s);
    if (!options.optimize_roll_pitch) {
        // First version estimates x/y/z/yaw. Roll and pitch are kept at their CAD prior by design.
    }

    ceres::Solver::Options solver_options;
    solver_options.max_num_iterations = std::max(1, options.max_iterations);
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    result.success = summary.IsSolutionUsable();
    result.message = summary.BriefReport();
    result.actuator.k_v = std::exp(actuator_log[0]);
    result.actuator.tau_v = std::exp(actuator_log[1]);
    result.actuator.k_w = std::exp(actuator_log[2]);
    result.actuator.tau_w = std::exp(actuator_log[3]);
    result.body_to_marker = options.body_to_marker_prior;
    result.body_to_marker.xyz = Eigen::Vector3d(extrinsic[0], extrinsic[1], extrinsic[2]);
    result.body_to_marker.rpy.z() = normalizeAngle(extrinsic[3]);
    result.delay_s = delay[0];
    result.initial_state.x = initial[0];
    result.initial_state.y = initial[1];
    result.initial_state.yaw = normalizeAngle(initial[2]);
    result.initial_state.linear_velocity = initial[3];
    result.initial_state.yaw_rate = initial[4];

    const std::vector<PoseSample> predicted =
        simulateMarkerTrajectory(commands, poseTimes(poses), result.actuator, result.body_to_marker,
                                 result.initial_state, result.delay_s);
    result.calibration = metricsOrEmpty(poses, predicted);
    return result;
#endif
}

}  // namespace ugv_identification
}  // namespace control_utils
