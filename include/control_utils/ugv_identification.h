#ifndef CONTROL_UTILS_UGV_IDENTIFICATION_H
#define CONTROL_UTILS_UGV_IDENTIFICATION_H

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace control_utils {
namespace ugv_identification {

struct CommandSample {
    double time_s{0.0};
    double linear_velocity_cmd{0.0};
    double yaw_rate_cmd{0.0};
};

struct PoseSample {
    double time_s{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct BodyState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double linear_velocity{0.0};
    double yaw_rate{0.0};
};

struct ExtrinsicEstimate {
    Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
};

struct ActuatorParameters {
    double k_v{1.0};
    double tau_v{0.2};
    double k_w{1.0};
    double tau_w{0.2};
};

struct IdentificationOptions {
    ActuatorParameters initial_actuator;
    ExtrinsicEstimate body_to_marker_prior;
    bool optimize_roll_pitch{false};
    double pose_position_weight{1.0};
    double pose_yaw_weight{1.0};
    double extrinsic_translation_prior_weight{0.0};
    double extrinsic_yaw_prior_weight{0.0};
    double delay_prior_weight{0.0};
    double initial_delay_s{0.0};
    double delay_bound_s{0.25};
    double robust_loss_scale{0.05};
    int max_iterations{80};
};

struct IdentificationMetrics {
    double pose_position_rmse{0.0};
    double yaw_rmse{0.0};
    double final_position_error{0.0};
    double final_yaw_error{0.0};
};

struct IdentificationResult {
    bool success{false};
    std::string message;
    ActuatorParameters actuator;
    ExtrinsicEstimate body_to_marker;
    double delay_s{0.0};
    BodyState initial_state;
    IdentificationMetrics calibration;
};

double normalizeAngle(double angle);
double yawFromQuaternion(const Eigen::Quaterniond& q);
Eigen::Quaterniond quaternionFromRpy(const Eigen::Vector3d& rpy);
Eigen::Quaterniond quaternionFromYaw(double yaw);

bool validateCommandSamples(const std::vector<CommandSample>& commands, std::string* error);
bool validatePoseSamples(const std::vector<PoseSample>& poses, std::string* error);

CommandSample interpolateCommandZeroOrder(const std::vector<CommandSample>& commands,
                                          double query_time_s);

double firstOrderExactStep(double state, double command, double gain, double tau_s, double dt_s);
BodyState stepActuatorAndUnicycle(const BodyState& state, double linear_velocity_cmd,
                                  double yaw_rate_cmd, const ActuatorParameters& actuator,
                                  double dt_s);

PoseSample markerPoseFromBodyState(const BodyState& body, const ExtrinsicEstimate& body_to_marker);
Eigen::Vector4d poseResidual(const PoseSample& measured_marker_pose,
                             const PoseSample& predicted_marker_pose, double position_weight,
                             double yaw_weight);

std::vector<PoseSample> simulateMarkerTrajectory(const std::vector<CommandSample>& commands,
                                                 const std::vector<double>& sample_times_s,
                                                 const ActuatorParameters& actuator,
                                                 const ExtrinsicEstimate& body_to_marker,
                                                 const BodyState& initial_state, double delay_s);

IdentificationMetrics computeMetrics(const std::vector<PoseSample>& measured,
                                     const std::vector<PoseSample>& predicted);

IdentificationResult identifyActuatorAndExtrinsic(const std::vector<CommandSample>& commands,
                                                  const std::vector<PoseSample>& poses,
                                                  const IdentificationOptions& options);

}  // namespace ugv_identification
}  // namespace control_utils

#endif  // CONTROL_UTILS_UGV_IDENTIFICATION_H
