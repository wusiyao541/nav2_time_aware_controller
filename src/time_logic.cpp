#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include <algorithm>
#include <cmath>

#include "angles/angles.h"
#include "nav2_time_aware_controller/controller_utils.hpp"

namespace nav2_time_aware_controller
{

void TimeAwareLQRController::generateReferenceTrajectory(double segment_time)
{
  reference_traj_.clear();
  plan_fastest_mode_ = false;

  const double path_length = path_s_.empty() ? 0.0 : path_s_.back();
  if (path_length <= kEps) {
    reference_traj_.push_back({});
    return;
  }

  double horizon = segment_time;
  double v_ref = 0.0;

  if (segment_time <= kEps) {
    plan_fastest_mode_ = true;
    v_ref = clamp(fastest_tracking_v_, min_tracking_v_, max_v_);
    horizon = std::max(path_length / std::max(v_ref, kEps), min_overtime_horizon_);
  } else {
    v_ref = path_length / segment_time;
    if (v_ref > max_v_) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN(
          node->get_logger(),
          "%s requested segment time %.2f s is physically infeasible for %.3f m: "
          "required_avg_speed %.3f m/s exceeds max_v %.3f m/s. Using fastest feasible timing.",
          plugin_name_.c_str(), segment_time, path_length, v_ref, max_v_);
      }
      v_ref = max_v_;
      horizon = path_length / std::max(v_ref, kEps);
      plan_fastest_mode_ = true;
    }
  }

  horizon = std::max(horizon, 1.0 / controller_frequency_);
  v_ref = clamp(v_ref, 0.0, max_v_);

  const double dt = 1.0 / controller_frequency_;
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon / dt)));
  reference_traj_.reserve(static_cast<size_t>(steps) + 1);

  for (int k = 0; k <= steps; ++k) {
    ReferencePoint ref;
    ref.t = std::min(k * dt, horizon);
    ref.s = std::min(v_ref * ref.t, path_length);
    ref.v = v_ref;
    ref.yaw = sampleYawAtArcLength(ref.s);
    ref.kappa = sampleCurvatureAtArcLength(ref.s);
    ref.w = ref.kappa * ref.v;
    reference_traj_.push_back(ref);
  }

  reference_traj_.back().t = horizon;
  reference_traj_.back().s = path_length;
  reference_traj_.back().v = v_ref;
  reference_traj_.back().yaw = sampleYawAtArcLength(path_length);
  reference_traj_.back().kappa = sampleCurvatureAtArcLength(path_length);
  reference_traj_.back().w = reference_traj_.back().kappa * reference_traj_.back().v;
}

ReferencePoint TimeAwareLQRController::sampleReferenceAtTime(double elapsed_time) const
{
  if (reference_traj_.empty()) {
    return {};
  }

  if (elapsed_time <= 0.0) {
    return reference_traj_.front();
  }
  if (elapsed_time >= reference_traj_.back().t) {
    return reference_traj_.back();
  }

  const double dt = 1.0 / controller_frequency_;
  const size_t idx = std::min(
    static_cast<size_t>(elapsed_time / dt),
    reference_traj_.size() - 2);
  const ReferencePoint & a = reference_traj_[idx];
  const ReferencePoint & b = reference_traj_[idx + 1];
  const double ratio = (b.t - a.t <= kEps) ? 0.0 : (elapsed_time - a.t) / (b.t - a.t);

  ReferencePoint ref;
  ref.t = elapsed_time;
  ref.s = a.s + ratio * (b.s - a.s);
  ref.v = a.v + ratio * (b.v - a.v);
  const double dyaw = angles::shortest_angular_distance(a.yaw, b.yaw);
  ref.yaw = angles::normalize_angle(a.yaw + ratio * dyaw);
  ref.kappa = a.kappa + ratio * (b.kappa - a.kappa);
  ref.w = ref.kappa * ref.v;
  return ref;
}

}
