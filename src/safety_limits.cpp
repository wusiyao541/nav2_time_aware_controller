#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include <algorithm>
#include <utility>

#include "nav2_time_aware_controller/controller_utils.hpp"

namespace nav2_time_aware_controller
{

double TimeAwareLQRController::computeGoalSlowdownScale(double distance_to_goal) const
{
  if (goal_slowdown_dist_ <= goal_tolerance_ + kEps) {
    return 1.0;
  }
  if (distance_to_goal >= goal_slowdown_dist_) {
    return 1.0;
  }
  if (distance_to_goal <= goal_tolerance_) {
    return 0.0;
  }

  const double scale =
    (distance_to_goal - goal_tolerance_) / (goal_slowdown_dist_ - goal_tolerance_);
  return clamp(scale, 0.0, 1.0);
}

double TimeAwareLQRController::applyAccelerationLimit(
  double target_value,
  double last_value,
  double negative_rate_limit,
  double positive_rate_limit,
  double dt) const
{
  const double safe_dt = dt > kEps ? dt : 1.0 / controller_frequency_;
  const double lower = last_value - std::max(0.0, negative_rate_limit) * safe_dt;
  const double upper = last_value + std::max(0.0, positive_rate_limit) * safe_dt;
  return clamp(target_value, lower, upper);
}

double TimeAwareLQRController::clamp(double value, double lower, double upper) const
{
  if (lower > upper) {
    std::swap(lower, upper);
  }
  return std::min(std::max(value, lower), upper);
}

const char * TimeAwareLQRController::modeToString(ControlMode mode) const
{
  switch (mode) {
    case ControlMode::NORMAL_LQR_TRACKING:
      return "FEASIBLE_TIME_TRACKING";
    case ControlMode::INFEASIBLE_TIME_TRACKING:
      return "INFEASIBLE_TIME_TRACKING";
    case ControlMode::GOAL_APPROACH:
      return "GOAL_APPROACH";
    case ControlMode::GOAL_REACHED:
      return "GOAL_REACHED";
  }
  return "UNKNOWN";
}

geometry_msgs::msg::TwistStamped TimeAwareLQRController::makeZeroCommand(
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = stamp;
  cmd.header.frame_id = base_frame_id_;
  cmd.twist.linear.x = 0.0;
  cmd.twist.angular.z = 0.0;
  return cmd;
}

}
