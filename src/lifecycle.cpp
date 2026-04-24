#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

#include "nav2_time_aware_controller/controller_utils.hpp"

namespace nav2_time_aware_controller
{

namespace
{
template<typename NodeT, typename T>
T declareOrGetParameter(
  const NodeT & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_value);
  }

  T value = default_value;
  node->get_parameter(name, value);
  return value;
}
}  // namespace

void TimeAwareLQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in TimeAwareLQRController::configure");
  }

  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;

  if (costmap_ros_) {
    base_frame_id_ = costmap_ros_->getBaseFrameID();
  }
  if (base_frame_id_.empty()) {
    base_frame_id_ = "base_link";
  }

  double server_frequency = controller_frequency_;
  if (node->has_parameter("controller_frequency")) {
    node->get_parameter("controller_frequency", server_frequency);
  } else {
    node->declare_parameter("controller_frequency", server_frequency);
  }

  desired_total_time_ = declareOrGetParameter(
    node, plugin_name_ + ".desired_total_time", desired_total_time_);
  controller_frequency_ = declareOrGetParameter(
    node, plugin_name_ + ".controller_frequency", server_frequency);
  max_v_ = declareOrGetParameter(node, plugin_name_ + ".max_v", max_v_);
  min_tracking_v_ = declareOrGetParameter(
    node, plugin_name_ + ".min_tracking_v", min_tracking_v_);
  max_w_ = declareOrGetParameter(node, plugin_name_ + ".max_w", max_w_);
  max_accel_ = declareOrGetParameter(node, plugin_name_ + ".max_accel", max_accel_);
  max_decel_ = declareOrGetParameter(node, plugin_name_ + ".max_decel", max_decel_);
  max_angular_accel_ = declareOrGetParameter(
    node, plugin_name_ + ".max_angular_accel", max_angular_accel_);
  command_filter_alpha_ = declareOrGetParameter(
    node, plugin_name_ + ".command_filter_alpha", command_filter_alpha_);

  q_s_ = declareOrGetParameter(node, plugin_name_ + ".q_s", q_s_);
  q_y_ = declareOrGetParameter(node, plugin_name_ + ".q_y", q_y_);
  q_psi_ = declareOrGetParameter(node, plugin_name_ + ".q_psi", q_psi_);
  r_v_ = declareOrGetParameter(node, plugin_name_ + ".r_v", r_v_);
  r_w_ = declareOrGetParameter(node, plugin_name_ + ".r_w", r_w_);
  qf_s_ = declareOrGetParameter(node, plugin_name_ + ".qf_s", qf_s_);
  qf_y_ = declareOrGetParameter(node, plugin_name_ + ".qf_y", qf_y_);
  qf_psi_ = declareOrGetParameter(node, plugin_name_ + ".qf_psi", qf_psi_);
  lqr_horizon_steps_ = declareOrGetParameter(
    node, plugin_name_ + ".lqr_horizon_steps", lqr_horizon_steps_);
  max_delta_v_ = declareOrGetParameter(node, plugin_name_ + ".max_delta_v", max_delta_v_);
  max_delta_w_ = declareOrGetParameter(node, plugin_name_ + ".max_delta_w", max_delta_w_);

  goal_tolerance_ = declareOrGetParameter(
    node, plugin_name_ + ".goal_tolerance", goal_tolerance_);
  goal_slowdown_dist_ = declareOrGetParameter(
    node, plugin_name_ + ".goal_slowdown_dist", goal_slowdown_dist_);
  goal_approach_dist_ = declareOrGetParameter(
    node, plugin_name_ + ".goal_approach_dist", goal_approach_dist_);
  goal_approach_gain_ = declareOrGetParameter(
    node, plugin_name_ + ".goal_approach_gain", goal_approach_gain_);
  goal_heading_gain_ = declareOrGetParameter(
    node, plugin_name_ + ".goal_heading_gain", goal_heading_gain_);
  goal_heading_stop_angle_ = declareOrGetParameter(
    node, plugin_name_ + ".goal_heading_stop_angle", goal_heading_stop_angle_);
  min_goal_approach_v_ = declareOrGetParameter(
    node, plugin_name_ + ".min_goal_approach_v", min_goal_approach_v_);
  max_goal_approach_v_ = declareOrGetParameter(
    node, plugin_name_ + ".max_goal_approach_v", max_goal_approach_v_);
  max_goal_approach_w_ = declareOrGetParameter(
    node, plugin_name_ + ".max_goal_approach_w", max_goal_approach_w_);
  near_path_end_dist_ = declareOrGetParameter(
    node, plugin_name_ + ".near_path_end_dist", near_path_end_dist_);

  allow_overtime_ = declareOrGetParameter(
    node, plugin_name_ + ".allow_overtime", allow_overtime_);
  fastest_tracking_v_ = declareOrGetParameter(
    node, plugin_name_ + ".fastest_tracking_v", fastest_tracking_v_);
  min_overtime_horizon_ = declareOrGetParameter(
    node, plugin_name_ + ".min_overtime_horizon", min_overtime_horizon_);

  allow_progress_backtracking_ = declareOrGetParameter(
    node, plugin_name_ + ".allow_progress_backtracking", allow_progress_backtracking_);
  max_backtrack_dist_ = declareOrGetParameter(
    node, plugin_name_ + ".max_backtrack_dist", max_backtrack_dist_);

  debug_ = declareOrGetParameter(node, plugin_name_ + ".debug", debug_);
  invert_lateral_error_sign_ = declareOrGetParameter(
    node, plugin_name_ + ".invert_lateral_error_sign", invert_lateral_error_sign_);
  invert_heading_error_sign_ = declareOrGetParameter(
    node, plugin_name_ + ".invert_heading_error_sign", invert_heading_error_sign_);
  invert_delta_w_sign_ = declareOrGetParameter(
    node, plugin_name_ + ".invert_delta_w_sign", invert_delta_w_sign_);
  debug_csv_path_ = declareOrGetParameter(
    node, plugin_name_ + ".debug_csv_path", debug_csv_path_);

  if (desired_total_time_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s desired_total_time must be positive. Falling back to 20.0 s.",
      plugin_name_.c_str());
    desired_total_time_ = 20.0;
  }
  if (controller_frequency_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s controller_frequency must be positive. Falling back to 10.0 Hz.",
      plugin_name_.c_str());
    controller_frequency_ = 10.0;
  }
  if (max_v_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s max_v must be positive. Falling back to 0.30 m/s.",
      plugin_name_.c_str());
    max_v_ = 0.30;
  }
  if (max_w_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s max_w must be positive. Falling back to 1.0 rad/s.",
      plugin_name_.c_str());
    max_w_ = 1.0;
  }

  min_tracking_v_ = clamp(min_tracking_v_, 0.0, max_v_);
  max_accel_ = std::max(0.0, max_accel_);
  max_decel_ = std::max(0.0, max_decel_);
  max_angular_accel_ = std::max(0.0, max_angular_accel_);
  command_filter_alpha_ = clamp(command_filter_alpha_, 0.0, 1.0);
  q_s_ = std::max(0.0, q_s_);
  q_y_ = std::max(0.0, q_y_);
  q_psi_ = std::max(0.0, q_psi_);
  r_v_ = std::max(kEps, r_v_);
  r_w_ = std::max(kEps, r_w_);
  qf_s_ = std::max(0.0, qf_s_);
  qf_y_ = std::max(0.0, qf_y_);
  qf_psi_ = std::max(0.0, qf_psi_);
  lqr_horizon_steps_ = std::max(1, lqr_horizon_steps_);
  max_delta_v_ = std::max(0.0, max_delta_v_);
  max_delta_w_ = std::max(0.0, max_delta_w_);
  goal_tolerance_ = std::max(0.0, goal_tolerance_);
  goal_slowdown_dist_ = std::max(goal_slowdown_dist_, goal_tolerance_);
  goal_approach_dist_ = std::max(goal_approach_dist_, goal_tolerance_);
  goal_approach_gain_ = std::max(0.0, goal_approach_gain_);
  goal_heading_gain_ = std::max(0.0, goal_heading_gain_);
  goal_heading_stop_angle_ = clamp(goal_heading_stop_angle_, 0.0, 3.14159265358979323846);
  min_goal_approach_v_ = clamp(min_goal_approach_v_, 0.0, max_v_);
  max_goal_approach_v_ = clamp(max_goal_approach_v_, min_goal_approach_v_, max_v_);
  max_goal_approach_w_ = clamp(max_goal_approach_w_, 0.0, max_w_);
  near_path_end_dist_ = std::max(0.0, near_path_end_dist_);
  fastest_tracking_v_ = clamp(fastest_tracking_v_, min_tracking_v_, max_v_);
  min_overtime_horizon_ = std::max(min_overtime_horizon_, 1.0 / controller_frequency_);
  max_backtrack_dist_ = std::max(0.0, max_backtrack_dist_);
  original_max_v_ = max_v_;

  projected_point_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/time_aware_lqr/projected_point", rclcpp::SystemDefaultsQoS());
  reference_point_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/time_aware_lqr/reference_point", rclcpp::SystemDefaultsQoS());
  debug_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/time_aware_lqr/debug_path", rclcpp::SystemDefaultsQoS());

  if (!debug_csv_path_.empty()) {
    debug_csv_stream_.open(debug_csv_path_, std::ios::out | std::ios::trunc);
    if (!debug_csv_stream_.is_open()) {
      RCLCPP_WARN(
        node->get_logger(),
        "%s failed to open debug_csv_path '%s'. CSV logging disabled.",
        plugin_name_.c_str(), debug_csv_path_.c_str());
    }
  }

  resetPlanState();
  resetCommandState();

  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s as Frenet LQR controller | desired_total_time=%.2f s, "
    "controller_frequency=%.2f Hz, max_v=%.2f m/s, max_w=%.2f rad/s",
    plugin_name_.c_str(), desired_total_time_, controller_frequency_, max_v_, max_w_);
}

void TimeAwareLQRController::cleanup()
{
  resetPlanState();
  resetCommandState();
  task_started_ = false;

  if (debug_csv_stream_.is_open()) {
    debug_csv_stream_.close();
  }
  projected_point_pub_.reset();
  reference_point_pub_.reset();
  debug_path_pub_.reset();
  tf_.reset();
  costmap_ros_.reset();
}

void TimeAwareLQRController::activate()
{
  active_ = true;
  if (projected_point_pub_) {
    projected_point_pub_->on_activate();
  }
  if (reference_point_pub_) {
    reference_point_pub_->on_activate();
  }
  if (debug_path_pub_) {
    debug_path_pub_->on_activate();
  }
}

void TimeAwareLQRController::deactivate()
{
  active_ = false;
  if (projected_point_pub_) {
    projected_point_pub_->on_deactivate();
  }
  if (reference_point_pub_) {
    reference_point_pub_->on_deactivate();
  }
  if (debug_path_pub_) {
    debug_path_pub_->on_deactivate();
  }
  resetCommandState();
}

void TimeAwareLQRController::resetPlanState()
{
  global_plan_.poses.clear();
  path_s_.clear();
  path_yaws_.clear();
  path_curvatures_.clear();
  reference_traj_.clear();
  valid_plan_ = false;
  plan_fastest_mode_ = false;
  have_controller_goal_ = false;
  have_last_projection_ = false;
  last_closest_segment_idx_ = 0;
  last_projected_s_ = 0.0;
}

void TimeAwareLQRController::resetCommandState()
{
  last_v_cmd_ = 0.0;
  last_w_cmd_ = 0.0;
  have_last_cmd_time_ = false;
}

void TimeAwareLQRController::setPlan(const nav_msgs::msg::Path & path)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in TimeAwareLQRController::setPlan");
  }

  const rclcpp::Time now = node->now();

  const bool had_projection = valid_plan_ && have_last_projection_ &&
    !global_plan_.poses.empty() && !path_s_.empty();
  geometry_msgs::msg::Point previous_projected_point;
  if (had_projection) {
    previous_projected_point = samplePathAtArcLength(last_projected_s_);
  }

  global_plan_ = path;
  valid_plan_ = false;
  path_s_.clear();
  path_yaws_.clear();
  path_curvatures_.clear();
  reference_traj_.clear();
  plan_fastest_mode_ = false;

  if (global_plan_.poses.size() < 2) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s received a plan with %zu pose(s). At least two poses are required.",
      plugin_name_.c_str(), global_plan_.poses.size());
    resetPlanState();
    return;
  }

  if (!computePathArcLengths() || path_s_.back() <= kEps) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s received a plan with no usable path length.",
      plugin_name_.c_str());
    resetPlanState();
    return;
  }

  computePathTangents();
  computePathCurvatures();

  updateControllerGoalFromPlan(global_plan_, now);

  if (!task_started_) {
    task_start_time_ = now;
    task_started_ = true;
  }
  plan_start_time_ = now;

  const double task_elapsed = std::max(0.0, (now - task_start_time_).seconds());
  double requested_time = desired_total_time_;
  if (task_elapsed > 0.0) {
    requested_time = desired_total_time_ - task_elapsed;
  }
  if (requested_time <= 0.0 && !allow_overtime_) {
    requested_time = path_s_.back() / std::max(max_v_, kEps);
  }

  generateReferenceTrajectory(requested_time);

  valid_plan_ = !reference_traj_.empty();

  have_last_projection_ = false;
  last_closest_segment_idx_ = 0;
  last_projected_s_ = 0.0;
  if (had_projection && valid_plan_) {
    geometry_msgs::msg::PoseStamped projected_pose;
    projected_pose.pose.position = previous_projected_point;
    const ProjectionResult projection = projectPoseToPath(projected_pose);
    const double remap_distance = std::hypot(
      projection.point.x - previous_projected_point.x,
      projection.point.y - previous_projected_point.y);
    if (remap_distance > 1.0) {
      have_last_projection_ = false;
      last_closest_segment_idx_ = 0;
      last_projected_s_ = 0.0;
    }
  }

  if (debug_path_pub_ && debug_path_pub_->is_activated()) {
    debug_path_pub_->publish(global_plan_);
  }

  RCLCPP_INFO(
    node->get_logger(),
    "%s accepted plan with %zu poses, length %.3f m, task_elapsed %.2f s, "
    "requested_time %.2f s, reference_horizon %.2f s, mode %s, "
    "path_end_to_controller_goal %.3f m.",
    plugin_name_.c_str(), global_plan_.poses.size(), path_s_.back(), task_elapsed,
    requested_time, reference_traj_.empty() ? 0.0 : reference_traj_.back().t,
    plan_fastest_mode_ ? "INFEASIBLE_TIME_TRACKING" : "NORMAL_LQR_TRACKING",
    computePathEndToControllerGoalDistance());
}

void TimeAwareLQRController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    const double ratio = clamp(speed_limit, 0.0, 100.0) / 100.0;
    max_v_ = original_max_v_ * ratio;
  } else {
    max_v_ = clamp(speed_limit, 0.0, original_max_v_);
  }

  min_tracking_v_ = clamp(min_tracking_v_, 0.0, max_v_);
  fastest_tracking_v_ = clamp(fastest_tracking_v_, min_tracking_v_, max_v_);
  max_goal_approach_v_ = clamp(max_goal_approach_v_, min_goal_approach_v_, max_v_);
  max_goal_approach_w_ = clamp(max_goal_approach_w_, 0.0, max_w_);

  if (valid_plan_ && task_started_) {
    auto node = node_.lock();
    if (node) {
      const double task_elapsed = std::max(0.0, (node->now() - task_start_time_).seconds());
      const double requested_time = desired_total_time_ - task_elapsed;
      generateReferenceTrajectory(requested_time);
    }
  }
}

}