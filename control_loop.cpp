#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include <algorithm>
#include <cmath>

#include "angles/angles.h"
#include "nav2_time_aware_controller/controller_utils.hpp"
#include "tf2/utils.h"

namespace nav2_time_aware_controller
{

geometry_msgs::msg::TwistStamped TimeAwareLQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto node = node_.lock();
  if (!node) {
    return makeZeroCommand(rclcpp::Time(0, 0, RCL_ROS_TIME));
  }

  const rclcpp::Time stamp = node->now();
  if (!active_ || !valid_plan_ || reference_traj_.empty()) {
    resetCommandState();
    return makeZeroCommand(stamp);
  }

  double control_dt = 1.0 / controller_frequency_;
  if (have_last_cmd_time_) {
    const double measured_dt = (stamp - last_cmd_time_).seconds();
    if (std::isfinite(measured_dt) && measured_dt > kEps && measured_dt < 1.0) {
      control_dt = measured_dt;
    }
  }

  const double distance_to_goal = computeDistanceToGoal(pose);
  const double distance_to_path_end = computeDistanceToPathEnd(pose);
  const double path_end_to_controller_goal = computePathEndToControllerGoalDistance();
  const bool goal_checker_reached = goal_checker && have_controller_goal_ &&
    goal_checker->isGoalReached(pose.pose, controller_goal_pose_, velocity);
  const bool fallback_goal_reached = !goal_checker && distance_to_goal <= goal_tolerance_;
  if (goal_checker_reached || fallback_goal_reached) {
    resetCommandState();
    last_cmd_time_ = stamp;
    have_last_cmd_time_ = true;
    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "%s debug | mode %s, distance_to_goal %.3f, distance_to_path_end %.3f, "
        "path_end_to_controller_goal %.3f, stop_reason %s",
        plugin_name_.c_str(), modeToString(ControlMode::GOAL_REACHED),
        distance_to_goal, distance_to_path_end, path_end_to_controller_goal,
        goal_checker_reached ? "goal_checker" : "fallback_distance");
    }
    return makeZeroCommand(stamp);
  }

  const double plan_elapsed = std::max(0.0, (stamp - plan_start_time_).seconds());
  const double task_elapsed = task_started_ ?
    std::max(0.0, (stamp - task_start_time_).seconds()) : 0.0;
  const bool reference_exhausted = plan_elapsed >= reference_traj_.back().t;
  const ProjectionResult projection = projectPoseToPath(pose);
  const double path_length = path_s_.empty() ? 0.0 : path_s_.back();
  const bool near_path_end =
    path_length > kEps && projection.s >= path_length - near_path_end_dist_;

  ControlMode mode = ControlMode::NORMAL_LQR_TRACKING;
  if (distance_to_goal < goal_approach_dist_ ||
    (near_path_end && distance_to_goal > goal_tolerance_))
  {
    mode = ControlMode::GOAL_APPROACH;
  } else if (plan_fastest_mode_ || reference_exhausted ||
    (allow_overtime_ && task_elapsed > desired_total_time_))
  {
    mode = ControlMode::INFEASIBLE_TIME_TRACKING;
  }

  ReferencePoint reference = sampleReferenceAtTime(plan_elapsed);
  if (mode == ControlMode::INFEASIBLE_TIME_TRACKING) {
    reference.s = projection.s;
    reference.v = clamp(fastest_tracking_v_, min_tracking_v_, max_v_);
    reference.yaw = projection.yaw;
    reference.kappa = projection.kappa;
    reference.w = reference.kappa * reference.v;
  } else if (mode == ControlMode::GOAL_APPROACH) {
    reference.s = projection.s;
    reference.v = clamp(
      goal_approach_gain_ * distance_to_goal,
      min_goal_approach_v_,
      max_goal_approach_v_);
    reference.yaw = projection.yaw;
    reference.kappa = projection.kappa;
    reference.w = reference.kappa * reference.v;
  }

  const double robot_yaw = tf2::getYaw(pose.pose.orientation);
  double e_s = clamp(projection.s, 0.0, path_length) - clamp(reference.s, 0.0, path_length);
  double e_y = projection.e_y;
  double e_psi = angles::shortest_angular_distance(projection.yaw, robot_yaw);

  // Sign convention: e_y > 0 means the robot is left of the path, and e_psi > 0 means
  // the robot yaw is counterclockwise from the path tangent. The LQR delta_w should
  // reduce both errors. The inversion flags below are only for quick sim diagnostics.
  if (invert_lateral_error_sign_) {
    e_y = -e_y;
  }
  if (invert_heading_error_sign_) {
    e_psi = -e_psi;
  }

  auto [delta_v, delta_w] = solveLQR(e_s, e_y, e_psi, reference.v, control_dt);
  delta_v = clamp(delta_v, -max_delta_v_, max_delta_v_);
  delta_w = clamp(delta_w, -max_delta_w_, max_delta_w_);

  const double w_ref = projection.kappa * reference.v;
  if (mode == ControlMode::INFEASIBLE_TIME_TRACKING) {
    delta_v = clamp(delta_v, -0.5 * max_delta_v_, 0.5 * max_delta_v_);
  } else if (mode == ControlMode::GOAL_APPROACH) {
    delta_v = 0.0;
    delta_w = 0.0;
  }
  if (invert_delta_w_sign_) {
    delta_w = -delta_w;
  }

  double v_cmd = reference.v + delta_v;
  double w_cmd = w_ref + delta_w;
  bool goal_approach_translation_allowed = true;

  if (mode == ControlMode::GOAL_APPROACH) {
    const auto & goal = have_controller_goal_ ?
      controller_goal_pose_.position : global_plan_.poses.back().pose.position;
    const double heading_to_goal = std::atan2(
      goal.y - pose.pose.position.y,
      goal.x - pose.pose.position.x);
    const double heading_error_to_goal =
      angles::shortest_angular_distance(robot_yaw, heading_to_goal);
    if (std::abs(heading_error_to_goal) > goal_heading_stop_angle_) {
      v_cmd = 0.0;
      goal_approach_translation_allowed = false;
    } else {
      const double heading_scale = std::max(0.0, std::cos(heading_error_to_goal));
      v_cmd = clamp(
        goal_approach_gain_ * distance_to_goal * heading_scale,
        min_goal_approach_v_,
        max_goal_approach_v_);
    }
    w_cmd = clamp(
      goal_heading_gain_ * heading_error_to_goal,
      -max_goal_approach_w_,
      max_goal_approach_w_);
  }

  v_cmd = clamp(v_cmd, 0.0, max_v_);
  w_cmd = clamp(w_cmd, -max_w_, max_w_);

  if (mode == ControlMode::GOAL_APPROACH) {
    const double slowdown_scale = goal_slowdown_dist_ > kEps ?
      clamp(distance_to_goal / goal_slowdown_dist_, 0.0, 1.0) : 1.0;
    v_cmd = clamp(v_cmd, 0.0, max_goal_approach_v_) * slowdown_scale;
    if (distance_to_goal > goal_tolerance_ && goal_approach_translation_allowed) {
      v_cmd = std::max(v_cmd, std::min(min_goal_approach_v_, max_goal_approach_v_));
    }
    w_cmd = clamp(w_cmd, -max_goal_approach_w_, max_goal_approach_w_);
  } else {
    const double slowdown_scale = computeGoalSlowdownScale(distance_to_goal);
    v_cmd *= slowdown_scale;
    if (distance_to_goal > goal_tolerance_) {
      v_cmd = std::max(v_cmd, std::min(min_tracking_v_, max_v_));
    }
  }

  v_cmd = applyAccelerationLimit(v_cmd, last_v_cmd_, max_decel_, max_accel_, control_dt);
  if (mode == ControlMode::GOAL_APPROACH) {
    v_cmd = clamp(v_cmd, 0.0, max_goal_approach_v_);
    if (distance_to_goal > goal_tolerance_ && goal_approach_translation_allowed) {
      v_cmd = std::max(v_cmd, std::min(min_goal_approach_v_, max_goal_approach_v_));
    }
  } else if (distance_to_goal > goal_tolerance_) {
    v_cmd = std::max(v_cmd, std::min(min_tracking_v_, max_v_));
  }

  w_cmd = clamp(w_cmd, -max_w_, max_w_);
  if (mode == ControlMode::GOAL_APPROACH) {
    w_cmd = clamp(w_cmd, -max_goal_approach_w_, max_goal_approach_w_);
  }
  w_cmd = applyAccelerationLimit(
    w_cmd, last_w_cmd_, max_angular_accel_, max_angular_accel_, control_dt);
  w_cmd = clamp(w_cmd, -max_w_, max_w_);
  if (mode == ControlMode::GOAL_APPROACH) {
    w_cmd = clamp(w_cmd, -max_goal_approach_w_, max_goal_approach_w_);
  }

  v_cmd = clamp(
    command_filter_alpha_ * v_cmd + (1.0 - command_filter_alpha_) * last_v_cmd_,
    0.0, max_v_);
  w_cmd = clamp(
    command_filter_alpha_ * w_cmd + (1.0 - command_filter_alpha_) * last_w_cmd_,
    -max_w_, max_w_);
  if (mode == ControlMode::GOAL_APPROACH) {
    v_cmd = clamp(v_cmd, 0.0, max_goal_approach_v_);
    w_cmd = clamp(w_cmd, -max_goal_approach_w_, max_goal_approach_w_);
    if (distance_to_goal > goal_tolerance_ && goal_approach_translation_allowed) {
      v_cmd = std::max(v_cmd, std::min(min_goal_approach_v_, max_goal_approach_v_));
    }
  } else if (distance_to_goal > goal_tolerance_) {
    v_cmd = std::max(v_cmd, std::min(min_tracking_v_, max_v_));
  }

  publishDebugVisualization(projection, reference, stamp);
  writeCsvDebug(
    stamp, mode, pose, projection, reference, e_s, e_y, e_psi,
    delta_v, delta_w, v_cmd, w_cmd, distance_to_goal);

  if (debug_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "%s debug | mode %s, robot(%.3f, %.3f, yaw %.3f), projected_s %.3f, "
      "reference_s %.3f, e_s %.3f, e_y %.3f, e_psi %.3f, v_ref %.3f, "
      "w_ref %.3f, delta_v %.3f, delta_w %.3f, v_cmd %.3f, w_cmd %.3f, "
      "distance_to_goal %.3f, distance_to_path_end %.3f, path_end_to_controller_goal %.3f, "
      "goal_checker_reached %s, closest_segment %zu, yaw_ref %.3f, kappa_ref %.3f, "
      "reference_exhausted %s, near_path_end %s",
      plugin_name_.c_str(), modeToString(mode),
      pose.pose.position.x, pose.pose.position.y, robot_yaw,
      projection.s, reference.s, e_s, e_y, e_psi,
      reference.v, w_ref, delta_v, delta_w, v_cmd, w_cmd,
      distance_to_goal, distance_to_path_end, path_end_to_controller_goal,
      goal_checker_reached ? "true" : "false",
      projection.segment_index, projection.yaw, projection.kappa,
      reference_exhausted ? "true" : "false", near_path_end ? "true" : "false");
  }

  last_v_cmd_ = v_cmd;
  last_w_cmd_ = w_cmd;
  last_cmd_time_ = stamp;
  have_last_cmd_time_ = true;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = stamp;
  cmd.header.frame_id = base_frame_id_;
  cmd.twist.linear.x = v_cmd;
  cmd.twist.angular.z = w_cmd;
  return cmd;
}

}