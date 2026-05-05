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
  const bool distance_goal_reached = !goal_checker && distance_to_goal <= goal_tolerance_;
  if (goal_checker_reached || distance_goal_reached) {
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
        goal_checker_reached ? "goal_checker" : "distance_tolerance");
    }
    return makeZeroCommand(stamp);
  }

  const double plan_elapsed = std::max(0.0, (stamp - plan_start_time_).seconds());
  const double task_elapsed = task_started_ ?
    std::max(0.0, (stamp - task_start_time_).seconds()) : 0.0;
  const bool reference_exhausted = plan_elapsed >= reference_traj_.back().t;
  const ProjectionResult projection = projectPoseToPath(pose);
  const double path_length = path_s_.empty() ? 0.0 : path_s_.back();
  const double path_remaining = path_length > kEps ?
    clamp(path_length - projection.s, 0.0, path_length) : 0.0;
  const double requested_time_remaining = std::max(0.0, desired_segment_time_ - task_elapsed);
  const double required_average_speed = requested_time_remaining > kEps ?
    path_remaining / requested_time_remaining : 0.0;
  const double expected_s = desired_segment_time_ > kEps ?
    clamp(path_length * task_elapsed / desired_segment_time_, 0.0, path_length) : path_length;
  const double schedule_error_s = expected_s - projection.s;
  const double aggressive_speed_before_clamp = required_average_speed * time_aggressiveness_;
  const double aggressive_speed_after_clamp = clamp(
    aggressive_speed_before_clamp, min_tracking_v_, max_v_);
  const bool near_path_end =
    path_length > kEps && projection.s >= path_length - near_path_end_dist_;

  ControlMode mode = ControlMode::NORMAL_LQR_TRACKING;
  if (distance_to_goal < goal_approach_dist_ ||
    (near_path_end && distance_to_goal > goal_tolerance_))
  {
    mode = ControlMode::GOAL_APPROACH;
  } else if (plan_fastest_mode_ || reference_exhausted ||
    (allow_overtime_ && task_elapsed > desired_segment_time_))
  {
    mode = ControlMode::INFEASIBLE_TIME_TRACKING;
  }

  ReferencePoint reference = sampleReferenceAtTime(plan_elapsed);
  const double raw_desired_linear_speed = reference.v;
  bool time_aggressiveness_applied = false;
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
  } else if (
    requested_time_remaining > kEps &&
    schedule_error_s > 0.05 &&
    aggressive_speed_after_clamp > reference.v + kEps)
  {
    reference.v = aggressive_speed_after_clamp;
    reference.w = reference.kappa * reference.v;
    time_aggressiveness_applied = true;
  }
  const double speed_after_time_logic = reference.v;
  const double speed_after_curvature_limit = reference.v;

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
  const double speed_after_lqr = v_cmd;
  bool goal_approach_translation_allowed = true;
  bool heading_error_limited = false;

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
      heading_error_limited = true;
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
  const double speed_after_max_limit = v_cmd;

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
  const double speed_after_goal_slowdown = v_cmd;

  v_cmd = applyAccelerationLimit(v_cmd, last_v_cmd_, max_decel_, max_accel_, control_dt);
  const double speed_after_accel_limit = v_cmd;
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
  const double speed_after_command_filter = v_cmd;
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
  const double final_v_cmd = v_cmd;

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

    const char * speed_reason = "none";
    if (mode == ControlMode::INFEASIBLE_TIME_TRACKING) {
      speed_reason = "infeasible_fastest_tracking_v";
    } else if (heading_error_limited) {
      speed_reason = "heading_error";
    } else if (mode == ControlMode::GOAL_APPROACH) {
      if (speed_after_goal_slowdown + kEps < speed_after_max_limit) {
        speed_reason = "near_goal_slowdown";
      } else if (speed_after_accel_limit + kEps < speed_after_goal_slowdown) {
        speed_reason = "acceleration_limit";
      } else if (speed_after_command_filter + kEps < speed_after_accel_limit) {
        speed_reason = "command_filter";
      } else {
        speed_reason = "goal_approach_limit";
      }
    } else if (speed_after_goal_slowdown + kEps < speed_after_max_limit) {
      speed_reason = "near_goal_slowdown";
    } else if (speed_after_accel_limit + kEps < speed_after_goal_slowdown) {
      speed_reason = "acceleration_limit";
    } else if (speed_after_command_filter + kEps < speed_after_accel_limit) {
      speed_reason = "command_filter";
    } else if (speed_after_max_limit + kEps < speed_after_lqr) {
      speed_reason = "max_speed_limit";
    } else if (time_aggressiveness_applied) {
      speed_reason = "time_aggressiveness";
    }

    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "[time_controller] dist=%.2fm path_rem=%.2fm time_left=%.2fs req_v=%.3f "
      "aggr=%.2f aggr_raw=%.3f aggr_v=%.3f sched_err=%.2fm raw_v=%.3f "
      "time_v=%.3f curve_v=%.3f lqr_v=%.3f max_v_stage=%.3f "
      "slow_v=%.3f accel_v=%.3f final_v=%.3f final_w=%.3f max_v_param=%.3f "
      "fastest_v_param=%.3f odom_vx=%.3f mode=%s reason=%s",
      distance_to_goal, path_remaining, requested_time_remaining, required_average_speed,
      time_aggressiveness_, aggressive_speed_before_clamp, aggressive_speed_after_clamp,
      schedule_error_s, raw_desired_linear_speed, speed_after_time_logic,
      speed_after_curvature_limit, speed_after_lqr, speed_after_max_limit,
      speed_after_goal_slowdown, speed_after_accel_limit, final_v_cmd, w_cmd,
      max_v_, fastest_tracking_v_, velocity.linear.x, modeToString(mode), speed_reason);
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
