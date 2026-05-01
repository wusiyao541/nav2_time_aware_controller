#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"
#include "nav2_time_aware_controller/controller_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_time_aware_controller
{

bool TimeAwareLQRController::computePathArcLengths()
{
  path_s_.clear();
  if (global_plan_.poses.empty()) {
    return false;
  }

  path_s_.reserve(global_plan_.poses.size());
  path_s_.push_back(0.0);
  for (size_t i = 1; i < global_plan_.poses.size(); ++i) {
    const double ds = nav2_util::geometry_utils::euclidean_distance(
      global_plan_.poses[i - 1].pose, global_plan_.poses[i].pose);
    path_s_.push_back(path_s_.back() + ds);
  }

  return path_s_.size() == global_plan_.poses.size();
}

void TimeAwareLQRController::computePathTangents()
{
  path_yaws_.assign(global_plan_.poses.size(), 0.0);
  if (global_plan_.poses.size() < 2) {
    return;
  }

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    size_t prev = i == 0 ? i : i - 1;
    size_t next = i + 1 >= global_plan_.poses.size() ? i : i + 1;
    if (prev == next) {
      next = std::min(i + 1, global_plan_.poses.size() - 1);
    }

    const auto & p0 = global_plan_.poses[prev].pose.position;
    const auto & p1 = global_plan_.poses[next].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    path_yaws_[i] = std::atan2(dy, dx);
  }
}

void TimeAwareLQRController::computePathCurvatures()
{
  path_curvatures_.assign(global_plan_.poses.size(), 0.0);
  if (global_plan_.poses.size() < 3 || path_yaws_.size() != global_plan_.poses.size()) {
    return;
  }

  for (size_t i = 1; i + 1 < global_plan_.poses.size(); ++i) {
    const double ds = path_s_[i + 1] - path_s_[i - 1];
    if (ds <= kEps) {
      path_curvatures_[i] = 0.0;
      continue;
    }
    const double dyaw = angles::shortest_angular_distance(path_yaws_[i - 1], path_yaws_[i + 1]);
    path_curvatures_[i] = dyaw / ds;
  }

  path_curvatures_.front() = path_curvatures_[1];
  path_curvatures_.back() = path_curvatures_[path_curvatures_.size() - 2];
}

geometry_msgs::msg::Point TimeAwareLQRController::samplePathAtArcLength(double s) const
{
  geometry_msgs::msg::Point result;
  if (global_plan_.poses.empty()) {
    return result;
  }
  if (global_plan_.poses.size() == 1 || path_s_.empty()) {
    return global_plan_.poses.front().pose.position;
  }

  s = clamp(s, 0.0, path_s_.back());
  auto upper = std::lower_bound(path_s_.begin(), path_s_.end(), s);
  if (upper == path_s_.begin()) {
    return global_plan_.poses.front().pose.position;
  }
  if (upper == path_s_.end()) {
    return global_plan_.poses.back().pose.position;
  }

  const size_t idx = static_cast<size_t>(std::distance(path_s_.begin(), upper));
  const size_t prev = idx - 1;
  const double s0 = path_s_[prev];
  const double s1 = path_s_[idx];
  const double ratio = (s1 - s0 <= kEps) ? 0.0 : (s - s0) / (s1 - s0);

  const auto & p0 = global_plan_.poses[prev].pose.position;
  const auto & p1 = global_plan_.poses[idx].pose.position;
  result.x = p0.x + ratio * (p1.x - p0.x);
  result.y = p0.y + ratio * (p1.y - p0.y);
  result.z = p0.z + ratio * (p1.z - p0.z);
  return result;
}

double TimeAwareLQRController::sampleYawAtArcLength(double s) const
{
  if (path_yaws_.empty() || path_s_.empty()) {
    return 0.0;
  }
  if (path_yaws_.size() == 1) {
    return path_yaws_.front();
  }

  s = clamp(s, 0.0, path_s_.back());
  auto upper = std::lower_bound(path_s_.begin(), path_s_.end(), s);
  if (upper == path_s_.begin()) {
    return path_yaws_.front();
  }
  if (upper == path_s_.end()) {
    return path_yaws_.back();
  }

  const size_t idx = static_cast<size_t>(std::distance(path_s_.begin(), upper));
  const size_t prev = idx - 1;
  const double s0 = path_s_[prev];
  const double s1 = path_s_[idx];
  const double ratio = (s1 - s0 <= kEps) ? 0.0 : (s - s0) / (s1 - s0);
  const double dyaw = angles::shortest_angular_distance(path_yaws_[prev], path_yaws_[idx]);
  return angles::normalize_angle(path_yaws_[prev] + ratio * dyaw);
}

double TimeAwareLQRController::sampleCurvatureAtArcLength(double s) const
{
  if (path_curvatures_.empty() || path_s_.empty()) {
    return 0.0;
  }
  if (path_curvatures_.size() == 1) {
    return path_curvatures_.front();
  }

  s = clamp(s, 0.0, path_s_.back());
  auto upper = std::lower_bound(path_s_.begin(), path_s_.end(), s);
  if (upper == path_s_.begin()) {
    return path_curvatures_.front();
  }
  if (upper == path_s_.end()) {
    return path_curvatures_.back();
  }

  const size_t idx = static_cast<size_t>(std::distance(path_s_.begin(), upper));
  const size_t prev = idx - 1;
  const double s0 = path_s_[prev];
  const double s1 = path_s_[idx];
  const double ratio = (s1 - s0 <= kEps) ? 0.0 : (s - s0) / (s1 - s0);
  return path_curvatures_[prev] + ratio * (path_curvatures_[idx] - path_curvatures_[prev]);
}

ProjectionResult TimeAwareLQRController::projectPoseToPath(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  ProjectionResult result;
  if (global_plan_.poses.size() < 2 || path_s_.size() != global_plan_.poses.size()) {
    return result;
  }

  const auto & position = pose.pose.position;
  const size_t last_segment = global_plan_.poses.size() - 2;
  size_t start_segment = 0;
  size_t end_segment = last_segment;
  if (have_last_projection_ && last_closest_segment_idx_ <= last_segment) {
    constexpr size_t kSearchWindow = 30;
    start_segment = last_closest_segment_idx_ > kSearchWindow ?
      last_closest_segment_idx_ - kSearchWindow : 0;
    end_segment = std::min(last_segment, last_closest_segment_idx_ + kSearchWindow);
  }

  double best_s = 0.0;
  double best_dist_sq = std::numeric_limits<double>::max();
  size_t best_segment = start_segment;

  for (size_t i = start_segment; i <= end_segment; ++i) {
    const auto & p0 = global_plan_.poses[i].pose.position;
    const auto & p1 = global_plan_.poses[i + 1].pose.position;
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double segment_len_sq = vx * vx + vy * vy;
    if (segment_len_sq <= kEps) {
      continue;
    }

    const double wx = position.x - p0.x;
    const double wy = position.y - p0.y;
    const double ratio = clamp((wx * vx + wy * vy) / segment_len_sq, 0.0, 1.0);
    const double proj_x = p0.x + ratio * vx;
    const double proj_y = p0.y + ratio * vy;
    const double dx = position.x - proj_x;
    const double dy = position.y - proj_y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_segment = i;
      best_s = path_s_[i] + ratio * std::sqrt(segment_len_sq);
    }
  }

  best_s = clamp(best_s, 0.0, path_s_.back());
  if (!allow_progress_backtracking_ && have_last_projection_) {
    best_s = std::max(best_s, last_projected_s_ - max_backtrack_dist_);
    best_s = clamp(best_s, 0.0, path_s_.back());
  }

  auto upper = std::lower_bound(path_s_.begin(), path_s_.end(), best_s);
  if (upper == path_s_.begin()) {
    best_segment = 0;
  } else if (upper == path_s_.end()) {
    best_segment = last_segment;
  } else {
    best_segment = static_cast<size_t>(std::distance(path_s_.begin(), upper) - 1);
    best_segment = std::min(best_segment, last_segment);
  }

  result.s = best_s;
  result.point = samplePathAtArcLength(best_s);
  result.yaw = sampleYawAtArcLength(best_s);
  result.kappa = sampleCurvatureAtArcLength(best_s);
  result.segment_index = best_segment;

  const double dx = position.x - result.point.x;
  const double dy = position.y - result.point.y;
  const double normal_left_x = -std::sin(result.yaw);
  const double normal_left_y = std::cos(result.yaw);
  result.e_y = dx * normal_left_x + dy * normal_left_y;

  last_projected_s_ = result.s;
  last_closest_segment_idx_ = result.segment_index;
  have_last_projection_ = true;
  return result;
}

double TimeAwareLQRController::computeDistanceToGoal(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (!have_controller_goal_ && global_plan_.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  if (have_controller_goal_) {
    const double dx = pose.pose.position.x - controller_goal_pose_.position.x;
    const double dy = pose.pose.position.y - controller_goal_pose_.position.y;
    return std::hypot(dx, dy);
  }

  return nav2_util::geometry_utils::euclidean_distance(
    pose.pose, global_plan_.poses.back().pose);
}

double TimeAwareLQRController::computeDistanceToPathEnd(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (global_plan_.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  return nav2_util::geometry_utils::euclidean_distance(
    pose.pose, global_plan_.poses.back().pose);
}

double TimeAwareLQRController::computePathEndToControllerGoalDistance() const
{
  if (!have_controller_goal_ || global_plan_.poses.empty()) {
    return 0.0;
  }

  const auto & path_end = global_plan_.poses.back().pose.position;
  const auto & goal = controller_goal_pose_.position;
  return std::hypot(path_end.x - goal.x, path_end.y - goal.y);
}

void TimeAwareLQRController::updateControllerGoalFromPlan(
  const nav_msgs::msg::Path & path,
  const rclcpp::Time & now)
{
  if (path.poses.empty()) {
    return;
  }

  const auto & path_goal = path.poses.back().pose;
  bool new_task_goal = !have_controller_goal_;
  if (have_controller_goal_) {
    const double dx = path_goal.position.x - controller_goal_pose_.position.x;
    const double dy = path_goal.position.y - controller_goal_pose_.position.y;
    new_task_goal = std::hypot(dx, dy) > kNewGoalDistance;
  }

  if (new_task_goal) {
    controller_goal_pose_ = path_goal;
    have_controller_goal_ = true;
    task_start_time_ = now;
    task_started_ = true;

    auto node = node_.lock();
    if (node) {
      RCLCPP_INFO(
        node->get_logger(),
        "%s set controller goal from path endpoint: x=%.3f, y=%.3f.",
        plugin_name_.c_str(), controller_goal_pose_.position.x, controller_goal_pose_.position.y);
    }
  }
}

}