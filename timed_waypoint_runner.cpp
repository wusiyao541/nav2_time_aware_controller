#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class TimedWaypointRunner : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  struct Waypoint
  {
    std::string name;
    double x;
    double y;
    double yaw;
    double dwell_time;
  };

  TimedWaypointRunner()
  : Node("timed_waypoint_runner")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");

    load_waypoints();
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
  }

  bool run()
  {
    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "No waypoints configured. Nothing to run.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Waiting for NavigateToPose action server '%s'...",
      action_name_.c_str());

    while (rclcpp::ok() && !action_client_->wait_for_action_server(1s)) {
    }

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for action server.");
      return false;
    }

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & waypoint = waypoints_[i];

      RCLCPP_INFO(
        get_logger(),
        "Starting waypoint %zu/%zu '%s' at x=%.3f, y=%.3f, yaw=%.3f.",
        i + 1, waypoints_.size(), waypoint.name.c_str(),
        waypoint.x, waypoint.y, waypoint.yaw);

      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose = make_pose(waypoint);

      auto goal_handle_future = action_client_->async_send_goal(goal_msg);
      const auto goal_status = rclcpp::spin_until_future_complete(
        shared_from_this(), goal_handle_future);

      if (goal_status != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(), "Failed while sending waypoint '%s'. Stopping sequence.",
          waypoint.name.c_str());
        return false;
      }

      auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(
          get_logger(), "Waypoint '%s' was rejected by the action server. Stopping sequence.",
          waypoint.name.c_str());
        return false;
      }

      auto result_future = action_client_->async_get_result(goal_handle);
      const auto result_status = rclcpp::spin_until_future_complete(
        shared_from_this(), result_future);

      if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(), "Failed while waiting for waypoint '%s' result. Stopping sequence.",
          waypoint.name.c_str());
        return false;
      }

      const auto result = result_future.get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(
          get_logger(), "Waypoint '%s' finished with status '%s'. Stopping sequence.",
          waypoint.name.c_str(), result_code_to_string(result.code).c_str());
        return false;
      }

      RCLCPP_INFO(get_logger(), "Waypoint '%s' succeeded.", waypoint.name.c_str());

      if (waypoint.dwell_time > 0.0) {
        RCLCPP_INFO(
          get_logger(), "Dwelling at waypoint '%s' for %.3f seconds.",
          waypoint.name.c_str(), waypoint.dwell_time);
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(waypoint.dwell_time)));
      }
    }

    RCLCPP_INFO(get_logger(), "All waypoints completed.");
    return true;
  }

private:
  void load_waypoints()
  {
    const auto waypoint_names = declare_parameter<std::vector<std::string>>(
      "waypoints.names", std::vector<std::string>{});

    waypoints_.reserve(waypoint_names.size());

    for (const auto & name : waypoint_names) {
      if (name.empty()) {
        RCLCPP_ERROR(get_logger(), "Encountered waypoint with an empty name. Skipping it.");
        continue;
      }

      Waypoint waypoint;
      waypoint.name = name;

      const auto prefix = "waypoints." + name + ".";
      waypoint.x = read_required_double(prefix + "x");
      waypoint.y = read_required_double(prefix + "y");
      waypoint.yaw = read_required_double(prefix + "yaw");
      waypoint.dwell_time = read_required_double(prefix + "dwell_time");

      if (waypoint.dwell_time < 0.0) {
        throw std::runtime_error(
          "Waypoint '" + name + "' has negative dwell_time; dwell_time must be >= 0.0");
      }

      waypoints_.push_back(waypoint);
    }
  }

  double read_required_double(const std::string & parameter_name)
  {
    const double missing_value = std::numeric_limits<double>::quiet_NaN();
    const auto value = declare_parameter<double>(parameter_name, missing_value);

    if (!std::isfinite(value)) {
      throw std::runtime_error("Missing required waypoint parameter '" + parameter_name + "'");
    }

    return value;
  }

  geometry_msgs::msg::PoseStamped make_pose(const Waypoint & waypoint)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.stamp = now();
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, waypoint.yaw);
    orientation.normalize();

    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();

    return pose;
  }

  std::string result_code_to_string(rclcpp_action::ResultCode code) const
  {
    switch (code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return "SUCCEEDED";
      case rclcpp_action::ResultCode::ABORTED:
        return "ABORTED";
      case rclcpp_action::ResultCode::CANCELED:
        return "CANCELED";
      case rclcpp_action::ResultCode::UNKNOWN:
      default:
        return "UNKNOWN";
    }
  }

  std::string frame_id_;
  std::string action_name_;
  std::vector<Waypoint> waypoints_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  bool succeeded = false;
  try {
    auto node = std::make_shared<TimedWaypointRunner>();
    succeeded = node->run();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("timed_waypoint_runner"), "%s", ex.what());
  }

  rclcpp::shutdown();
  return succeeded ? 0 : 1;
}
