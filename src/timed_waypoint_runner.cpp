#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class TimedWaypointRunner : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  struct Waypoint
  {
    std::string name;
    double x;
    double y;
    double yaw;
    double dwell_time;
    double target_time;
  };

  TimedWaypointRunner()
  : Node("timed_waypoint_runner")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
    total_loops_ = declare_parameter<int>("total_loops", 1);
    controller_parameter_node_ = declare_parameter<std::string>(
      "controller_parameter_node", "controller_server");
    controller_target_time_parameter_ = declare_parameter<std::string>(
      "controller_target_time_parameter", "FollowPath.desired_segment_time");
    set_controller_target_time_ = declare_parameter<bool>("set_controller_target_time", true);

    if (total_loops_ < 1) {
      RCLCPP_WARN(
        get_logger(), "total_loops must be >= 1. Using total_loops=1.");
      total_loops_ = 1;
    }

    load_waypoints();
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    controller_parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      this, controller_parameter_node_);
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

    bool saw_failed_waypoint = false;

    for (int loop = 1; rclcpp::ok() && loop <= total_loops_; ++loop) {
      RCLCPP_INFO(get_logger(), "Starting loop %d/%d", loop, total_loops_);

      for (std::size_t i = 0; rclcpp::ok() && i < waypoints_.size(); ++i) {
        const auto & waypoint = waypoints_[i];

        if (std::isfinite(waypoint.target_time)) {
          RCLCPP_INFO(
            get_logger(),
            "Sending waypoint %zu/%zu '%s' at x=%.3f, y=%.3f, yaw=%.3f, target_time=%.3f s.",
            i + 1, waypoints_.size(), waypoint.name.c_str(),
            waypoint.x, waypoint.y, waypoint.yaw, waypoint.target_time);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Sending waypoint %zu/%zu '%s' at x=%.3f, y=%.3f, yaw=%.3f.",
            i + 1, waypoints_.size(), waypoint.name.c_str(),
            waypoint.x, waypoint.y, waypoint.yaw);
        }

        apply_target_time(waypoint);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = make_pose(waypoint);

        auto goal_handle_future = action_client_->async_send_goal(goal_msg);
        const auto goal_status = rclcpp::spin_until_future_complete(
          shared_from_this(), goal_handle_future);

        if (goal_status != rclcpp::FutureReturnCode::SUCCESS) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(
              get_logger(), "Interrupted while sending waypoint '%s'.",
              waypoint.name.c_str());
            return false;
          }
          saw_failed_waypoint = true;
          RCLCPP_ERROR(
            get_logger(), "Failed while sending waypoint '%s'; continuing to the next waypoint.",
            waypoint.name.c_str());
          continue;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
          saw_failed_waypoint = true;
          RCLCPP_ERROR(
            get_logger(),
            "Waypoint '%s' was rejected by the action server; continuing to the next waypoint.",
            waypoint.name.c_str());
          continue;
        }

        auto result_future = action_client_->async_get_result(goal_handle);
        const auto result_status = rclcpp::spin_until_future_complete(
          shared_from_this(), result_future);

        if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(
              get_logger(), "Interrupted while waiting for waypoint '%s' result.",
              waypoint.name.c_str());
            return false;
          }
          saw_failed_waypoint = true;
          RCLCPP_ERROR(
            get_logger(),
            "Failed while waiting for waypoint '%s' result; continuing to the next waypoint.",
            waypoint.name.c_str());
          continue;
        }

        const auto result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          saw_failed_waypoint = true;
          RCLCPP_ERROR(
            get_logger(), "Waypoint '%s' finished with status '%s'; continuing to the next waypoint.",
            waypoint.name.c_str(), result_code_to_string(result.code).c_str());
          continue;
        }

        RCLCPP_INFO(get_logger(), "Waypoint reached: '%s'.", waypoint.name.c_str());

        if (waypoint.dwell_time > 0.0) {
          RCLCPP_INFO(
            get_logger(), "Dwelling at waypoint '%s' for %.3f seconds.",
            waypoint.name.c_str(), waypoint.dwell_time);
          rclcpp::sleep_for(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(waypoint.dwell_time)));
        }
      }

      RCLCPP_INFO(get_logger(), "Loop %d completed", loop);
    }

    RCLCPP_INFO(get_logger(), "All loops completed");
    if (saw_failed_waypoint) {
      RCLCPP_WARN(
        get_logger(),
        "One or more waypoints failed, but the requested loop count was completed.");
    }
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
      waypoint.dwell_time = read_optional_double(prefix + "dwell_time", 0.0);
      waypoint.target_time = read_waypoint_target_time(prefix);

      if (waypoint.dwell_time < 0.0) {
        throw std::runtime_error(
          "Waypoint '" + name + "' has negative dwell_time; dwell_time must be >= 0.0");
      }
      if (std::isfinite(waypoint.target_time) && waypoint.target_time <= 0.0) {
        throw std::runtime_error(
          "Waypoint '" + name + "' has non-positive target_time; target_time must be > 0.0");
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

  double read_optional_double(const std::string & parameter_name, double default_value)
  {
    return declare_parameter<double>(parameter_name, default_value);
  }

  double read_waypoint_target_time(const std::string & prefix)
  {
    const double missing_value = std::numeric_limits<double>::quiet_NaN();
    const double target_time = declare_parameter<double>(prefix + "target_time", missing_value);

    if (std::isfinite(target_time)) {
      return target_time;
    }

    return missing_value;
  }

  bool apply_target_time(const Waypoint & waypoint)
  {
    if (!set_controller_target_time_) {
      return true;
    }
    if (!std::isfinite(waypoint.target_time)) {
      return true;
    }

    if (!controller_parameters_client_->wait_for_service(1s)) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter service for '%s' is not available; waypoint '%s' will use the controller's current target time.",
        controller_parameter_node_.c_str(), waypoint.name.c_str());
      return false;
    }

    if (set_controller_time_parameter(
        controller_target_time_parameter_, waypoint.target_time, waypoint.name))
    {
      return true;
    }

    return false;
  }

  bool set_controller_time_parameter(
    const std::string & parameter_name,
    double target_time,
    const std::string & waypoint_name)
  {
    try {
      const auto results = controller_parameters_client_->set_parameters(
        {rclcpp::Parameter(parameter_name, target_time)});

      if (results.empty() || !results.front().successful) {
        const std::string reason = results.empty() ?
          "no result returned" : results.front().reason;
        RCLCPP_WARN(
          get_logger(),
          "Failed to set %s.%s to %.3f s for waypoint '%s': %s.",
          controller_parameter_node_.c_str(), parameter_name.c_str(),
          target_time, waypoint_name.c_str(), reason.c_str());
        return false;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Exception while setting %s.%s for waypoint '%s': %s.",
        controller_parameter_node_.c_str(), parameter_name.c_str(),
        waypoint_name.c_str(), ex.what());
      return false;
    }

    return true;
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
  int total_loops_;
  std::string controller_parameter_node_;
  std::string controller_target_time_parameter_;
  bool set_controller_target_time_;
  std::vector<Waypoint> waypoints_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::SyncParametersClient::SharedPtr controller_parameters_client_;
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
