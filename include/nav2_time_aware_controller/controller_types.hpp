#ifndef NAV2_TIME_AWARE_CONTROLLER__CONTROLLER_TYPES_HPP_
#define NAV2_TIME_AWARE_CONTROLLER__CONTROLLER_TYPES_HPP_

#include <cstddef>

#include "geometry_msgs/msg/point.hpp"

namespace nav2_time_aware_controller
{

struct ReferencePoint
{
  double t {0.0};
  double s {0.0};
  double v {0.0};
  double yaw {0.0};
  double kappa {0.0};
  double w {0.0};
};

struct ProjectionResult
{
  double s {0.0};
  geometry_msgs::msg::Point point;
  double yaw {0.0};
  double kappa {0.0};
  std::size_t segment_index {0};
  double e_y {0.0};
};

enum class ControlMode
{
  NORMAL_LQR_TRACKING,
  INFEASIBLE_TIME_TRACKING,
  GOAL_APPROACH,
  GOAL_REACHED
};

}  // namespace nav2_time_aware_controller

#endif
