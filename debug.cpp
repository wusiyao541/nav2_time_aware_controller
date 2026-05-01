#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include <string>

#include "tf2/utils.h"

namespace nav2_time_aware_controller
{

void TimeAwareLQRController::publishDebugVisualization(
  const ProjectionResult & projection,
  const ReferencePoint & reference,
  const rclcpp::Time & stamp) const
{
  const std::string frame_id = global_plan_.header.frame_id.empty() ?
    "map" : global_plan_.header.frame_id;

  if (projected_point_pub_ && projected_point_pub_->is_activated()) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.point = projection.point;
    projected_point_pub_->publish(msg);
  }

  if (reference_point_pub_ && reference_point_pub_->is_activated()) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.point = samplePathAtArcLength(reference.s);
    reference_point_pub_->publish(msg);
  }

  if (debug_path_pub_ && debug_path_pub_->is_activated()) {
    debug_path_pub_->publish(global_plan_);
  }
}

void TimeAwareLQRController::writeCsvDebug(
  const rclcpp::Time & stamp,
  ControlMode mode,
  const geometry_msgs::msg::PoseStamped & pose,
  const ProjectionResult & projection,
  const ReferencePoint & reference,
  double e_s,
  double e_y,
  double e_psi,
  double delta_v,
  double delta_w,
  double v_cmd,
  double w_cmd,
  double distance_to_goal)
{
  if (!debug_csv_stream_.is_open()) {
    return;
  }

  if (!debug_csv_header_written_) {
    debug_csv_stream_
      << "time,mode,robot_x,robot_y,robot_yaw,projected_s,reference_s,"
      << "e_s,e_y,e_psi,v_ref,w_ref,delta_v,delta_w,v_cmd,w_cmd,"
      << "distance_to_goal,kappa_ref\n";
    debug_csv_header_written_ = true;
  }

  debug_csv_stream_
    << stamp.seconds() << ','
    << modeToString(mode) << ','
    << pose.pose.position.x << ','
    << pose.pose.position.y << ','
    << tf2::getYaw(pose.pose.orientation) << ','
    << projection.s << ','
    << reference.s << ','
    << e_s << ','
    << e_y << ','
    << e_psi << ','
    << reference.v << ','
    << reference.w << ','
    << delta_v << ','
    << delta_w << ','
    << v_cmd << ','
    << w_cmd << ','
    << distance_to_goal << ','
    << projection.kappa << '\n';
}

} 