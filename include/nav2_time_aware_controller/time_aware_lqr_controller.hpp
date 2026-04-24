#ifndef NAV2_TIME_AWARE_CONTROLLER__TIME_AWARE_LQR_CONTROLLER_HPP_
#define NAV2_TIME_AWARE_CONTROLLER__TIME_AWARE_LQR_CONTROLLER_HPP_

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_time_aware_controller/controller_types.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_time_aware_controller
{

class TimeAwareLQRController : public nav2_core::Controller
{
public:
  TimeAwareLQRController();
  ~TimeAwareLQRController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void resetPlanState();
  void resetCommandState();

  bool computePathArcLengths();
  void computePathTangents();
  void computePathCurvatures();
  void generateReferenceTrajectory(double requested_time);

  geometry_msgs::msg::Point samplePathAtArcLength(double s) const;
  double sampleYawAtArcLength(double s) const;
  double sampleCurvatureAtArcLength(double s) const;
  ReferencePoint sampleReferenceAtTime(double elapsed_time) const;
  ProjectionResult projectPoseToPath(const geometry_msgs::msg::PoseStamped & pose) const;

  std::pair<double, double> solveLQR(
    double e_s,
    double e_y,
    double e_psi,
    double v_ref,
    double dt) const;

  double computeDistanceToGoal(const geometry_msgs::msg::PoseStamped & pose) const;
  double computeDistanceToPathEnd(const geometry_msgs::msg::PoseStamped & pose) const;
  double computePathEndToControllerGoalDistance() const;
  void updateControllerGoalFromPlan(const nav_msgs::msg::Path & path, const rclcpp::Time & now);
  double computeGoalSlowdownScale(double distance_to_goal) const;
  double applyAccelerationLimit(
    double target_value,
    double last_value,
    double negative_rate_limit,
    double positive_rate_limit,
    double dt) const;
  double clamp(double value, double lower, double upper) const;
  const char * modeToString(ControlMode mode) const;

  void publishDebugVisualization(
    const ProjectionResult & projection,
    const ReferencePoint & reference,
    const rclcpp::Time & stamp) const;
  void writeCsvDebug(
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
    double distance_to_goal);
  geometry_msgs::msg::TwistStamped makeZeroCommand(const rclcpp::Time & stamp) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
    projected_point_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
    reference_point_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;

  std::string plugin_name_;
  std::string base_frame_id_ {"base_link"};

  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::Pose controller_goal_pose_;
  std::vector<double> path_s_;
  std::vector<double> path_yaws_;
  std::vector<double> path_curvatures_;
  std::vector<ReferencePoint> reference_traj_;

  rclcpp::Time task_start_time_;
  rclcpp::Time plan_start_time_;
  rclcpp::Time last_cmd_time_;
  bool active_ {false};
  bool valid_plan_ {false};
  bool task_started_ {false};
  bool have_last_cmd_time_ {false};
  bool plan_fastest_mode_ {false};
  bool have_controller_goal_ {false};

  mutable bool have_last_projection_ {false};
  mutable size_t last_closest_segment_idx_ {0};
  mutable double last_projected_s_ {0.0};

  double desired_total_time_ {20.0};
  double controller_frequency_ {10.0};
  double max_v_ {0.3};
  double original_max_v_ {0.3};
  double min_tracking_v_ {0.06};
  double max_w_ {1.0};
  double max_accel_ {0.25};
  double max_decel_ {0.45};
  double max_angular_accel_ {1.2};
  double command_filter_alpha_ {0.6};

  double q_s_ {1.0};
  double q_y_ {12.0};
  double q_psi_ {8.0};
  double r_v_ {3.0};
  double r_w_ {2.5};
  double qf_s_ {2.0};
  double qf_y_ {16.0};
  double qf_psi_ {10.0};
  int lqr_horizon_steps_ {20};
  double max_delta_v_ {0.12};
  double max_delta_w_ {0.8};

  double goal_tolerance_ {0.18};
  double goal_slowdown_dist_ {0.35};
  double goal_approach_dist_ {0.45};
  double goal_approach_gain_ {0.4};
  double goal_heading_gain_ {1.0};
  double goal_heading_stop_angle_ {0.8};
  double min_goal_approach_v_ {0.02};
  double max_goal_approach_v_ {0.08};
  double max_goal_approach_w_ {0.4};
  double near_path_end_dist_ {0.3};

  bool allow_overtime_ {true};
  double fastest_tracking_v_ {0.18};
  double min_overtime_horizon_ {5.0};

  bool allow_progress_backtracking_ {false};
  double max_backtrack_dist_ {0.05};

  bool debug_ {false};
  bool invert_lateral_error_sign_ {false};
  bool invert_heading_error_sign_ {false};
  bool invert_delta_w_sign_ {false};
  std::string debug_csv_path_;
  std::ofstream debug_csv_stream_;
  bool debug_csv_header_written_ {false};

  double last_v_cmd_ {0.0};
  double last_w_cmd_ {0.0};
};

}  // namespace nav2_time_aware_controller

#endif
