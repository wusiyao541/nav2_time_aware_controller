#include "nav2_time_aware_controller/time_aware_lqr_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace nav2_time_aware_controller
{

TimeAwareLQRController::TimeAwareLQRController()
{
}

TimeAwareLQRController::~TimeAwareLQRController()
{
  if (debug_csv_stream_.is_open()) {
    debug_csv_stream_.close();
  }
}

}

PLUGINLIB_EXPORT_CLASS(nav2_time_aware_controller::TimeAwareLQRController, nav2_core::Controller)
