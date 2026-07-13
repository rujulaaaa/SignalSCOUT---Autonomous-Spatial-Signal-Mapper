#ifndef NAV2_READ_RSSI_AT_WAYPOINT__READ_RSSI_AT_WAYPOINT_HPP_
#define NAV2_READ_RSSI_AT_WAYPOINT__READ_RSSI_AT_WAYPOINT_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "rosbot_interfaces/msg/rssi_at_waypoint.hpp"

namespace nav2_read_rssi_at_waypoint
{

class ReadRssiAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  ReadRssiAtWaypoint();
  ~ReadRssiAtWaypoint();

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_pose_index) override;

protected:
  bool is_enabled_;
  int n_measurements_;
  // Simulated Access Point parameters (one entry per AP; supports multiple APs)
  std::vector<double> aps_x_;
  std::vector<double> aps_y_;
  std::vector<double> tx_powers_;
  std::vector<double> path_loss_exponents_;

  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<rosbot_interfaces::msg::RssiAtWaypoint>>
    rssi_data_publisher;
};

}  // namespace nav2_read_rssi_at_waypoint

#endif  // NAV2_READ_RSSI_AT_WAYPOINT__READ_RSSI_AT_WAYPOINT_HPP_
