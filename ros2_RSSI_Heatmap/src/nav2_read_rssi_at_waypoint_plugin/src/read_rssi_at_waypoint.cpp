#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

#include "nav2_read_rssi_at_waypoint/read_rssi_at_waypoint.hpp"
#include "nav2_read_rssi_at_waypoint/simulated_rssi.hpp"
#include "nav2_util/node_utils.hpp"
#include "rosbot_interfaces/msg/rssi_at_waypoint.hpp"

namespace nav2_read_rssi_at_waypoint
{

ReadRssiAtWaypoint::ReadRssiAtWaypoint()
: is_enabled_(true), n_measurements_(10),
  aps_x_({0.0}), aps_y_({0.0}), tx_powers_({-30.0}), path_loss_exponents_({3.0}) {}

ReadRssiAtWaypoint::~ReadRssiAtWaypoint() {}

void ReadRssiAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in ReadRssiAtWaypoint plugin!"};
  }
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Initializing ReadRssiAtWaypoint (simulation mode)...");

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".number_of_measurements", rclcpp::ParameterValue(10));
  // Simulated Access Point parameters — one entry per AP, supports multiple APs
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".ap_x", rclcpp::ParameterValue(std::vector<double>{0.0}));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".ap_y", rclcpp::ParameterValue(std::vector<double>{0.0}));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".tx_power", rclcpp::ParameterValue(std::vector<double>{-30.0}));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".path_loss_exponent", rclcpp::ParameterValue(std::vector<double>{3.0}));

  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".number_of_measurements", n_measurements_);
  node->get_parameter(plugin_name + ".ap_x", aps_x_);
  node->get_parameter(plugin_name + ".ap_y", aps_y_);
  node->get_parameter(plugin_name + ".tx_power", tx_powers_);
  node->get_parameter(plugin_name + ".path_loss_exponent", path_loss_exponents_);

  if (aps_y_.size() != aps_x_.size()) {
    throw std::runtime_error{"ap_x and ap_y parameters must have the same length!"};
  }

  if (n_measurements_ == 0) {
    is_enabled_ = false;
  }

  if (is_enabled_) {
    RCLCPP_INFO(logger_,
      "Simulated RSSI plugin enabled  —  %zu access point(s) configured",
      aps_x_.size());
    for (size_t i = 0; i < aps_x_.size(); ++i) {
      RCLCPP_INFO(logger_, "  AP %zu at (%.1f, %.1f)", i, aps_x_[i], aps_y_[i]);
    }
    rssi_data_publisher =
      node->create_publisher<rosbot_interfaces::msg::RssiAtWaypoint>("rssi_data", 10);
    rssi_data_publisher->on_activate();
  }
}

namespace
{
double param_at(const std::vector<double> & values, size_t index)
{
  if (values.empty()) {
    return 0.0;
  }
  return values[index < values.size() ? index : values.size() - 1];
}
}  // namespace

bool ReadRssiAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose,
  const int & curr_pose_index)
{
  (void)curr_pose_index;
  if (!is_enabled_) {
    return true;
  }

  RCLCPP_INFO(logger_, "Measuring simulated RSSI at waypoint %d ...", curr_pose_index);

  double robot_x = curr_pose.pose.position.x;
  double robot_y = curr_pose.pose.position.y;

  auto msg = rosbot_interfaces::msg::RssiAtWaypoint();
  msg.coordinates.x = robot_x;
  msg.coordinates.y = robot_y;
  msg.coordinates.z = curr_pose.pose.position.z;

  int best_rssi = -101;
  size_t best_ap = 0;

  for (size_t ap = 0; ap < aps_x_.size(); ++ap) {
    double tx_power = param_at(tx_powers_, ap);
    double path_loss_exponent = param_at(path_loss_exponents_, ap);

    int total_rssi = 0;
    int valid_count = 0;

    for (int i = 0; i < n_measurements_; ++i) {
      int sample = simulate_rssi(
        robot_x, robot_y, aps_x_[ap], aps_y_[ap], tx_power, path_loss_exponent);
      if (sample <= 0) {
        total_rssi += sample;
        ++valid_count;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    int avg_rssi = valid_count > 0 ?
      static_cast<int>(std::lround(static_cast<double>(total_rssi) / valid_count)) :
      -100;

    if (avg_rssi > best_rssi) {
      best_rssi = avg_rssi;
      best_ap = ap;
    }
  }

  msg.rssi = best_rssi;
  msg.serving_ap = static_cast<int8_t>(best_ap);

  RCLCPP_INFO(logger_, "  RSSI = %d dBm at (%.2f, %.2f), served by AP %zu",
    msg.rssi, robot_x, robot_y, best_ap);
  rssi_data_publisher->publish(msg);

  return true;
}

}  // namespace nav2_read_rssi_at_waypoint

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_read_rssi_at_waypoint::ReadRssiAtWaypoint,
  nav2_core::WaypointTaskExecutor)
