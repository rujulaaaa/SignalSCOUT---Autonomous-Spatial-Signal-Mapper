#ifndef SIMULATED_RSSI_HPP_
#define SIMULATED_RSSI_HPP_

#include <cmath>
#include <random>

/**
 * Simulate WiFi RSSI using a log-distance path-loss model.
 *
 *   RSSI = tx_power - 10 * n * log10(d / d0) + noise
 *
 * @param robot_x  Robot X position in map frame
 * @param robot_y  Robot Y position in map frame
 * @param ap_x     Access-point X position
 * @param ap_y     Access-point Y position
 * @param tx_power Transmit power (dBm) at reference distance
 * @param n        Path-loss exponent (2=free-space, 3-4=indoor)
 */
inline int simulate_rssi(
  double robot_x, double robot_y,
  double ap_x, double ap_y,
  double tx_power = -30.0,
  double n = 3.0)
{
  double dx = robot_x - ap_x;
  double dy = robot_y - ap_y;
  double dist = std::sqrt(dx * dx + dy * dy);

  const double d0 = 1.0;
  if (dist < d0) {
    dist = d0;
  }

  double rssi = tx_power - 10.0 * n * std::log10(dist / d0);

  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::normal_distribution<double> noise(0.0, 2.0);
  rssi += noise(gen);

  int rssi_rounded = static_cast<int>(std::lround(rssi));

  if (rssi_rounded > 0) {
    rssi_rounded = 0;
  } else if (rssi_rounded < -100) {
    rssi_rounded = -100;
  }

  return rssi_rounded;
}

#endif  // SIMULATED_RSSI_HPP_
